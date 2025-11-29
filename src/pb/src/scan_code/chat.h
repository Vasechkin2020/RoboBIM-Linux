// scan_node.cpp
// ROS1 node для детекции 4-х круглых столбов (диаметр читается из параметров).
// Реализованы:
//  - Сбор N сканов и агрегация (медиана по лучам)
//  - Предобработка (NaN отбрасывание, фильтрации)
//  - Детекторы: Gap detection, DBSCAN (XY clustering), Shadow detection
//  - Fit окружности (Taubin) -> центр, радиус, RMSE
//  - Matching (перестановки) внутри каждого метода
//  - Fusion (взвешенное усреднение по σ)
//  - Umeyama похожее преобразование (scale, rotate, translate) для приведения в глобальную систему
//  - Запись результатов в rosparam /pb_config/...
//
// Комментарии: всё подробно документировано.
// Логирование: используем logi.log(fmt, ...) (printf-like).
//
// Зависимости: ROS1 (roscpp, sensor_msgs), eigen3 (для SVD/матриц).
//
// Автор: ChatGPT (генерация по запросу пользователя)

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <numeric>
#include <functional>
#include <Eigen/Dense> // Для Umeyama / SVD
#include <tuple>
#include <chrono>
#include <sstream>
#include <iomanip>

// Предполагается, что у тебя есть асинхронный логгер объявленный где-то в проекте,
// как в предыдущих кусках кода: extern AsyncFileLogger logi;
extern AsyncFileLogger logi;

// Удобный макрос логирования (использует printf-подобный формат)
#define LOG(fmt, ...)                   \
    do                                  \
    {                                   \
        logi.log((fmt), ##__VA_ARGS__); \
    } while (0)

// ---- Константы ----
const double DEFAULT_MIN_RANGE = 0.05;
const double DEFAULT_MAX_RANGE = 30.0;

// ---- Вспомогательные структуры ----
struct Point2D
{
    double x;
    double y;
};

struct Cluster
{
    std::vector<Point2D> pts; // точки в XY
    std::vector<int> idxs;    // индексы лучей из исходного скана (если нужны)
    double angular_span_deg = 0.0;
    int N = 0;
    // Результат фитинга окружности:
    Point2D center{0.0, 0.0};
    double radius = 0.0;
    double rmse = 1e9;
    double sigma_center = 1.0; // оценка неопределенности центра (м)
    double confidence = 0.0;   // 0..1
    std::string method;        // "gap"/"dbscan"/"shadow"
};

// ---- Класс для агрегации сканов (медиана по лучам) ----
class ScanAggregator
{
public:
    ScanAggregator(int angular_steps)
        : angular_steps_(angular_steps)
    {
        buffer_.resize(angular_steps_);
    }

    // Добавить новый scan (rays в формате vector<double> размером angular_steps_)
    void addScan(const std::vector<double> &ranges)
    {
        if ((int)ranges.size() != angular_steps_)
        {
            LOG("[ScanAggregator] Ошибка: размер ranges != angular_steps (%d vs %ld)\n",
                angular_steps_, (long)ranges.size());
            return;
        }
        for (int i = 0; i < angular_steps_; ++i)
        {
            buffer_[i].push_back(ranges[i]);
            // ограничим глубину буфера по каждому лучу (необязательно, но гарантируем bounded mem)
            if (buffer_[i].size() > max_scans_)
                buffer_[i].pop_front();
        }
        collected_++;
    }

    // Вернуть true если накоплено >= need_scans
    bool ready() const
    {
        return collected_ >= need_scans_;
    }

    // Очистить накопленное
    void reset()
    {
        for (int i = 0; i < angular_steps_; ++i)
            buffer_[i].clear();
        collected_ = 0;
    }

    // Установить требуемое число сканов
    void setNeedScans(int n) { need_scans_ = n; }

    // Получить агрегированный (медианный) скан. Вернёт вектор размера angular_steps_
    std::vector<double> getMedianScan(double min_range = DEFAULT_MIN_RANGE, double max_range = DEFAULT_MAX_RANGE) const
    {
        std::vector<double> out(angular_steps_, std::numeric_limits<double>::quiet_NaN());
        for (int i = 0; i < angular_steps_; ++i)
        {
            auto tmp = buffer_[i];
            if ((int)tmp.size() == 0)
            {
                out[i] = std::numeric_limits<double>::quiet_NaN();
                continue;
            }
            // фильтруем NaN и внедиапазон
            std::vector<double> valid;
            valid.reserve(tmp.size());
            for (double v : tmp)
            {
                if (!std::isnan(v) && v >= min_range && v <= max_range)
                    valid.push_back(v);
            }
            if (valid.empty())
            {
                out[i] = std::numeric_limits<double>::quiet_NaN();
                continue;
            }
            std::nth_element(valid.begin(), valid.begin() + valid.size() / 2, valid.end());
            double med = valid[valid.size() / 2];
            out[i] = med;
        }
        return out;
    }

private:
    int angular_steps_;
    int need_scans_ = 100;
    size_t max_scans_ = 200;
    std::vector<std::deque<double>> buffer_;
    int collected_ = 0;
};

// ---- Предобработка: перевод в XY и базовые фильтры ----
class Preprocessor
{
public:
    Preprocessor(double angle_min, double angle_increment, double min_range, double max_range)
        : angle_min_(angle_min), angle_inc_(angle_increment),
          min_range_(min_range), max_range_(max_range) {}

    // Конвертация scan -> вектор XY (NaN пропускаются)
    void scanToXY(const std::vector<double> &ranges, std::vector<Point2D> &out_pts, std::vector<int> &out_idxs)
    {
        out_pts.clear();
        out_idxs.clear();
        int N = ranges.size();
        out_pts.reserve(N);
        for (int i = 0; i < N; ++i)
        {
            double r = ranges[i];
            if (std::isnan(r))
                continue;
            if (r < min_range_ || r > max_range_)
                continue;
            double angle = angle_min_ + i * angle_inc_;
            double x = r * cos(angle);
            double y = r * sin(angle);
            out_pts.push_back({x, y});
            out_idxs.push_back(i);
        }
    }

    // Возвращает XY точки, но сохраняя позиции для моментных сегментов (полезно для gap)
    void scanToXYFull(const std::vector<double> &ranges, std::vector<Point2D> &out_pts_full)
    {
        out_pts_full.resize(ranges.size());
        for (size_t i = 0; i < ranges.size(); ++i)
        {
            double r = ranges[i];
            double angle = angle_min_ + i * angle_inc_;
            if (std::isnan(r) || r < min_range_ || r > max_range_)
            {
                out_pts_full[i] = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
            }
            else
            {
                out_pts_full[i] = {r * cos(angle), r * sin(angle)};
            }
        }
    }

private:
    double angle_min_;
    double angle_inc_;
    double min_range_;
    double max_range_;
};

// ---- Detector 1: Gap detection (по разрывам по дальности) ----
class GapDetector
{
public:
    GapDetector(double gap_thresh, int min_pts, double min_angle_deg, double angle_inc_deg)
        : gap_thresh_(gap_thresh), min_pts_(min_pts), min_angle_deg_(min_angle_deg), angle_inc_deg_(angle_inc_deg) {}

    // Возвращает массив кластеров, используя входной агрегированный scan (ranges) в угловой последовательности
    std::vector<Cluster> detect(const std::vector<double> &ranges)
    {
        std::vector<Cluster> clusters;
        int N = ranges.size();
        if (N == 0)
            return clusters;

        // Найдём границы разрывов: большие скачки |r[i]-r[i+1]| > gap_thresh
        std::vector<int> break_idxs;
        for (int i = 0; i < N - 1; ++i)
        {
            double a = ranges[i], b = ranges[i + 1];
            if (std::isnan(a) || std::isnan(b))
                continue;
            if (std::abs(a - b) > gap_thresh_)
            {
                break_idxs.push_back(i);
            }
        }
        // Если нет крупных разрывов — возможно нет явно выделяемых сегментов
        if (break_idxs.empty())
        {
            // ничего не нашлось
            LOG("[GapDetector] Не найдено разрывов (break_idxs == 0)\n");
            return clusters;
        }

        // Между разрывами считаем сегменты: сегмент от (break_idxs[k]+1) до break_idxs[k+1]
        // Учитываем "кольцевой" случай? Для простоты считаем линейно (scan обычно -pi..+pi)
        int start = 0;
        for (size_t bi = 0; bi <= break_idxs.size(); ++bi)
        {
            int seg_start = (bi == 0) ? start : break_idxs[bi - 1] + 1;
            int seg_end = (bi < break_idxs.size()) ? break_idxs[bi] : N - 1;
            // Соберём точки внутри seg_start..seg_end
            Cluster cl;
            cl.method = "gap";
            for (int k = seg_start; k <= seg_end; ++k)
            {
                if (!std::isnan(ranges[k]))
                {
                    double angle = k * angle_inc_deg_ * M_PI / 180.0; // angle not used directly here
                    // мы сохраним индексы; точки создадим позже в fitter
                    cl.idxs.push_back(k);
                }
            }
            cl.N = cl.idxs.size();
            cl.angular_span_deg = (cl.N > 0) ? (cl.N * angle_inc_deg_) : 0.0;
            if (cl.N >= min_pts_ && cl.angular_span_deg >= min_angle_deg_)
            {
                clusters.push_back(cl);
                LOG("[GapDetector] Найден кластер gap: idxs %d..%d, N=%d, span=%.2fdeg\n",
                    (int)cl.idxs.front(), (int)cl.idxs.back(), cl.N, cl.angular_span_deg);
            }
            else
            {
                LOG("[GapDetector] Отброшен маленький сегмент gap: N=%d span=%.2fdeg\n", cl.N, cl.angular_span_deg);
            }
        }
        LOG("[GapDetector] Всего кластеров найдено: %zu\n", clusters.size());
        return clusters;
    }

private:
    double gap_thresh_;
    int min_pts_;
    double min_angle_deg_;
    double angle_inc_deg_;
};

// ---- Detector 2: DBSCAN (простая реализация по XY) ----
class DBSCAN
{
public:
    DBSCAN(double eps, int min_pts)
        : eps_(eps), min_pts_(min_pts) {}

    // Простой DBSCAN по XY точкам (вектор Point2D)
    std::vector<Cluster> cluster(const std::vector<Point2D> &pts, const std::vector<int> &idxs_in_scan)
    {
        std::vector<Cluster> clusters;
        int N = pts.size();
        if (N == 0)
            return clusters;

        enum PtState
        {
            UNVISITED = 0,
            VISITED,
            NOISE,
            CLUSTERED
        };
        std::vector<int> state(N, UNVISITED);
        for (int i = 0; i < N; ++i)
        {
            if (state[i] != UNVISITED)
                continue;
            std::vector<int> neigh = regionQuery(pts, i);
            if ((int)neigh.size() < min_pts_)
            {
                state[i] = NOISE;
            }
            else
            {
                Cluster cl;
                expandCluster(pts, idxs_in_scan, state, i, neigh, cl);
                if ((int)cl.pts.size() >= min_pts_)
                {
                    cl.method = "dbscan";
                    clusters.push_back(cl);
                    LOG("[DBSCAN] Новый кластер: N=%d\n", (int)cl.pts.size());
                }
            }
        }

        LOG("[DBSCAN] Всего кластеров: %zu\n", clusters.size());
        return clusters;
    }

private:
    double eps_;
    int min_pts_;

    std::vector<int> regionQuery(const std::vector<Point2D> &pts, int idx)
    {
        std::vector<int> ret;
        double ex = eps_ * eps_;
        for (int j = 0; j < (int)pts.size(); ++j)
        {
            double dx = pts[j].x - pts[idx].x;
            double dy = pts[j].y - pts[idx].y;
            double d2 = dx * dx + dy * dy;
            if (d2 <= ex)
                ret.push_back(j);
        }
        return ret;
    }

    void expandCluster(const std::vector<Point2D> &pts, const std::vector<int> &idxs_in_scan,
                       std::vector<int> &state, int idx, std::vector<int> &neigh, Cluster &cl)
    {
        // создаём новый кластер
        cl.pts.push_back(pts[idx]);
        cl.idxs.push_back(idxs_in_scan[idx]);
        state[idx] = CLUSTERED;
        std::deque<int> q;
        for (int n : neigh)
            q.push_back(n);
        while (!q.empty())
        {
            int v = q.front();
            q.pop_front();
            if (state[v] == UNVISITED)
            {
                state[v] = VISITED;
                std::vector<int> neigh2 = regionQuery(pts, v);
                if ((int)neigh2.size() >= min_pts_)
                {
                    for (int t : neigh2)
                        q.push_back(t);
                }
            }
            if (state[v] != CLUSTERED)
            {
                cl.pts.push_back(pts[v]);
                cl.idxs.push_back(idxs_in_scan[v]);
                state[v] = CLUSTERED;
            }
        }
        cl.N = cl.pts.size();
    }
};

// ---- Detector 3: Shadow detection (тень) ----
class ShadowDetector
{
public:
    ShadowDetector(int bg_window, double shadow_thresh, int min_pts, double min_angle_deg, double angle_inc_deg)
        : bg_window_(bg_window), shadow_thresh_(shadow_thresh), min_pts_(min_pts),
          min_angle_deg_(min_angle_deg), angle_inc_deg_(angle_inc_deg) {}

    // detect: вход — агрегированный scan range[] (по углам)
    std::vector<Cluster> detect(const std::vector<double> &ranges)
    {
        int N = ranges.size();
        std::vector<Cluster> clusters;
        if (N == 0)
            return clusters;

        // посчитаем локальный фон для каждого луча (медиана окна ±bg_window_)
        std::vector<double> bg(N, std::numeric_limits<double>::quiet_NaN());
        for (int i = 0; i < N; ++i)
        {
            std::vector<double> window;
            int a = std::max(0, i - bg_window_);
            int b = std::min(N - 1, i + bg_window_);
            for (int j = a; j <= b; ++j)
            {
                double r = ranges[j];
                if (!std::isnan(r))
                    window.push_back(r);
            }
            if (!window.empty())
            {
                std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
                bg[i] = window[window.size() / 2];
            }
        }
        // пометим лучи, у которых bg - r > threshold
        std::vector<int> mark(N, 0);
        for (int i = 0; i < N; ++i)
        {
            if (std::isnan(ranges[i]) || std::isnan(bg[i]))
                continue;
            if (bg[i] - ranges[i] > shadow_thresh_)
                mark[i] = 1;
        }
        // соберём последовательные помеченные
        int i = 0;
        while (i < N)
        {
            if (mark[i] == 0)
            {
                ++i;
                continue;
            }
            int a = i;
            while (i < N && mark[i] == 1)
                ++i;
            int b = i - 1;
            Cluster cl;
            cl.method = "shadow";
            for (int k = a; k <= b; ++k)
                cl.idxs.push_back(k);
            cl.N = cl.idxs.size();
            cl.angular_span_deg = cl.N * angle_inc_deg_;
            if (cl.N >= min_pts_ && cl.angular_span_deg >= min_angle_deg_)
            {
                clusters.push_back(cl);
                LOG("[ShadowDetector] Найден сегмент shadow: idxs %d..%d N=%d span=%.2fdeg\n",
                    a, b, cl.N, cl.angular_span_deg);
            }
            else
            {
                LOG("[ShadowDetector] Отброшен маленький сегмент shadow N=%d span=%.2fdeg\n", cl.N, cl.angular_span_deg);
            }
        }

        LOG("[ShadowDetector] Всего кластеров shadow: %zu\n", clusters.size());
        return clusters;
    }

private:
    int bg_window_;
    double shadow_thresh_;
    int min_pts_;
    double min_angle_deg_;
    double angle_inc_deg_;
};

// ---- Circle fitter (Taubin algebraic fit) ----
// Возвращает центр (cx,cy), radius, rmse. Если не удалось, возвращает rmse large.
class CircleFitter
{
public:
    // fit по набору точек: Taubin method
    static std::tuple<Point2D, double, double> fitCircle(const std::vector<Point2D> &pts)
    {
        Point2D center{0.0, 0.0};
        double radius = 0.0;
        double rmse = 1e9;
        int n = pts.size();
        if (n < 3)
            return {center, radius, rmse};

        // центровать данные
        double meanx = 0.0, meany = 0.0;
        for (auto &p : pts)
        {
            meanx += p.x;
            meany += p.y;
        }
        meanx /= n;
        meany /= n;
        Eigen::MatrixXd Z(n, 3);
        for (int i = 0; i < n; ++i)
        {
            double xi = pts[i].x - meanx;
            double yi = pts[i].y - meany;
            Z(i, 0) = xi * xi + yi * yi;
            Z(i, 1) = xi;
            Z(i, 2) = yi;
        }
        Eigen::Vector3d M = Z.colwise().mean();
        // Центрируем матрицу
        Eigen::MatrixXd Zc = Z.rowwise() - M.transpose();
        Eigen::Matrix3d Cov = (Zc.adjoint() * Zc) / double(n);
        // SVD для решения обобщенной задачи - Taubin приближение
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Cov);
        Eigen::Vector3d evals = es.eigenvalues();
        Eigen::Matrix3d evecs = es.eigenvectors();
        int minIdx = 0;
        for (int i = 1; i < 3; ++i)
            if (evals[i] < evals[minIdx])
                minIdx = i;
        Eigen::Vector3d A = evecs.col(minIdx);
        // A = [a, b, c] соответствуют уравнению a*(x^2+y^2) + b*x + c*y + d =0
        double a = A(0), b = A(1), c = A(2);
        if (fabs(a) < 1e-12)
        {
            // Degenerate
            return {center, radius, rmse};
        }
        // восстановим параметры
        double cx = -b / (2 * a) + meanx;
        double cy = -c / (2 * a) + meany;
        // radius
        double rsum = 0.0;
        for (int i = 0; i < n; ++i)
        {
            double dx = pts[i].x - cx;
            double dy = pts[i].y - cy;
            rsum += sqrt(dx * dx + dy * dy);
        }
        radius = rsum / n;
        // rmse
        double sq = 0.0;
        for (int i = 0; i < n; ++i)
        {
            double dx = pts[i].x - cx;
            double dy = pts[i].y - cy;
            double di = sqrt(dx * dx + dy * dy);
            double err = di - radius;
            sq += err * err;
        }
        rmse = sqrt(sq / n);
        center = {cx, cy};
        return {center, radius, rmse};
    }
};

// ---- Matching (перестановки) и выбор лучшей перестановки ----
class Matcher
{
public:
    Matcher(const std::array<double, 6> &d_true)
    {
        // d_true array: пары [01,02,03,12,13,23] (в удобной компоновке)
        // Но проще: мы ожидаем систему 4 точек: RB, RT, LT, LB (в каком порядке? договоримся)
        d_true_ = d_true;
    }

    // Построим эталонные координаты Q_i (i=0..3) из попарных расстояний d_true
    // Схема: 0=(0,0) (RB), 1=(d01,0) (RT on +X), 2 = intersection of circle(0,d02) and circle(1,d12) choose positive Y, 3 = remaining
    std::vector<Point2D> buildReference()
    {
        // распакуем
        double d01 = d_true_[0];
        double d02 = d_true_[1];
        double d03 = d_true_[2];
        double d12 = d_true_[3];
        double d13 = d_true_[4];
        double d23 = d_true_[5];
        // 0 at (0,0)
        Point2D q0{0.0, 0.0};
        Point2D q1{d01, 0.0};
        // Compute q2 from circle intersections of (0,d02) and (q1,d12)
        auto inter = circleIntersection(q0, d02, q1, d12);
        Point2D q2;
        if (!inter.second)
        {
            // Если пересечения нет (из-за несогласованных расстояний), приближенно разместим по x
            q2 = {d02 * 0.5, sqrt(std::max(0.0, d02 * d02 - (d02 * 0.5) * (d02 * 0.5)))};
        }
        else
        {
            // Возьмем положительную Y (верхний)
            q2 = inter.first.first.y >= inter.first.second.y ? inter.first.first : inter.first.second;
        }
        // q3 можно найти как пересечение окружностей с q0,d03 и q1,d13 (или q2,d23)
        auto inter2 = circleIntersection(q0, d03, q1, d13);
        Point2D q3;
        if (!inter2.second)
        {
            q3 = {d03 * 0.5, -sqrt(std::max(0.0, d03 * d03 - (d03 * 0.5) * (d03 * 0.5)))};
        }
        else
        {
            // выбрать ту точку, которая ближе к ожидаемой позиции (нижний)
            // просто выберем точку с меньшим y
            q3 = inter2.first.first.y <= inter2.first.second.y ? inter2.first.first : inter2.first.second;
        }

        std::vector<Point2D> Q = {q0, q1, q2, q3};
        LOG("[Matcher] Reference coords Q0..Q3: (%.3f,%.3f) (%.3f,%.3f) (%.3f,%.3f) (%.3f,%.3f)\n",
            Q[0].x, Q[0].y, Q[1].x, Q[1].y, Q[2].x, Q[2].y, Q[3].x, Q[3].y);
        return Q;
    }

    // Сравнение: у нас метод возвращает ровно 4 центра (если меньше — special handling);
    // Возвращаем best permutation и её RMS ошибки по 6 попарным расстояниям.
    std::pair<std::vector<Point2D>, double> matchFour(const std::vector<Point2D> &found4)
    {
        std::vector<Point2D> ref = buildReference();
        if (found4.size() != 4)
        {
            LOG("[Matcher] Warning: matchFour called with found4.size() = %zu (expected 4)\n", found4.size());
        }
        std::vector<int> perm = {0, 1, 2, 3};
        double bestErr = 1e9;
        std::vector<Point2D> bestAssign;
        // Перебор всех перестановок
        do
        {
            // переставим found4[perm[i]] -> candidate for Q[i]
            std::vector<Point2D> assigned(4);
            for (int i = 0; i < 4 && i < (int)found4.size(); ++i)
                assigned[i] = found4[perm[i]];
            // Если меньше 4 найдено, оставшиеся assigned будут (0,0) - это плохо; будем считать большая ошибка
            double err = computePairwiseRMS(assigned, ref);
            if (err < bestErr)
            {
                bestErr = err;
                bestAssign = assigned;
            }
        } while (std::next_permutation(perm.begin(), perm.end()));
        LOG("[Matcher] Best perm RMS error = %.4f m\n", bestErr);
        return {bestAssign, bestErr};
    }

    // Вспомогательное: вычислить RMS по 6 попарным расстояниям между assigned и ref
    double computePairwiseRMS(const std::vector<Point2D> &asg, const std::vector<Point2D> &ref)
    {
        // пары (0,1),(0,2),(0,3),(1,2),(1,3),(2,3)
        std::vector<std::pair<int, int>> pairs = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
        double sumsq = 0.0;
        int cnt = 0;
        for (auto &pr : pairs)
        {
            int i = pr.first, j = pr.second;
            if (i >= (int)asg.size() || j >= (int)asg.size())
            {
                sumsq += 1.0;
                cnt++;
                continue;
            }
            double dx = asg[i].x - ref[i].x;
            double dy = asg[i].y - ref[i].y;
            // НО правильно сравнивать попарные расстояния:
            double dmeas = hypot(asg[i].x - asg[j].x, asg[i].y - asg[j].y);
            double dtrue = getTrueDistance(i, j);
            double diff = dmeas - dtrue;
            sumsq += diff * diff;
            cnt++;
        }
        if (cnt == 0)
            return 1e9;
        return sqrt(sumsq / cnt);
    }

private:
    std::array<double, 6> d_true_;

    // Возвращает истинное расстояние между индексами i,j в той же нотации (i<j)
    double getTrueDistance(int i, int j) const
    {
        // mapping pair->index in d_true_
        // pairs(0,1)->0; (0,2)->1; (0,3)->2; (1,2)->3; (1,3)->4; (2,3)->5
        int idx = -1;
        if (i == 0 && j == 1)
            idx = 0;
        else if (i == 0 && j == 2)
            idx = 1;
        else if (i == 0 && j == 3)
            idx = 2;
        else if (i == 1 && j == 2)
            idx = 3;
        else if (i == 1 && j == 3)
            idx = 4;
        else if (i == 2 && j == 3)
            idx = 5;
        if (idx == -1)
            return 0.0;
        return d_true_[idx];
    }

    // intersection of two circles: center p0 r0, p1 r1
    // returns pair (Point2D a, Point2D b) and bool success
    std::pair<std::pair<Point2D, Point2D>, bool> circleIntersection(const Point2D &p0, double r0, const Point2D &p1, double r1)
    {
        double x0 = p0.x, y0 = p0.y;
        double x1 = p1.x, y1 = p1.y;
        double dx = x1 - x0;
        double dy = y1 - y0;
        double d = sqrt(dx * dx + dy * dy);
        if (d < 1e-9)
            return {{{0, 0}, {0, 0}}, false}; // совпадают
        if (d > r0 + r1 + 1e-6)
            return {{{0, 0}, {0, 0}}, false}; // нет пересечений
        if (d < fabs(r0 - r1) - 1e-6)
            return {{{0, 0}, {0, 0}}, false}; // одна внутри другой
        double a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
        double h2 = r0 * r0 - a * a;
        if (h2 < 0)
            h2 = 0;
        double xm = x0 + a * dx / d;
        double ym = y0 + a * dy / d;
        double rx = -dy * sqrt(h2) / d;
        double ry = dx * sqrt(h2) / d;
        Point2D pa{xm + rx, ym + ry};
        Point2D pb{xm - rx, ym - ry};
        return {{pa, pb}, true};
    }
};

// ---- Fusion: взвешенное усреднение по идентифицированным одноимённым центрам ----
class Fusion
{
public:
    // inputs: for each named pillar (0..3) есть vector of pairs (Point2D, sigma)
    // weights: w = 1/sigma^2
    static std::pair<Point2D, double> weightedAverage(const std::vector<std::pair<Point2D, double>> &samples)
    {
        Point2D out{0.0, 0.0};
        double sumw = 0.0;
        if (samples.empty())
            return {out, 1e9};
        for (auto &s : samples)
        {
            double sigma = s.second;
            double w = 1.0 / (sigma * sigma + 1e-12);
            out.x += s.first.x * w;
            out.y += s.first.y * w;
            sumw += w;
        }
        out.x /= sumw;
        out.y /= sumw;
        double sigma_final = sqrt(1.0 / sumw);
        return {out, sigma_final};
    }
};

// ---- Umeyama (2D) для similarity: scale, rotation, translation ----
struct Similarity
{
    double scale;
    double rot; // angle radians
    Point2D trans;
};

class Umeyama2D
{
public:
    // input: vectors of matched points P (source) and Q (target), same size
    // output: Similarity transform s,R,t minimizing || s R P + t - Q ||
    static Similarity estimate(const std::vector<Point2D> &P, const std::vector<Point2D> &Q, bool estimate_scale = true)
    {
        size_t n = P.size();
        Similarity out;
        out.scale = 1.0;
        out.rot = 0.0;
        out.trans = {0.0, 0.0};
        if (n == 0)
            return out;
        // centroids
        Point2D muP{0.0, 0.0}, muQ{0.0, 0.0};
        for (size_t i = 0; i < n; ++i)
        {
            muP.x += P[i].x;
            muP.y += P[i].y;
            muQ.x += Q[i].x;
            muQ.y += Q[i].y;
        }
        muP.x /= n;
        muP.y /= n;
        muQ.x /= n;
        muQ.y /= n;
        // covariance
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < n; ++i)
        {
            Eigen::Vector2d p(P[i].x - muP.x, P[i].y - muP.y);
            Eigen::Vector2d q(Q[i].x - muQ.x, Q[i].y - muQ.y);
            cov += q * p.transpose();
        }
        cov /= double(n);
        // SVD
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        double det = (U * V.transpose()).determinant();
        Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
        if (det < 0)
            S(1, 1) = -1;
        Eigen::Matrix2d R = U * S * V.transpose();
        double scale = 1.0;
        if (estimate_scale)
        {
            double varP = 0.0;
            for (size_t i = 0; i < n; ++i)
            {
                varP += (P[i].x - muP.x) * (P[i].x - muP.x) + (P[i].y - muP.y) * (P[i].y - muP.y);
            }
            varP /= double(n);
            double traceSR = 0.0;
            Eigen::Matrix2d tmp = R.transpose() * cov;
            traceSR = tmp.trace();
            if (fabs(varP) < 1e-12)
                scale = 1.0;
            else
                scale = traceSR / varP;
        }
        Eigen::Vector2d tvec(muQ.x, muQ.y);
        Eigen::Vector2d muPvec(muP.x, muP.y);
        Eigen::Vector2d trans = tvec - scale * R * muPvec;
        out.scale = scale;
        out.rot = atan2(R(1, 0), R(0, 0)); // извлекаем угол из матрицы
        out.trans = {trans(0), trans(1)};
        LOG("[Umeyama2D] scale=%.6f rot=%.6fdeg trans=(%.4f, %.4f)\n", scale, out.rot * 180.0 / M_PI, out.trans.x, out.trans.y);
        return out;
    }

    // Apply transform: P' = s R P + t
    static Point2D apply(const Similarity &s, const Point2D &p)
    {
        double ca = cos(s.rot), sa = sin(s.rot);
        double x = s.scale * (ca * p.x - sa * p.y) + s.trans.x;
        double y = s.scale * (sa * p.x + ca * p.y) + s.trans.y;
        return {x, y};
    }
};

// ---- Основной узел scan_node ----
class ScanNode
{
public:
    ScanNode(ros::NodeHandle &nh)
        : nh_(nh)
    {
        // Чтение параметров из rosparam / yaml
        // Чтение всех 6 расстояний между центрами (пример)
        nh_.param<double>("/pb_config/distance/pillar_0_1", dist_pillar[0], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_0_2", dist_pillar[1], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_0_3", dist_pillar[2], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_1_2", dist_pillar[3], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_1_3", dist_pillar[4], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_2_3", dist_pillar[5], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_diametr", pillar_diametr_, 0.315);

        // Лидар параметры (англ. шаг и начало угла) читаем из параметров или по умолчанию
        nh_.param<double>("/scan_node/angle_min", angle_min_, -M_PI);
        nh_.param<double>("/scan_node/angle_max", angle_max_, M_PI);
        nh_.param<double>("/scan_node/angle_increment", angle_inc_, 0.0052359877); // ~0.3deg default
        nh_.param<int>("/scan_node/expected_scans", expected_scans_, 100);
        nh_.param<double>("/scan_node/min_range", min_range_, 0.05);
        nh_.param<double>("/scan_node/max_range", max_range_, 30.0);
        nh_.param<double>("/scan_node/sensor_sigma", sensor_sigma_, 0.03); // 30mm default

        // Параметры детекторов (начальные)
        nh_.param<double>("/scan_node/gap_thresh", gap_thresh_, 0.5);
        nh_.param<int>("/scan_node/gap_min_pts", gap_min_pts_, 8);
        nh_.param<double>("/scan_node/min_angle_deg", min_angle_deg_, 6.0);

        nh_.param<double>("/scan_node/dbscan_eps", dbscan_eps_, 0.10);
        nh_.param<int>("/scan_node/dbscan_minpts", dbscan_minpts_, 10);

        nh_.param<int>("/scan_node/shadow_bg_window", shadow_bg_window_, 20);
        nh_.param<double>("/scan_node/shadow_thresh", shadow_thresh_, 0.25);
        nh_.param<int>("/scan_node/shadow_min_pts", shadow_min_pts_, 8);

        // Подготовим внутренние объекты: angular steps
        // Вычислим шаг угла в градусах и число шагов по параметрам
        angle_inc_deg_ = angle_inc_ * 180.0 / M_PI;
        angular_steps_ = int(std::round((angle_max_ - angle_min_) / angle_inc_)) + 1;
        LOG("[ScanNode] angle_min=%.3f angle_max=%.3f angle_inc=%.6f deg=%.6f steps=%d\n",
            angle_min_, angle_max_, angle_inc_, angle_inc_deg_, angular_steps_);

        // Инициализируем агрегатор
        aggregator_.reset(new ScanAggregator(angular_steps_));
        aggregator_->setNeedScans(expected_scans_);

        // Preprocessor
        preproc_.reset(new Preprocessor(angle_min_, angle_inc_, min_range_, max_range_));

        // Детекторы
        gap_detector_.reset(new GapDetector(gap_thresh_, gap_min_pts_, min_angle_deg_, angle_inc_deg_));
        dbscan_detector_.reset(new DBSCAN(dbscan_eps_, dbscan_minpts_));
        shadow_detector_.reset(new ShadowDetector(shadow_bg_window_, shadow_thresh_, shadow_min_pts_, min_angle_deg_, angle_inc_deg_));

        // Matcher с эталонными расстояниями (взятые из параметров)
        std::array<double, 6> dtrue;
        for (int i = 0; i < 6; ++i)
            dtrue[i] = dist_pillar[i];
        matcher_.reset(new Matcher(dtrue));

        // Подписка на LaserScan
        sub_scan_ = nh_.subscribe("/scan", 10, &ScanNode::scanCallback, this);

        // Публичный таймер для периодической проверки (например, запуск обработки каждые 0.5 s если накоплено)
        timer_ = nh_.createTimer(ros::Duration(0.5), &ScanNode::timerCb, this);

        LOG("[ScanNode] Инициализирован. Ожидаем %d сканов для агрегации\n", expected_scans_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_scan_;
    ros::Timer timer_;

    std::unique_ptr<ScanAggregator> aggregator_;
    std::unique_ptr<Preprocessor> preproc_;
    std::unique_ptr<GapDetector> gap_detector_;
    std::unique_ptr<DBSCAN> dbscan_detector_;
    std::unique_ptr<ShadowDetector> shadow_detector_;
    std::unique_ptr<Matcher> matcher_;

    // Параметры
    double angle_min_, angle_max_, angle_inc_;
    double angle_inc_deg_;
    int angular_steps_;
    int expected_scans_;
    double min_range_, max_range_;
    double sensor_sigma_;
    double pillar_diametr_;
    double dist_pillar[6];

    // detector params
    double gap_thresh_;
    int gap_min_pts_;
    double min_angle_deg_;
    double dbscan_eps_;
    int dbscan_minpts_;
    int shadow_bg_window_;
    double shadow_thresh_;
    int shadow_min_pts_;

    // Временное хранилище последнего агрегированного скана
    std::vector<double> last_agg_scan_;

    // Callback для входящих сканов
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // Преобразуем LaserScan -> vector<double> ranges
        int N = msg->ranges.size();
        std::vector<double> ranges(N);
        for (int i = 0; i < N; ++i)
            ranges[i] = msg->ranges[i];
        // Добавим в агрегатор
        aggregator_->addScan(ranges);
        LOG("[scanCallback] Добавлен scan. Накоплено: %s\n", aggregator_->ready() ? "ready" : "collecting");
    }

    // timerCb - проверяем, готовы ли мы к агрегации и начинаем конвейер обработки
    void timerCb(const ros::TimerEvent &)
    {
        if (!aggregator_->ready())
        {
            LOG("[timerCb] Ещё не накоплено нужное число сканов\n");
            return;
        }
        LOG("[timerCb] Накоплено %d сканов. Начинаем агрегацию и обработку...\n", expected_scans_);
        // 1) Получаем медианный скан
        auto scan_med = aggregator_->getMedianScan(min_range_, max_range_);
        last_agg_scan_ = scan_med; // сохраним
        // 2) Предобработка: получаем XY full
        std::vector<Point2D> pts_full;
        preproc_->scanToXYFull(scan_med, pts_full);
        // 3) Запускаем детекторы
        auto clusters_gap = gap_detector_->detect(scan_med);
        // Для dbscan нужно собрать XY и индексы
        std::vector<Point2D> pts_all;
        std::vector<int> idxs_all;
        for (int i = 0; i < angular_steps_; ++i)
        {
            if (!std::isnan(scan_med[i]) && scan_med[i] >= min_range_ && scan_med[i] <= max_range_)
            {
                double ang = angle_min_ + i * angle_inc_;
                Point2D p{scan_med[i] * cos(ang), scan_med[i] * sin(ang)};
                pts_all.push_back(p);
                idxs_all.push_back(i);
            }
        }
        auto clusters_dbscan = dbscan_detector_->cluster(pts_all, idxs_all);
        auto clusters_shadow = shadow_detector_->detect(scan_med);

        // 4) Для каждого кластера: подставим XY точки (для gap/shadow используем idxs -> pts_full)
        fillClusterPoints(clusters_gap, pts_full);
        evaluateClusters(clusters_gap);
        evaluateClusters(clusters_dbscan);
        fillClusterPoints(clusters_shadow, pts_full);
        evaluateClusters(clusters_shadow);

        // 5) Matching внутри каждого метода: для простоты рассматриваем, что каждый метод возвращает ровно 4 центра (попробуем форсировать выбор)
        // Если метод вернул больше или меньше 4, — попытаемся выбрать 4 лучших (по rmse/size). Для простоты: если найдено >4 — берем 4 лучших.
        auto named_by_method1 = produceNamedByMethod("gap", clusters_gap);
        auto named_by_method2 = produceNamedByMethod("dbscan", clusters_dbscan);
        auto named_by_method3 = produceNamedByMethod("shadow", clusters_shadow);

        // 6) Собрать таблицу одноимённых точек и выполнить взвешенное усреднение
        std::vector<std::vector<std::pair<Point2D, double>>> samples_per_name(4); // RB,RT,LT,LB -> 0..3
        // Каждый named_by_methodX это вектор size 4 (или меньше), на позициях где нет - пропуски
        collectSamplesInto(samples_per_name, named_by_method1);
        collectSamplesInto(samples_per_name, named_by_method2);
        collectSamplesInto(samples_per_name, named_by_method3);

        // Усреднение
        std::vector<Point2D> final_coords(4);
        std::vector<double> final_sigmas(4);
        for (int i = 0; i < 4; ++i)
        {
            auto pr = Fusion::weightedAverage(samples_per_name[i]);
            final_coords[i] = pr.first;
            final_sigmas[i] = pr.second;
            LOG("[Fusion] Pillar %d final = (%.4f, %.4f), sigma=%.4f (samples=%zu)\n",
                i, final_coords[i].x, final_coords[i].y, final_sigmas[i], samples_per_name[i].size());
        }

        // 7) final validation: сравнить попарные расстояния с эталоном
        double E_all = computePairwiseRMS(final_coords);
        LOG("[Validation] RMS popar distances = %.4f m\n", E_all);

        // 8) если всё ок, то применим Umeyama чтобы привести к глобальной системе (RB at 0,0)
        // Построим эталонные Q_i (target global positions) на основе d_true
        std::vector<Point2D> Q = matcher_->buildReference(); // Q в системе, где RB=(0,0)
        // P = final_coords (в локальной системе лидара), Q = эталон
        Similarity sim = Umeyama2D::estimate(final_coords, Q, true);
        // Преобразуем final_coords
        std::vector<Point2D> final_global(4);
        for (int i = 0; i < 4; ++i)
        {
            final_global[i] = Umeyama2D::apply(sim, final_coords[i]);
            LOG("[Global] Pillar %d global = (%.4f, %.4f)\n", i, final_global[i].x, final_global[i].y);
            // Запишем в rosparam /pb_config/pillar_i_x и y
            std::ostringstream sx, sy;
            ros::param::set("/pb_config/pillar_" + std::to_string(i) + "_x", final_global[i].x);
            ros::param::set("/pb_config/pillar_" + std::to_string(i) + "_y", final_global[i].y);
            // также сохраним sigma
            ros::param::set("/pb_config/pillar_" + std::to_string(i) + "_sigma", final_sigmas[i]);
        }
        // Запишем диаметр туда же
        ros::param::set("/pb_config/distance/pillar_diametr", pillar_diametr_);

        LOG("[ScanNode] Результаты записаны в /pb_config. Заканчиваем цикл обработки.\n");

        // После успешной обработки очистим агрегатор и начнём новый цикл
        aggregator_->reset();
    }

    // Вспомогательные: заполняем кластер.pts по idxs из pts_full
    void fillClusterPoints(std::vector<Cluster> &clusters, const std::vector<Point2D> &pts_full)
    {
        for (auto &cl : clusters)
        {
            cl.pts.clear();
            for (int idx : cl.idxs)
            {
                if (idx >= 0 && idx < (int)pts_full.size())
                {
                    auto p = pts_full[idx];
                    if (!std::isnan(p.x) && !std::isnan(p.y))
                        cl.pts.push_back(p);
                }
            }
            cl.N = cl.pts.size();
        }
    }

    // Оценим каждый кластер: fit circle, sigma, confidence
    void evaluateClusters(std::vector<Cluster> &clusters)
    {
        for (auto &cl : clusters)
        {
            if (cl.pts.size() < 3)
            {
                cl.confidence = 0.0;
                cl.rmse = 1e9;
                continue;
            }
            auto [center, radius, rmse] = CircleFitter::fitCircle(cl.pts);
            cl.center = center;
            cl.radius = radius;
            cl.rmse = rmse;
            // Оценка sigma_center (элементарная модель)
            double sigma_center = sqrt((sensor_sigma_ * sensor_sigma_) / std::max(1, cl.N) + (rmse * rmse));
            cl.sigma_center = sigma_center;
            // Confidence: инверсия rmse, нормированная
            cl.confidence = std::max(0.0, 1.0 - (rmse / 0.1)); // rmse=0.0 -> conf=1, rmse=0.1 -> conf~0
            LOG("[evaluateClusters] method=%s N=%d center=(%.4f,%.4f) r=%.4f rmse=%.4f sigma=%.4f conf=%.3f\n",
                cl.method.c_str(), cl.N, cl.center.x, cl.center.y, cl.radius, cl.rmse, cl.sigma_center, cl.confidence);
        }
    }

    // produceNamedByMethod: возвращает вектор size 4 (RB..LB) с центрами Point2D и sigma
    // если кластеров >4 - берем 4 лучших по confidence; если <4 - оставляем пустые
    std::vector<std::pair<Point2D, double>> produceNamedByMethod(const std::string &method, const std::vector<Cluster> &clusters)
    {
        std::vector<std::pair<Point2D, double>> res(4, {{0.0, 0.0}, 1e9});
        // Отберем все кластеры этого метода
        std::vector<Cluster> sel;
        for (auto &c : clusters)
            if (c.method == method)
                sel.push_back(c);
        if (sel.empty())
        {
            LOG("[produceNamedByMethod] method=%s: нет кластеров\n", method.c_str());
            return res;
        }
        // Сортировка по confidence (убывание)
        std::sort(sel.begin(), sel.end(), [](const Cluster &a, const Cluster &b)
                  { return a.confidence > b.confidence; });
        // Возьмём up to 4 лучших центров
        std::vector<Point2D> centers;
        std::vector<double> sigmas;
        for (size_t i = 0; i < sel.size() && centers.size() < 4; ++i)
        {
            centers.push_back(sel[i].center);
            sigmas.push_back(sel[i].sigma_center);
        }
        // Если нашли меньше 4, заполним пустыми и пометим как не найденные
        if (centers.size() < 4)
        {
            LOG("[produceNamedByMethod] method=%s: найдено только %zu центров (нужны 4)\n", method.c_str(), centers.size());
            // Возвращаем пустую (при таком подходе метод не участвует для отсутствующих столбов)
            // Альтернатива: можно попытаться разделить крупные кластеры на 4 сегмента, но это сложнее.
        }
        // Теперь делаем matching: centers -> reference Q
        if (centers.size() >= 4)
        {
            std::vector<Point2D> first4(4);
            for (int i = 0; i < 4; ++i)
                first4[i] = centers[i];
            auto matched = matcher_->matchFour(first4);
            // matched.first уже aligned to ref order (RB,RT,LT,LB)
            for (int i = 0; i < 4; ++i)
            {
                res[i] = {matched.first[i], sigmas[i]};
                LOG("[produceNamedByMethod] method=%s assigned idx %d -> (%.4f,%.4f) sigma=%.4f\n",
                    method.c_str(), i, res[i].first.x, res[i].first.y, res[i].second);
            }
        }
        else
        {
            // Если centers.size()<4, возвращаем пустые, по которым в fusion не будет данных.
        }
        return res;
    }

    // collectSamplesInto: добавить точки named (size 4 vector) в samples_per_name
    void collectSamplesInto(std::vector<std::vector<std::pair<Point2D, double>>> &samples_per_name,
                            const std::vector<std::pair<Point2D, double>> &named)
    {
        for (int i = 0; i < 4 && i < (int)named.size(); ++i)
        {
            if (named[i].second < 1e8)
            {
                samples_per_name[i].push_back(named[i]);
            }
        }
    }

    // Compute RMS of pairwise distances for final coords vs true distances
    double computePairwiseRMS(const std::vector<Point2D> &P)
    {
        // if any P is empty (0,0) and sigma large, treat as error
        std::vector<std::pair<int, int>> pairs = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
        double sumsq = 0.0;
        int cnt = 0;
        for (auto &pr : pairs)
        {
            int i = pr.first, j = pr.second;
            double dmeas = hypot(P[i].x - P[j].x, P[i].y - P[j].y);
            double dtrue = dist_pillar[mapPairToIdx(i, j)];
            double diff = dmeas - dtrue;
            sumsq += diff * diff;
            cnt++;
            LOG("[computePairwiseRMS] pair %d-%d: meas=%.4f true=%.4f diff=%.4f\n", i, j, dmeas, dtrue, diff);
        }
        if (cnt == 0)
            return 1e9;
        return sqrt(sumsq / cnt);
    }

    int mapPairToIdx(int i, int j) const
    {
        // (0,1)->0; (0,2)->1; (0,3)->2; (1,2)->3; (1,3)->4; (2,3)->5
        if (i == 0 && j == 1)
            return 0;
        if (i == 0 && j == 2)
            return 1;
        if (i == 0 && j == 3)
            return 2;
        if (i == 1 && j == 2)
            return 3;
        if (i == 1 && j == 3)
            return 4;
        if (i == 2 && j == 3)
            return 5;
        return 0;
    }
};

// ---- main ----
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_node");
    ros::NodeHandle nh("~");

    LOG("[main] Запуск scan_node...\n");
    ScanNode node(nh);

    ros::spin();
    return 0;
}
