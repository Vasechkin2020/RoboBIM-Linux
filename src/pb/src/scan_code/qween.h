#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <queue>

// Предполагается, что у тебя есть свой логгер с методом log(const char*, ...)
extern struct
{
    void log(const char *fmt, ...);
} logi;

// ========================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ========================

// Преобразование угла из градусов в радианы
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
// Из радиан в градусы
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// Вычисление расстояния между двумя 2D точками
double dist2D(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return (a - b).norm();
}

// Аппроксимация центра окружности с фиксированным радиусом методом итеративного уточнения (упрощённый)
// Используем начальную оценку как центроид, затем корректируем радиус до target_radius
Eigen::Vector2d fitCircleFixedRadius(const std::vector<Eigen::Vector2d> &points, double target_radius)
{
    if (points.empty())
        return Eigen::Vector2d::Zero();

    // Начальная оценка — центр масс
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (const auto &p : points)
        center += p;
    center /= points.size();

    // Итеративное уточнение (1–2 итерации достаточно)
    for (int iter = 0; iter < 2; ++iter)
    {
        Eigen::Vector2d grad = Eigen::Vector2d::Zero();
        double sum_w = 0.0;
        for (const auto &p : points)
        {
            double d = (p - center).norm();
            if (d < 1e-6)
                continue;
            double w = 1.0; // можно взвесить, но для простоты — равномерно
            grad += w * (1.0 - target_radius / d) * (p - center);
            sum_w += w;
        }
        if (sum_w > 1e-6)
            center += grad / sum_w;
    }
    return center;
}

// Umeyama similarity transform (масштаб + поворот + сдвиг) для 2D
// Возвращает: (scale, rotation_matrix, translation)
std::tuple<double, Eigen::Matrix2d, Eigen::Vector2d>
umeyama2D(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &dst)
{
    assert(src.size() == dst.size() && src.size() >= 2);

    size_t n = src.size();
    Eigen::Vector2d src_mean = Eigen::Vector2d::Zero();
    Eigen::Vector2d dst_mean = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < n; ++i)
    {
        src_mean += src[i];
        dst_mean += dst[i];
    }
    src_mean /= n;
    dst_mean /= n;

    double src_var = 0.0;
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < n; ++i)
    {
        Eigen::Vector2d xs = src[i] - src_mean;
        Eigen::Vector2d xd = dst[i] - dst_mean;
        src_var += xs.squaredNorm();
        cov += xd * xs.transpose();
    }
    src_var /= n;
    cov /= n;

    if (src_var < 1e-10)
    {
        logi.log("Umeyama: zero variance in source points\n");
        return std::make_tuple(1.0, Eigen::Matrix2d::Identity(), dst_mean - src_mean);
    }

    double det_cov = cov.determinant();
    if (det_cov < 0)
    {
        logi.log("Umeyama: reflection detected, forcing rotation\n");
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    // Коррекция на отражение
    Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
    if (det_cov < 0)
    {
        S(1, 1) = -1.0;
    }

    Eigen::Matrix2d R = U * S * V.transpose();
    double scale = svd.singularValues().dot(S.diagonal()) / src_var;
    Eigen::Vector2d t = dst_mean - scale * R * src_mean;

    return std::make_tuple(scale, R, t);
}

// ========================
// ОСНОВНОЙ КЛАСС НОДА
// ========================

class PillarDetector
{
public:
    PillarDetector(ros::NodeHandle &nh) : nh_(nh), scan_count_(0), processing_done_(false)
    {
        // --- Загрузка параметров из YAML ---
        nh_.param<double>("/pb_config/distance/pillar_diametr", pillar_diam_, 0.315);
        pillar_radius_ = pillar_diam_ / 2.0;

        // Загружаем 6 расстояний между поверхностями
        nh_.param<double>("/pb_config/distance/pillar_0_1", d_surf[0], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_0_2", d_surf[1], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_0_3", d_surf[2], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_1_2", d_surf[3], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_1_3", d_surf[4], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_2_3", d_surf[5], 0.0);

        // Преобразуем в расстояния между центрами
        for (int i = 0; i < 6; ++i)
        {
            d_center[i] = d_surf[i] + pillar_diam_;
        }

        // Проверка: все расстояния заданы?
        bool valid = true;
        for (int i = 0; i < 6; ++i)
        {
            if (d_center[i] <= 0.0)
                valid = false;
        }
        if (!valid)
        {
            logi.log("ERROR: Some pillar distances are missing or zero!\n");
            ros::shutdown();
            return;
        }
        logi.log("Loaded pillar diameter = %.3f m\n", pillar_diam_);
        logi.log("Center distances: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                 d_center[0], d_center[1], d_center[2], d_center[3], d_center[4], d_center[5]);

        // --- Подписка на сканы ---
        scan_sub_ = nh_.subscribe("/scan", 100, &PillarDetector::scanCallback, this);

        // --- Таймер: через 10 секунд обработать накопленные данные ---
        process_timer_ = nh_.createTimer(ros::Duration(10.0), &PillarDetector::processScans, this, true, false);
    }

private:
    ros::NodeHandle &nh_;
    ros::Subscriber scan_sub_;
    ros::Timer process_timer_;
    std::vector<sensor_msgs::LaserScan> scans_;
    int scan_count_;
    bool processing_done_;

    // Параметры из YAML
    double pillar_diam_;
    double pillar_radius_;
    double d_surf[6];   // расстояния между поверхностями
    double d_center[6]; // расстояния между центрами

    // ===== CALLBACK: накопление сканов =====
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        if (processing_done_)
            return;
        scans_.push_back(*msg);
        scan_count_++;
        if (scan_count_ % 10 == 0)
        {
            logi.log("Collected %d scans...\n", scan_count_);
        }
    }

    // ===== ОСНОВНАЯ ОБРАБОТКА =====
    void processScans(const ros::TimerEvent &event)
    {
        if (scans_.empty())
        {
            logi.log("No scans received!\n");
            return;
        }
        logi.log("\n=== STARTING PROCESSING (%zu scans) ===\n", scans_.size());

        processing_done_ = true;

        // ----------------------------
        // Этап 1: Сбор и фильтрация
        // ----------------------------
        logi.log("\n--- Stage 1: Collecting and filtering points ---\n");
        std::vector<Eigen::Vector2d> all_points;
        double min_angle = scans_[0].angle_min;
        double max_angle = scans_[0].angle_max;
        double angle_inc = scans_[0].angle_increment;

        // Собираем все точки из всех сканов
        for (const auto &scan : scans_)
        {
            for (size_t i = 0; i < scan.ranges.size(); ++i)
            {
                float r = scan.ranges[i];
                if (r < scan.range_min || r > scan.range_max)
                    continue;
                if (std::isnan(r) || std::isinf(r))
                    continue;
                if (r > 15.0)
                    continue; // фильтр по радиусу

                double angle = min_angle + i * angle_inc;
                double x = r * cos(angle);
                double y = r * sin(angle);
                all_points.emplace_back(x, y);
            }
        }
        logi.log("Total raw points: %zu\n", all_points.size());

        // Простая фильтрация выбросов (статистическая)
        // В реальности можно использовать PCL, но для минимализма — простой порог
        std::vector<Eigen::Vector2d> filtered_points;
        if (all_points.size() > 100)
        {
            // Удалим точки, у которых мало соседей в радиусе 0.3 м
            for (size_t i = 0; i < all_points.size(); ++i)
            {
                int neighbors = 0;
                for (size_t j = 0; j < all_points.size(); ++j)
                {
                    if (i == j)
                        continue;
                    if (dist2D(all_points[i], all_points[j]) < 0.3)
                        neighbors++;
                }
                if (neighbors >= 3)
                    filtered_points.push_back(all_points[i]);
            }
        }
        else
        {
            filtered_points = all_points;
        }
        logi.log("Points after filtering: %zu\n", filtered_points.size());

        // ----------------------------
        // Этап 2: Три метода детекции
        // ----------------------------
        logi.log("\n--- Stage 2: Running 3 detection methods ---\n");
        std::vector<std::vector<Eigen::Vector2d>> candidates_segments;

        // Метод 1: Разрывы в дальности (работаем с последним сканом для простоты)
        detectByRangeDiscontinuities(scans_.back(), candidates_segments);

        // Метод 2: Кластеризация
        detectByClustering(filtered_points, candidates_segments);

        // Метод 3: Локальный минимум + симметрия
        detectByLocalMinima(scans_.back(), candidates_segments);

        logi.log("Total candidate segments from 3 methods: %zu\n", candidates_segments.size());

        // ----------------------------
        // Этап 3: Аппроксимация и веса
        // ----------------------------
        logi.log("\n--- Stage 3: Fitting circles and computing weights ---\n");
        struct Candidate
        {
            Eigen::Vector2d center;
            double weight;
            size_t n_points;
            double rmse;
        };
        std::vector<Candidate> candidates;

        std::vector<double> method_weights = {1.0, 0.9, 0.85}; // веса методов 1,2,3

        size_t seg_idx = 0;
        for (const auto &seg : candidates_segments)
        {
            if (seg.size() < 5)
                continue; // слишком мало точек

            Eigen::Vector2d center = fitCircleFixedRadius(seg, pillar_radius_);
            // Оценка RMSE
            double sum_sq = 0.0;
            for (const auto &p : seg)
            {
                double d = (p - center).norm();
                sum_sq += (d - pillar_radius_) * (d - pillar_radius_);
            }
            double rmse = sqrt(sum_sq / seg.size());
            double radius_dev = std::abs((seg[0] - center).norm() - pillar_radius_); // грубая оценка

            // Вес
            double w =
                exp(-rmse / 0.02) *
                log(1.0 + seg.size()) *
                exp(-radius_dev / 0.01);

            // Определяем, от какого метода сегмент (очень грубо: по размеру)
            int method_id = 0;
            if (seg_idx < 10)
                method_id = 0; // метод 1 — первые
            else if (seg_idx < 30)
                method_id = 1; // метод 2
            else
                method_id = 2; // метод 3
            w *= method_weights[method_id];

            if (w > 0.01)
            { // порог значимости
                candidates.push_back({center, w, seg.size(), rmse});
                logi.log("Cand[%zu]: center=(%.3f, %.3f), n=%zu, rmse=%.1f mm, weight=%.3f\n",
                         candidates.size() - 1, center.x(), center.y(), seg.size(), rmse * 1000, w);
            }
            seg_idx++;
        }

        if (candidates.empty())
        {
            logi.log("ERROR: No valid candidates found!\n");
            return;
        }

        // ----------------------------
        // Этап 4: Fusion (объединение)
        // ----------------------------
        logi.log("\n--- Stage 4: Fusion of candidates ---\n");
        std::vector<Candidate> fused;

        std::vector<bool> used(candidates.size(), false);
        for (size_t i = 0; i < candidates.size(); ++i)
        {
            if (used[i])
                continue;
            Candidate merged = candidates[i];
            double total_weight = candidates[i].weight;
            Eigen::Vector2d weighted_center = candidates[i].center * candidates[i].weight;

            for (size_t j = i + 1; j < candidates.size(); ++j)
            {
                if (used[j])
                    continue;
                if (dist2D(candidates[i].center, candidates[j].center) < 0.25)
                {
                    used[j] = true;
                    total_weight += candidates[j].weight;
                    weighted_center += candidates[j].center * candidates[j].weight;
                }
            }
            merged.center = weighted_center / total_weight;
            merged.weight = total_weight;
            fused.push_back(merged);
        }

        logi.log("Fused candidates: %zu\n", fused.size());
        for (size_t i = 0; i < fused.size(); ++i)
        {
            logi.log("Fused[%zu]: center=(%.3f, %.3f), weight=%.3f\n",
                     i, fused[i].center.x(), fused[i].center.y(), fused[i].weight);
        }

        if (fused.size() < 4)
        {
            logi.log("WARNING: Only %zu fused candidates found (expected 4)\n", fused.size());
        }

        // ----------------------------
        // Этап 5: Построение эталонной геометрии
        // ----------------------------
        logi.log("\n--- Stage 5: Building reference geometry from distances ---\n");
        std::vector<Eigen::Vector2d> Q(4);
        Q[0] = Eigen::Vector2d(0.0, 0.0);         // правый нижний
        Q[1] = Eigen::Vector2d(d_center[0], 0.0); // вдоль X

        // Q[2]: пересечение окружностей вокруг Q0 (r = d02) и Q1 (r = d12)
        double x2 = (d_center[1] * d_center[1] - d_center[3] * d_center[3] + d_center[0] * d_center[0]) / (2.0 * d_center[0]);
        double y2_sq = d_center[1] * d_center[1] - x2 * x2;
        if (y2_sq < 0)
        {
            logi.log("ERROR: Invalid geometry for pillar 2!\n");
            return;
        }
        double y2 = sqrt(y2_sq); // предполагаем Y > 0 (помещение против часовой)
        Q[2] = Eigen::Vector2d(x2, y2);

        // Q[3]: через Q0 и Q1
        double x3 = (d_center[2] * d_center[2] - d_center[4] * d_center[4] + d_center[0] * d_center[0]) / (2.0 * d_center[0]);
        double y3_sq = d_center[2] * d_center[2] - x3 * x3;
        if (y3_sq < 0)
        {
            logi.log("ERROR: Invalid geometry for pillar 3!\n");
            return;
        }
        // Выбор знака: должен быть того же знака, что и Q[2]
        double y3 = (Q[2].y() > 0) ? sqrt(y3_sq) : -sqrt(y3_sq);
        Q[3] = Eigen::Vector2d(x3, y3);

        logi.log("Reference pillars (Q):\n");
        for (int i = 0; i < 4; ++i)
        {
            logi.log("  Q[%d] = (%.3f, %.3f)\n", i, Q[i].x(), Q[i].y());
        }

        // ----------------------------
        // Этап 6: Калибровка (Umeyama)
        // ----------------------------
        logi.log("\n--- Stage 6: Calibration via Umeyama ---\n");
        if (fused.size() < 4)
        {
            logi.log("ERROR: Need at least 4 candidates for calibration!\n");
            return;
        }

        // Возьмём 4 лучших кандидата
        std::sort(fused.begin(), fused.end(), [](const Candidate &a, const Candidate &b)
                  { return a.weight > b.weight; });
        std::vector<Eigen::Vector2d> P(4);
        for (int i = 0; i < 4; ++i)
            P[i] = fused[i].center;

        // Применяем Umeyama
        auto [scale, R, t] = umeyama2D(P, Q);

        logi.log("Umeyama result: scale=%.4f, translation=(%.3f, %.3f)\n", scale, t.x(), t.y());
        logi.log("Rotation matrix:\n  [%.4f %.4f]\n  [%.4f %.4f]\n", R(0, 0), R(0, 1), R(1, 0), R(1, 1));

        // Проверка остатков
        double rms = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            Eigen::Vector2d transformed = scale * R * P[i] + t;
            double err = (transformed - Q[i]).norm();
            rms += err * err;
            logi.log("  Residual[%d]: %.1f mm\n", i, err * 1000);
        }
        rms = sqrt(rms / 4.0);
        logi.log("RMS residual: %.1f mm\n", rms * 1000);

        if (rms > 0.05)
        {
            logi.log("WARNING: RMS residual > 50 mm — result may be unreliable!\n");
        }

        // ----------------------------
        // Этап 7: Применение преобразования и запись результата
        // ----------------------------
        logi.log("\n--- Stage 7: Final coordinates and ROS param output ---\n");
        std::vector<Eigen::Vector2d> final_coords(4);
        for (int i = 0; i < 4; ++i)
        {
            final_coords[i] = scale * R * P[i] + t;
        }

        // Сдвиг: убедимся, что Q[0] = (0,0)
        // (в нашем случае он уже (0,0), но для надёжности)
        Eigen::Vector2d offset = final_coords[0];
        for (int i = 0; i < 4; ++i)
        {
            final_coords[i] -= offset;
        }

        // Вывод в лог
        logi.log("Final calibrated pillar coordinates (global frame, origin = pillar 0):\n");
        for (int i = 0; i < 4; ++i)
        {
            logi.log("  Pillar %d: (%.4f, %.4f) m\n", i, final_coords[i].x(), final_coords[i].y());
        }

        // Запись в ROS параметры
        nh_.setParam("/pb_config/pillar_0_x", final_coords[0].x());
        nh_.setParam("/pb_config/pillar_0_y", final_coords[0].y());
        nh_.setParam("/pb_config/pillar_1_x", final_coords[1].x());
        nh_.setParam("/pb_config/pillar_1_y", final_coords[1].y());
        nh_.setParam("/pb_config/pillar_2_x", final_coords[2].x());
        nh_.setParam("/pb_config/pillar_2_y", final_coords[2].y());
        nh_.setParam("/pb_config/pillar_3_x", final_coords[3].x());
        nh_.setParam("/pb_config/pillar_3_y", final_coords[3].y());

        logi.log("Coordinates saved to ROS param /pb_config/pillar_*\n");
        logi.log("=== PROCESSING COMPLETE ===\n");
    }

    // ===== МЕТОД 1: Разрывы в дальности =====
    void detectByRangeDiscontinuities(const sensor_msgs::LaserScan &scan, std::vector<std::vector<Eigen::Vector2d>> &segments)
    {
        logi.log("  Method 1: Range discontinuities...\n");
        double min_angle = scan.angle_min;
        double angle_inc = scan.angle_increment;
        const std::vector<float> &ranges = scan.ranges;

        std::vector<size_t> gaps;
        for (size_t i = 0; i < ranges.size() - 1; ++i)
        {
            if (std::isnan(ranges[i]) || std::isnan(ranges[i + 1]))
                continue;
            if (ranges[i] < scan.range_min || ranges[i] > scan.range_max)
                continue;
            if (ranges[i + 1] < scan.range_min || ranges[i + 1] > scan.range_max)
                continue;
            if (std::abs(ranges[i + 1] - ranges[i]) > 0.5)
            {
                gaps.push_back(i);
            }
        }
        gaps.push_back(ranges.size() - 1); // конец

        if (gaps.empty())
            return;

        size_t start = 0;
        for (size_t gap : gaps)
        {
            if (gap - start < 3)
            { // минимум 3 точки
                start = gap + 1;
                continue;
            }
            std::vector<Eigen::Vector2d> seg;
            for (size_t i = start; i <= gap; ++i)
            {
                double r = ranges[i];
                double angle = min_angle + i * angle_inc;
                seg.emplace_back(r * cos(angle), r * sin(angle));
            }
            // Проверка углового размера
            double ang_span = (gap - start) * angle_inc;
            if (ang_span > deg2rad(1.0) && ang_span < deg2rad(6.0))
            {
                segments.push_back(seg);
            }
            start = gap + 1;
        }
        logi.log("    Found %zu segments\n", segments.size());
    }

    // ===== МЕТОД 2: Кластеризация =====
    void detectByClustering(const std::vector<Eigen::Vector2d> &points, std::vector<std::vector<Eigen::Vector2d>> &segments)
    {
        logi.log("  Method 2: Clustering...\n");
        if (points.size() < 10)
            return;

        // Очень простая кластеризация (в реальности — PCL::EuclideanClusterExtraction)
        std::vector<bool> visited(points.size(), false);
        double eps = 0.12; // метров
        int min_pts = 6;

        for (size_t i = 0; i < points.size(); ++i)
        {
            if (visited[i])
                continue;
            std::vector<size_t> cluster;
            std::queue<size_t> q;
            q.push(i);
            visited[i] = true;

            while (!q.empty())
            {
                size_t idx = q.front();
                q.pop();
                cluster.push_back(idx);
                for (size_t j = 0; j < points.size(); ++j)
                {
                    if (visited[j])
                        continue;
                    if (dist2D(points[idx], points[j]) < eps)
                    {
                        visited[j] = true;
                        q.push(j);
                    }
                }
            }

            if (cluster.size() >= min_pts)
            {
                std::vector<Eigen::Vector2d> seg;
                for (size_t idx : cluster)
                    seg.push_back(points[idx]);
                // Проверка диаметра
                double max_d = 0.0;
                for (size_t a = 0; a < seg.size(); ++a)
                {
                    for (size_t b = a + 1; b < seg.size(); ++b)
                    {
                        max_d = std::max(max_d, dist2D(seg[a], seg[b]));
                    }
                }
                if (max_d > 0.25 && max_d < 0.45)
                {
                    segments.push_back(seg);
                }
            }
        }
        logi.log("    Found %zu clusters\n", segments.size());
    }

    // ===== МЕТОД 3: Локальный минимум + симметрия =====
    void detectByLocalMinima(const sensor_msgs::LaserScan &scan, std::vector<std::vector<Eigen::Vector2d>> &segments)
    {
        logi.log("  Method 3: Local minima + symmetry...\n");
        const std::vector<float> &ranges = scan.ranges;
        double min_angle = scan.angle_min;
        double angle_inc = scan.angle_increment;

        // Сглаживание
        std::vector<float> smoothed = ranges;
        if (ranges.size() > 5)
        {
            for (size_t i = 2; i < ranges.size() - 2; ++i)
            {
                smoothed[i] = (ranges[i - 2] + ranges[i - 1] + ranges[i] + ranges[i + 1] + ranges[i + 2]) / 5.0;
            }
        }

        // Поиск локальных минимумов
        for (size_t i = 5; i < smoothed.size() - 5; ++i)
        {
            if (smoothed[i] > 15.0)
                continue;
            bool is_min = true;
            for (int j = -5; j <= 5; ++j)
            {
                if (j == 0)
                    continue;
                if (smoothed[i] >= smoothed[i + j])
                {
                    is_min = false;
                    break;
                }
            }
            if (!is_min)
                continue;

            // Определяем угловое окно (зависит от расстояния)
            double R = smoothed[i];
            double theta_deg = (R > 0) ? rad2deg(0.315 / R) : 3.0;
            double window_deg = std::max(2.0, 1.3 * theta_deg); // минимум 2 градуса
            int half_win = std::max(3, static_cast<int>(window_deg / rad2deg(angle_inc) / 2));

            int start = i - half_win;
            int end = i + half_win;
            if (start < 0 || end >= (int)smoothed.size())
                continue;

            // Проверка симметрии
            double mse = 0.0;
            int count = 0;
            for (int j = 1; j <= half_win; ++j)
            {
                if (start + j >= end - j)
                    break;
                float left = smoothed[start + j];
                float right = smoothed[end - j];
                mse += (left - right) * (left - right);
                count++;
            }
            if (count == 0)
                continue;
            mse /= count;
            if (mse > 0.0016)
                continue; // (40 mm)^2

            // Собираем точки
            std::vector<Eigen::Vector2d> seg;
            for (int j = start; j <= end; ++j)
            {
                if (std::isnan(ranges[j]))
                    continue;
                double angle = min_angle + j * angle_inc;
                seg.emplace_back(ranges[j] * cos(angle), ranges[j] * sin(angle));
            }
            if (seg.size() >= 5)
            {
                segments.push_back(seg);
            }
        }
        logi.log("    Found %zu symmetric segments\n", segments.size());
    }
};

// ========================
// MAIN
// ========================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_node");
    ros::NodeHandle nh("~");

    PillarDetector detector(nh);

    ros::spin(); // таймер сработает через 10 сек, затем можно выходить или продолжать
    return 0;
}