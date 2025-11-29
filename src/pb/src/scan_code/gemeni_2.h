/*
 * Версия: 2.6_Final
 * Дата: 2025-11-29
 * Описание: Финальная версия. Внедрен медианный фильтр (v2.5), полный Umeyama (v2.4)
 * и исправлен расчет межосевого расстояния (v2.6).
 */

#include <ros/ros.h>               // Подключение основной библиотеки ROS
#include <sensor_msgs/LaserScan.h> // Подключение типа сообщения для данных лидара
#include <ros/topic.h>             // Необходим для ros::topic::waitForMessage
#include <vector>                  // Работа с динамическими массивами
#include <cmath>                   // Математические функции (sqrt, pow, cos, sin, atan2)
#include <string>                  // Работа со строками
#include <algorithm>               // Использование std::sort, std::max, std::nth_element
#include <stdarg.h>                // Для реализации printf-подобного логгера
#include <Eigen/Dense>             // Обязательно: библиотека линейной алгебры

// Константа для математических вычислений (pi)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --------------------------------------------------------------------------------------
// 1. Вспомогательные классы и структуры
// --------------------------------------------------------------------------------------

// Обертка для логирования
class LoggerWrapper
{
public:
    void log(const char *format, ...) // Метод для вывода сообщений в консоль
    {
        va_list args;           // Объявление списка аргументов переменной длины
        va_start(args, format); // Инициализация списка аргументов
        vprintf(format, args);  // Вывод в консоль (printf-style)
        va_end(args);           // Завершение работы со списком аргументов
    }
};

LoggerWrapper logi; // Глобальный инстанс логгера

// Структура кандидата (найденного потенциального столба)
struct PillarCandidate
{
    Eigen::Vector2d center; // Координаты (X, Y) центра столба в системе лидара
    double rmse;            // Среднеквадратичная ошибка вписывания окружности
    int num_points;         // Число точек, использованных для аппроксимации
    double weight;          // Интегральный вес (0..1) - показатель надежности детекции
    int method_id;          // ID метода, которым найден кандидат (1, 2 или 3)
};

// Структура финального столба (после идентификации)
struct FinalPillar
{
    std::string name;       // Имя: RB, RT, LT, LB
    Eigen::Vector2d local;  // Координаты центра в системе лидара (до калибровки)
    Eigen::Vector2d global; // Координаты центра в глобальной системе (после калибровки)
};

// Класс математических утилит
class MathUtils
{
public:
    // Расстояние между двумя точками Eigen
    static double dist2D(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
    {
        return (p1 - p2).norm(); // Возвращает евклидово расстояние (норму разности векторов)
    }

    // Аппроксимация окружности (Метод Kasa - Linear Least Squares)
    static bool fitCircle(const std::vector<Eigen::Vector2d> &points, double expected_radius,
                          Eigen::Vector2d &out_center, double &out_rmse)
    {
        size_t n = points.size();
        if (n < 3)
            return false;

        Eigen::MatrixXd A(n, 3);
        Eigen::VectorXd B(n);

        for (size_t i = 0; i < n; ++i)
        {
            A(i, 0) = 2.0 * points[i].x();
            A(i, 1) = 2.0 * points[i].y();
            A(i, 2) = 1.0;
            B(i) = points[i].x() * points[i].x() + points[i].y() * points[i].y();
        }

        // Решаем систему Ax = B методом SVD
        Eigen::Vector3d sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

        out_center.x() = sol(0);
        out_center.y() = sol(1);

        double r_sq = sol(2) + sol(0) * sol(0) + sol(1) * sol(1);
        double r_fit = (r_sq > 0) ? sqrt(r_sq) : expected_radius;

        double sum_sq = 0;
        for (const auto &p : points)
        {
            double d = dist2D(p, out_center);
            sum_sq += pow(d - r_fit, 2);
        }
        out_rmse = sqrt(sum_sq / n);
        return true;
    }

    // Функция для нахождения медианы
    static double getMedian(std::vector<double> &values)
    {
        if (values.empty())
            return 0.0;            // Если вектор пустой, возвращаем 0
        size_t n = values.size();  // Количество элементов
        size_t median_idx = n / 2; // Индекс правого центрального элемента (или единственного)

        // Находим элемент на позиции median_idx
        std::nth_element(values.begin(), values.begin() + median_idx, values.end());

        if (n % 2 != 0)
        {
            return values[median_idx]; // Если нечетное число элементов, это и есть медиана
        }
        else
        {
            // Для четного числа элементов берем среднее двух центральных
            double v2 = values[median_idx]; // Элемент на позиции N/2

            // Находим v1 - максимальный элемент в левой половине (до median_idx)
            double v1 = *std::max_element(values.begin(), values.begin() + median_idx);

            return (v1 + v2) / 2.0; // Возвращаем среднее арифметическое v1 и v2
        }
    }
};

// --------------------------------------------------------------------------------------
// 2. Основной Класс Ноды
// --------------------------------------------------------------------------------------

class PillarScanNode
{
private:
    ros::NodeHandle nh;       // Обработчик ROS
    ros::Subscriber scan_sub; // Подписчик на данные лидара

    // --- Параметры конфигурации ---
    double pillar_diam_;   // Диаметр столба (м)
    double pillar_radius_; // Радиус столба
    double d_surf[6];      // Расстояния между поверхностями столбов
    double d_center[6];    // Расстояния между центрами столбов (расчетные)

    // Фильтрация
    double min_range_filter;
    double max_range_filter;
    double neighbor_radius_filter;
    int min_neighbors_filter;

    // Детекция
    double jump_dist_threshold;
    double cluster_dist_threshold;

    // Fusion & Weights
    double rmse_max_tolerance;
    int n_max_points_norm;
    double fusion_group_radius;
    double w_method[4];

    // Эталонная система
    std::vector<Eigen::Vector2d> reference_centers_; // 4 точки эталонной системы (RB, RT, LT, LB)

    // Состояние сбора данных
    int scans_collected;              // Счетчик накопленных сканов
    const int SCANS_TO_COLLECT = 100; // Требуемое количество сканов для медианы

    // Накопление ВСЕХ дальностей для медианной фильтрации
    std::vector<std::vector<double>> accumulated_ranges;
    sensor_msgs::LaserScan meta_scan; // Метаданные (углы) из первого скана

public:
    PillarScanNode() : scans_collected(0) // Конструктор класса
    {
        logi.log("=== PillarScanNode v2.6 Started (Final) ===\n");

        loadParameters(); // 1. Загрузка параметров
        if (!ros::ok())
            return;

        initReferenceSystem(); // 2. Построение эталона (Q_global)

        // --- ПРОВЕРКА НАЛИЧИЯ ДАННЫХ ЛИДАРА ---
        logi.log("Checking /scan topic availability (timeout 30s)...\n");

        sensor_msgs::LaserScan::ConstPtr first_scan = // Блокирующий вызов
            ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(30));

        if (!first_scan)
        {
            logi.log("ERROR: Timed out waiting for /scan topic. Shutting down.\n");
            ros::shutdown();
            return;
        }

        // Инициализация структур
        size_t num_rays = first_scan->ranges.size();
        accumulated_ranges.resize(num_rays);
        meta_scan = *first_scan;

        logi.log("LiDAR initialized. Rays: %lu. Starting accumulation.\n", num_rays);

        scan_sub = nh.subscribe("/scan", 100, &PillarScanNode::scanCallback, this);
        logi.log("Waiting for %d laser scans on /scan topic...\n", SCANS_TO_COLLECT);
    }

    // ----------------------------------------------------------------------------------
    // Загрузка параметров (Расчет d_center ИСПРАВЛЕН)
    // ----------------------------------------------------------------------------------
    void loadParameters()
    {
        logi.log("\n--- Loading YAML Parameters ---\n"); // Логирование начала загрузки параметров

        // 1. Геометрия
        nh.param<double>("/pb_config/scan_node/pillar_diametr", pillar_diam_, 0.315); // Чтение диаметра столба из ROS Parameter Server
        logi.log("  pillar_diametr: %.4f\n", pillar_diam_);                           // Логирование прочитанного диаметра
        pillar_radius_ = pillar_diam_ / 2.0;                                          // Расчет радиуса столба

        const char *dist_names[] = {// Список имен параметров расстояний между столбами
                                    "pillar_0_1", "pillar_0_2", "pillar_0_3",
                                    "pillar_1_2", "pillar_1_3", "pillar_2_3"};

        bool dist_valid = true; // Флаг для проверки логической корректности расстояний
        for (int i = 0; i < 6; ++i)
        {
            std::string path = "/pb_config/scan_node/" + std::string(dist_names[i]); // Формирование полного пути к параметру
            nh.param<double>(path, d_surf[i], 0.0);                                  // Чтение расстояния между поверхностями (d_surf)

            // !!! ИСПРАВЛЕНО: Расчет межосевого расстояния (центр-центр) !!!
            d_center[i] = d_surf[i] + 2.0 * pillar_radius_; // Межосевое расстояние = d_surf + 2 * R

            logi.log("  %s (surf): %.4f -> (center): %.4f\n", dist_names[i], d_surf[i], d_center[i]); // Логирование

            if (d_center[i] <= pillar_diam_)
                dist_valid = false; // Проверка: межосевое расстояние должно быть больше диаметра
        }

        if (!dist_valid)
        {
            logi.log("ERROR: Invalid distances! Check YAML. Shutting down.\n"); // Если проверка не пройдена, выводим ошибку
            ros::shutdown();
            return; // Завершаем работу ROS-ноды
        }

        // 2. Фильтрация
        nh.param<double>("/pb_config/scan_node/filter/min_range_filter", min_range_filter, 0.2);             // Мин. дальность для фильтрации
        nh.param<double>("/pb_config/scan_node/filter/max_range_filter", max_range_filter, 15.0);            // Макс. дальность для фильтрации
        nh.param<double>("/pb_config/scan_node/filter/neighbor_radius_filter", neighbor_radius_filter, 0.3); // Радиус для фильтрации шума
        nh.param<int>("/pb_config/scan_node/filter/min_neighbors_filter", min_neighbors_filter, 3);          // Мин. соседей для фильтрации

        // 3. Детекция
        nh.param<double>("/pb_config/scan_node/detection/jump_dist_threshold", jump_dist_threshold, 0.5);        // Порог для метода "Разрыв"
        nh.param<double>("/pb_config/scan_node/detection/cluster_dist_threshold", cluster_dist_threshold, 0.05); // Порог для метода "Кластеризация"

        // 4. Fusion
        nh.param<double>("/pb_config/scan_node/fusion/rmse_max_tolerance", rmse_max_tolerance, 0.01);  // Макс. RMSE для нормализации веса
        nh.param<int>("/pb_config/scan_node/fusion/n_max_points_norm", n_max_points_norm, 100);        // Макс. число точек для нормализации веса
        nh.param<double>("/pb_config/scan_node/fusion/fusion_group_radius", fusion_group_radius, 0.2); // Радиус для объединения кандидатов в один столб

        // 5. Веса
        nh.param<double>("/pb_config/scan_node/weights/w_method_1_jump", w_method[1], 1.0);    // Вес метода 1 (Jump Distance)
        nh.param<double>("/pb_config/scan_node/weights/w_method_2_cluster", w_method[2], 0.9); // Вес метода 2 (Clustering)
        nh.param<double>("/pb_config/scan_node/weights/w_method_3_minima", w_method[3], 0.8);  // Вес метода 3 (Local Minima)
        logi.log("--- Parameters Loaded Successfully ---\n");                                  // Логирование успешного завершения
    }

    // ----------------------------------------------------------------------------------
    // Инициализация эталона (Global Reference System)
    // ----------------------------------------------------------------------------------
    void initReferenceSystem()
    {
        reference_centers_.resize(4); // Выделяем место для 4 опорных точек

        // P0 (RB) -> (0,0)
        reference_centers_[0] = Eigen::Vector2d(0, 0);

        // P1 (RT) -> (d_01, 0)
        reference_centers_[1] = Eigen::Vector2d(d_center[0], 0);

        // P2 (LT) -> Левый Верхний: Триангуляция от P0 и P1
        double d01 = d_center[0];
        double d02 = d_center[1];
        double d12 = d_center[3];

        double x2 = (pow(d01, 2) + pow(d02, 2) - pow(d12, 2)) / (2 * d01);
        double y2 = sqrt(std::max(0.0, pow(d02, 2) - pow(x2, 2)));
        reference_centers_[2] = Eigen::Vector2d(x2, y2); // P2: Y положительный

        // P3 (LB) -> Левый Нижний: Триангуляция от P0 и P1
        double d03 = d_center[2];
        double d13 = d_center[4];

        double x3 = (pow(d01, 2) + pow(d03, 2) - pow(d13, 2)) / (2 * d01);
        double y3 = sqrt(std::max(0.0, pow(d03, 2) - pow(x3, 2)));
        reference_centers_[3] = Eigen::Vector2d(x3, -y3); // P3: Y отрицательный

        logi.log("\nReference System Initialized:\n");
        const char *names[] = {"RB (0)", "RT (1)", "LT (2)", "LB (3)"};
        for (int i = 0; i < 4; ++i)
            logi.log("  %s: [%.3f, %.3f]\n", names[i], reference_centers_[i].x(), reference_centers_[i].y());
    }

    // ----------------------------------------------------------------------------------
    // Callback скана (Накопление всех значений для медианы)
    // ----------------------------------------------------------------------------------
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        if (scans_collected >= SCANS_TO_COLLECT)
            return;

        if (scan->ranges.size() != accumulated_ranges.size())
        {
            logi.log("WARNING: Scan size changed! Skipping scan.\n");
            return;
        }

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float r = scan->ranges[i];

            // Фильтрация мусора
            if (std::isnan(r) || std::isinf(r))
                continue;
            if (r < min_range_filter || r > max_range_filter)
                continue;

            accumulated_ranges[i].push_back(r); // Накопление значения
        }

        scans_collected++;
        if (scans_collected % 10 == 0)
            logi.log("Collecting scans: %d/%d\n", scans_collected, SCANS_TO_COLLECT);

        if (scans_collected == SCANS_TO_COLLECT)
        {
            processPipeline();
        }
    }

    // ----------------------------------------------------------------------------------
    // Основной конвейер обработки (Медианный фильтр)
    // ----------------------------------------------------------------------------------
    void processPipeline()
    {
        logi.log("\n=== Starting Processing Pipeline (Median Filtered Scan) ===\n");

        std::vector<Eigen::Vector2d> clean_points;

        // 1. ФОРМИРОВАНИЕ МЕДИАННОГО СКАНА И ПЕРЕВОД В ТОЧКИ
        for (size_t i = 0; i < accumulated_ranges.size(); ++i)
        {
            if (accumulated_ranges[i].empty())
                continue;

            // Находим медиану
            std::vector<double> current_ray_data = accumulated_ranges[i]; // Копирование для безопасного nth_element
            double median_r = MathUtils::getMedian(current_ray_data);

            // Вычисляем угол
            double angle = meta_scan.angle_min + i * meta_scan.angle_increment;

            // Переводим в XY
            clean_points.emplace_back(median_r * cos(angle), median_r * sin(angle));
        }

        if (clean_points.empty())
        {
            logi.log("ERROR: No valid points after filtering. Shutting down.\n");
            ros::shutdown();
            return;
        }
        logi.log("Total median filtered points: %lu\n", clean_points.size());

        // 2. Статистическая фильтрация
        std::vector<Eigen::Vector2d> filtered_points;
        if (clean_points.size() > 2)
        {
            for (size_t i = 1; i < clean_points.size() - 1; ++i)
            {
                double d_prev = MathUtils::dist2D(clean_points[i], clean_points[i - 1]);
                double d_next = MathUtils::dist2D(clean_points[i], clean_points[i + 1]);

                if (d_prev < 0.2 || d_next < 0.2)
                {
                    filtered_points.push_back(clean_points[i]);
                }
            }
        }
        else
        {
            filtered_points = clean_points;
        }
        logi.log("Points after simple filter: %lu\n", filtered_points.size());

        // 3. Детекция (3 метода)
        std::vector<PillarCandidate> all_candidates;

        auto c1 = detectGenericClustering(filtered_points, jump_dist_threshold, 1);
        all_candidates.insert(all_candidates.end(), c1.begin(), c1.end());

        auto c2 = detectGenericClustering(filtered_points, cluster_dist_threshold, 2);
        all_candidates.insert(all_candidates.end(), c2.begin(), c2.end());

        auto c3 = detectLocalMinima(filtered_points, 3);
        all_candidates.insert(all_candidates.end(), c3.begin(), c3.end());

        logi.log("Total candidates found: %lu (M1=%lu, M2=%lu, M3=%lu)\n",
                 all_candidates.size(), c1.size(), c2.size(), c3.size());

        // 4. Fusion
        std::vector<FinalPillar> final_pillars = fuseCandidates(all_candidates);

        // 5. Калибровка (Full Umeyama)
        performCalibration(final_pillars);

        // 6. Сохранение
        saveResults(final_pillars);

        logi.log("Pipeline completed. Shutting down node...\n");
        ros::shutdown();
    }

    // --- Методы детектирования ---

    std::vector<PillarCandidate> detectGenericClustering(const std::vector<Eigen::Vector2d> &pts, double threshold, int method_id)
    {
        std::vector<PillarCandidate> results;
        if (pts.empty())
            return results;

        std::vector<Eigen::Vector2d> current_cluster;
        current_cluster.push_back(pts[0]);

        for (size_t i = 1; i < pts.size(); ++i)
        {
            double d = MathUtils::dist2D(pts[i], pts[i - 1]);
            if (d > threshold)
            {
                processCluster(current_cluster, method_id, results);
                current_cluster.clear();
            }
            current_cluster.push_back(pts[i]);
        }
        processCluster(current_cluster, method_id, results);
        return results;
    }

    std::vector<PillarCandidate> detectLocalMinima(const std::vector<Eigen::Vector2d> &pts, int method_id)
    {
        std::vector<PillarCandidate> results;
        if (pts.size() < 10)
            return results;

        for (size_t i = 5; i < pts.size() - 5; ++i)
        {
            double r = pts[i].norm();
            bool is_min = true;
            for (int k = -5; k <= 5; ++k)
            {
                if (k == 0)
                    continue;
                if (pts[i + k].norm() < r)
                    is_min = false;
            }

            if (is_min)
            {
                std::vector<Eigen::Vector2d> cluster;
                cluster.push_back(pts[i]);

                for (int k = -1; i + k >= 0; --k)
                {
                    if (pts[i + k].norm() > r + 0.2 || MathUtils::dist2D(pts[i + k], pts[i + k + 1]) > 0.1)
                        break;
                    cluster.push_back(pts[i + k]);
                }
                for (size_t k = 1; i + k < pts.size(); ++k)
                {
                    if (pts[i + k].norm() > r + 0.2 || MathUtils::dist2D(pts[i + k], pts[i + k - 1]) > 0.1)
                        break;
                    cluster.push_back(pts[i + k]);
                }
                processCluster(cluster, method_id, results);
                i += cluster.size();
            }
        }
        return results;
    }

    void processCluster(const std::vector<Eigen::Vector2d> &cluster, int method_id, std::vector<PillarCandidate> &out)
    {
        if (cluster.size() < 5)
            return;
        double width = MathUtils::dist2D(cluster.front(), cluster.back());
        if (width < 0.05 || width > 0.5)
            return;

        Eigen::Vector2d center;
        double rmse;
        if (MathUtils::fitCircle(cluster, pillar_radius_, center, rmse))
        {
            double w_rmse = std::max(0.0, 1.0 - (rmse / rmse_max_tolerance));
            double w_n = std::min(1.0, sqrt((double)cluster.size()) / sqrt((double)n_max_points_norm));
            double w_m = w_method[method_id];

            double w_total = w_rmse * w_n * w_m;

            if (w_total > 0.1)
            {
                PillarCandidate cand;
                cand.center = center;
                cand.rmse = rmse;
                cand.num_points = (int)cluster.size();
                cand.weight = w_total;
                cand.method_id = method_id;
                out.push_back(cand);
            }
        }
    }

    // --- Логика слияния (Fusion) ---
    std::vector<FinalPillar> fuseCandidates(const std::vector<PillarCandidate> &candidates)
    {
        std::vector<FinalPillar> final_pillars;
        if (candidates.empty())
            return final_pillars;

        std::vector<bool> processed(candidates.size(), false);
        std::vector<Eigen::Vector2d> found_centers;

        for (size_t i = 0; i < candidates.size(); ++i)
        {
            if (processed[i])
                continue;
            double sum_w = 0;
            Eigen::Vector2d w_center(0, 0);

            for (size_t j = i; j < candidates.size(); ++j)
            {
                if (processed[j])
                    continue;
                if (MathUtils::dist2D(candidates[i].center, candidates[j].center) < fusion_group_radius)
                {
                    w_center += candidates[j].center * candidates[j].weight;
                    sum_w += candidates[j].weight;
                    processed[j] = true;
                }
            }

            if (sum_w > 0)
            {
                found_centers.push_back(w_center / sum_w);
            }
        }

        logi.log("Fusion: Found %lu unique pillars.\n", found_centers.size());

        if (found_centers.size() != 4)
        {
            logi.log("WARNING: Fusion found %lu pillars. Umeyama calibration requires 4. Skipping.\n", found_centers.size());
            return final_pillars;
        }

        // Временно присваиваем имена для целей калибровки
        std::sort(found_centers.begin(), found_centers.end(),
                  [](const Eigen::Vector2d &a, const Eigen::Vector2d &b)
                  {
                      return atan2(a.y(), a.x()) < atan2(b.y(), b.x());
                  });

        for (size_t i = 0; i < 4; ++i)
        {
            FinalPillar fp;
            fp.local = found_centers[i];
            fp.name = "Pillar_" + std::to_string(i);
            final_pillars.push_back(fp);
        }

        return final_pillars;
    }

    // --- 6. Калибровка: Полноценный Umeyama's Algorithm (SVD) ---
    void performCalibration(std::vector<FinalPillar> &pillars)
    {
        if (pillars.size() != 4)
        {
            logi.log("Calibration skipped: Need exactly 4 pillars.\n");
            return;
        }

        logi.log("\n--- Performing FULL Umeyama Calibration (SVD) ---\n");

        // 1. Идентификация
        int match_index[4] = {-1, -1, -1, -1};
        std::vector<bool> pillar_used(4, false);

        int idx_RB = -1;
        int idx_RT = -1;
        double best_err = 1000.0;

        // Идентификация RB (0) и RT (1)
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                if (i == j)
                    continue;
                double d = MathUtils::dist2D(pillars[i].local, pillars[j].local);
                double err = std::abs(d - d_center[0]);
                if (err < 0.2 && err < best_err)
                {
                    best_err = err;
                    if (pillars[i].local.x() < pillars[j].local.x())
                    {
                        idx_RB = i;
                        idx_RT = j;
                    }
                    else
                    {
                        idx_RB = j;
                        idx_RT = i;
                    }
                }
            }
        }

        if (idx_RB == -1)
        {
            logi.log("Calibration Failed: Could not identify RB-RT pair.\n");
            return;
        }

        match_index[0] = idx_RB;
        pillar_used[idx_RB] = true;
        match_index[1] = idx_RT;
        pillar_used[idx_RT] = true;
        pillars[idx_RB].name = "RB";
        pillars[idx_RT].name = "RT";

        // Идентификация LT (2) и LB (3) по Y-координате
        int rem_idx[2];
        int rem_count = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (!pillar_used[i])
                rem_idx[rem_count++] = i;
        }

        if (pillars[rem_idx[0]].local.y() > pillars[rem_idx[1]].local.y())
        {
            match_index[2] = rem_idx[0];
            match_index[3] = rem_idx[1];
        }
        else
        {
            match_index[2] = rem_idx[1];
            match_index[3] = rem_idx[0];
        }

        pillars[match_index[2]].name = "LT";
        pillars[match_index[3]].name = "LB";

        logi.log("Identified: RB(%d), RT(%d), LT(%d), LB(%d)\n",
                 match_index[0], match_index[1], match_index[2], match_index[3]);

        // 2. Подготовка матриц
        Eigen::Matrix<double, 2, 4> X;
        Eigen::Matrix<double, 2, 4> Y;

        for (int i = 0; i < 4; ++i)
        {
            X.col(i) = pillars[match_index[i]].local;
            Y.col(i) = reference_centers_[i];
        }

        // 3. Центрирование
        Eigen::Vector2d mu_x = X.rowwise().mean();
        Eigen::Vector2d mu_y = Y.rowwise().mean();

        Eigen::Matrix<double, 2, 4> X_c = X.colwise() - mu_x;
        Eigen::Matrix<double, 2, 4> Y_c = Y.colwise() - mu_y;

        // 4. Матрица Ковариации H
        Eigen::Matrix2d H = X_c * Y_c.transpose() / 4.0;

        // 5. SVD
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();

        // 6. Поворот (R)
        Eigen::Matrix2d D;
        D.setIdentity();
        if (U.determinant() * V.determinant() < 0.0)
        {
            D(1, 1) = -1.0;
        }
        Eigen::Matrix2d R = V * D * U.transpose();

        // 7. Масштаб (c)
        double c_numerator = svd.singularValues().sum();
        double c_denominator = (X_c.cwiseAbs2()).sum() / 4.0;

        double c = c_numerator / c_denominator;

        // 8. Сдвиг (T)
        Eigen::Vector2d T = mu_y - c * R * mu_x;

        logi.log("Umeyama Transform:\n");
        logi.log("  Scale (c): %.4f\n", c);
        // !!! ИСПРАВЛЕНО: atan2 для полного диапазона угла (-180..180°) !!!
        double angle_rad = atan2(R(1, 0), R(0, 0));                     // atan2(sin_theta, cos_theta)
        logi.log("  Rotation (deg): %.3f\n", angle_rad * 180.0 / M_PI); // Логируем угол в градусах

        logi.log("  Translation (T): [%.3f, %.3f]\n", T.x(), T.y());

        // 9. Применение
        double final_rmse_sum_sq = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            FinalPillar &p = pillars[match_index[i]];

            p.global = c * R * p.local + T;

            logi.log("Pillar %s -> Global: [%.3f, %.3f]\n", p.name.c_str(), p.global.x(), p.global.y());

            Eigen::Vector2d error = p.global - reference_centers_[i];
            final_rmse_sum_sq += error.squaredNorm();
        }

        double final_rmse = sqrt(final_rmse_sum_sq / 4.0);
        logi.log("Final Alignment RMSE: %.5f meters\n", final_rmse);

        // !!! ДОБАВЛЕНО: Проверка на высокую ошибку !!!
        if (final_rmse > 0.05)
        {
            logi.log("WARNING: High calibration error (%.1f cm)! Check pillars visibility and geometry.\n", final_rmse * 100.0);
        }
    }

    // --- Сохранение результатов в ROS Parameter Server ---
    void saveResults(const std::vector<FinalPillar> &pillars)
    {
        for (const auto &p : pillars)
        {
            if (p.name.find("Pillar_") != std::string::npos)
                continue;
            std::string base = "/pb_config/result/" + p.name;
            nh.setParam(base + "/x", p.global.x());
            nh.setParam(base + "/y", p.global.y());
        }
        logi.log("Results saved to rosparam.\n");
    }
};

int main(int argc, char **argv)
{
    // Инициализация ROS
    ros::init(argc, argv, "scan_pillar_node");

    // Создание объекта
    PillarScanNode node;

    // Запуск цикла обработки
    ros::spin();

    // Завершение работы
    logi.log("Node finished execution gracefully.\n");
    return 0;
}