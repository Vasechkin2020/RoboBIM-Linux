/*
 * Версия: 6.0
 * Дата: 2025-11-29
 */

#include <ros/ros.h>               // Подключение основной библиотеки ROS
#include <sensor_msgs/LaserScan.h> // Подключение типа сообщения для данных лидара
#include <ros/topic.h>             // Необходим для ros::topic::waitForMessage
#include <vector>                  // Работа с динамическими массивами
#include <cmath>                   // Математические функции
#include <string>                  // Работа со строками
#include <algorithm>               // Различные алгоритмы (включая min/max)
#include <stdarg.h>                // Для реализации printf-подобного логгера
#include <Eigen/Dense>             // Обязательно: библиотека линейной алгебры
#include <limits>                  // Для std::numeric_limits
#include <numeric>                 // Для std::accumulate
#include <deque>                   // Требуется для std::deque

// --- Заголовки для RViz (Только стандартные ROS) ---
#include <visualization_msgs/MarkerArray.h> // Для публикации MarkerArray
#include <visualization_msgs/Marker.h>      // Для публикации одиночного Marker
#include <geometry_msgs/Point.h>            // Для точек внутри маркеров

// Константа для математических вычислений (pi)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --------------------------------------------------------------------------------------
// 1. Вспомогательные классы и структуры (Без изменений)
// --------------------------------------------------------------------------------------

// ВНЕШНЕЕ ОБЪЯВЛЕНИЕ ЛОГГЕРА
class AsyncFileLogger;
extern AsyncFileLogger logi;

// Структура кандидата
struct PillarCandidate
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector2f center;
    double rmse;
    int num_points;
    double weight;
    int method_id;
};

// Структура финального столба
struct FinalPillar
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string name;
    Eigen::Vector2f local;
    Eigen::Vector2d global;
    double total_weight = 0.0; // ← добавляем
    int ref_index = -1; // ← ДОБАВИТЬ ЭТУ СТРОКУ для хранения индекса эталона
};

// Тип для выровненного вектора FinalPillar
// НОВОЕ: Мы указываем контейнеру использовать специальный аллокатор Eigen
using AlignedPillarVector = std::vector<FinalPillar, Eigen::aligned_allocator<FinalPillar>>;


// Тип для выровненного вектора Eigen::Vector2f
using AlignedVector2f = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

// Класс математических утилит (Без изменений)
class MathUtils
{
public:
    static double dist2D(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2)
    {
        return (p1 - p2).norm();
    }

    static bool fitCircle(const AlignedVector2f &points, double expected_radius,
                          Eigen::Vector2f &out_center, double &out_rmse)
    {
        size_t n = points.size();
        if (n < 3)
            return false;

        Eigen::MatrixXd A(n, 3);
        Eigen::VectorXd B(n);

        for (size_t i = 0; i < n; ++i)
        {
            double x = (double)points[i].x();
            double y = (double)points[i].y();

            A(i, 0) = 2.0 * x;
            A(i, 1) = 2.0 * y;
            A(i, 2) = 1.0;
            B(i) = x * x + y * y;
        }

        Eigen::Vector3d sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(B);

        out_center.x() = (float)sol(0);
        out_center.y() = (float)sol(1);

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

    static double getMedian(std::vector<double> &values)
    {
        if (values.empty())
            return 0.0;
        size_t n = values.size();
        size_t median_idx = n / 2;

        std::nth_element(values.begin(), values.begin() + median_idx, values.end());

        if (n % 2 != 0)
        {
            return values[median_idx];
        }
        else
        {
            double v2 = values[median_idx];
            double v1 = *std::max_element(values.begin(), values.begin() + median_idx);

            return (v1 + v2) / 2.0;
        }
    }
};

// --------------------------------------------------------------------------------------
// 2. Основной Класс Ноды
// --------------------------------------------------------------------------------------

class PillarScanNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Timer publish_timer_;

    // --- Паблишеры ---
    ros::Publisher pub_filtered_scan;

    // Вместо одного pub_method_clusters теперь три:
    ros::Publisher pub_method_1; // Jump (Красный)
    ros::Publisher pub_method_2; // Cluster (Синий)
    ros::Publisher pub_method_3; // Minima (Желтый)

    ros::Publisher pub_fused_pillars;
    ros::Publisher pub_final_markers;

    // --- Параметры ---
    double pillar_diam_;
    double pillar_radius_;
    double d_surf[6];
    double d_center[6];
    double min_range_filter;
    double max_range_filter;
    double neighbor_radius_filter;
    int min_neighbors_filter;
    double jump_dist_threshold;
    double cluster_dist_threshold;
    double rmse_max_tolerance;
    int n_max_points_norm;
    double w_method[4];

    // --- Параметры кластеризации (НОВЫЕ, v5.8) ---
    double min_cluster_width_; // Минимальная физическая ширина кластера [м]
    double max_cluster_width_; // Максимальная физическая ширина кластера [м]
    int min_cluster_points_;   // Минимальное количество точек в кластере (НОВЫЙ, v5.9)

    // --- Параметры DBSCAN Fusion (НОВЫЙ, v6.1) ---
    double fusion_group_radius;
    int min_dbscan_points_; // minPts для DBSCAN-слияния

    // ПАРАМЕТРЫ ДЛЯ ФИЛЬТРАЦИИ ХВОСТОВ (v4.4)
    double intensity_min_threshold;
    double edge_angle_threshold;

    AlignedVector2f reference_centers_;

    // --- Калибровка и данные ---
    int scans_collected;
    const int SCANS_TO_COLLECT = 100;
    std::vector<std::vector<double>> accumulated_ranges;
    std::vector<std::vector<double>> accumulated_intensities;
    sensor_msgs::LaserScan meta_scan;
    sensor_msgs::LaserScan::ConstPtr last_raw_scan_ptr_;

    // Глобальный счетчик для лучей, отброшенных по интенсивности за все 100 сканов (v4.7)
    long long total_rays_removed_by_zero_intensity;
    long long total_rays_removed_by_low_intensity;
    long long total_rays_removed_by_initial_intensity;

    bool calibration_done_;
    AlignedPillarVector final_pillars_results_;

    // Хранение промежуточных результатов для постоянной публикации
    sensor_msgs::LaserScan filtered_scan_results_;

    // ИЗМЕНЕНО: Вместо массива маркеров храним каждый маркер отдельно
    visualization_msgs::Marker marker_m1_results_;
    visualization_msgs::Marker marker_m2_results_;
    visualization_msgs::Marker marker_m3_results_;

    AlignedVector2f fused_centers_results_;
    AlignedVector2f clean_points_results_; // <-- НОВОЕ: Для сохранения точек после углового фильтра

    // ----------------------------------------------------------------------------------
    // 3. ПРИВАТНЫЕ МЕТОДЫ
    // ----------------------------------------------------------------------------------

    // НОВАЯ ФУНКЦИЯ: logRawScan (v4.7)
    // Выводит все лучи первого сырого скана
    void logRawScan()
    {
        if (scans_collected != SCANS_TO_COLLECT)
            return;

        logi.log("\n--- RAW SCAN DUMP (First Scan of %d) ---\n", SCANS_TO_COLLECT);
        logi.log("Index | Angle(deg) | Raw_Range(m) | Raw_Intens\n");
        logi.log("------------------------------------------\n");

        size_t num_rays = meta_scan.ranges.size();
        for (size_t i = 0; i < num_rays; ++i)
        {
            double angle = meta_scan.angle_min + i * meta_scan.angle_increment;

            // Берем данные первого сохраненного сырого скана
            float raw_r = accumulated_ranges[i].empty() ? 0.0f : (float)accumulated_ranges[i][0];
            float raw_i = accumulated_intensities[i].empty() ? 0.0f : (float)accumulated_intensities[i][0];

            logi.log("%5lu | %10.3f | %12.4f | %10.1f\n",
                     i, angle * 180.0 / M_PI,
                     raw_r, raw_i);
        }
        logi.log("--- END RAW SCAN DUMP ---\n");
    }

    // --- ИЗМЕНЕНА: logFinalFilteredScan (Полностью - v4.8 FIX) ---

    // Приватный метод для детального логирования данных итогового отфильтрованного скана
    void logFinalFilteredScan(const sensor_msgs::LaserScan &filtered_scan)
    {
        if (scans_collected != SCANS_TO_COLLECT)
            return;

        logi.log("\n--- FINAL FILTERED SCAN DUMP (After Median & I_min Filter) ---\n");
        logi.log("Index | Angle(deg) | Filtered_Range(m) | Filtered_Intens\n");
        logi.log("---------------------------------------------------\n");

        size_t num_rays = filtered_scan.ranges.size(); // <--- ИСПРАВЛЕНО
        for (size_t i = 0; i < num_rays; ++i)
        {
            double angle = meta_scan.angle_min + i * meta_scan.angle_increment;

            // Получаем данные итогового отфильтрованного скана
            float filtered_r = filtered_scan.ranges[i];
            float filtered_i = filtered_scan.intensities[i];

            // Заменяем inf на NaN для удобства чтения
            if (std::isinf(filtered_r))
                filtered_r = std::numeric_limits<float>::quiet_NaN();

            logi.log("%5lu | %10.3f | %17.4f | %15.1f\n",
                     i, angle * 180.0 / M_PI,
                     filtered_r, filtered_i);
        }
        logi.log("--- END FINAL FILTERED SCAN DUMP ---\n");
    }

    // НОВАЯ ФУНКЦИЯ: publishMarkerInArray (Без изменений)
    void publishMarkerInArray(const visualization_msgs::Marker &marker, ros::Publisher &pub)
    {
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        pub.publish(marker_array);
    }

    // НОВАЯ ФУНКЦИЯ: createClusterMarkers (Без изменений)
    visualization_msgs::MarkerArray createClusterMarkers(const std::vector<PillarCandidate> &candidates, const std::string &frame_id)
    {
        visualization_msgs::MarkerArray marker_array;
        return marker_array;
    }

    // ИЗМЕНЕНА: createPointsMarker (Без изменений)
    visualization_msgs::Marker createPointsMarker(const AlignedVector2f &points, const std::string &frame_id, const std::string &ns, int id, float r, float g, float b, float scale)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = scale;
        marker.scale.y = scale;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        for (const auto &p : points)
        {
            geometry_msgs::Point pt;
            pt.x = p.x();
            pt.y = p.y();
            pt.z = 0.0;
            marker.points.push_back(pt);
        }

        return marker;
    }

    // Метод для публикации маркеров финальных столбов (Без изменений)
    void publishFinalMarkers(const AlignedPillarVector &pillars)
    {
        if (pillars.empty())
            return;

        visualization_msgs::MarkerArray marker_array;

        std::string laser_frame_id = meta_scan.header.frame_id;

        for (size_t i = 0; i < pillars.size(); ++i)
        {
            const FinalPillar &p = pillars[i];

            Eigen::Vector2d ref = Eigen::Vector2d(0, 0);
            if (p.name == "RB")
                ref = reference_centers_[0].cast<double>();
            else if (p.name == "RT")
                ref = reference_centers_[1].cast<double>();
            else if (p.name == "LT")
                ref = reference_centers_[2].cast<double>();
            else if (p.name == "LB")
                ref = reference_centers_[3].cast<double>();

            // 1. Маркер-сфера
            visualization_msgs::Marker sphere_marker;
            sphere_marker.header.frame_id = laser_frame_id;
            sphere_marker.header.stamp = ros::Time::now();
            sphere_marker.ns = "final_pillars";
            sphere_marker.id = (int)i * 2;
            sphere_marker.type = visualization_msgs::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::Marker::ADD;
            sphere_marker.pose.position.x = p.global.x();
            sphere_marker.pose.position.y = p.global.y();
            sphere_marker.pose.position.z = 0.0;
            sphere_marker.scale.x = pillar_diam_;
            sphere_marker.scale.y = pillar_diam_;
            sphere_marker.scale.z = 0.1;
            sphere_marker.color.a = 0.5;
            sphere_marker.color.r = 0.0;
            sphere_marker.color.g = 0.0;
            sphere_marker.color.b = 1.0;
            marker_array.markers.push_back(sphere_marker);

            // 2. Текстовый маркер
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = laser_frame_id;
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "pillar_names";
            text_marker.id = (int)i * 2 + 1;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position.x = p.global.x();
            text_marker.pose.position.y = p.global.y();
            text_marker.pose.position.z = 0.5;
            text_marker.scale.z = 0.3;
            text_marker.color.a = 1.0;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.text = p.name + "\nRef:(" + std::to_string((int)(ref.x() * 100)) + "," + std::to_string((int)(ref.y() * 100)) + ")";
            marker_array.markers.push_back(text_marker);
        }

        pub_final_markers.publish(marker_array);
    }

    // ИЗМЕНЕНА: publishResultsTimerCallback (v5.6)
    void publishResultsTimerCallback(const ros::TimerEvent &event)
    {
        ROS_INFO("+++ publishResultsTimerCallback");

        // 1. Публикация ЧИСТЫХ ТОЧЕК (clean_points_results_) как Marker::POINTS
        if (clean_points_results_.size() > 0 && meta_scan.header.frame_id != "")
        {
            // Создаем маркер POINTS из вектора 2D-точек clean_points_results_
            // Используем pub_filtered_scan, но публикуем Marker вместо LaserScan
            pub_filtered_scan.publish(createPointsMarker(clean_points_results_,
                                                         meta_scan.header.frame_id,                    // Используем frame_id из meta_scan
                                                         "clean_points", 0, 0.7f, 0.7f, 0.7f, 0.05f)); // Серый цвет, мелкие точки
            ROS_INFO("    0 - clean_points_results_ published as Marker::POINTS");
        }
        else if (filtered_scan_results_.ranges.size() > 0)
        {
            // Исходный LaserScan больше не публикуется
            ROS_INFO("      NOTE: filtered_scan_results_ is no longer published as LaserScan.");
        }

        // 2. Публикация маркеров кластеров (РАЗДЕЛЕНА)
        // Метод 1: Jump (Красный)
        if (marker_m1_results_.points.size() > 0)
        {
            marker_m1_results_.header.stamp = ros::Time::now(); // Обновляем время
            pub_method_1.publish(marker_m1_results_);
            ROS_INFO("    1 - marker_m1_results_");
        }
        // Метод 2: Cluster (Синий)
        if (marker_m2_results_.points.size() > 0)
        {
            marker_m2_results_.header.stamp = ros::Time::now();
            pub_method_2.publish(marker_m2_results_);
            ROS_INFO("    2 - marker_m2_results_");
        }
        // Метод 3: Minima (Желтый)
        if (marker_m3_results_.points.size() > 0)
        {
            marker_m3_results_.header.stamp = ros::Time::now();
            pub_method_3.publish(marker_m3_results_);
            ROS_INFO("    3 - marker_m3_results_");
        }

        // 3. Публикация центров Fusion (fused_centers_results_)
        if (fused_centers_results_.size() > 0 && meta_scan.header.frame_id != "")
        {
            // Нужно пересоздать маркер, используя сохраненные данные и frame_id
            pub_fused_pillars.publish(createPointsMarker(fused_centers_results_,
                                                         meta_scan.header.frame_id, // Используем frame_id из meta_scan
                                                         "fused_centers", 4, 0.0f, 1.0f, 0.0f, 0.15f));
            ROS_INFO("    4 - pub_fused_pillars");
        }

        // 4. Публикация финальных откалиброванных маркеров (final_pillars_results_)
        if (!final_pillars_results_.empty())
        {
            publishFinalMarkers(final_pillars_results_);
            ROS_INFO("    5 - publishFinalMarkers");
        }
    }

    // ИЗМЕНЕНА: processCluster (v5.9)
    // Обработка одного кластера, фильтрация по количеству точек и физической ширине
    void processCluster(const AlignedVector2f &cluster, int method_id,
                        std::vector<PillarCandidate> &out, AlignedVector2f &out_cluster_points)
    {
        // Фильтрация по минимальному количеству точек (ИЗМЕНЕНО: Используем параметр)
        if (cluster.size() < min_cluster_points_)
        {
            logi.log_w("Rejecting cluster from method %d (size %lu): too few points (%lu < %d).\n",
                       method_id, cluster.size(), cluster.size(), min_cluster_points_); // Подробный лог
            return;
        }

        // Расчет ширины кластера (Евклидово расстояние между крайними точками)
        double width = MathUtils::dist2D(cluster.front(), cluster.back());

        // Фильтрация по ширине (Используем параметры min_cluster_width_ и max_cluster_width_)
        if (width < min_cluster_width_ || width > max_cluster_width_)
        {
            logi.log_w("Rejecting cluster from method %d (size %lu): width %.3f m is outside bounds (%.3f to %.3f).\n",
                       method_id, cluster.size(), width, min_cluster_width_, max_cluster_width_);
            return;
        }

        Eigen::Vector2f center;
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

                for (const auto &p : cluster)
                {
                    out_cluster_points.push_back(p);
                }
            }
        }
    }

    // ИЗМЕНЕНА: Детекция на основе разрыва/плотности (v5.8)
    // Исправлено: 1. Циклическая кластеризация. 2. Устранена ошибка инвалидации ссылок.
    std::vector<PillarCandidate> detectGenericClustering(const AlignedVector2f &pts, double threshold, int method_id,
                                                         AlignedVector2f &out_cluster_points)
    {
        std::vector<PillarCandidate> results;
        // Минимальное число точек для надежной обработки кластеризации
        if (pts.size() < min_cluster_points_)
            return results;

        // Вектор для временного хранения всех кластеров перед циклической проверкой
        std::vector<AlignedVector2f> clusters;
        AlignedVector2f current;
        current.reserve(500); // Резервирование памяти для повышения эффективности

        // === 1. Линейная кластеризация ===
        current.push_back(pts[0]);
        for (size_t i = 1; i < pts.size(); ++i)
        {
            // Проверка расстояния между соседними точками
            if (MathUtils::dist2D(pts[i], pts[i - 1]) > threshold)
            {
                // Фильтрация: сохраняем только кластеры, содержащие минимум 5 точек
                if (current.size() >= min_cluster_points_)
                    clusters.push_back(std::move(current)); // Используем std::move

                current.clear(); // Начинаем новый кластер
            }
            current.push_back(pts[i]);
        }
        // Сохраняем последний кластер
        if (current.size() >= min_cluster_points_)
            clusters.push_back(std::move(current));
        current.clear();

        // === 2. Циклическое замыкание ===
        // Проверяем, можно ли объединить первый и последний кластеры.
        if (clusters.size() >= 2)
        {
            // КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: КОПИРУЕМ КЛАСТЕРЫ, чтобы избежать INVALIDATION (невалидности ссылок)
            const AlignedVector2f first = clusters.front(); // Копия первого кластера
            const AlignedVector2f last = clusters.back();   // Копия последнего кластера

            // Проверяем расстояние между последней точкой последнего кластера и первой точкой первого кластера
            if (MathUtils::dist2D(last.back(), first.front()) < threshold)
            {
                // Объединяем в новом векторе, сохраняя угловой порядок: (Последний + Первый)
                AlignedVector2f merged;
                merged.reserve(last.size() + first.size());

                // Добавляем точки последнего кластера (в правильном порядке)
                merged.insert(merged.end(), last.begin(), last.end());
                // Добавляем точки первого кластера (в правильном порядке)
                merged.insert(merged.end(), first.begin(), first.end());

                // Удаляем старые кластеры из списка
                clusters.erase(clusters.begin()); // Удаляем первый
                clusters.pop_back();              // Удаляем последний

                // Добавляем объединенный кластер (merged)
                clusters.push_back(std::move(merged));
            }
        }

        // === 3. Обработка и фильтрация кластеров ===
        // Для каждого финального кластера запускаем processCluster
        for (const auto &cluster : clusters)
        {
            // out_cluster_points добавляется только внутри processCluster, без дублирования.
            processCluster(cluster, method_id, results, out_cluster_points);
        }

        return results;
    }

    // ----------------------------------------------------------------------------------
    // Детекция на основе локальных минимумов дальности (ОКОНЧАТЕЛЬНАЯ v5.13)
    // Внедрены: предвычисление радиусов, сравнение квадратов расстояний, защита от
    // захвата уже обработанных лучей и улучшенные условия остановки.
    // ----------------------------------------------------------------------------------
    std::vector<PillarCandidate> detectLocalMinima(const AlignedVector2f &pts, int method_id,
                                                   AlignedVector2f &out_cluster_points)
    {
        std::vector<PillarCandidate> results;
        size_t N = pts.size();
        if (N < min_cluster_points_)
            return results;

        // Параметры
        const int MAX_SEG_POINTS = 40;            // Жесткий лимит точек в одном направлении
        const float MAX_RADIAL_DEVIATION = 0.10f; // Макс отклонение по радиусу (м)
        const float MAX_NEIGHBOR_DIST = 0.10f;    // Порог разрыва между соседними точками (м)

        // Оптимизация: используем квадраты расстояний для избежания sqrt()
        const float MAX_NEIGHBOR_DIST2 = MAX_NEIGHBOR_DIST * MAX_NEIGHBOR_DIST;

        // Вектор флагов для маркировки обработанных точек.
        std::vector<char> processed_flags(N, 0);
        const int WINDOW_SIZE = 5;

        // Оптимизация: Предвычисляем все нормы (расстояния до лидара)
        std::vector<float> r_vals(N);
        for (size_t i = 0; i < N; ++i)
            r_vals[i] = pts[i].norm();

        // Вспомогательная лямбда: квадрат расстояния между двумя точками
        auto dist2_sq = [&](size_t a, size_t b) -> float
        {
            float dx = pts[a].x() - pts[b].x();
            float dy = pts[a].y() - pts[b].y();
            return dx * dx + dy * dy;
        };

        // 1. Поиск локального минимума
        for (size_t i = 0; i < N; ++i)
        {
            if (processed_flags[i])
                continue;

            float r = r_vals[i]; // Используем предвычисленное значение
            bool is_min = true;

            // Проверка локального минимума в циклическом окне
            for (int k = -WINDOW_SIZE; k <= WINDOW_SIZE; ++k)
            {
                if (k == 0)
                    continue;
                size_t j = (i + k + N) % N;
                // Сравнение дальностей
                if (r_vals[j] < r)
                {
                    is_min = false;
                    break;
                }
            }
            if (!is_min)
                continue;

            // 2. Расширение: собираем только индексы
            std::vector<size_t> back_idx;
            back_idx.reserve(MAX_SEG_POINTS);
            int back_count = 0;

            // Cегмент назад (от i-1)
            for (int k = -1; k >= -((int)N); --k)
            {
                size_t cur = (i + k + N) % N;
                size_t next = (i + k + 1 + N) % N;

                // Условие 1: Не захватываем уже обработанные точки
                if (processed_flags[cur])
                    break;

                // Условие 2: Жесткий лимит кол-ва точек
                if (++back_count > MAX_SEG_POINTS)
                    break;

                // Условие 3: Радиальное условие (абсолютная разница)
                if (std::fabs(r_vals[cur] - r) > MAX_RADIAL_DEVIATION)
                    break;

                // Условие 4: Разрыв между соседями (используем квадраты)
                if (dist2_sq(cur, next) > MAX_NEIGHBOR_DIST2)
                    break;

                back_idx.push_back(cur);
            }
            // Переворачиваем, чтобы получить порядок от дальнего к ближнему
            std::reverse(back_idx.begin(), back_idx.end());

            // Cегмент вперед (от i+1)
            std::vector<size_t> fwd_idx;
            fwd_idx.reserve(MAX_SEG_POINTS);
            int fwd_count = 0;
            for (size_t k = 1; k < N; ++k)
            {
                size_t cur = (i + k) % N;
                size_t prev = (i + k - 1 + N) % N;

                // Условие 1: Не захватываем уже обработанные точки
                if (processed_flags[cur])
                    break;

                // Условие 2: Жесткий лимит кол-ва точек
                if (++fwd_count > MAX_SEG_POINTS)
                    break;

                // Условие 3: Радиальное условие
                if (std::fabs(r_vals[cur] - r) > MAX_RADIAL_DEVIATION)
                    break;

                // Условие 4: Разрыв между соседями
                if (dist2_sq(cur, prev) > MAX_NEIGHBOR_DIST2)
                    break;

                fwd_idx.push_back(cur);
            }

            // Защита от захвата всего скана (back + center + fwd не должны превышать N)
            // Обрезаем fwd, если превышен общий лимит (N)
            if (back_idx.size() + 1 + fwd_idx.size() > N)
            {
                size_t allowed = (N > 1) ? (N - 1 - back_idx.size()) : 0;
                if (fwd_idx.size() > allowed)
                    fwd_idx.resize(allowed);
            }

            // 3. Формируем кластер по индексам (гарантированная синхронизация)
            AlignedVector2f cluster;
            cluster.reserve(back_idx.size() + 1 + fwd_idx.size());

            // P_far_back ... P_i-1
            for (size_t idx : back_idx)
                cluster.push_back(pts[idx]);

            cluster.push_back(pts[i]); // P_i (центр)

            // P_i+1 ... P_far_forward
            for (size_t idx : fwd_idx)
                cluster.push_back(pts[idx]);

            // 4. Обработка и маркировка
            if (cluster.size() >= min_cluster_points_)
            {
                processCluster(cluster, method_id, results, out_cluster_points);

                // Маркировка по индексам (синхронизированы с cluster)
                for (size_t idx : back_idx)
                    processed_flags[idx] = 1;
                for (size_t idx : fwd_idx)
                    processed_flags[idx] = 1;
                processed_flags[i] = 1; // Маркируем центр
            }
        } // for i

        return results;
    }

    // Логика слияния (Fusion) (Без изменений)

    // ИЗМЕНЕНА: fuseCandidates (v6.0 - Добавлено подробное логирование)
    // Логика слияния (Fusion)
    // Объединяет кандидатов, найденных разными методами, в единые уникальные столбы.
    // ИЗМЕНЕНА: fuseCandidates (v6.0 - Исправлена ошибка 'size', используется 'num_points')
    // Логика слияния (Fusion)
    // Объединяет кандидатов, найденных разными методами, в единые уникальные столбы.

    AlignedPillarVector fuseCandidates(const std::vector<PillarCandidate> &candidates)
    // AlignedPillarVector fuseCandidatesDBSCAN(const std::vector<PillarCandidate> &candidates)
    {
        AlignedPillarVector final_pillars;
        if (candidates.empty())
        {
            logi.log("DBSCAN Fusion skipped: No candidates.\n");
            return final_pillars;
        }

        logi.log("\n--- DBSCAN Fusion Start: %lu candidates ---\n", candidates.size());

        // Параметры DBSCAN
        const double eps = fusion_group_radius; // радиус объединения кластеров (обычно 0.25–0.35 м)
        const double eps2 = eps * eps;          // квадрат радиуса для оптимизации (избегаем sqrt)
        // const int minPts = 2;                        // минимальное количество точек для образования кластера
        // Заменить жестко заданное minPts = 2 на переменную:
        const int minPts = min_dbscan_points_;

        size_t N = candidates.size();
        std::vector<int> labels(N, -1); // -1 = непосещенная, -2 = шум, >=0 = ID кластера
        int clusterId = 0;              // счетчик кластеров

        // Лямбда для вычисления квадрата расстояния между кандидатами
        auto dist2 = [&](size_t a, size_t b)
        {
            return (candidates[a].center - candidates[b].center).squaredNorm();
        };

        // ---- Фаза 1: Алгоритм DBSCAN ----
        for (size_t i = 0; i < N; ++i)
        {
            if (labels[i] != -1)
                continue; // Пропускаем уже обработанные точки

            // Поиск всех соседей в пределах eps
            std::vector<size_t> neighbors;
            neighbors.reserve(16); // Предварительное выделение памяти

            for (size_t j = 0; j < N; ++j)
                if (dist2(i, j) <= eps2)
                    neighbors.push_back(j);

            // Проверка на core point: достаточно ли соседей?
            if (neighbors.size() < minPts)
            {
                labels[i] = -2; // Помечаем как шум
                continue;
            }

            // Создание нового кластера
            labels[i] = clusterId;

            // Очередь для расширения кластера (начинаем с соседей seed точки)
            std::deque<size_t> q(neighbors.begin(), neighbors.end());

            // Расширение кластера
            while (!q.empty())
            {
                size_t n = q.front();
                q.pop_front();

                if (labels[n] == -2) // Если точка была шумом - переклассифицируем в кластер
                    labels[n] = clusterId;

                if (labels[n] != -1) // Пропускаем уже посещенные точки
                    continue;

                labels[n] = clusterId; // Помечаем как принадлежащую текущему кластеру

                // Поиск соседей для текущей точки
                std::vector<size_t> neigh2;
                neigh2.reserve(16);

                for (size_t k = 0; k < N; ++k)
                    if (dist2(n, k) <= eps2)
                        neigh2.push_back(k);

                // Если точка является core point - добавляем ее соседей в очередь
                if (neigh2.size() >= minPts)
                {
                    for (size_t x : neigh2)
                        if (labels[x] == -1 || labels[x] == -2) // Добавляем только непосещенные или шумовые
                            q.push_back(x);
                }
            }

            clusterId++; // Переходим к следующему кластеру
        }

        // Подсчет и логирование шумовых точек
        int noise_count = 0;
        for (size_t i = 0; i < N; ++i)
            if (labels[i] == -2)
                noise_count++;
        logi.log("DBSCAN: %d noise points filtered out.\n", noise_count);

        if (clusterId == 0)
        {
            logi.log_w("DBSCAN: No clusters found.\n");
            return final_pillars;
        }

        logi.log("DBSCAN: %d initial clusters detected.\n", clusterId);

        // ---- Фаза 2: Аккумуляторы веса для кластеров ----
        struct Acc
        {
            double wsum = 0.0;        // Суммарный вес кластера
            std::vector<size_t> idxs; // Индексы кандидатов в кластере
        };

        std::vector<Acc> acc(clusterId); // Аккумуляторы для каждого кластера

        // Сбор статистики по кластерам
        for (size_t i = 0; i < N; ++i)
        {
            int cid = labels[i];
            if (cid >= 0) // Игнорируем шумовые точки (cid = -2)
            {
                acc[cid].wsum += candidates[i].weight;
                acc[cid].idxs.push_back(i);
            }
        }

        // ---- Фаза 3: Формируем центры кластеров взвешенным усреднением ----
        for (int cid = 0; cid < clusterId; ++cid)
        {
            if (acc[cid].idxs.size() < 1) // Пропускаем пустые кластеры
                continue;

            Eigen::Vector2f c(0, 0); // Взвешенный центр
            double w = 0.0;          // Сумма весов

            // Вычисление взвешенного центра
            for (size_t k : acc[cid].idxs)
            {
                c += candidates[k].center * (float)candidates[k].weight;
                w += candidates[k].weight;
            }

            // Защита от деления на ноль
            if (w < 1e-9)
                continue;

            c /= (float)w; // Нормализация по сумме весов

            // Создание финального столба
            FinalPillar fp;
            fp.local = c;
            fp.total_weight = w;
            fp.name = "Cluster_" + std::to_string(cid);

            logi.log("DBSCAN Cluster %d: center (%.3f, %.3f), points=%lu, weight=%.3f\n",
                     cid, c.x(), c.y(), acc[cid].idxs.size(), w);

            final_pillars.push_back(fp);
        }

        if (final_pillars.empty())
        {
            logi.log_w("DBSCAN: No valid clusters after accumulation.\n");
            return final_pillars;
        }

        // ---- Фаза 4: Если кластеров больше 4 — выбираем top-4 по весу ----
        if (final_pillars.size() > 4)
        {
            logi.log_w("DBSCAN: %lu clusters => selecting top 4 by weight...\n",
                       final_pillars.size());

            // Сортировка по убыванию суммарного веса
            std::sort(final_pillars.begin(), final_pillars.end(),
                      [](const FinalPillar &a, const FinalPillar &b)
                      {
                          return a.total_weight > b.total_weight;
                      });

            final_pillars.resize(4); // Оставляем только 4 лучших кластера

            logi.log_g("DBSCAN: Top 4 clusters selected.\n");
        }

        // // ---- Фаза 5: Сортировка по углу (для согласованного порядка) ----
        // std::sort(final_pillars.begin(), final_pillars.end(),
        //           [](const FinalPillar &a, const FinalPillar &b)
        //           {
        //               return atan2(a.local.y(), a.local.x()) < atan2(b.local.y(), b.local.x());
        //           });

        // ---- Фаза 6: Финальное логирование результатов ----
        for (size_t i = 0; i < final_pillars.size(); ++i)
        {
            double angle = atan2(final_pillars[i].local.y(), final_pillars[i].local.x()) * 180.0 / M_PI;
            double range = final_pillars[i].local.norm();

            logi.log_g("  Pillar %lu: (%.3f, %.3f), R=%.3f m, Angle=%.1f°, W=%.3f\n",
                       i,
                       final_pillars[i].local.x(),
                       final_pillars[i].local.y(),
                       range,
                       angle,
                       final_pillars[i].total_weight);
        }

        logi.log("--- DBSCAN Fusion End: %lu pillars created ---\n", final_pillars.size());

        return final_pillars;
    }

    // НОВАЯ ФУНКЦИЯ: reorderPillars (v6.5 - Ротационно-инвариантная сортировка)
    // Выполняет геометрическое сопоставление и пересортировывает вектор pillars в эталонный порядок.
    // Алгоритм: полный перебор 24 перестановок с минимизацией суммы квадратов ошибок расстояний.
    void reorderPillars(AlignedPillarVector &pillars)
    {
            logi.log_b("\n--- reorderPillars --- \n");
        // --- ПРОВЕРКА ВХОДНЫХ ДАННЫХ ---
        if (pillars.size() != 4)
        {
            logi.log_r("    Reorder failed: Expected 4 pillars, found %lu.\n", pillars.size());
            return; // Выход если количество столбов не соответствует ожидаемому
        }

        // --- КОНФИГУРАЦИЯ АЛГОРИТМА ---
        const double ACCEPTABLE_RMSE = 0.05; // Допустимая среднеквадратичная ошибка (5 см)

        // Эталонные имена столбов в правильном порядке
        std::vector<std::string> names = {"RB", "RT", "LT", "LB"};

        // Пары индексов для 6 расстояний между столбами и их соответствие d_center
        // d_center: 0=RB-RT, 1=RB-LT, 2=RB-LB, 3=RT-LT, 4=RT-LB, 5=LT-LB
        std::vector<std::pair<int, int>> ref_pairs = {
            {0, 1}, // RB-RT = d_center[0]
            {0, 2}, // RB-LT = d_center[1]
            {0, 3}, // RB-LB = d_center[2]
            {1, 2}, // RT-LT = d_center[3]
            {1, 3}, // RT-LB = d_center[4]
            {2, 3}  // LT-LB = d_center[5]
        };

        // --- ПЕРЕБОР ВСЕХ ПЕРЕСТАНОВОК ---
        std::vector<int> current_match = {0, 1, 2, 3};        // Текущая перестановка индексов (начинаем с тождественной)
        std::vector<int> best_match(4);                       // Лучшая найденная перестановка
        double min_cost = std::numeric_limits<double>::max(); // Минимальная стоимость (инициализируем максимумом)
        bool perfect_match_found = false;                     // Флаг идеального соответствия

        logi.log("--- Starting geometric matching of %lu pillars...\n", pillars.size());

        do
        {
            // ВЫЧИСЛЕНИЕ СТОИМОСТИ ТЕКУЩЕЙ ПЕРЕСТАНОВКИ
            double current_cost = 0.0; // Сумма квадратов ошибок для текущей перестановки

            // Проходим по всем 6 парам расстояний
            for (int k = 0; k < 6; ++k)
            {
                // Получаем индексы эталонной пары
                int i_ref = ref_pairs[k].first;
                int j_ref = ref_pairs[k].second;

                // Получаем соответствующие индексы в текущей перестановке
                int i_pillar_idx = current_match[i_ref];
                int j_pillar_idx = current_match[j_ref];

                // Вычисляем измеренное расстояние между столбами
                double measured_dist = MathUtils::dist2D(pillars[i_pillar_idx].local,
                                                         pillars[j_pillar_idx].local);

                // Получаем ожидаемое расстояние из эталонных данных
                double expected_dist = d_center[k];

                // Вычисляем квадрат ошибки и добавляем к общей стоимости
                double error = measured_dist - expected_dist;
                current_cost += error * error; // Используем L2-норму для устойчивости
            }

            // ПРОВЕРКА НА ИДЕАЛЬНОЕ СООТВЕТСТВИЕ (для оптимизации)
            if (current_cost < 1e-6 && !perfect_match_found)
            {
                logi.log_g("    Perfect match found! Cost=%.2e\n", current_cost);
                perfect_match_found = true;
                best_match = current_match;
                min_cost = current_cost;
                break; // Выходим из цикла перебора - нашли идеальное решение
            }

            // ОБНОВЛЕНИЕ ЛУЧШЕГО РЕШЕНИЯ
            if (current_cost < min_cost)
            {
                min_cost = current_cost;
                best_match = current_match; // Сохраняем лучшую перестановку
            }

        } while (std::next_permutation(current_match.begin(), current_match.end())); // Перебираем все 24 варианта

        // --- ВАЛИДАЦИЯ РЕЗУЛЬТАТА ---
        double rmse = std::sqrt(min_cost / 6.0); // Среднеквадратичная ошибка по всем 6 расстояниям

        if (rmse > ACCEPTABLE_RMSE)
        {
            logi.log_r("WARNING: High matching error (RMSE=%.3f m). Identification may be unreliable.\n", rmse);
        }
        else
        {
            logi.log_g("Geometric matching successful. RMSE=%.3f m\n", rmse);
        }

        // --- ДЕТАЛЬНОЕ ЛОГИРОВАНИЕ ЛУЧШЕЙ КОНФИГУРАЦИИ ---
        logi.log("Best match configuration analysis:\n");
        for (int k = 0; k < 6; ++k)
        {
            int i_ref = ref_pairs[k].first;
            int j_ref = ref_pairs[k].second;
            int i_actual = best_match[i_ref];
            int j_actual = best_match[j_ref];

            double measured = MathUtils::dist2D(pillars[i_actual].local,
                                                pillars[j_actual].local);
            double expected = d_center[k];
            double error = std::abs(measured - expected);

            // Цветное логирование в зависимости от величины ошибки
            if (error <= 0.001)
                logi.log_g("  %s-%s: %.3f (exp: %.3f) ✓ Δ=%.1f mm\n",
                           names[i_ref].c_str(), names[j_ref].c_str(), measured, expected, error * 1000);
            else if (error <= 0.010)
                logi.log_w("  %s-%s: %.3f (exp: %.3f) ~ Δ=%.1f mm\n",
                           names[i_ref].c_str(), names[j_ref].c_str(), measured, expected, error * 1000);
            else
                logi.log_r("  %s-%s: %.3f (exp: %.3f) ✗ Δ=%.1f mm\n",
                           names[i_ref].c_str(), names[j_ref].c_str(), measured, expected, error * 1000);
        }

        // --- ПЕРЕСОРТИРОВКА И ПЕРЕИМЕНОВАНИЕ СТОЛБОВ ---
        AlignedPillarVector sorted_pillars(4);
        std::string mapping_log = "Pillar mapping result: ";

        for (int i = 0; i < 4; ++i)
        {
            // best_match[i] содержит индекс исходного столба, который должен занять позицию i
            int original_index = best_match[i];

            // Копируем столб из исходной позиции в правильную позицию
            sorted_pillars[i] = pillars[original_index];

            // Присваиваем эталонное имя согласно новой позиции
            sorted_pillars[i].name = names[i];

            // Сохраняем индекс эталона для последующей калибровки
            sorted_pillars[i].ref_index = i;

            // Формируем строку лога для отладки
            mapping_log += names[i] + "←P" + std::to_string(original_index) + " ";
        }

        // --- ФИНАЛЬНАЯ ЗАМЕНА И ЛОГИРОВАНИЕ ---
        pillars = sorted_pillars; // Заменяем исходный вектор отсортированной версией

        logi.log("%s\n", mapping_log.c_str());
        logi.log_g("Pillars successfully reordered and renamed.\n");

        // Дополнительный лог финальных координат
        logi.log("Final pillar coordinates:\n");
        for (const auto &pillar : pillars)
        {
            double angle = atan2(pillar.local.y(), pillar.local.x()) * 180.0 / M_PI;
            logi.log("  %s: [%.3f, %.3f] (angle: %.1f°)\n",
                     pillar.name.c_str(), pillar.local.x(), pillar.local.y(), angle);
        }
    }

    // Сохранение результатов в ROS Parameter Server (Без изменений)
    void saveResults(const AlignedPillarVector &pillars)
    {
        for (const auto &p : pillars)
        {
            if (p.name.find("Pillar_") != std::string::npos)
                continue;
            std::string base = "/pb/scan/result/" + p.name;
            nh.setParam(base + "/x", p.global.x());
            nh.setParam(base + "/y", p.global.y());
        }
        logi.log_g("Results saved to rosparam.\n");
    }

    // --- ИЗМЕНЕНА: removeEdgeArtifacts (Полностью - v5.5) ---
    // Удаление фантомных точек (хвостов) с помощью углового фильтра
    AlignedVector2f removeEdgeArtifacts(const AlignedVector2f &points, const std::vector<double> &intensities, int &points_removed_by_angle_filter)
    {
        logi.log("\n--- removeEdgeArtifacts ---\n");
        // Очищаем счетчик удаленных точек
        points_removed_by_angle_filter = 0;

        // Получаем общее количество точек до фильтрации
        size_t initial_point_count = points.size();

        // Проверка на минимальный размер скана
        if (initial_point_count < 2)
        {
            return points;
        }

        AlignedVector2f clean_points;                            // Вектор для хранения очищенных точек
        const double ANGLE_THRESHOLD_RAD = edge_angle_threshold; // Порог угла в радианах (берется из params.yaml)

        // ВРЕМЕННЫЙ ЛОГ ЗАГОЛОВКА (v5.5)
        logi.log("\n--- DETAILED ANGLE FILTER DEBUG LOG (Angle Threshold: %.1f deg) ---\n", edge_angle_threshold * 180.0 / M_PI);
        logi.log("P_IDX | P_Curr_X | P_Curr_Y | Lidar_Ang | P_Next_X | P_Next_Y | Angle(deg) | ABS_Check_Ang | Decision\n");
        logi.log("---------------------------------------------------------------------------------------------------\n");

        // Всегда добавляем первую точку (points[0])
        clean_points.push_back(points[0]);

        // Итерируем до предпоследней точки (points[N-2])
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            const Eigen::Vector2f &P_curr = points[i];     // Текущая точка
            const Eigen::Vector2f &P_next = points[i + 1]; // Следующая точка

            Eigen::Vector2f V_ray = P_curr;               // Вектор луча (от (0,0) до P_curr)
            Eigen::Vector2f V_seg_next = P_next - P_curr; // Вектор сегмента P_curr -> P_next
            double angle_rad = M_PI;                      // Исходный угол [0, 180]
            double angle_check = M_PI / 2.0;              // Угол для проверки параллельности [0, 90]

            // Расчет угла между V_ray и V_seg_next
            if (V_ray.norm() > 0.001 && V_seg_next.norm() > 0.001)
            {
                double dot_prod = V_ray.normalized().dot(V_seg_next.normalized());
                angle_rad = std::acos(std::max(-1.0, std::min(dot_prod, 1.0)));

                // Расчет угла, насколько сегмент параллелен лучу, независимо от направления (v5.3)
                // Это решает проблему симметрии 0 градусов и 180 градусов
                angle_check = std::min(angle_rad, M_PI - angle_rad);
            }

            double angle_deg = angle_rad * 180.0 / M_PI;
            double angle_check_deg = angle_check * 180.0 / M_PI;

            // Расчет абсолютного угла лидара для P_curr (v5.4)
            double lidar_angle_rad = std::atan2(P_curr.y(), P_curr.x());
            double lidar_angle_deg = lidar_angle_rad * 180.0 / M_PI;

            // --- УСЛОВИЕ ФАНТОМА (v5.3) ---
            // Если угол параллельности меньше порога
            if (std::abs(angle_check) < ANGLE_THRESHOLD_RAD)
            {
                points_removed_by_angle_filter++;

                // ВРЕМЕННЫЙ ЛОГ ТЕКУЩЕЙ ИТЕРАЦИИ (v5.5)
                logi.log("%5lu | %8.3f | %8.3f | %9.3f | %8.3f | %8.3f | %10.3f | %13.3f | %s\n",
                         i + 1, P_curr.x(), P_curr.y(), lidar_angle_deg, P_next.x(), P_next.y(),
                         angle_deg, angle_check_deg,
                         "REMOVED_P_NEXT");

                // Пропускаем P_next (удаляем ее)
                i++;
                continue;
            }

            // Если условие не сработало, P_next — валидная точка, добавляем ее.
            clean_points.push_back(P_next);

            // ВРЕМЕННЫЙ ЛОГ ТЕКУЩЕЙ ИТЕРАЦИИ (v5.5)
            // logi.log("%5lu | %8.3f | %8.3f | %9.3f | %8.3f | %8.3f | %10.3f | %13.3f | %s\n",
            //          i + 1, P_curr.x(), P_curr.y(), lidar_angle_deg, P_next.x(), P_next.y(),
            //          angle_deg, angle_check_deg,
            //          "KEPT_P_NEXT");
        }

        logi.log("--- END DETAILED ANGLE FILTER DEBUG LOG ---\n");

        // Вывод статистики в конце функции (v5.5)
        size_t final_point_count = clean_points.size();

        logi.log_b("ANGLE FILTER STATS: Initial points: %lu. Removed: %d. Final points: %lu.\n",
                   initial_point_count, points_removed_by_angle_filter, final_point_count);

        return clean_points;
    }

    // --- Калибровка: Полноценный Umeyama's Algorithm (Без изменений) ---
    void performCalibration(AlignedPillarVector &pillars)
    {
        if (pillars.size() != 4)
        {
            logi.log_w("Calibration skipped: Need exactly 4 pillars (Found %lu). Not setting flag.\n", pillars.size());
            return;
        }

        logi.log("\n--- Performing FULL Umeyama Calibration (SVD) ---\n");

        // 1. Идентификация
        int match_index[4] = {-1, -1, -1, -1};
        std::vector<bool> pillar_used(4, false);

        int idx_RB = -1;
        int idx_RT = -1;
        double best_err = 1000.0;

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
            logi.log_r("Calibration Failed: Could not identify RB-RT pair.\n");
            return;
        }

        match_index[0] = idx_RB;
        pillar_used[idx_RB] = true;
        match_index[1] = idx_RT;
        pillar_used[idx_RT] = true;
        pillars[idx_RB].name = "RB";
        pillars[idx_RT].name = "RT";

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

        // 2. Подготовка матриц (DOUBLE)
        Eigen::Matrix<double, 2, 4> X;
        Eigen::Matrix<double, 2, 4> Y;

        for (int i = 0; i < 4; ++i)
        {
            X.col(i) = pillars[match_index[i]].local.cast<double>();
            Y.col(i) = reference_centers_[i].cast<double>();
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

        logi.log_g("Umeyama Transform:\n");
        logi.log_g("  Scale (c): %.4f\n", c);

        double angle_rad = atan2(R(1, 0), R(0, 0));
        logi.log_g("  Rotation (deg): %.3f\n", angle_rad * 180.0 / M_PI);

        logi.log_g("  Translation (T): [%.3f, %.3f]\n", T.x(), T.y());

        // 9. Применение R,c,T (сначала без фиксации)
        Eigen::Vector2d global_RB;
        for (int i = 0; i < 4; ++i)
        {
            FinalPillar &p = pillars[match_index[i]];
            Eigen::Vector2d p_local_double = p.local.cast<double>();
            p.global = c * R * p_local_double + T;

            if (i == 0) // match_index[0] = RB
                global_RB = p.global;
        }

        logi.log_w("RB before fix: [%.3f, %.3f]\n", global_RB.x(), global_RB.y());

        // ---- Фиксация RB в (0,0) ----
        Eigen::Vector2d delta = -global_RB;

        logi.log_b("Fixing RB to global (0,0). Delta applied: [%.3f, %.3f]\n",
                   delta.x(), delta.y());

        for (int i = 0; i < 4; ++i)
        {
            pillars[match_index[i]].global += delta;
        }

        // 10. Пересчёт RMSE уже после фиксации!
        double final_rmse_sum_sq = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            FinalPillar &p = pillars[match_index[i]];

            logi.log_g("Pillar %s -> Global FIXED: [%.3f, %.3f]\n",
                       p.name.c_str(), p.global.x(), p.global.y());

            Eigen::Vector2d ref_double = reference_centers_[i].cast<double>();
            Eigen::Vector2d error = p.global - ref_double;
            final_rmse_sum_sq += error.squaredNorm();
        }

        double final_rmse = sqrt(final_rmse_sum_sq / 4.0);
        logi.log_g("Final Alignment RMSE (after RB fix): %.5f meters\n", final_rmse);

        // --- ХРАНЕНИЕ РЕЗУЛЬТАТОВ ---
        if (final_rmse <= 0.05)
        {
            final_pillars_results_ = pillars;
            calibration_done_ = true;

            publishFinalMarkers(final_pillars_results_);
        }
        else
        {
            logi.log_r("Calibration FAILED: High final alignment error (RMSE > 5 cm, current: %.5f). Not setting flag.\n", final_rmse);
        }
    }

    // ----------------------------------------------------------------------------------
    // 4. ПУБЛИЧНЫЕ МЕТОДЫ
    // ----------------------------------------------------------------------------------
public:
    PillarScanNode() : scans_collected(0), calibration_done_(false),
                       total_rays_removed_by_zero_intensity(0),   // <--- ОБНУЛЕНИЕ
                       total_rays_removed_by_low_intensity(0),    // <--- ОБНУЛЕНИЕ
                       total_rays_removed_by_initial_intensity(0) // <--- ОБНУЛЕНИЕ
    {
        logi.log("\n=== PillarScanNode v5.9 Started (Configurable Filters) ===\n"); // Обновление версии

        loadParameters();
        if (!ros::ok())
            return;

        // Инициализация паблишеров
        pub_filtered_scan = nh.advertise<visualization_msgs::Marker>("/rviz/filtered_scan", 1);

        // Три отдельных топика для методов
        pub_method_1 = nh.advertise<visualization_msgs::Marker>("/rviz/method_1_jump", 1);
        pub_method_2 = nh.advertise<visualization_msgs::Marker>("/rviz/method_2_cluster", 1);
        pub_method_3 = nh.advertise<visualization_msgs::Marker>("/rviz/method_3_minima", 1);

        pub_fused_pillars = nh.advertise<visualization_msgs::Marker>("/rviz/fused_pillars", 1);
        pub_final_markers = nh.advertise<visualization_msgs::MarkerArray>("/rviz/final_pillars", 1);

        initReferenceSystem();

        logi.log("Checking /scan topic availability (timeout 30s)...\n");

        // Ожидание первого сообщения /scan
        sensor_msgs::LaserScan::ConstPtr first_scan =
            ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(30));

        if (!first_scan)
        {
            logi.log_r("Timed out waiting for /scan topic. Is the LiDAR node running? Shutting down.\n");
            ros::shutdown();
            return;
        }

        size_t num_rays = first_scan->ranges.size();
        accumulated_ranges.resize(num_rays);
        accumulated_intensities.resize(num_rays);
        meta_scan = *first_scan;

        logi.log_b("LiDAR initialized. Rays: %lu. Starting accumulation.\n", num_rays);

        scan_sub = nh.subscribe("/scan", 100, &PillarScanNode::scanCallback, this);
        logi.log("Waiting for %d laser scans on /scan topic to complete initial calibration.\n", SCANS_TO_COLLECT);

        publish_timer_ = nh.createTimer(ros::Duration(1.0), &PillarScanNode::publishResultsTimerCallback, this);
    }

    void loadParameters()
    {
        logi.log_r("\n--- Loading YAML Parameters ---\n");

        // Вспомогательная лямбда для безопасного чтения double
        auto loadParam = [this](const std::string &key, double &var, double default_val, const char *name)
        {
            if (!nh.getParam(key, var))
            {
                logi.log_r("WARNING: Param '%s' not found, using default = %.4f\n", name, default_val);
                var = default_val;
                return false;
            }
            else
            {
                logi.log("Loaded '%s' = %.4f\n", name, var);
                return true;
            }
        };

        // Вспомогательная лямбда для int
        auto loadParamInt = [this](const std::string &key, int &var, int default_val, const char *name)
        {
            if (!nh.getParam(key, var))
            {
                logi.log_r("WARNING: Param '%s' not found, using default = %d\n", name, default_val);
                var = default_val;
                return false;
            }
            else
            {
                logi.log("Loaded '%s' = %d\n", name, var);
                return true;
            }
        };

        // 1. Диаметр столба и радиус
        loadParam("/pb_config/scan_node/pillar_diametr", pillar_diam_, 0.315, "pillar_diametr");
        pillar_radius_ = pillar_diam_ / 2.0;

        // 2. Эталонные расстояния между поверхностями (6 штук)
        loadParam("/pb_config/scan_node/pillar_0_1", d_surf[0], 10.5, "pillar_0_1");
        loadParam("/pb_config/scan_node/pillar_0_2", d_surf[1], 12.0, "pillar_0_2");
        loadParam("/pb_config/scan_node/pillar_0_3", d_surf[2], 4.8, "pillar_0_3");
        loadParam("/pb_config/scan_node/pillar_1_2", d_surf[3], 5.5, "pillar_1_2");
        loadParam("/pb_config/scan_node/pillar_1_3", d_surf[4], 11.5, "pillar_1_3");
        loadParam("/pb_config/scan_node/pillar_2_3", d_surf[5], 4.8, "pillar_2_3");

        // Вычисление расстояний между центрами
        for (int i = 0; i < 6; ++i)
        {
            d_center[i] = d_surf[i] + pillar_diam_;
        }

        // Логирование расстояний (исправлены индексы!)
        logi.log_b("    pillar_diametr: %.4f\n", pillar_diam_);
        logi.log("    pillar_0_1 RB-RT (surf): %.4f -> (center): %.4f\n", d_surf[0], d_center[0]);
        logi.log("    pillar_0_2 RB-LT (surf): %.4f -> (center): %.4f\n", d_surf[1], d_center[1]);
        logi.log("    pillar_0_3 RB-LB (surf): %.4f -> (center): %.4f\n", d_surf[2], d_center[2]);
        logi.log("    pillar_1_2 RT-LT (surf): %.4f -> (center): %.4f\n", d_surf[3], d_center[3]);
        logi.log("    pillar_1_3 RT-LB (surf): %.4f -> (center): %.4f\n", d_surf[4], d_center[4]);
        logi.log("    pillar_2_3 LT-LB (surf): %.4f -> (center): %.4f\n", d_surf[5], d_center[5]);

        // --- Параметры фильтрации кластеров ---
        loadParam("/pb_config/scan_node/min_cluster_width", min_cluster_width_, 0.20, "min_cluster_width");
        loadParam("/pb_config/scan_node/max_cluster_width", max_cluster_width_, 0.40, "max_cluster_width");
        loadParamInt("/pb_config/scan_node/min_cluster_points", min_cluster_points_, 11, "min_cluster_points");

        logi.log_b("    Cluster Filter Config:\n");
        logi.log("    Min Cluster Width: %.3f m\n", min_cluster_width_);
        logi.log("    Max Cluster Width: %.3f m\n", max_cluster_width_);
        logi.log("    Min Cluster Points: %d\n", min_cluster_points_);

        // Параметры детекции
        loadParam("/pb_config/scan_node/jump_dist_threshold", jump_dist_threshold, 0.33, "jump_dist_threshold");
        loadParam("/pb_config/scan_node/cluster_dist_threshold", cluster_dist_threshold, 0.05, "cluster_dist_threshold");
        logi.log("    Detection: jump_dist_threshold=%.2f, cluster_dist_threshold=%.2f\n", jump_dist_threshold, cluster_dist_threshold);

        // Веса методов
        loadParam("/pb_config/scan_node/w_method_1_jump", w_method[1], 1.0, "w_method_1_jump");
        loadParam("/pb_config/scan_node/w_method_2_cluster", w_method[2], 0.9, "w_method_2_cluster");
        loadParam("/pb_config/scan_node/w_method_3_minima", w_method[3], 0.8, "w_method_3_minima");
        logi.log_b("    Weights: method_1=%.2f, method_2=%.2f, method_3=%.2f\n", w_method[1], w_method[2], w_method[3]);

        // Параметры фильтрации сканов
        loadParam("/pb_config/scan_node/min_range_filter", min_range_filter, 0.2, "min_range");
        loadParam("/pb_config/scan_node/max_range_filter", max_range_filter, 15.0, "max_range");
        loadParam("/pb_config/scan_node/neighbor_radius_filter", neighbor_radius_filter, 0.3, "neighbor_radius");
        loadParamInt("/pb_config/scan_node/min_neighbors_filter", min_neighbors_filter, 3, "min_neighbors");
        loadParam("/pb_config/scan_node/intensity_min_threshold", intensity_min_threshold, 10.0, "intensity_min_threshold");
        loadParam("/pb_config/scan_node/edge_angle_threshold_deg", edge_angle_threshold, 15.0, "edge_angle_threshold_deg");
        edge_angle_threshold *= M_PI / 180.0; // в радианы

        logi.log("    Filter Range: [%.2f, %.2f], KNN: R=%.2f, N=%d\n", min_range_filter, max_range_filter, neighbor_radius_filter, min_neighbors_filter);
        logi.log("    Artifact Filter: I_min=%.2f, Angle_rad=%.4f (%.1f deg)\n", intensity_min_threshold, edge_angle_threshold, edge_angle_threshold * 180.0 / M_PI);

        // Параметры Fusion
        // ИСПРАВЛЕНО: убран лишний слэш в начале
        loadParam("/pb_config/scan_node/rmse_max_tolerance", rmse_max_tolerance, 0.01, "rmse_max_tolerance");
        loadParamInt("/pb_config/scan_node/n_max_points_norm", n_max_points_norm, 100, "n_max_points_norm");
        loadParam("/pb_config/scan_node/fusion_group_radius", fusion_group_radius, 0.2, "fusion_group_radius");
        loadParamInt("/pb_config/scan_node/min_dbscan_points", min_dbscan_points_, 2, "min_dbscan_points");

        logi.log_b("    DBSCAN Fusion Config:\n");
        logi.log("    Fusion: RMSE_max=%.4f, N_max=%d \n", rmse_max_tolerance, n_max_points_norm);
        logi.log("    Min DBSCAN Points (minPts): %d\n", min_dbscan_points_);
        logi.log("    Fusion Group Radius: %.3f m\n", fusion_group_radius);

        logi.log_r("--- Parameters Loaded ---\n");
    }

    // ИЗМЕНЕНА: initReferenceSystem (v6.1 - LM Optimization)
    // Находит координаты эталонной системы с помощью нелинейной оптимизации
    // Levenberg-Marquardt, минимизируя невязки по всем 6 расстояниям.
    void initReferenceSystem()
    {
        reference_centers_.resize(4);

        logi.log("\n=== Reference System Initialization (Levenberg-Marquardt) ===\n");

        // Начальное приближение — трилатерация
        double L = d_center[0];
        reference_centers_[0] = Eigen::Vector2f(0, 0);        // RB (0)
        reference_centers_[1] = Eigen::Vector2f((float)L, 0); // RT (1)

        // LT (2) - начальное приближение
        // x2 = (d_0_1^2 + d_0_2^2 - d_1_2^2) / (2*d_0_1)
        double x2 = (L * L + d_center[1] * d_center[1] - d_center[3] * d_center[3]) / (2 * L);
        double y2 = std::sqrt(std::max(0.0, d_center[1] * d_center[1] - x2 * x2));
        reference_centers_[2] = Eigen::Vector2f((float)x2, (float)y2);

        // LB (3) - начальное приближение
        // x3 = (d_0_1^2 + d_0_3^2 - d_1_3^2) / (2*d_0_1)
        double x3 = (L * L + d_center[2] * d_center[2] - d_center[4] * d_center[4]) / (2 * L);
        double y3 = std::sqrt(std::max(0.0, d_center[2] * d_center[2] - x3 * x3));
        reference_centers_[3] = Eigen::Vector2f((float)x3, (float)y3);

        logi.log_b("Initial approximation:\n");
        for (int i = 0; i < 4; ++i)
            logi.log(" P%d: [%.4f, %.4f]\n", i, reference_centers_[i].x(), reference_centers_[i].y());
        logi.log("End approximation:\n");

        // --- LM ОПТИМИЗАЦИЯ ---
        // Параметры: x2, y2, x3, y3
        Eigen::Vector4d params;
        params << reference_centers_[2].x(), reference_centers_[2].y(),
            reference_centers_[3].x(), reference_centers_[3].y();

        const int max_iter = 50;
        const double tol = 1e-12; // Более строгий критерий сходимости
        double lambda = 0.001;
        double best_cost = std::numeric_limits<double>::max();
        Eigen::Vector4d best_params = params;

        for (int iter = 0; iter < max_iter; ++iter)
        {
            Eigen::Vector2d p2(params[0], params[1]); // LT (2)
            Eigen::Vector2d p3(params[2], params[3]); // LB (3)
            Eigen::Vector2d rb(0, 0);                 // RB (0)
            Eigen::Vector2d rt(L, 0);                 // RT (1)

            // Остатки (6 расстояний) - ИСПРАВЛЕННЫЕ ИНДЕКСЫ
            Eigen::VectorXd residuals(6);
            residuals[0] = (rt - rb).norm() - d_center[0]; // RB-RT = 4.0150
            residuals[1] = (p2 - rb).norm() - d_center[1]; // RB-LT = 6.0930
            residuals[2] = (p3 - rb).norm() - d_center[2]; // RB-LB = 3.5350
            residuals[3] = (p2 - rt).norm() - d_center[3]; // RT-LT = 3.8250
            residuals[4] = (p3 - rt).norm() - d_center[4]; // RT-LB = 5.0990
            residuals[5] = (p2 - p3).norm() - d_center[5]; // LT-LB = 4.4940

            double cost = residuals.squaredNorm();

            if (cost < best_cost)
            {
                best_cost = cost;
                best_params = params;
            }

            if (cost < tol)
            {
                logi.log(" Converged after %d iterations (cost=%.2e)\n", iter, cost);
                break;
            }

            if (iter % 10 == 0)
            {
                logi.log(" Iter %d: cost=%.2e, lambda=%.2e\n", iter, cost, lambda);
            }

            // Якобиан 6x4 (J) - ИСПРАВЛЕННЫЕ ИНДЕКСЫ
            Eigen::Matrix<double, 6, 4> J = Eigen::Matrix<double, 6, 4>::Zero();

            auto jacobian_row = [&](int row, const Eigen::Vector2d &p, const Eigen::Vector2d &fixed, int px, int py)
            {
                double d = (p - fixed).norm();
                if (d > 1e-12)
                {
                    J(row, px) = (p.x() - fixed.x()) / d;
                    J(row, py) = (p.y() - fixed.y()) / d;
                }
            };

            jacobian_row(1, p2, rb, 0, 1); // residuals[1] = RB-LT
            jacobian_row(2, p3, rb, 2, 3); // residuals[2] = RB-LB
            jacobian_row(3, p2, rt, 0, 1); // residuals[3] = RT-LT
            jacobian_row(4, p3, rt, 2, 3); // residuals[4] = RT-LB

            // residuals[5] = LT-LB
            double d_lt_lb = (p2 - p3).norm();
            if (d_lt_lb > 1e-12)
            {
                J(5, 0) = (p2.x() - p3.x()) / d_lt_lb;
                J(5, 1) = (p2.y() - p3.y()) / d_lt_lb;
                J(5, 2) = (p3.x() - p2.x()) / d_lt_lb;
                J(5, 3) = (p3.y() - p2.y()) / d_lt_lb;
            }

            // RB-RT (r5) - строка остается нулевой, т.к. P0 и P1 фиксированы.

            Eigen::Matrix4d H = J.transpose() * J;
            Eigen::Vector4d g = J.transpose() * residuals;

            // LM итерация
            bool step_accepted = false;
            for (int lm_iter = 0; lm_iter < 10; ++lm_iter)
            {
                Eigen::Matrix4d H_lm = H + lambda * Eigen::Matrix4d::Identity();
                Eigen::Vector4d delta = H_lm.ldlt().solve(-g);

                if (!delta.allFinite())
                {
                    lambda *= 10;
                    continue;
                }

                Eigen::Vector4d params_new = params + delta;
                Eigen::Vector2d p2_new(params_new[0], params_new[1]);
                Eigen::Vector2d p3_new(params_new[2], params_new[3]);

                // Новые остатки (после исправления)
                Eigen::VectorXd res_new(6);
                res_new[0] = (rt - rb).norm() - d_center[0];         // RB-RT
                res_new[1] = (p2_new - rb).norm() - d_center[1];     // RB-LT
                res_new[2] = (p3_new - rb).norm() - d_center[2];     // RB-LB
                res_new[3] = (p2_new - rt).norm() - d_center[3];     // RT-LT
                res_new[4] = (p3_new - rt).norm() - d_center[4];     // RT-LB
                res_new[5] = (p2_new - p3_new).norm() - d_center[5]; // LT-LB

                double new_cost = res_new.squaredNorm();

                if (new_cost < cost)
                {
                    params = params_new;
                    lambda = std::max(lambda / 10.0, 1e-12);
                    step_accepted = true;
                    break;
                }
                else
                {
                    lambda *= 10;
                }
            }

            if (!step_accepted)
            {
                logi.log(" LM: No best improvement, stopping at iter %d\n", iter);
                break;
            }
        }

        // Используем лучшие параметры
        params = best_params;
        reference_centers_[2] = Eigen::Vector2f((float)params[0], (float)params[1]);
        reference_centers_[3] = Eigen::Vector2f((float)params[2], (float)params[3]);

        // Детальная проверка ошибок
        logi.log("=== Distance Validation ===\n");
        double max_error = 0.0;

        std::vector<std::string> dist_names = {"RB-RT", "RB-LT", "RB-LB", "RT-LT", "RT-LB", "LT-LB"};
        std::vector<std::pair<int, int>> indices = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

        for (int i = 0; i < 6; ++i)
        {
            double measured = MathUtils::dist2D(reference_centers_[indices[i].first],
                                                reference_centers_[indices[i].second]);
            double expected = d_center[i];
            double error = std::abs(measured - expected);

            if (error > max_error)
                max_error = error;

            if (error <= 0.001) // 1 мм
                logi.log_g(" %s: %.4f (expected: %.4f) ✓ Δ=%.1f mm\n",
                           dist_names[i].c_str(), measured, expected, error * 1000);
            else if (error <= 0.010) // 10 мм
                logi.log_w(" %s: %.4f (expected: %.4f) ~ Δ=%.1f mm\n",
                           dist_names[i].c_str(), measured, expected, error * 1000);
            else
                logi.log_r(" %s: %.4f (expected: %.4f) ✗ Δ=%.1f mm\n",
                           dist_names[i].c_str(), measured, expected, error * 1000);
        }

        // Финальное решение
        if (max_error > 0.010) // 10 мм
        {
            logi.log_r("\nFATAL: Pillar geometry inconsistent! Max error = %.1f mm\n", max_error * 1000);
            reference_centers_.clear();
        }
        else
        {
            logi.log_g("\nGeometry validated. Max residual = %.1f mm ✓\n", max_error * 1000);
            logi.log_g("Final coordinates:\n");
            std::vector<std::string> names = {"RB", "RT", "LT", "LB"};
            for (int i = 0; i < 4; ++i)
                logi.log_g(" %s: [%.4f, %.4f]\n", names[i].c_str(),
                           reference_centers_[i].x(), reference_centers_[i].y());
        }

        logi.log("==============================================\n");
    }

    // МЕТОД: scanCallback
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        if (calibration_done_ || scans_collected >= SCANS_TO_COLLECT)
        {
            return;
        }

        if (scans_collected == 0)
        {
            meta_scan = *scan;
            accumulated_ranges.resize(scan->ranges.size());
            accumulated_intensities.resize(scan->ranges.size());
            logi.log("LiDAR initialized. Rays: %lu. Starting accumulation.\n", scan->ranges.size());
        }

        bool has_intensities = (scan->intensities.size() == scan->ranges.size());

        int removed_by_zero_in_current_scan = 0;
        int removed_by_low_in_current_scan = 0;
        int current_scan_added_count = 0;

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float r = scan->ranges[i];
            float intensity = has_intensities ? scan->intensities[i] : 0.0f;

            // ФИЛЬТРАЦИЯ ПО ИНТЕНСИВНОСТИ
            if (has_intensities)
            {
                if (intensity == 0.0f)
                {
                    removed_by_zero_in_current_scan++;
                    total_rays_removed_by_zero_intensity++;
                    total_rays_removed_by_initial_intensity++;
                    continue;
                }

                if (intensity < intensity_min_threshold)
                {
                    removed_by_low_in_current_scan++;
                    total_rays_removed_by_low_intensity++;
                    total_rays_removed_by_initial_intensity++;
                    continue;
                }
            }
            if (std::isinf(r) || std::isnan(r))
                continue;
            if (r < min_range_filter || r > max_range_filter)
                continue;
            accumulated_ranges[i].push_back(r);
            accumulated_intensities[i].push_back(intensity); // Сохраняем  только если прошли фильтр
            current_scan_added_count++;
        }

        scans_collected++;

        logi.log("Collecting scans: %d/%d. Kept: %d. Removed: Zero=%d, Low=%d. Total Intensity Removed: %lld\n",
                 scans_collected, SCANS_TO_COLLECT, current_scan_added_count,
                 removed_by_zero_in_current_scan, removed_by_low_in_current_scan,
                 total_rays_removed_by_initial_intensity);

        if (scans_collected == SCANS_TO_COLLECT)
        {
            logi.log("Accumulation complete. Starting processing pipeline...\n");
            processPipeline();
        }
    }

    // ИЗМЕНЕНА: processPipeline (v5.7)
    // Сохранение результатов в разные переменные маркеров
    void processPipeline()
    {
        logi.log("\n=== Starting Processing Pipeline (Median Filtered Scan) ===\n");

        int total_initial_rays = 0;
        int points_removed_by_angle_filter = 0;

        // 1. ФОРМИРОВАНИЕ МЕДИАННОГО СКАНА
        sensor_msgs::LaserScan current_filtered_scan = meta_scan;
        current_filtered_scan.ranges.clear();
        current_filtered_scan.intensities.clear();
        AlignedVector2f initial_points;
        std::vector<double> median_intensities;
        float nan_val = std::numeric_limits<float>::quiet_NaN();

        for (size_t i = 0; i < accumulated_ranges.size(); ++i)
        {
            total_initial_rays++;
            if (accumulated_ranges[i].empty())
            {
                current_filtered_scan.ranges.push_back(nan_val);
                current_filtered_scan.intensities.push_back(nan_val);
                continue;
            }

            double median_r = MathUtils::getMedian(accumulated_ranges[i]);
            double median_i = 0.0;
            if (accumulated_intensities[i].size() > 0)
                median_i = MathUtils::getMedian(accumulated_intensities[i]);

            median_intensities.push_back(median_i);

            double angle = meta_scan.angle_min + i * meta_scan.angle_increment;
            current_filtered_scan.ranges.push_back((float)median_r);
            current_filtered_scan.intensities.push_back((float)median_i);

            if (!std::isnan(current_filtered_scan.ranges.back()))
            {
                initial_points.emplace_back((float)(median_r * cos(angle)), (float)(median_r * sin(angle)));
            }
        }
        // СОХРАНЕНИЕ 1: Сохраняем отфильтрованный скан (LaserScan)
        filtered_scan_results_ = current_filtered_scan; // Сохраняем LaserScan (нужен для заголовка)

        // 2. ЛОГИРОВАНИЕ СЫРОГО СКАНА (v4.7)
        // logRawScan(); // Логирование сырых данных

        // 3. ЛОГИРОВАНИЕ ИТОГОВОГО ОТФИЛЬТРОВАННОГО СКАНА
        // logFinalFilteredScan(filtered_scan_results_); // Логирование медианного скана

        logi.log("\n--- FILTERING STATISTICS ---\n");                      // Логирование статистики
        logi.log("1. Total rays in scan (max): %d\n", total_initial_rays); // Общее число лучей
        logi.log_r("2. Total rays removed by ZERO Intensity (I=0.0) over %d scans: %lld\n",
                   SCANS_TO_COLLECT, total_rays_removed_by_zero_intensity); // Удалено I=0
        logi.log_b("3. Total rays removed by LOW Intensity (0.0 < I < %.2f) over %d scans: %lld\n",
                   intensity_min_threshold, SCANS_TO_COLLECT, total_rays_removed_by_low_intensity);                         // Удалено I<I_min
        logi.log_b("4. Total accumulated intensity removed (Sum of 2+3): %lld\n", total_rays_removed_by_initial_intensity); // Общая сумма
        logi.log("5. Points remaining before Angle Filter: %lu\n", initial_points.size());                                  // Точки до углового фильтра

        if (initial_points.empty())
        {
            logi.log_r("No valid points after median filtering. Exiting pipeline.\n"); // Выход, если нет точек
            return;
        }

        // 4. УДАЛЕНИЕ ФАНТОМНЫХ АРТЕФАКТОВ
        AlignedVector2f clean_points = removeEdgeArtifacts(initial_points, median_intensities, points_removed_by_angle_filter); // Применение углового фильтра

        // СОХРАНЕНИЕ 2 (v5.6): Сохраняем чистые точки для постоянной публикации
        clean_points_results_ = clean_points; // <--- НОВОЕ: Сохранение чистых точек

        logi.log_b("4. Removed by Angle Filter (<%.1f deg): %d\n", edge_angle_threshold * 180.0 / M_PI, points_removed_by_angle_filter); // Лог удаленных
        logi.log_b("5. Final clean points for clustering: %lu\n", clean_points.size());                                                  // Лог финального числа
        logi.log("----------------------------\n");

        if (clean_points.empty())
        {
            logi.log_r("No points left after artifact filtering. Exiting pipeline.\n"); // Выход, если нет точек
            return;
        }

        // 5. Детекция (3 метода)
        std::vector<PillarCandidate> all_candidates; // Все кандидаты
        // ... (Код детекции и fusion)
        AlignedVector2f clusters_m1, clusters_m2, clusters_m3; // Точки кластеров для RVIZ

        auto c1 = detectGenericClustering(clean_points, jump_dist_threshold, 1, clusters_m1); // Метод 1
        all_candidates.insert(all_candidates.end(), c1.begin(), c1.end());

        auto c2 = detectGenericClustering(clean_points, cluster_dist_threshold, 2, clusters_m2); // Метод 2
        all_candidates.insert(all_candidates.end(), c2.begin(), c2.end());

        auto c3 = detectLocalMinima(clean_points, 3, clusters_m3); // Метод 3
        all_candidates.insert(all_candidates.end(), c3.begin(), c3.end());

        // СОЗДАНИЕ И СОХРАНЕНИЕ ОТДЕЛЬНЫХ МАРКЕРОВ (ИЗМЕНЕНО)
        // Метод 1: Красный
        marker_m1_results_ = createPointsMarker(clusters_m1, meta_scan.header.frame_id,
                                                "method_1_jump", 1, 1.0f, 0.0f, 0.0f, 0.08f);

        // Метод 2: Синий
        marker_m2_results_ = createPointsMarker(clusters_m2, meta_scan.header.frame_id,
                                                "method_2_cluster", 2, 0.0f, 0.0f, 1.0f, 0.08f);

        // Метод 3: Желтый
        marker_m3_results_ = createPointsMarker(clusters_m3, meta_scan.header.frame_id,
                                                "method_3_minima", 3, 1.0f, 1.0f, 0.0f, 0.08f);

        logi.log("Total candidates found: %lu (M1=%lu, M2=%lu, M3=%lu)\n",
                 all_candidates.size(), c1.size(), c2.size(), c3.size());

        // 6. Fusion
        AlignedPillarVector final_pillars = fuseCandidates(all_candidates); // Слияние кандидатов

        if (final_pillars.size() == 4)
        {
            reorderPillars(final_pillars); // Геометрическая сортировка
        }

        AlignedVector2f current_fused_centers;
        for (const auto &fp : final_pillars)
        {
            current_fused_centers.push_back(fp.local); // Сохранение центров
        }
        fused_centers_results_ = current_fused_centers; // Сохранение центров Fusion

        // 7. Калибровка (Full Umeyama)
        performCalibration(final_pillars); // Выполнение калибровки

        // 8. Сохранение и лог
        if (calibration_done_)
        {
            saveResults(final_pillars_results_); // Сохранение результатов
            logi.log_g("Calibration successful. Node remains active, publishing results every 1 second.\n");
        }
        else
        {
            logi.log_w("Initial calibration attempt ended without success. Node remains active, check logs for details.\n");
        }
    }
};
/*
// --------------------------------------------------------------------------------------
// 5. MAIN (Без изменений)
// --------------------------------------------------------------------------------------
int main(int argc, char** argv)
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
*/