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
    int ref_index = -1;        // ← ДОБАВИТЬ ЭТУ СТРОКУ для хранения индекса эталона
        // НОВЫЕ ПОЛЯ:
    bool is_estimated = false;           // true если столб оценен (не измерен)
    double estimation_confidence = 1.0;  // Уверенность (1.0 = измерен, <1.0 = оценен)
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
    const int SCANS_TO_COLLECT = 10;
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

    // Структура для хранения калибровки лидара
    struct LidarCalibration
    {
        double scale_factor = 1.0;
        double rotation_deg = 0.0;
        Eigen::Vector2d position = Eigen::Vector2d(0, 0);
        Eigen::Matrix2d rotation_matrix = Eigen::Matrix2d::Identity();

        void clear()
        {
            scale_factor = 1.0;
            rotation_deg = 0.0;
            position = Eigen::Vector2d(0, 0);
            rotation_matrix = Eigen::Matrix2d::Identity();
        }
    };

    LidarCalibration lidar_calibration_; //  для хранения калибровки

    private:
    // Структура для хранения гипотезы калибровки (для 3 точек)
struct CalibrationHypothesis {
    int missing_pillar_idx;          // Какой столб отсутствует: 0=RB, 1=RT, 2=LT, 3=LB
    std::vector<int> visible_indices; // Индексы видимых столбов (3 индекса)
    std::vector<int> visible_indices_permutation; // Лучшая перестановка измеренных точек
    double transformation_error;     // Ошибка трансформации 3 точек
    double prediction_error;         // Ошибка предсказания 4-го столба
    double geometric_error;          // Геометрическая ошибка
    double total_score;              // Общий score (меньше = лучше)
    Eigen::Vector2d estimated_missing; // Оценка положения отсутствующего столба
    LidarCalibration calib_params;   // Параметры калибровки
    
    CalibrationHypothesis() : missing_pillar_idx(-1), 
                              transformation_error(0.0),
                              prediction_error(0.0),
                              geometric_error(0.0),
                              total_score(1e9) {}
};
    
    // Имена столбов для удобства
    const std::vector<std::string> PILLAR_NAMES = {"RB", "RT", "LT", "LB"};


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

 /*
 * Ротационно-инвариантная сортировка (поддерживает 3 и 4 столба)
 * Для 3 точек: только проверяет геометрическую согласованность
 * Для 4 точек: полная сортировка и переименование
 */
void reorderPillars(AlignedPillarVector &pillars)
{
    logi.log("\n=== reorderPillars (Flexible) ===\n");
    
        // Общие структуры данных (используются для всех случаев)
        std::vector<std::string> names = {"RB", "RT", "LT", "LB"};
        std::vector<std::pair<int, int>> ref_pairs = {
            {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}
        };
    
    if (pillars.size() == 4) {
        // --- ОРИГИНАЛЬНАЯ ЛОГИКА ДЛЯ 4 ТОЧЕК ---
        logi.log("Processing 4 pillars (full geometric matching)...\n");
        
        const double ACCEPTABLE_RMSE = 0.05;
        
        std::vector<int> current_match = {0, 1, 2, 3};
        std::vector<int> best_match(4);
        double min_cost = std::numeric_limits<double>::max();
        bool perfect_match_found = false;
        
        do {
            double current_cost = 0.0;
            for (int k = 0; k < 6; ++k) {
                int i_ref = ref_pairs[k].first;
                int j_ref = ref_pairs[k].second;
                int i_pillar_idx = current_match[i_ref];
                int j_pillar_idx = current_match[j_ref];
                
                double measured_dist = MathUtils::dist2D(pillars[i_pillar_idx].local,
                                                        pillars[j_pillar_idx].local);
                double expected_dist = d_center[k];
                double error = measured_dist - expected_dist;
                current_cost += error * error;
            }
            
            if (current_cost < 1e-6 && !perfect_match_found) {
                logi.log_g("    Perfect match found! Cost=%.2e\n", current_cost);
                perfect_match_found = true;
                best_match = current_match;
                min_cost = current_cost;
                break;
            }
            
            if (current_cost < min_cost) {
                min_cost = current_cost;
                best_match = current_match;
            }
            
        } while (std::next_permutation(current_match.begin(), current_match.end()));
        
        double rmse = std::sqrt(min_cost / 6.0);
        
        if (rmse > ACCEPTABLE_RMSE) {
            logi.log_r("WARNING: High matching error (RMSE=%.3f m). Identification may be unreliable.\n", rmse);
        } else {
            logi.log_g("Geometric matching successful. RMSE=%.3f m\n", rmse);
        }
        
        // Детальное логирование
        logi.log("Best match configuration analysis:\n");
        for (int k = 0; k < 6; ++k) {
            int i_ref = ref_pairs[k].first;
            int j_ref = ref_pairs[k].second;
            int i_actual = best_match[i_ref];
            int j_actual = best_match[j_ref];
            
            double measured = MathUtils::dist2D(pillars[i_actual].local,
                                               pillars[j_actual].local);
            double expected = d_center[k];
            double error = std::abs(measured - expected);
            
            if (error <= 0.001)
                logi.log_g("  %s-%s: %.3f (exp: %.3f) ✓ Δ=%.1f mm\n",
                          names[i_ref].c_str(), names[j_ref].c_str(), 
                          measured, expected, error * 1000);
            else if (error <= 0.010)
                logi.log_w("  %s-%s: %.3f (exp: %.3f) ~ Δ=%.1f mm\n",
                          names[i_ref].c_str(), names[j_ref].c_str(),
                          measured, expected, error * 1000);
            else
                logi.log_r("  %s-%s: %.3f (exp: %.3f) ✗ Δ=%.1f mm\n",
                          names[i_ref].c_str(), names[j_ref].c_str(),
                          measured, expected, error * 1000);
        }
        
        // Пересортировка и переименование
        AlignedPillarVector sorted_pillars(4);
        std::string mapping_log = "Pillar mapping result: ";
        
        for (int i = 0; i < 4; ++i) {
            int original_index = best_match[i];
            sorted_pillars[i] = pillars[original_index];
            sorted_pillars[i].name = names[i];
            sorted_pillars[i].ref_index = i;
            mapping_log += names[i] + "←P" + std::to_string(original_index) + " ";
        }
        
        pillars = sorted_pillars;
        logi.log("%s\n", mapping_log.c_str());
        logi.log_g("Pillars successfully reordered and renamed.\n");
        
        // Дополнительный лог
        logi.log("Final pillar coordinates:\n");
        for (const auto &pillar : pillars) {
            double angle = atan2(pillar.local.y(), pillar.local.x()) * 180.0 / M_PI;
            logi.log("  %s: [%.3f, %.3f] (angle: %.1f°)\n",
                     pillar.name.c_str(), pillar.local.x(), pillar.local.y(), angle);
        }
    }
    else if (pillars.size() == 3) {
        // --- НОВАЯ ЛОГИКА ДЛЯ 3 ТОЧЕК ---
        logi.log("Processing 3 pillars (partial geometric validation)...\n");
        
        // Для 3 точек мы не можем определить какие именно это столбы,
        // но можем проверить геометрическую согласованность
        
        // 1. Проверить все возможные комбинации (3 из 4 столбов)
        std::vector<std::vector<int>> possible_combinations = {
            {0, 1, 2}, // RB, RT, LT (нет LB)
            {0, 1, 3}, // RB, RT, LB (нет LT)
            {0, 2, 3}, // RB, LT, LB (нет RT)
            {1, 2, 3}  // RT, LT, LB (нет RB)
        };
        
        std::vector<std::string> comb_names = {
            "RB-RT-LT (no LB)",
            "RB-RT-LB (no LT)", 
            "RB-LT-LB (no RT)",
            "RT-LT-LB (no RB)"
        };
        
        std::vector<double> combination_errors(4, 0.0);
        std::vector<std::vector<std::pair<int, int>>> combination_pairs = {
            {{0,1}, {0,2}, {1,2}}, // Пары для RB-RT-LT
            {{0,1}, {0,3}, {1,3}}, // Пары для RB-RT-LB
            {{0,2}, {0,3}, {2,3}}, // Пары для RB-LT-LB
            {{1,2}, {1,3}, {2,3}}  // Пары для RT-LT-LB
        };
        
        // 2. Для каждой возможной комбинации проверяем геометрию
        logi.log("Testing possible pillar combinations:\n");
        
        for (int comb_idx = 0; comb_idx < 4; ++comb_idx) {
            const auto& comb = possible_combinations[comb_idx];
            const auto& pairs = combination_pairs[comb_idx];
            
            // Перебираем все 6 перестановок 3 измеренных точек
            std::vector<int> perm = {0, 1, 2};
            double best_comb_error = 1e9;
            
            do {
                double current_error = 0.0;
                
                // Проверяем 3 расстояния для этой перестановки
                for (int p = 0; p < 3; ++p) {
                    int i_ref = pairs[p].first;
                    int j_ref = pairs[p].second;
                    
                    // Находим индексы в reference_centers_
                    int ref_i_idx = comb[i_ref];
                    int ref_j_idx = comb[j_ref];
                    
                    // Соответствующие измеренные точки
                    int meas_i_idx = perm[i_ref];
                    int meas_j_idx = perm[j_ref];
                    
                    double measured_dist = MathUtils::dist2D(pillars[meas_i_idx].local,
                                                            pillars[meas_j_idx].local);
                    double expected_dist = 0.0;
                    
                    // Находим какое это расстояние между эталонными столбами
                    for (int k = 0; k < 6; ++k) {
                        if ((ref_pairs[k].first == ref_i_idx && ref_pairs[k].second == ref_j_idx) ||
                            (ref_pairs[k].first == ref_j_idx && ref_pairs[k].second == ref_i_idx)) {
                            expected_dist = d_center[k];
                            break;
                        }
                    }
                    
                    double error = measured_dist - expected_dist;
                    current_error += error * error;
                }
                
                if (current_error < best_comb_error) {
                    best_comb_error = current_error;
                }
                
            } while (std::next_permutation(perm.begin(), perm.end()));
            
            combination_errors[comb_idx] = std::sqrt(best_comb_error / 3.0);
            
            // Логирование
            if (combination_errors[comb_idx] <= 0.05) { // 5 см
                logi.log_g("  %s: RMSE = %.1f mm ✓\n", 
                          comb_names[comb_idx].c_str(), 
                          combination_errors[comb_idx] * 1000);
            } else {
                logi.log_r("  %s: RMSE = %.1f mm ✗\n",
                          comb_names[comb_idx].c_str(),
                          combination_errors[comb_idx] * 1000);
            }
        }
        
        // 3. Найти лучшую комбинацию
        int best_comb_idx = std::min_element(combination_errors.begin(), 
                                           combination_errors.end()) - 
                           combination_errors.begin();
        
        double best_error = combination_errors[best_comb_idx];
        
        if (best_error <= 0.05) { // 5 см порог
            logi.log_g("\n✅ Best combination: %s (RMSE = %.1f mm)\n",
                      comb_names[best_comb_idx].c_str(), best_error * 1000);
            
            // Сохраняем информацию о том, какие столбы вероятно видны
            // Эта информация будет использована в performCalibrationThreePillars
            const auto& best_comb = possible_combinations[best_comb_idx];
            
            logi.log("Probable visible pillars: ");
            for (int idx : best_comb) {
                logi.log("%s ", PILLAR_NAMES[idx].c_str());
            }
            logi.log("\n");
            
            // Временно присваиваем имена (могут быть изменены позже)
            for (size_t i = 0; i < pillars.size(); ++i) {
                pillars[i].name = "Pillar_" + std::to_string(i);
                pillars[i].ref_index = -1; // Неизвестно точно
            }
            
            logi.log_w("⚠️  Note: Pillar identification is tentative for 3 points.\n");
            logi.log_w("    Final identification will be done during calibration.\n");
        } else {
            logi.log_r("\n❌ No geometrically consistent combination found (min RMSE = %.1f mm)\n",
                      best_error * 1000);
            logi.log_r("    Pillar identification may be unreliable.\n");
            
            // Все равно присваиваем временные имена
            for (size_t i = 0; i < pillars.size(); ++i) {
                pillars[i].name = "Pillar_" + std::to_string(i);
                pillars[i].ref_index = -1;
            }
        }
    }
    else if (pillars.size() == 2) {
        // --- МИНИМАЛЬНАЯ ОБРАБОТКА ДЛЯ 2 ТОЧЕК ---
        logi.log_w("Only 2 pillars detected - minimal processing.\n");
        
        // Просто присваиваем временные имена
        for (size_t i = 0; i < pillars.size(); ++i) {
            pillars[i].name = "Pillar_" + std::to_string(i);
            pillars[i].ref_index = -1;
        }
        
        // Проверяем расстояние между ними
        double dist = MathUtils::dist2D(pillars[0].local, pillars[1].local);
        logi.log("Distance between pillars: %.3f m\n", dist);
        
        // Сравниваем с возможными эталонными расстояниями
        bool matches_any = false;
        for (int k = 0; k < 6; ++k) {
            if (std::abs(dist - d_center[k]) < 0.1) { // 10 см порог
                matches_any = true;
                break;
            }
        }
        
        if (matches_any) {
            logi.log_g("  Distance is plausible for pillar pair ✓\n");
        } else {
            logi.log_r("  Distance does not match any expected pillar pair ✗\n");
        }
    }
    else {
        // --- ОБРАБОТКА ДРУГИХ СЛУЧАЕВ ---
        logi.log_r("Unexpected number of pillars: %lu\n", pillars.size());
        
        for (size_t i = 0; i < pillars.size(); ++i) {
            pillars[i].name = "Pillar_" + std::to_string(i);
            pillars[i].ref_index = -1;
        }
    }
    
    logi.log("=== reorderPillars complete ===\n");
}
   
/* * Преобразование координат из системы лидара в мировую систему с применением калибровочных параметров */
Eigen::Vector2d lidarToWorld(const Eigen::Vector2d& lidar_point)
{
    if (!calibration_done_) {
        logi.log_r("Калибровка не выполнена, преобразование невозможно\n");
        return lidar_point;
    }
    
    return lidar_calibration_.scale_factor * 
           lidar_calibration_.rotation_matrix * 
           lidar_point + 
           lidar_calibration_.position;
}

/* * Преобразование координат из мировой системы в систему лидара * (обратное преобразование) */
Eigen::Vector2d worldToLidar(const Eigen::Vector2d& world_point)
{
    if (!calibration_done_) {
        logi.log_r("Калибровка не выполнена, преобразование невозможно\n");
        return world_point;
    }
    
    // Обратное преобразование: P = (1/c) * Rᵀ * (Q - T)
    Eigen::Matrix2d R_inv = lidar_calibration_.rotation_matrix.transpose();
    double c_inv = 1.0 / lidar_calibration_.scale_factor;
    
    return c_inv * R_inv * (world_point - lidar_calibration_.position);
}

void saveResults(const AlignedPillarVector &pillars)
{
    logi.log("--- 💾 СОХРАНЕНИЕ РЕЗУЛЬТАТОВ:\n");
    
    // 1. Сохраняем идеальные координаты столбов (для обратной совместимости)
    for (const auto &p : pillars)
    {
        if (p.name.find("Pillar_") != std::string::npos)
            continue;
        
        std::string base = "/pb/scan/result/" + p.name;
        nh.setParam(base + "/x", p.global.x());
        nh.setParam(base + "/y", p.global.y());
        
        logi.log("  %s: [%.6f, %.6f]\n", 
                 p.name.c_str(), p.global.x(), p.global.y());
    }
    logi.log_g("✅ РЕЗУЛЬТАТЫ СОХРАНЕНЫ\n");
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

/*
 * Сохранение параметров калибровки лидара в ROS Parameter Server
 */
void saveCalibrationParameters()
{
    if (!calibration_done_) {
        logi.log_r("Cannot save calibration: calibration not completed\n");
        return;
    }
    
    logi.log("--- 💾 SAVING CALIBRATION PARAMETERS:\n");
    
    std::string base_path = "/lidar_calibration";
    
    // 1. Позиция лидара
    nh.setParam(base_path + "/position_x", lidar_calibration_.position.x());
    nh.setParam(base_path + "/position_y", lidar_calibration_.position.y());
    
    logi.log("  LiDAR position saved:\n");
    logi.log("    position_x = %.6f m\n", lidar_calibration_.position.x());
    logi.log("    position_y = %.6f m\n", lidar_calibration_.position.y());
    
    // 2. Угол поворота
    nh.setParam(base_path + "/rotation_deg", lidar_calibration_.rotation_deg);
    nh.setParam(base_path + "/rotation_rad", lidar_calibration_.rotation_deg * M_PI / 180.0);
    
    logi.log("  Rotation angle saved:\n");
    logi.log("    rotation_deg = %.6f°\n", lidar_calibration_.rotation_deg);
    logi.log("    rotation_rad = %.6f rad\n", lidar_calibration_.rotation_deg * M_PI / 180.0);
    
    // 3. Масштабный коэффициент
    nh.setParam(base_path + "/scale_factor", lidar_calibration_.scale_factor);
    
    logi.log("  Scale factor saved:\n");
    logi.log("    scale_factor = %.6f\n", lidar_calibration_.scale_factor);
    logi.log("    Scale error: %+.2f%%\n", 
             (lidar_calibration_.scale_factor - 1.0) * 100.0);
    
    
    // 4. Матрица поворота (поэлементно)
    nh.setParam(base_path + "/rotation_matrix/r00", lidar_calibration_.rotation_matrix(0,0));
    nh.setParam(base_path + "/rotation_matrix/r01", lidar_calibration_.rotation_matrix(0,1));
    nh.setParam(base_path + "/rotation_matrix/r10", lidar_calibration_.rotation_matrix(1,0));
    nh.setParam(base_path + "/rotation_matrix/r11", lidar_calibration_.rotation_matrix(1,1));
    
    logi.log("  Rotation matrix saved:\n");
    logi.log("    [%10.6f %10.6f]\n", 
             lidar_calibration_.rotation_matrix(0,0),
             lidar_calibration_.rotation_matrix(0,1));
    logi.log("    [%10.6f %10.6f]\n", 
             lidar_calibration_.rotation_matrix(1,0),
             lidar_calibration_.rotation_matrix(1,1));
    
    // 5. Временная метка - FIXED: нужно привести к int
    ros::Time now = ros::Time::now();
    nh.setParam(base_path + "/timestamp_sec", static_cast<int>(now.sec));
    nh.setParam(base_path + "/timestamp_nsec", static_cast<int>(now.nsec));
    nh.setParam(base_path + "/calibration_done", true);
    
    logi.log("  Metadata:\n");
    logi.log("    timestamp: %d.%09d\n", now.sec, now.nsec);
    logi.log_g("    ✅ Calibration parameters saved to ROS Parameter Server\n");
    logi.log("   Use command: rosparam get /lidar_calibration\n");
}

/*
 * Калибровка по 4 точкам с подробным логированием
 */
bool performCalibrationFourPillars(AlignedPillarVector &pillars)
{
    logi.log("=== 4-PILLAR CALIBRATION ===\n");
    
    // Проверка, что pillars уже отсортированы
    if (pillars.size() != 4) {
        logi.log_r("Internal error: expected 4 pillars, got %lu\n", pillars.size());
        return false;
    }
    
    logi.log("📊 INPUT DATA:\n");
    logi.log("  Reference coordinates (from YAML):\n");
    for (int i = 0; i < 4; ++i) {
        logi.log("    %s: [%.4f, %.4f]\n", 
                 pillars[i].name.c_str(),
                 reference_centers_[i].x(), reference_centers_[i].y());
    }
    
    logi.log("  Measured coordinates (by LiDAR):\n");
    for (int i = 0; i < 4; ++i) {
        double angle = atan2(pillars[i].local.y(), pillars[i].local.x()) * 180.0 / M_PI;
        logi.log("    %s: [%.4f, %.4f] (angle: %.1f°, distance: %.3f m)\n", 
                 pillars[i].name.c_str(),
                 pillars[i].local.x(), pillars[i].local.y(),
                 angle, pillars[i].local.norm());
    }
    
    // 1. ПОДГОТОВКА МАТРИЦ ДАННЫХ
    Eigen::Matrix2Xd P(2, 4), Q(2, 4);
    for (int i = 0; i < 4; ++i) {
        P.col(i) = pillars[i].local.cast<double>();
        Q.col(i) = reference_centers_[i].cast<double>();
    }
    
    logi.log("\n📐 UMEYAMA EQUATION SOLUTION:\n");
    logi.log("  Finding transformation: Q = c * R * P + T\n");
    logi.log("  where: Q - reference (world), P - measurements (LiDAR)\n");
    
    // 2. ВЫЧИСЛЕНИЕ ЦЕНТРОИДОВ
    Eigen::Vector2d mu_P = P.rowwise().mean();
    Eigen::Vector2d mu_Q = Q.rowwise().mean();
    
    logi.log("  Measurements centroid P̄: [%.4f, %.4f]\n", mu_P.x(), mu_P.y());
    logi.log("  Reference centroid Q̄:   [%.4f, %.4f]\n", mu_Q.x(), mu_Q.y());
    
    // 3. ЦЕНТРИРОВАНИЕ
    Eigen::Matrix2Xd P_centered = P.colwise() - mu_P;
    Eigen::Matrix2Xd Q_centered = Q.colwise() - mu_Q;
    
    // 4. КОВАРИАЦИОННАЯ МАТРИЦА
    Eigen::Matrix2d H = P_centered * Q_centered.transpose();
    logi.log("  Covariance matrix H:\n");
    logi.log("    [%10.6f %10.6f]\n", H(0,0), H(0,1));
    logi.log("    [%10.6f %10.6f]\n", H(1,0), H(1,1));
    
    // 5. SVD РАЗЛОЖЕНИЕ
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();
    
    logi.log("  Singular values: %.6f, %.6f\n", 
             svd.singularValues()(0), svd.singularValues()(1));
    
    // 6. МАТРИЦА ВРАЩЕНИЯ R
    Eigen::Matrix2d D = Eigen::Matrix2d::Identity();
    if (U.determinant() * V.determinant() < 0.0) {
        D(1, 1) = -1.0;
        logi.log_w("  ⚠️  Different sign determinants → adding reflection\n");
    }
    
    Eigen::Matrix2d R = V * D * U.transpose();
    
    // Проверка, что R - матрица вращения
    double det_R = R.determinant();
    if (std::abs(det_R - 1.0) > 1e-6) {
        logi.log_r("  ❗ Matrix R is not pure rotation matrix: det=%.6f\n", det_R);
    }
    
    // 7. ВЫЧИСЛЕНИЕ МАСШТАБА c
    double denominator = P_centered.squaredNorm();
    if (denominator < 1e-12) {
        logi.log_r("  ❗ Zero denominator in scale calculation\n");
        return false;
    }
    
    double c = svd.singularValues().sum() / denominator;
    
    // 8. ВЕКТОР СДВИГА T (позиция лидара)
    Eigen::Vector2d T = mu_Q - c * R * mu_P;
    
    logi.log("\n🎯 FOUND LiDAR PARAMETERS:\n");
    
    // 8.1. Масштабный коэффициент
    lidar_calibration_.scale_factor = c;
    double scale_error_percent = (c - 1.0) * 100.0;
    logi.log("  🔍 Scale factor (c): %.6f\n", c);
    logi.log("     Scale error: %+.2f%%\n", scale_error_percent);
    
    if (std::abs(scale_error_percent) > 5.0) {
        logi.log_r("     ⚠️  Warning: large scale error (>5%%)\n");
    } else if (std::abs(scale_error_percent) > 2.0) {
        logi.log_w("     ⚠️  Noticeable scale error (>2%%)\n");
    } else {
        logi.log_g("     ✓ Scale within normal range\n");
    }
    
    // 8.2. Угол поворота
    double rotation_rad = atan2(R(1, 0), R(0, 0));
    double rotation_deg = rotation_rad * 180.0 / M_PI;
    lidar_calibration_.rotation_deg = rotation_deg;
    lidar_calibration_.rotation_matrix = R;
    
    logi.log("  🧭 LiDAR rotation angle: %.2f°\n", rotation_deg);
    logi.log("     Rotation matrix R:\n");
    logi.log("     [%8.5f %8.5f]\n", R(0,0), R(0,1));
    logi.log("     [%8.5f %8.5f]\n", R(1,0), R(1,1));
    
    // 8.3. Позиция лидара в мире
    lidar_calibration_.position = T;
    logi.log("  📍 LiDAR position in world system (T):\n");
    logi.log("     X = %.4f m\n", T.x());
    logi.log("     Y = %.4f m\n", T.y());
    logi.log("     Distance from origin: %.3f m\n", T.norm());
    
    // 9. ВАЛИДАЦИЯ: ПРИМЕНЕНИЕ ПРЕОБРАЗОВАНИЯ
    logi.log("\n🔬 CALIBRATION VALIDATION:\n");
    logi.log("  Applying transformation to measured points:\n");
    logi.log("  Pillar | Measured (P) | Transformed | Reference (Q) | Error (mm)\n");
    logi.log("  -------------------------------------------------------------\n");
    
    double max_error = 0.0;
    double total_error_sq = 0.0;
    
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector2d p = P.col(i);
        Eigen::Vector2d q_expected = Q.col(i);
        
        // Преобразование: Q' = c*R*P + T
        Eigen::Vector2d q_calculated = c * R * p + T;
        
        double error = (q_calculated - q_expected).norm();
        total_error_sq += error * error;
        if (error > max_error) max_error = error;
        
        // Цветное логирование в зависимости от ошибки
        if (error <= 0.001) { // ≤1 мм
            logi.log_g("  %s  [%6.3f,%6.3f] → [%6.3f,%6.3f] ≈ [%6.3f,%6.3f]  %.1f✓\n",
                      pillars[i].name.c_str(),
                      p.x(), p.y(),
                      q_calculated.x(), q_calculated.y(),
                      q_expected.x(), q_expected.y(),
                      error * 1000);
        } else if (error <= 0.010) { // ≤10 мм
            logi.log_w("  %s  [%6.3f,%6.3f] → [%6.3f,%6.3f] ≈ [%6.3f,%6.3f]  %.1f~\n",
                      pillars[i].name.c_str(),
                      p.x(), p.y(),
                      q_calculated.x(), q_calculated.y(),
                      q_expected.x(), q_expected.y(),
                      error * 1000);
        } else {
            logi.log_r("  %s  [%6.3f,%6.3f] → [%6.3f,%6.3f] ≠ [%6.3f,%6.3f]  %.1f✗\n",
                      pillars[i].name.c_str(),
                      p.x(), p.y(),
                      q_calculated.x(), q_calculated.y(),
                      q_expected.x(), q_expected.y(),
                      error * 1000);
        }
    }
    
    // 10. ВЫЧИСЛЕНИЕ RMSE
    double rmse = std::sqrt(total_error_sq / 4.0);
    logi.log("\n📈 ACCURACY STATISTICS:\n");
    logi.log("  Maximum error: %.1f mm\n", max_error * 1000);
    logi.log("  Root Mean Square Error (RMSE): %.1f mm\n", rmse * 1000);
    logi.log("  Sum of squared errors: %.6f m²\n", total_error_sq);
    
    // 11. ПРИНЯТИЕ РЕШЕНИЯ О КАЛИБРОВКЕ
    const double RMSE_THRESHOLD = 0.05; // 5 см порог
    
    if (rmse <= RMSE_THRESHOLD) {
        calibration_done_ = true;
        
        // Столбы получают идеальные координаты (а не преобразованные)
        for (int i = 0; i < 4; ++i) {
            pillars[i].global = reference_centers_[i].cast<double>();
        }
        
        // Сохранение финальных результатов
        final_pillars_results_ = pillars;
        
        logi.log_g("✅ 4-PILLAR CALIBRATION SUCCESSFUL! RMSE = %.1f mm < %.1f mm\n", 
                   rmse * 1000, RMSE_THRESHOLD * 1000);
        
        // Сохраняем параметры калибровки
        saveCalibrationParameters();
        
        // Визуализация
        publishFinalMarkers(final_pillars_results_);
        
        // 12. ПРИМЕР ИСПОЛЬЗОВАНИЯ
        logi.log_g("\n💡 CALIBRATION USAGE EXAMPLE:\n");
        logi.log("  For correcting LiDAR measurements:\n");
        logi.log("    Q = %.4f * R * P + [%.4f, %.4f]\n", 
                 lidar_calibration_.scale_factor,
                 lidar_calibration_.position.x(),
                 lidar_calibration_.position.y());
        
        // Пример: объект на расстоянии 5м прямо перед лидаром
        Eigen::Vector2d example_lidar_point(5.0, 0.0);
        Eigen::Vector2d example_world_point = 
            lidar_calibration_.scale_factor * lidar_calibration_.rotation_matrix * 
            example_lidar_point + lidar_calibration_.position;
        
        logi.log("  Example: object at LiDAR coordinates [5.0, 0.0]\n");
        logi.log("           in world system will be: [%.3f, %.3f]\n",
                 example_world_point.x(), example_world_point.y());
        
        logi.log("═══════════════════════════════════════════════════════════\n");
        
        return true;
    } else {
        logi.log_r("❌ 4-PILLAR CALIBRATION FAILED: RMSE = %.1f mm > threshold = %.1f mm\n",
                   rmse * 1000, RMSE_THRESHOLD * 1000);
        lidar_calibration_.clear();
        return false;
    }
}

/*
 * Калибровка по 3 точкам с подробным логированием (подробный лог как для 4 точек)
 */
bool performCalibrationThreePillars(AlignedPillarVector &pillars)
{
    logi.log("\n═══════════════════════════════════════════════════════════\n");
    logi.log("🔍 3-PILLAR CALIBRATION (Advanced Hypothesis Testing)\n");
    logi.log("═══════════════════════════════════════════════════════════\n");
    
    if (pillars.size() != 3) {
        logi.log_r("❌ Internal error: expected 3 pillars, got %lu\n", pillars.size());
        return false;
    }
    
    // Временная копия pillars для модификации
    AlignedPillarVector original_pillars = pillars;
    const std::vector<std::string> pillar_names = {"RB", "RT", "LT", "LB"};
    
    logi.log("📊 INPUT DATA (3 measured pillars):\n");
    for (int i = 0; i < 3; ++i) {
        double angle = atan2(original_pillars[i].local.y(), 
                           original_pillars[i].local.x()) * 180.0 / M_PI;
        logi.log("  Pillar_%d: [%.4f, %.4f] (angle: %.1f°, distance: %.3f m)\n",
                 i, original_pillars[i].local.x(), original_pillars[i].local.y(),
                 angle, original_pillars[i].local.norm());
    }
    
    logi.log("\n📊 REFERENCE DATA (4 expected pillars from YAML):\n");
    for (int i = 0; i < 4; ++i) {
        logi.log("  %s: [%.4f, %.4f]\n",
                 pillar_names[i].c_str(),
                 reference_centers_[i].x(), reference_centers_[i].y());
    }
    
    logi.log("\n🧮 STARTING HYPOTHESIS TESTING (4 possibilities):\n");
    
    // 1. ГЕНЕРАЦИЯ ВСЕХ ВОЗМОЖНЫХ ГИПОТЕЗ (4 варианта)
    std::vector<CalibrationHypothesis> hypotheses(4);
    
    for (int missing_idx = 0; missing_idx < 4; ++missing_idx) {
        CalibrationHypothesis &hyp = hypotheses[missing_idx];
        hyp.missing_pillar_idx = missing_idx;
        
        // Индексы видимых столбов (все кроме missing)
        for (int i = 0; i < 4; ++i) {
            if (i != missing_idx) hyp.visible_indices.push_back(i);
        }
        
        logi.log("\n--- Testing Hypothesis %d ---\n", missing_idx);
        logi.log("  Missing pillar: %s\n", pillar_names[missing_idx].c_str());
        logi.log("  Visible pillars assumed: ");
        for (int idx : hyp.visible_indices) {
            logi.log("%s ", pillar_names[idx].c_str());
        }
        logi.log("\n");
        
        // 2. ВЫПОЛНИТЬ UMEYAMA ДЛЯ 3 ТОЧЕК
        if (!performUmeyamaForThreePoints(original_pillars, hyp)) {
            logi.log_r("  ❌ Hypothesis failed: Umeyama computation impossible\n");
            hyp.total_score = 1e9;
            continue;
        }
        
        // 3. ДЕТАЛЬНОЕ ЛОГИРОВАНИЕ НАЙДЕННЫХ ПАРАМЕТРОВ
        logi.log("  ✅ Umeyama solution found:\n");
        logi.log("     Scale factor: %.6f (error: %+.2f%%)\n", 
                 hyp.calib_params.scale_factor,
                 (hyp.calib_params.scale_factor - 1.0) * 100.0);
        logi.log("     Rotation angle: %.2f°\n", hyp.calib_params.rotation_deg);
        logi.log("     LiDAR position: [%.4f, %.4f] m\n",
                 hyp.calib_params.position.x(), hyp.calib_params.position.y());
        
        // 4. ВАЛИДАЦИЯ ГИПОТЕЗЫ
        logi.log("  🔍 Validating hypothesis...\n");
        
        double transformation_error = 0.0;
        int valid_points = 0;
        
        // Применяем преобразование к видимым точкам
        for (size_t i = 0; i < 3; ++i) {
            Eigen::Vector2d p = original_pillars[i].local.cast<double>();
            Eigen::Vector2d q_expected = reference_centers_[hyp.visible_indices[i]].cast<double>();
            Eigen::Vector2d q_calculated = hyp.calib_params.scale_factor * 
                                          hyp.calib_params.rotation_matrix * p + 
                                          hyp.calib_params.position;
            
            double error = (q_calculated - q_expected).norm();
            transformation_error += error * error;
            valid_points++;
            
            logi.log("     %s → %s: error = %.1f mm %s\n",
                     ("Pillar_" + std::to_string(i)).c_str(),
                     pillar_names[hyp.visible_indices[i]].c_str(),
                     error * 1000,
                     error <= 0.02 ? "✓" : "✗");
        }
        
        hyp.transformation_error = std::sqrt(transformation_error / valid_points);
        
        // 5. ОЦЕНКА ПОЛОЖЕНИЯ ОТСУТСТВУЮЩЕГО СТОЛБА
        logi.log("  🔮 Estimating missing pillar position...\n");
        
        // Находим, какая из измеренных точек соответствует какой эталонной
        // (упрощенно - предполагаем порядок)
        Eigen::Vector2d estimated_missing;
        double prediction_error = 0.0;
        
        // Для простоты берем среднюю точку измерений как кандидата на отсутствующий столб
        Eigen::Vector2d avg_measured(0, 0);
        for (const auto& p : original_pillars) {
            avg_measured += p.local.cast<double>();
        }
        avg_measured /= 3.0;
        
        // Преобразуем в мировые координаты
        estimated_missing = hyp.calib_params.scale_factor * 
                          hyp.calib_params.rotation_matrix * avg_measured + 
                          hyp.calib_params.position;
        
        // Сравниваем с эталонным положением отсутствующего столба
        Eigen::Vector2d expected_missing = reference_centers_[missing_idx].cast<double>();
        prediction_error = (estimated_missing - expected_missing).norm();
        hyp.prediction_error = prediction_error;
        hyp.estimated_missing = estimated_missing;
        
        logi.log("     Estimated %s position: [%.4f, %.4f]\n",
                 pillar_names[missing_idx].c_str(),
                 estimated_missing.x(), estimated_missing.y());
        logi.log("     Expected %s position:  [%.4f, %.4f]\n",
                 pillar_names[missing_idx].c_str(),
                 expected_missing.x(), expected_missing.y());
        logi.log("     Prediction error: %.1f mm %s\n",
                 prediction_error * 1000,
                 prediction_error <= 0.05 ? "✓" : "✗");
        
        // 6. ГЕОМЕТРИЧЕСКАЯ ПРОВЕРКА
        logi.log("  📐 Geometric consistency check...\n");
        
        double geometric_error = 0.0;
        int pair_count = 0;
        
        // Проверяем расстояния между измеренными точками
        for (int i = 0; i < 2; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                double measured_dist = MathUtils::dist2D(original_pillars[i].local,
                                                        original_pillars[j].local);
                
                // Находим, каким эталонным столбам они могут соответствовать
                // (упрощенно - берем среднее ожидаемое расстояние)
                double expected_dist = 4.0; // Примерное среднее расстояние
                
                double error = std::abs(measured_dist - expected_dist);
                geometric_error += error;
                pair_count++;
                
                logi.log("     Distance P%d-P%d: %.3f m (≈ expected: %.3f m, Δ=%.1f mm)\n",
                         i, j, measured_dist, expected_dist, error * 1000);
            }
        }
        
        if (pair_count > 0) {
            hyp.geometric_error = geometric_error / pair_count;
        }
        
        // 7. ВЫЧИСЛЕНИЕ ОБЩЕГО SCORE
        hyp.total_score = hyp.transformation_error * 1000.0 +  // мм
                         hyp.prediction_error * 2000.0 +       // вес x2
                         hyp.geometric_error * 1000.0 +        // мм
                         std::abs(hyp.calib_params.scale_factor - 1.0) * 10.0;
        
        logi.log("  📊 Hypothesis score breakdown:\n");
        logi.log("     Transformation error: %.1f mm\n", hyp.transformation_error * 1000);
        logi.log("     Prediction error:     %.1f mm\n", hyp.prediction_error * 1000);
        logi.log("     Geometric error:      %.1f mm\n", hyp.geometric_error * 1000);
        logi.log("     Scale penalty:        %.3f\n", std::abs(hyp.calib_params.scale_factor - 1.0) * 10.0);
        logi.log("     TOTAL SCORE:          %.3f\n", hyp.total_score);
    }
    
    // 8. ВЫБОР ЛУЧШЕЙ ГИПОТЕЗЫ
    logi.log("\n🎯 SELECTING BEST HYPOTHESIS:\n");
    
    int best_hyp_idx = -1;
    double best_score = 1e9;
    
    for (int i = 0; i < 4; ++i) {
        if (hypotheses[i].total_score < best_score) {
            best_score = hypotheses[i].total_score;
            best_hyp_idx = i;
        }
    }
    
    if (best_hyp_idx == -1 || best_score > 1000.0) {
        logi.log_r("❌ No valid hypothesis found (all scores > 1000)\n");
        logi.log_r("    Possible reasons:\n");
        logi.log_r("    - Geometry doesn't match any expected pillar configuration\n");
        logi.log_r("    - Measurement errors too large\n");
        logi.log_r("    - Wrong pillar identification\n");
        return false;
    }
    
    CalibrationHypothesis &best_hyp = hypotheses[best_hyp_idx];
    
    logi.log_g("✅ BEST HYPOTHESIS SELECTED:\n");
    logi.log("   Missing pillar: %s\n", pillar_names[best_hyp.missing_pillar_idx].c_str());
    logi.log("   Total score:    %.3f\n", best_score);
    logi.log("   Transformation RMSE: %.1f mm\n", best_hyp.transformation_error * 1000);
    logi.log("   Prediction error:    %.1f mm\n", best_hyp.prediction_error * 1000);
    
    // 9. ПРИМЕНЕНИЕ ЛУЧШЕЙ ГИПОТЕЗЫ
    logi.log("\n🔧 APPLYING BEST HYPOTHESIS...\n");
    
    // Копируем параметры калибровки
    lidar_calibration_ = best_hyp.calib_params;
    
    // Восстанавливаем полный набор из 4 столбов
    pillars.clear();
    pillars.resize(4);
    
    // Для видимых столбов
    for (int i = 0; i < 3; ++i) {
        int pillar_idx = best_hyp.visible_indices[i];
        pillars[pillar_idx] = original_pillars[i];
        pillars[pillar_idx].name = pillar_names[pillar_idx];
        pillars[pillar_idx].global = reference_centers_[pillar_idx].cast<double>();
        pillars[pillar_idx].is_estimated = false;
        pillars[pillar_idx].estimation_confidence = 1.0;
        
        logi.log("  %s: Measured → Reference [%.4f, %.4f]\n",
                 pillar_names[pillar_idx].c_str(),
                 pillars[pillar_idx].global.x(), pillars[pillar_idx].global.y());
    }
    
    // Для отсутствующего столба (оцениваемого)
    int missing_idx = best_hyp.missing_pillar_idx;
    pillars[missing_idx].name = pillar_names[missing_idx];
    pillars[missing_idx].global = reference_centers_[missing_idx].cast<double>();
    pillars[missing_idx].local = best_hyp.estimated_missing.cast<float>();
    pillars[missing_idx].is_estimated = true;
    pillars[missing_idx].estimation_confidence = 0.7; // 70% уверенности
    
    logi.log("  %s: Estimated → Reference [%.4f, %.4f] (confidence: 70%%)\n",
             pillar_names[missing_idx].c_str(),
             pillars[missing_idx].global.x(), pillars[missing_idx].global.y());
    
    // 10. ФИНАЛЬНАЯ ВАЛИДАЦИЯ
    logi.log("\n🔬 FINAL VALIDATION (using 3 measured pillars):\n");
    logi.log("  Pillar | Measured (P) | Transformed | Reference (Q) | Error (mm)\n");
    logi.log("  -------------------------------------------------------------\n");
    
    double max_error = 0.0;
    double total_error_sq = 0.0;
    int valid_points = 0;
    
    for (int i = 0; i < 4; ++i) {
        if (i == missing_idx) continue; // Пропускаем оцененный столб
        
        Eigen::Vector2d p = pillars[i].local.cast<double>();
        Eigen::Vector2d q_expected = reference_centers_[i].cast<double>();
        Eigen::Vector2d q_calculated = lidar_calibration_.scale_factor * 
                                      lidar_calibration_.rotation_matrix * p + 
                                      lidar_calibration_.position;
        
        double error = (q_calculated - q_expected).norm();
        total_error_sq += error * error;
        valid_points++;
        
        if (error > max_error) max_error = error;
        
        if (error <= 0.001) {
            logi.log_g("  %s  [%6.3f,%6.3f] → [%6.3f,%6.3f] ≈ [%6.3f,%6.3f]  %.1f✓\n",
                      pillar_names[i].c_str(),
                      p.x(), p.y(),
                      q_calculated.x(), q_calculated.y(),
                      q_expected.x(), q_expected.y(),
                      error * 1000);
        } else if (error <= 0.010) {
            logi.log_w("  %s  [%6.3f,%6.3f] → [%6.3f,%6.3f] ≈ [%6.3f,%6.3f]  %.1f~\n",
                      pillar_names[i].c_str(),
                      p.x(), p.y(),
                      q_calculated.x(), q_calculated.y(),
                      q_expected.x(), q_expected.y(),
                      error * 1000);
        } else {
            logi.log_r("  %s  [%6.3f,%6.3f] → [%6.3f,%6.3f] ≠ [%6.3f,%6.3f]  %.1f✗\n",
                      pillar_names[i].c_str(),
                      p.x(), p.y(),
                      q_calculated.x(), q_calculated.y(),
                      q_expected.x(), q_expected.y(),
                      error * 1000);
        }
    }
    
    if (valid_points == 0) {
        logi.log_r("❌ No valid points for validation\n");
        return false;
    }
    
    double rmse = std::sqrt(total_error_sq / valid_points);
    
    logi.log("\n📈 ACCURACY STATISTICS:\n");
    logi.log("  Maximum error: %.1f mm\n", max_error * 1000);
    logi.log("  Root Mean Square Error (RMSE): %.1f mm\n", rmse * 1000);
    logi.log("  Valid points used: %d (1 pillar estimated)\n", valid_points);
    
    // 11. ПРИНЯТИЕ РЕШЕНИЯ (более строгий порог для 3 точек)
    const double RMSE_THRESHOLD = 0.03; // 3 см (строже чем для 4 точек)
    
    if (rmse <= RMSE_THRESHOLD) {
        calibration_done_ = true;
        final_pillars_results_ = pillars;
        
        logi.log_g("\n✅ 3-PILLAR CALIBRATION SUCCESSFUL! RMSE = %.1f mm\n", rmse * 1000);
        logi.log_w("⚠️  NOTE: Pillar %s is ESTIMATED (confidence: 70%%)\n",
                   pillar_names[missing_idx].c_str());
        
        // 12. ПРИМЕР ИСПОЛЬЗОВАНИЯ
        logi.log_g("\n💡 CALIBRATION USAGE EXAMPLE:\n");
        logi.log("  For correcting LiDAR measurements:\n");
        logi.log("    Q = %.4f * R * P + [%.4f, %.4f]\n", 
                 lidar_calibration_.scale_factor,
                 lidar_calibration_.position.x(),
                 lidar_calibration_.position.y());
        
        // Пример: объект на расстоянии 5м прямо перед лидаром
        Eigen::Vector2d example_lidar_point(5.0, 0.0);
        Eigen::Vector2d example_world_point = 
            lidar_calibration_.scale_factor * lidar_calibration_.rotation_matrix * 
            example_lidar_point + lidar_calibration_.position;
        
        logi.log("  Example: object at LiDAR coordinates [5.0, 0.0]\n");
        logi.log("           in world system will be: [%.3f, %.3f]\n",
                 example_world_point.x(), example_world_point.y());
        
        logi.log("═══════════════════════════════════════════════════════════\n");
        
        saveCalibrationParameters();
        publishFinalMarkers(final_pillars_results_);
        return true;
    } else {
        logi.log_r("\n❌ 3-PILLAR CALIBRATION FAILED: RMSE = %.1f mm > threshold = %.1f mm\n",
                   rmse * 1000, RMSE_THRESHOLD * 1000);
        logi.log_r("    Threshold is stricter for 3-point calibration (30 mm vs 50 mm)\n");
        lidar_calibration_.clear();
        return false;
    }
}

/*
 * Выполнение Umeyama для 3 точек с детальным логированием
 */
bool performUmeyamaForThreePoints(const AlignedPillarVector &measured_pillars,
                                  CalibrationHypothesis &hyp)
{
    if (measured_pillars.size() != 3) {
        logi.log_r("    ❌ Expected 3 measured pillars, got %lu\n", measured_pillars.size());
        return false;
    }
    
    logi.log("    🧮 Starting Umeyama for 3 points...\n");
    
    // У нас есть 3 измеренных столба и 3 соответствующих эталонных
    // Но мы не знаем, какие именно эталонные соответствуют измеренным
    
    std::vector<int> perm = {0, 1, 2}; // индексы измеренных точек
    const std::vector<std::string> pillar_names = {"RB", "RT", "LT", "LB"};
    
    double best_error = 1e9;
    std::vector<int> best_perm = {-1, -1, -1};
    LidarCalibration best_calib;
    Eigen::Matrix2Xd best_P, best_Q;
    
    int permutation_count = 0;
    
    do {
        permutation_count++;
        logi.log("      Testing permutation %d: [P%d, P%d, P%d] → [%s, %s, %s]\n",
                 permutation_count, 
                 perm[0], perm[1], perm[2],
                 pillar_names[hyp.visible_indices[0]].c_str(),
                 pillar_names[hyp.visible_indices[1]].c_str(),
                 pillar_names[hyp.visible_indices[2]].c_str());
        
        // Подготовка матриц P (измеренные) и Q (эталонные) для этой перестановки
        Eigen::Matrix2Xd P(2, 3), Q(2, 3);
        
        for (int i = 0; i < 3; ++i) {
            // measured_pillars[perm[i]] соответствует reference_centers_[hyp.visible_indices[i]]
            P.col(i) = measured_pillars[perm[i]].local.cast<double>();
            Q.col(i) = reference_centers_[hyp.visible_indices[i]].cast<double>();
        }
        
        // Логирование входных данных для этой перестановки
        logi.log("        Measured points (P):\n");
        for (int i = 0; i < 3; ++i) {
            logi.log("          P%d: [%.4f, %.4f] → %s: [%.4f, %.4f]\n",
                     perm[i],
                     P(0, i), P(1, i),
                     pillar_names[hyp.visible_indices[i]].c_str(),
                     Q(0, i), Q(1, i));
        }
        
        // 1. ВЫЧИСЛЕНИЕ ЦЕНТРОИДОВ
        Eigen::Vector2d mu_P = P.rowwise().mean();
        Eigen::Vector2d mu_Q = Q.rowwise().mean();
        
        logi.log("        Centroids: P̄ = [%.4f, %.4f], Q̄ = [%.4f, %.4f]\n",
                 mu_P.x(), mu_P.y(), mu_Q.x(), mu_Q.y());
        
        // 2. ЦЕНТРИРОВАНИЕ
        Eigen::Matrix2Xd P_centered = P.colwise() - mu_P;
        Eigen::Matrix2Xd Q_centered = Q.colwise() - mu_Q;
        
        // 3. КОВАРИАЦИОННАЯ МАТРИЦА
        Eigen::Matrix2d H = P_centered * Q_centered.transpose();
        
        logi.log("        Covariance matrix H:\n");
        logi.log("          [%10.6f %10.6f]\n", H(0,0), H(0,1));
        logi.log("          [%10.6f %10.6f]\n", H(1,0), H(1,1));
        
        // 4. SVD РАЗЛОЖЕНИЕ
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        
        double sv1 = svd.singularValues()(0);
        double sv2 = svd.singularValues()(1);
        
        logi.log("        Singular values: %.6f, %.6f (ratio: %.3f)\n", 
                 sv1, sv2, sv1 / (sv2 + 1e-12));
        
        // Проверка сингулярных значений
        if (sv2 < 1e-6) {
            logi.log_r("        ⚠️  Second singular value too small (%.2e), skipping\n", sv2);
            continue;
        }
        
        // 5. МАТРИЦА ВРАЩЕНИЯ R
        Eigen::Matrix2d D = Eigen::Matrix2d::Identity();
        double det_U = U.determinant();
        double det_V = V.determinant();
        
        logi.log("        Determinants: det(U)=%.6f, det(V)=%.6f\n", det_U, det_V);
        
        if (det_U * det_V < 0.0) {
            D(1, 1) = -1.0;
            logi.log_w("        ⚠️  Different sign determinants → adding reflection\n");
        }
        
        Eigen::Matrix2d R = V * D * U.transpose();
        
        // Проверка, что R - матрица вращения
        double det_R = R.determinant();
        logi.log("        Rotation matrix determinant: %.6f\n", det_R);
        
        if (std::abs(det_R - 1.0) > 1e-4) {
            logi.log_r("        ❗ Matrix R is not pure rotation (det=%.6f)\n", det_R);
            continue;
        }
        
        // 6. ВЫЧИСЛЕНИЕ МАСШТАБА c
        double denominator = P_centered.squaredNorm();
        logi.log("        Denominator for scale: %.6f\n", denominator);
        
        if (denominator < 1e-12) {
            logi.log_r("        ❗ Zero denominator in scale calculation\n");
            continue;
        }
        
        double c = svd.singularValues().sum() / denominator;
        logi.log("        Scale factor c = %.6f (error: %+.2f%%)\n", 
                 c, (c - 1.0) * 100.0);
        
        // Проверка разумности масштаба
        if (c < 0.7 || c > 1.3) {
            logi.log_r("        ❗ Unrealistic scale factor (c=%.3f)\n", c);
            continue;
        }
        
        // 7. ВЕКТОР СДВИГА T
        Eigen::Vector2d T = mu_Q - c * R * mu_P;
        logi.log("        Translation T = [%.4f, %.4f]\n", T.x(), T.y());
        
        // 8. ВЫЧИСЛЕНИЕ УГЛА ПОВОРОТА
        double rotation_rad = atan2(R(1, 0), R(0, 0));
        double rotation_deg = rotation_rad * 180.0 / M_PI;
        logi.log("        Rotation angle: %.2f°\n", rotation_deg);
        
        // Проверка разумности угла
        if (std::abs(rotation_deg) > 45.0 && std::abs(rotation_deg) < 135.0) {
            logi.log_r("        ❗ Unusual rotation angle (%.1f°)\n", rotation_deg);
            continue;
        }
        
        // 9. ВЫЧИСЛЕНИЕ ОШИБКИ ДЛЯ ЭТОЙ ПЕРЕСТАНОВКИ
        double error = 0.0;
        double max_point_error = 0.0;
        
        logi.log("        Point-wise errors:\n");
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector2d q_calc = c * R * P.col(i) + T;
            Eigen::Vector2d q_ref = Q.col(i);
            
            double point_error = (q_calc - q_ref).norm();
            error += point_error * point_error;
            
            if (point_error > max_point_error) {
                max_point_error = point_error;
            }
            
            logi.log("          %s: error = %.1f mm %s\n",
                     pillar_names[hyp.visible_indices[i]].c_str(),
                     point_error * 1000,
                     point_error <= 0.02 ? "✓" : "✗");
        }
        
        error = std::sqrt(error / 3.0);
        
        logi.log("        Transformation RMSE: %.1f mm, Max error: %.1f mm\n",
                 error * 1000, max_point_error * 1000);
        
        // 10. ПРОВЕРКА ГЕОМЕТРИЧЕСКОЙ СОГЛАСОВАННОСТИ
        // Вычисляем расстояния между преобразованными точками
        std::vector<double> transformed_distances;
        for (int i = 0; i < 2; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                Eigen::Vector2d p1 = c * R * P.col(i) + T;
                Eigen::Vector2d p2 = c * R * P.col(j) + T;
                double dist = (p1 - p2).norm();
                transformed_distances.push_back(dist);
            }
        }
        
        // Сравниваем с эталонными расстояниями
        double geometric_consistency_error = 0.0;
        logi.log("        Geometric consistency:\n");
        
        // Пары эталонных расстояний для видимых столбов
        std::vector<std::pair<int, int>> ref_pairs = {
            {hyp.visible_indices[0], hyp.visible_indices[1]},
            {hyp.visible_indices[0], hyp.visible_indices[2]},
            {hyp.visible_indices[1], hyp.visible_indices[2]}
        };
        
        for (size_t idx = 0; idx < transformed_distances.size(); ++idx) {
            double transformed_dist = transformed_distances[idx];
            
            // Находим соответствующее эталонное расстояние
            double expected_dist = 0.0;
            for (int k = 0; k < 6; ++k) {
                if ((ref_pairs[idx].first == k/4 && ref_pairs[idx].second == k%4) ||
                    (ref_pairs[idx].first == k%4 && ref_pairs[idx].second == k/4)) {
                    expected_dist = d_center[k];
                    break;
                }
            }
            
            double dist_error = std::abs(transformed_dist - expected_dist);
            geometric_consistency_error += dist_error;
            
            logi.log("          %s-%s: %.3f m (exp: %.3f m, Δ=%.1f mm)\n",
                     pillar_names[ref_pairs[idx].first].c_str(),
                     pillar_names[ref_pairs[idx].second].c_str(),
                     transformed_dist, expected_dist, dist_error * 1000);
        }
        
        geometric_consistency_error /= transformed_distances.size();
        logi.log("        Avg geometric error: %.1f mm\n", geometric_consistency_error * 1000);
        
        // 11. ОБЩАЯ ОЦЕНКА ЭТОЙ ПЕРЕСТАНОВКИ
        double total_permutation_score = error * 1000.0 + // мм
                                        geometric_consistency_error * 1000.0 + // мм
                                        std::abs(c - 1.0) * 500.0 + // штраф за масштаб
                                        std::abs(rotation_deg) * 2.0; // штраф за большой угол
        
        logi.log("        Total permutation score: %.3f\n", total_permutation_score);
        
        // 12. ОБНОВЛЕНИЕ ЛУЧШЕГО РЕШЕНИЯ
        if (total_permutation_score < best_error) {
            best_error = total_permutation_score;
            best_perm = perm;
            best_calib.scale_factor = c;
            best_calib.rotation_deg = rotation_deg;
            best_calib.rotation_matrix = R;
            best_calib.position = T;
            best_P = P;
            best_Q = Q;
            
            logi.log_g("        🎯 NEW BEST PERMUTATION FOUND! (score: %.3f)\n", best_error);
        }
        
        logi.log("\n");
        
    } while (std::next_permutation(perm.begin(), perm.end()));
    
    // 13. ПРОВЕРКА РЕЗУЛЬТАТОВ
    if (best_error > 1e8) {
        logi.log_r("    ❌ No valid permutation found for this hypothesis\n");
        logi.log_r("    Possible reasons:\n");
        logi.log_r("    - Geometry doesn't match\n");
        logi.log_r("    - Large measurement errors\n");
        logi.log_r("    - Invalid scale or rotation\n");
        return false;
    }
    
    // 14. ФИНАЛЬНОЕ ЛОГИРОВАНИЕ ЛУЧШЕЙ ПЕРЕСТАНОВКИ
    logi.log_g("    ✅ BEST PERMUTATION SELECTED:\n");
    logi.log("       Permutation: [P%d, P%d, P%d] → [%s, %s, %s]\n",
             best_perm[0], best_perm[1], best_perm[2],
             pillar_names[hyp.visible_indices[0]].c_str(),
             pillar_names[hyp.visible_indices[1]].c_str(),
             pillar_names[hyp.visible_indices[2]].c_str());
    
    logi.log("       Transformation parameters:\n");
    logi.log("         Scale factor:      %.6f (error: %+.2f%%)\n", 
             best_calib.scale_factor, (best_calib.scale_factor - 1.0) * 100.0);
    logi.log("         Rotation angle:    %.2f°\n", best_calib.rotation_deg);
    logi.log("         LiDAR position:    [%.4f, %.4f] m\n", 
             best_calib.position.x(), best_calib.position.y());
    logi.log("         Distance from origin: %.3f m\n", best_calib.position.norm());
    
    // 15. ФИНАЛЬНАЯ ВАЛИДАЦИЯ ЛУЧШЕГО РЕШЕНИЯ
    logi.log("       Final validation of best permutation:\n");
    double final_error = 0.0;
    
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector2d q_calc = best_calib.scale_factor * best_calib.rotation_matrix * best_P.col(i) + 
                                best_calib.position;
        Eigen::Vector2d q_ref = best_Q.col(i);
        
        double point_error = (q_calc - q_ref).norm();
        final_error += point_error * point_error;
        
        logi.log("         %s: error = %.1f mm %s\n",
                 pillar_names[hyp.visible_indices[i]].c_str(),
                 point_error * 1000,
                 point_error <= 0.02 ? "✓" : "✗");
    }
    
    final_error = std::sqrt(final_error / 3.0);
    logi.log("       Final transformation RMSE: %.1f mm\n", final_error * 1000);
    
    // 16. СОХРАНЕНИЕ РЕЗУЛЬТАТОВ
    hyp.calib_params = best_calib;
    hyp.transformation_error = final_error;
    
    // Сохраняем информацию о лучшей перестановке
    hyp.visible_indices_permutation = best_perm;
    
    logi.log_g("    ✅ Umeyama computation successful for this hypothesis\n");
    
    return true;
}


/*
 * Оценка качества гипотезы
 */
void evaluateHypothesis(CalibrationHypothesis &hyp, 
                       const AlignedPillarVector &measured_pillars)
{
    LidarCalibration &calib = hyp.calib_params;
    
    // 1. Ошибка трансформации уже вычислена в performUmeyamaForThreePoints
    // hyp.transformation_error
    
    // 2. Ошибка предсказания 4-го столба
    // Находим "лишнюю" измеренную точку (ту, которая не использовалась в калибровке)
    // Это сложно без знания соответствия, поэтому пропускаем этот шаг
    hyp.prediction_error = 0.0;
    
    // 3. Геометрическая ошибка
    // Проверяем расстояния между видимыми столбами
    double geom_error = 0.0;
    int geom_pairs = 0;
    
    // У нас есть 3 измеренные точки, проверяем их расстояния
    for (int i = 0; i < 2; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            double measured_dist = MathUtils::dist2D(measured_pillars[i].local,
                                                    measured_pillars[j].local);
            
            // Находим, каким эталонным столбам они должны соответствовать
            // Это сложно без знания соответствия, поэтому упрощаем
            geom_error += std::abs(measured_dist - 4.0); // Примерное среднее расстояние
            geom_pairs++;
        }
    }
    
    if (geom_pairs > 0) {
        hyp.geometric_error = geom_error / geom_pairs;
    }
    
    // 4. Физическая правдоподобность параметров
    double scale_penalty = std::abs(calib.scale_factor - 1.0) * 10.0;
    double rotation_penalty = std::abs(calib.rotation_deg) > 45.0 ? 10.0 : 0.0;
    
    // 5. Общий score
    hyp.total_score = hyp.transformation_error * 1000.0 +  // мм
                      hyp.prediction_error * 2000.0 +      // вес x2
                      hyp.geometric_error * 1000.0 +       // мм
                      scale_penalty +
                      rotation_penalty;
}

/*
 * Применение лучшей гипотезы
 */
bool applyBestHypothesis(const CalibrationHypothesis &best_hyp,
                         AlignedPillarVector &pillars,
                         const AlignedPillarVector &original_pillars)
{
    // 1. Копируем параметры калибровки
    lidar_calibration_ = best_hyp.calib_params;
    
    // 2. Восстанавливаем полный набор из 4 столбов
    pillars.clear();
    pillars.resize(4);
    
    // 3. Для видимых столбов
    for (int i = 0; i < 3; ++i) {
        int pillar_idx = best_hyp.visible_indices[i];
        pillars[pillar_idx] = original_pillars[i];
        pillars[pillar_idx].name = PILLAR_NAMES[pillar_idx];
        pillars[pillar_idx].global = reference_centers_[pillar_idx].cast<double>();
    }
    
    // 4. Для отсутствующего столба (оцениваемого)
    int missing_idx = best_hyp.missing_pillar_idx;
    pillars[missing_idx].name = PILLAR_NAMES[missing_idx];
    pillars[missing_idx].global = reference_centers_[missing_idx].cast<double>();
    pillars[missing_idx].is_estimated = true;
    pillars[missing_idx].estimation_confidence = 0.7; // 70% уверенности
    
    logi.log_g("✅ Reconstructed 4 pillars (1 estimated: %s)\n",
               PILLAR_NAMES[missing_idx].c_str());
    
    // 5. Валидация
    double max_error = 0.0;
    double total_error_sq = 0.0;
    int valid_points = 0;
    
    for (int i = 0; i < 4; ++i) {
        if (i == missing_idx) continue; // Пропускаем оцененный столб
        
        Eigen::Vector2d p = pillars[i].local.cast<double>();
        Eigen::Vector2d q_expected = reference_centers_[i].cast<double>();
        Eigen::Vector2d q_calculated = lidar_calibration_.scale_factor * 
                                       lidar_calibration_.rotation_matrix * p + 
                                       lidar_calibration_.position;
        
        double error = (q_calculated - q_expected).norm();
        total_error_sq += error * error;
        valid_points++;
        
        if (error > max_error) max_error = error;
        
        logi.log("  %s: error = %.1f mm %s\n",
                 PILLAR_NAMES[i].c_str(), error * 1000,
                 error <= 0.02 ? "✓" : "✗");
    }
    
    if (valid_points == 0) {
        logi.log_r("❌ No valid points for validation\n");
        return false;
    }
    
    double rmse = std::sqrt(total_error_sq / valid_points);
    
    // 6. Принятие решения (более строгий порог для 3 точек)
    const double RMSE_THRESHOLD = 0.03; // 3 см (строже чем для 4 точек)
    
    if (rmse <= RMSE_THRESHOLD) {
        calibration_done_ = true;
        final_pillars_results_ = pillars;
        
        logi.log_g("\n✅ 3-PILLAR CALIBRATION SUCCESSFUL! RMSE = %.1f mm\n", rmse * 1000);
        logi.log_w("⚠️  Note: Pillar %s is estimated (confidence: 70%%)\n",
                   PILLAR_NAMES[missing_idx].c_str());
        
        saveCalibrationParameters();
        publishFinalMarkers(final_pillars_results_);
        return true;
    } else {
        logi.log_r("❌ 3-PILLAR CALIBRATION FAILED: RMSE = %.1f mm > %.1f mm\n",
                   rmse * 1000, RMSE_THRESHOLD * 1000);
        lidar_calibration_.clear();
        return false;
    }
}

/*
 * Выбор лучших 4 столбов из большего количества
 */
void selectBestFourPillars(AlignedPillarVector &pillars)
{
    if (pillars.size() <= 4) return;
    
    logi.log("Selecting best 4 pillars from %lu candidates...\n", pillars.size());
    
    // Сортируем по убыванию total_weight
    std::sort(pillars.begin(), pillars.end(),
              [](const FinalPillar &a, const FinalPillar &b) {
                  return a.total_weight > b.total_weight;
              });
    
    // Оставляем только 4 лучших
    pillars.resize(4);
    
    // Переименовываем
    for (size_t i = 0; i < pillars.size(); ++i) {
        pillars[i].name = "Pillar_" + std::to_string(i);
    }
    
    logi.log_g("Selected 4 pillars with highest weights\n");
}

/*
 * Универсальная калибровка - работает с 3 или 4 столбами
 */
void performCalibration(AlignedPillarVector &pillars)
{
    logi.log("\n═══════════════════════════════════════════════════════════\n");
    logi.log("🚀 UNIVERSAL LiDAR CALIBRATION (v8.0 - Supports 3 & 4 pillars)\n");
    logi.log("═══════════════════════════════════════════════════════════\n");
    
    // ========== ПРОВЕРКА ВХОДНЫХ ДАННЫХ ==========
    if (pillars.size() == 4) {
        logi.log_g("✓ 4 pillars detected - using standard calibration\n");
        performCalibrationFourPillars(pillars);
    }
    else if (pillars.size() == 3) {
        logi.log_w("⚠️  3 pillars detected - attempting advanced calibration\n");
        performCalibrationThreePillars(pillars);
    }
    else if (pillars.size() > 4) {
        logi.log_w("⚠️  %lu pillars detected - selecting best 4\n", pillars.size());
        selectBestFourPillars(pillars);
        if (pillars.size() == 4) {
            performCalibrationFourPillars(pillars);
        } else {
            logi.log_r("❌ Failed to select 4 pillars\n");
            lidar_calibration_.clear();
        }
    }
    else {
        logi.log_r("❌ Insufficient pillars for calibration: %lu (need 3 or 4)\n", pillars.size());
        lidar_calibration_.clear();
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


        AlignedVector2f current_fused_centers;
        for (const auto &fp : final_pillars)
        {
            current_fused_centers.push_back(fp.local); // Сохранение центров
        }
        fused_centers_results_ = current_fused_centers; // Сохранение центров Fusion

        // 7. Калибровка (Full Umeyama)
        // performCalibration(final_pillars); // Выполнение калибровки
        // Стало:
        if (final_pillars.size() >= 3) {
            // Для 4+ столбов сначала сортируем
            if (final_pillars.size() >= 4) {
                reorderPillars(final_pillars);
            }
            // Универсальная калибровка
            performCalibration(final_pillars);
        } else {
            logi.log_r("Insufficient pillars for calibration: %lu (need at least 3)\n",
                    final_pillars.size());
        }

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