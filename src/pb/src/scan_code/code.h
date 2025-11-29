/*
 * Версия: 5.6
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
};

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
    ros::Publisher pub_method_clusters;
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
    double fusion_group_radius;
    double w_method[4];

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
    std::vector<FinalPillar> final_pillars_results_;

    // Хранение промежуточных результатов для постоянной публикации
    sensor_msgs::LaserScan filtered_scan_results_;
    visualization_msgs::MarkerArray cluster_markers_results_;
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
    void publishFinalMarkers(const std::vector<FinalPillar> &pillars)
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
        ROS_INFO("    0-publishResultsTimerCallback");

        // 1. Публикация ЧИСТЫХ ТОЧЕК (clean_points_results_) как Marker::POINTS
        if (clean_points_results_.size() > 0 && meta_scan.header.frame_id != "")
        {
            // Создаем маркер POINTS из вектора 2D-точек clean_points_results_
            // Используем pub_filtered_scan, но публикуем Marker вместо LaserScan
            pub_filtered_scan.publish(createPointsMarker(clean_points_results_,
                                                         meta_scan.header.frame_id,                    // Используем frame_id из meta_scan
                                                         "clean_points", 0, 0.7f, 0.7f, 0.7f, 0.05f)); // Серый цвет, мелкие точки
            ROS_INFO("      clean_points_results_ published as Marker::POINTS");
        }
        else if (filtered_scan_results_.ranges.size() > 0)
        {
            // Исходный LaserScan больше не публикуется
            ROS_INFO("      NOTE: filtered_scan_results_ is no longer published as LaserScan.");
        }

        // 2. Публикация маркеров кластеров (cluster_markers_results_)
        if (cluster_markers_results_.markers.size() > 0)
        {
            pub_method_clusters.publish(cluster_markers_results_);
            ROS_INFO("      cluster_markers_results_");
        }

        // 3. Публикация центров Fusion (fused_centers_results_)
        if (fused_centers_results_.size() > 0 && meta_scan.header.frame_id != "")
        {
            // Нужно пересоздать маркер, используя сохраненные данные и frame_id
            pub_fused_pillars.publish(createPointsMarker(fused_centers_results_,
                                                         meta_scan.header.frame_id, // Используем frame_id из meta_scan
                                                         "fused_centers", 4, 0.0f, 1.0f, 0.0f, 0.15f));
            ROS_INFO("      pub_fused_pillars");
        }

        // 4. Публикация финальных откалиброванных маркеров (final_pillars_results_)
        if (!final_pillars_results_.empty())
        {
            publishFinalMarkers(final_pillars_results_);
            ROS_INFO("      publishFinalMarkers");
        }
    }

    // Обработка одного кластера (Без изменений)
    void processCluster(const AlignedVector2f &cluster, int method_id,
                        std::vector<PillarCandidate> &out, AlignedVector2f &out_cluster_points)
    {
        if (cluster.size() < 5)
            return;
        double width = MathUtils::dist2D(cluster.front(), cluster.back());
        if (width < 0.05 || width > 0.5)
            return;

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

    // Детекция на основе разрыва/плотности (Без изменений)
    std::vector<PillarCandidate> detectGenericClustering(const AlignedVector2f &pts, double threshold, int method_id,
                                                         AlignedVector2f &out_cluster_points)
    {
        std::vector<PillarCandidate> results;
        if (pts.empty())
            return results;

        AlignedVector2f current_cluster;
        current_cluster.push_back(pts[0]);

        for (size_t i = 1; i < pts.size(); ++i)
        {
            double d = MathUtils::dist2D(pts[i], pts[i - 1]);
            if (d > threshold)
            {
                processCluster(current_cluster, method_id, results, out_cluster_points);
                current_cluster.clear();
            }
            current_cluster.push_back(pts[i]);
        }
        processCluster(current_cluster, method_id, results, out_cluster_points);
        return results;
    }

    // Детекция на основе локальных минимумов дальности (Без изменений)
    std::vector<PillarCandidate> detectLocalMinima(const AlignedVector2f &pts, int method_id,
                                                   AlignedVector2f &out_cluster_points)
    {
        std::vector<PillarCandidate> results;
        if (pts.size() < 10)
            return results;

        for (size_t i = 5; i < pts.size() - 5; ++i)
        {
            float r = pts[i].norm();
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
                AlignedVector2f cluster;
                cluster.push_back(pts[i]);

                // Расширение кластера назад
                for (int k = -1; i + k >= 0; --k)
                {
                    if (pts[i + k].norm() > r + 0.2 || MathUtils::dist2D(pts[i + k], pts[i + k + 1]) > 0.1)
                        break;
                    cluster.push_back(pts[i + k]);
                }
                // Расширение кластера вперед
                for (size_t k = 1; i + k < pts.size(); ++k)
                {
                    if (pts[i + k].norm() > r + 0.2 || MathUtils::dist2D(pts[i + k], pts[i + k - 1]) > 0.1)
                        break;
                    cluster.push_back(pts[i + k]);
                }
                processCluster(cluster, method_id, results, out_cluster_points);
                i += cluster.size();
            }
        }
        return results;
    }

    // Логика слияния (Fusion) (Без изменений)
    std::vector<FinalPillar> fuseCandidates(const std::vector<PillarCandidate> &candidates)
    {
        std::vector<FinalPillar> final_pillars;
        if (candidates.empty())
            return final_pillars;

        std::vector<bool> processed(candidates.size(), false);
        AlignedVector2f found_centers;

        for (size_t i = 0; i < candidates.size(); ++i)
        {
            if (processed[i])
                continue;
            double sum_w = 0;
            Eigen::Vector2f w_center(0.0f, 0.0f);

            for (size_t j = i; j < candidates.size(); ++j)
            {
                if (processed[j])
                    continue;
                // Слияние, если центры близки
                if (MathUtils::dist2D(candidates[i].center, candidates[j].center) < fusion_group_radius)
                {
                    w_center += candidates[j].center * (float)candidates[j].weight;
                    sum_w += candidates[j].weight;
                    processed[j] = true;
                }
            }

            if (sum_w > 0)
            {
                found_centers.push_back(w_center / (float)sum_w);
            }
        }

        logi.log("Fusion: Found %lu unique pillars.\n", found_centers.size());

        if (found_centers.size() != 4)
        {
            logi.log_w("Fusion found %lu pillars. Umeyama calibration requires 4. Skipping.\n", found_centers.size());
            return final_pillars;
        }

        // Сортировка по углу
        std::sort(found_centers.begin(), found_centers.end(),
                  [](const Eigen::Vector2f &a, const Eigen::Vector2f &b)
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

    // Сохранение результатов в ROS Parameter Server (Без изменений)
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
        logi.log_g("Results saved to rosparam.\n");
    }

    // --- ИЗМЕНЕНА: removeEdgeArtifacts (Полностью - v5.5) ---
    // Удаление фантомных точек (хвостов) с помощью углового фильтра
    AlignedVector2f removeEdgeArtifacts(const AlignedVector2f &points, const std::vector<double> &intensities, int &points_removed_by_angle_filter)
    {
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
            logi.log("%5lu | %8.3f | %8.3f | %9.3f | %8.3f | %8.3f | %10.3f | %13.3f | %s\n",
                     i + 1, P_curr.x(), P_curr.y(), lidar_angle_deg, P_next.x(), P_next.y(),
                     angle_deg, angle_check_deg,
                     "KEPT_P_NEXT");
        }

        logi.log("--- END DETAILED ANGLE FILTER DEBUG LOG ---\n");

        // Вывод статистики в конце функции (v5.5)
        size_t final_point_count = clean_points.size();

        logi.log_b("ANGLE FILTER STATS: Initial points: %lu. Removed: %d. Final points: %lu.\n",
                   initial_point_count, points_removed_by_angle_filter, final_point_count);

        return clean_points;
    }

    // --- Калибровка: Полноценный Umeyama's Algorithm (Без изменений) ---
    void performCalibration(std::vector<FinalPillar> &pillars)
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

        // 9. Применение (Сохранение в Vector2d)
        double final_rmse_sum_sq = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            FinalPillar &p = pillars[match_index[i]];

            Eigen::Vector2d p_local_double = p.local.cast<double>();
            p.global = c * R * p_local_double + T;

            logi.log_g("Pillar %s -> Global: [%.3f, %.3f]\n", p.name.c_str(), p.global.x(), p.global.y());

            Eigen::Vector2d ref_double = reference_centers_[i].cast<double>();
            Eigen::Vector2d error = p.global - ref_double;
            final_rmse_sum_sq += error.squaredNorm();
        }

        double final_rmse = sqrt(final_rmse_sum_sq / 4.0);
        logi.log_g("Final Alignment RMSE: %.5f meters\n", final_rmse);

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
        logi.log("=== PillarScanNode v5.2 Started (Intensity & Angle Debug) ===\n");

        loadParameters();
        if (!ros::ok())
            return;

        // Инициализация паблишеров
        pub_filtered_scan = nh.advertise<visualization_msgs::Marker>("/rviz/filtered_scan", 1);
        pub_method_clusters = nh.advertise<visualization_msgs::MarkerArray>("/rviz/method_clusters", 1);
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

    // ----------------------------------------------------------------------------------
    // Загрузка параметров (Без изменений)
    // ----------------------------------------------------------------------------------
    void loadParameters()
    {
        logi.log("\n--- Loading YAML Parameters ---\n");

        // 1. Диаметр столба и радиус
        nh.param<double>("/pb_config/pillar_params/pillar_diametr", pillar_diam_, 0.315);
        pillar_radius_ = pillar_diam_ / 2.0;

        // 2. Эталонные расстояния (в метрах)
        nh.param<double>("/pb_config/reference_dists/pillar_0_1", d_surf[0], 10.5);
        nh.param<double>("/pb_config/reference_dists/pillar_0_2", d_surf[1], 12.0);
        nh.param<double>("/pb_config/reference_dists/pillar_0_3", d_surf[2], 4.8);
        nh.param<double>("/pb_config/reference_dists/pillar_1_2", d_surf[3], 5.5);
        nh.param<double>("/pb_config/reference_dists/pillar_1_3", d_surf[4], 11.5);
        nh.param<double>("/pb_config/reference_dists/pillar_2_3", d_surf[5], 4.8);

        // Вычисление расстояний между центрами
        for (int i = 0; i < 6; ++i)
        {
            d_center[i] = d_surf[i] + pillar_diam_;
        }

        // Лог параметров расстояний
        logi.log("  pillar_diametr: %.4f\n", pillar_diam_);
        logi.log("  pillar_0_1 (surf): %.4f -> (center): %.4f\n", d_surf[0], d_center[0]);
        logi.log("  pillar_0_2 (surf): %.4f -> (center): %.4f\n", d_surf[1], d_center[1]);
        // ...

        // 3. Параметры фильтрации
        nh.param<double>("/pb_config/filters/min_range", min_range_filter, 0.2);
        nh.param<double>("/pb_config/filters/max_range", max_range_filter, 15.0);
        nh.param<double>("/pb_config/filters/neighbor_radius", neighbor_radius_filter, 0.3);
        nh.param<int>("/pb_config/filters/min_neighbors", min_neighbors_filter, 3);

        // НОВЫЕ ПАРАМЕТРЫ ФИЛЬТРАЦИИ ХВОСТОВ (v4.4)
        nh.param<double>("/pb_config/filters/intensity_min_threshold", intensity_min_threshold, 10.0);
        nh.param<double>("/pb_config/filters/edge_angle_threshold_deg", edge_angle_threshold, 15.0);
        edge_angle_threshold *= M_PI / 180.0; // Конвертация в радианы

        logi.log("  Filter Range: [%.2f, %.2f], KNN: R=%.2f, N=%d\n",
                 min_range_filter, max_range_filter, neighbor_radius_filter, min_neighbors_filter);
        logi.log("  Artifact Filter: I_min=%.2f, Angle_rad=%.4f (%.1f deg)\n",
                 intensity_min_threshold, edge_angle_threshold, edge_angle_threshold * 180.0 / M_PI);

        // 4. Параметры детекции
        nh.param<double>("/pb_config/detection/jump_threshold", jump_dist_threshold, 0.5);
        nh.param<double>("/pb_config/detection/cluster_threshold", cluster_dist_threshold, 0.05);
        logi.log("  Detection: JumpThresh=%.2f, ClusterThresh=%.2f\n", jump_dist_threshold, cluster_dist_threshold);

        // 5. Параметры Fusion
        nh.param<double>("/pb_config/fusion/rmse_max_tolerance", rmse_max_tolerance, 0.01);
        nh.param<int>("/pb_config/fusion/n_max_points_norm", n_max_points_norm, 100);
        nh.param<double>("/pb_config/fusion/group_radius", fusion_group_radius, 0.2);
        logi.log("  Fusion: RMSE_max=%.4f, N_max=%d, GroupR=%.2f\n",
                 rmse_max_tolerance, n_max_points_norm, fusion_group_radius);

        // 6. Веса методов
        nh.param<double>("/pb_config/weights/method_1", w_method[1], 1.0);
        nh.param<double>("/pb_config/weights/method_2", w_method[2], 0.9);
        nh.param<double>("/pb_config/weights/method_3", w_method[3], 0.8);
        logi.log("  Weights: M1=%.2f, M2=%.2f, M3=%.2f\n", w_method[1], w_method[2], w_method[3]);

        logi.log("--- Parameters Loaded Successfully ---\n");
    }

    // ----------------------------------------------------------------------------------
    // Инициализация эталона (Reference System) (Без изменений)
    // ----------------------------------------------------------------------------------
    void initReferenceSystem()
    {
        reference_centers_.clear();

        double L = d_center[0];
        double Y_LT = d_center[3];
        double X_LB = (d_center[1] * d_center[1] - d_center[4] * d_center[4] + L * L) / (2 * L);
        double Y_LB = sqrt(d_center[2] * d_center[2] - X_LB * X_LB);

        // 0. RB (Right Bottom)
        reference_centers_.emplace_back(0.0f, 0.0f);

        // 1. RT (Right Top)
        reference_centers_.emplace_back((float)L, 0.0f);

        // 2. LT (Left Top)
        reference_centers_.emplace_back((float)L, (float)Y_LT);

        // 3. LB (Left Bottom)
        reference_centers_.emplace_back((float)X_LB, -(float)Y_LB);

        logi.log("\nReference System Initialized:\n");
        logi.log("  RB (0): [%.3f, %.3f]\n", reference_centers_[0].x(), reference_centers_[0].y());
        logi.log("  RT (1): [%.3f, %.3f]\n", reference_centers_[1].x(), reference_centers_[1].y());
        logi.log("  LT (2): [%.3f, %.3f]\n", reference_centers_[2].x(), reference_centers_[2].y());
        logi.log("  LB (3): [%.3f, %.3f]\n", reference_centers_[3].x(), reference_centers_[3].y());
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
            accumulated_intensities[i].push_back(intensity);  // Сохраняем  только если прошли фильтр
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

    // ИЗМЕНЕНА: Основной конвейер обработки (Полностью - v5.6)
    void processPipeline()
    {
        logi.log("=== Starting Processing Pipeline (Median Filtered Scan) ===\n"); // Лог начала конвейера

        int total_initial_rays = 0;             // Общее число лучей в скане (фиксировано: 3585)
        int points_removed_by_angle_filter = 0; // Счетчик точек, удаленных угловым фильтром

        // 1. ФОРМИРОВАНИЕ МЕДИАННОГО СКАНА И ПЕРЕВОД В ТОЧКИ

        sensor_msgs::LaserScan current_filtered_scan = meta_scan; // Копирование мета-данных скана
        current_filtered_scan.ranges.clear();                     // Очистка дальностей
        current_filtered_scan.intensities.clear();                // Очистка интенсивностей
        AlignedVector2f initial_points;                           // Вектор 2D точек до углового фильтра
        std::vector<double> median_intensities;                   // Вектор медианных интенсивностей

        float nan_val = std::numeric_limits<float>::quiet_NaN(); // Значение NaN

        for (size_t i = 0; i < accumulated_ranges.size(); ++i)
        {
            total_initial_rays++; // Считаем общее число лучей

            if (accumulated_ranges[i].empty()) // Если луч был полностью отфильтрован по интенсивности
            {
                // Устанавливаем NaN и для дальности, и для интенсивности
                current_filtered_scan.ranges.push_back(nan_val);
                current_filtered_scan.intensities.push_back(nan_val); // NaN для интенсивности
                continue;                                             // Пропускаем итерацию
            }

            // Расчет медианной дальности
            std::vector<double> current_ray_data = accumulated_ranges[i];
            double median_r = MathUtils::getMedian(current_ray_data);

            // Расчет медианной интенсивности
            double median_i = 0.0;
            if (accumulated_intensities[i].size() > 0)
            {
                std::vector<double> current_int_data = accumulated_intensities[i];
                median_i = MathUtils::getMedian(current_int_data);
            }
            median_intensities.push_back(median_i); // Сохраняем медианную интенсивность

            double angle = meta_scan.angle_min + i * meta_scan.angle_increment; // Угол луча

            current_filtered_scan.ranges.push_back((float)median_r);      // Сохраняем медианную дальность
            current_filtered_scan.intensities.push_back((float)median_i); // Сохраняем медианную интенсивность

            // Только если луч не NaN, переводим его в 2D точку для кластеризации (защита от NaN)
            if (!std::isnan(current_filtered_scan.ranges.back()))
            {
                initial_points.emplace_back((float)(median_r * cos(angle)), (float)(median_r * sin(angle))); // Перевод в 2D
            }
        }

        // СОХРАНЕНИЕ 1: Сохраняем отфильтрованный скан (LaserScan)
        filtered_scan_results_ = current_filtered_scan; // Сохраняем LaserScan (нужен для заголовка)

        // 2. ЛОГИРОВАНИЕ СЫРОГО СКАНА (v4.7)
        logRawScan(); // Логирование сырых данных

        // 3. ЛОГИРОВАНИЕ ИТОГОВОГО ОТФИЛЬТРОВАННОГО СКАНА
        logFinalFilteredScan(filtered_scan_results_); // Логирование медианного скана

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

        // СОЗДАНИЕ МАРКЕРОВ КЛАСТЕРОВ ДЛЯ СОХРАНЕНИЯ
        visualization_msgs::MarkerArray cluster_markers;
        cluster_markers.markers.push_back(createPointsMarker(clusters_m1, meta_scan.header.frame_id,
                                                             "method_1_jump", 1, 1.0f, 0.0f, 0.0f, 0.08f)); // Красные
        cluster_markers.markers.push_back(createPointsMarker(clusters_m2, meta_scan.header.frame_id,
                                                             "method_2_cluster", 2, 0.0f, 0.0f, 1.0f, 0.08f)); // Синие
        cluster_markers.markers.push_back(createPointsMarker(clusters_m3, meta_scan.header.frame_id,
                                                             "method_3_minima", 3, 1.0f, 1.0f, 0.0f, 0.08f)); // Желтые

        cluster_markers_results_ = cluster_markers; // Сохранение маркеров

        logi.log("Total candidates found: %lu (M1=%lu, M2=%lu, M3=%lu)\n",
                 all_candidates.size(), c1.size(), c2.size(), c3.size()); // Лог кандидатов

        // 6. Fusion
        std::vector<FinalPillar> final_pillars = fuseCandidates(all_candidates); // Слияние кандидатов

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