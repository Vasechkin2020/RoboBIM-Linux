/*
 * Версия: 3.2_Active
 * Дата: 2025-11-29
 * Описание: Версия с активным режимом работы.
 * - Нода не завершается после калибровки.
 * - Используется ros::Timer для публикации финальных маркеров 1 раз в секунду.
 * - Фрейм для всех визуализаций унифицирован в "laser_frame_id".
 */

#include <ros/ros.h>                                       // Подключение основной библиотеки ROS
#include <sensor_msgs/LaserScan.h>                         // Подключение типа сообщения для данных лидара
#include <ros/topic.h>                                     // Необходим для ros::topic::waitForMessage
#include <vector>                                          // Работа с динамическими массивами
#include <cmath>                                           // Математические функции
#include <string>                                          // Работа со строками
#include <algorithm>                                       // Различные алгоритмы
#include <stdarg.h>                                        // Для реализации printf-подобного логгера 
#include <Eigen/Dense>                                     // Обязательно: библиотека линейной алгебры

// --- Заголовки для RViz ---
#include <sensor_msgs/PointCloud2.h>                   // Для публикации облаков точек
#include <visualization_msgs/MarkerArray.h>            // Для публикации маркеров (столбы)
#include <pcl_conversions/pcl_conversions.h>           // Конвертация между ROS и PCL
#include <pcl/point_types.h>                           // Типы точек PCL

// Константа для математических вычислений (pi)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --------------------------------------------------------------------------------------
// 1. Вспомогательные классы и структуры
// --------------------------------------------------------------------------------------

// Обертка для логирования (v2.8)
class LoggerWrapper
{
private:
    void v_log(const char* format, va_list args)           // Приватный базовый метод для printf
    {
        vprintf(format, args);
    }

public:
    void log(const char* format, ...)                      // Базовый вывод в лог (информационный)
    {
        va_list args;
        va_start(args, format);
        v_log(format, args);
        va_end(args);
    }

    void log_r(const char* format, ...)                    // Вывод ошибок [ERROR]
    {
        log("ERROR: ");
        va_list args;
        va_start(args, format);
        v_log(format, args);
        va_end(args);
    }
    
    void log_b(const char* format, ...)                    // Вывод входных данных [INPUT]
    {
        log("INPUT: ");
        va_list args;
        va_start(args, format);
        v_log(format, args);
        va_end(args);
    }
    
    void log_g(const char* format, ...)                    // Вывод выходных данных [OUTPUT]
    {
        log("OUTPUT: ");
        va_list args;
        va_start(args, format);
        v_log(format, args);
        va_end(args);
    }
    
    void log_w(const char* format, ...)                    // Вывод предупреждений [WARNING]
    {
        log("WARNING: ");
        va_list args;
        va_start(args, format);
        v_log(format, args);
        va_end(args);
    }
};

LoggerWrapper logi;                                        // Глобальный инстанс логгера

// Структура кандидата (найденного потенциального столба)
struct PillarCandidate
{
    Eigen::Vector2d center;                                // Координаты (X, Y) центра столба в системе лидара
    double rmse;                                           // Среднеквадратичная ошибка вписывания окружности
    int num_points;                                        // Число точек, использованных для аппроксимации
    double weight;                                         // Интегральный вес (0..1) - показатель надежности детекции
    int method_id;                                         // ID метода, которым найден кандидат (1, 2 или 3)
};

// Структура финального столба (после идентификации)
struct FinalPillar
{
    std::string name;                                      // Имя: RB, RT, LT, LB
    Eigen::Vector2d local;                                 // Координаты центра в системе лидара (до калибровки)
    Eigen::Vector2d global;                                // Координаты центра в глобальной системе (после калибровки)
};

// Класс математических утилит (v2.7)
class MathUtils
{
public:
    // Расстояние между двумя точками Eigen
    static double dist2D(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
    {
        return (p1 - p2).norm();                           // Возвращает евклидово расстояние (норму разности векторов)
    }

    // Аппроксимация окружности (Метод Kasa - Linear Least Squares)
    static bool fitCircle(const std::vector<Eigen::Vector2d>& points, double expected_radius, 
                          Eigen::Vector2d& out_center, double& out_rmse)
    {
        size_t n = points.size();
        if (n < 3) return false;

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
        Eigen::Vector3d sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(B);

        out_center.x() = sol(0);
        out_center.y() = sol(1);
        
        double r_sq = sol(2) + sol(0)*sol(0) + sol(1)*sol(1);
        double r_fit = (r_sq > 0) ? sqrt(r_sq) : expected_radius;

        double sum_sq = 0;
        for (const auto& p : points)
        {
            double d = dist2D(p, out_center);
            sum_sq += pow(d - r_fit, 2);
        }
        out_rmse = sqrt(sum_sq / n);
        return true;
    }
    
    // Функция для нахождения медианы
    static double getMedian(std::vector<double>& values)
    {
        if (values.empty()) return 0.0;
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
    ros::NodeHandle nh;                                    // Обработчик ROS
    ros::Subscriber scan_sub;                              // Подписчик на данные лидара
    ros::Timer publish_timer_;                             // Таймер для периодической публикации

    // --- Паблишеры для RViz ---
    ros::Publisher pub_filtered_scan;
    ros::Publisher pub_method_clusters;
    ros::Publisher pub_fused_pillars;
    ros::Publisher pub_final_markers;


    // --- Параметры конфигурации ---
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
    std::vector<Eigen::Vector2d> reference_centers_;

    // Состояние сбора данных
    int scans_collected;
    const int SCANS_TO_COLLECT = 100;
    std::vector<std::vector<double>> accumulated_ranges;        
    sensor_msgs::LaserScan meta_scan;

    // ЧЛЕНЫ ДЛЯ АКТИВНОГО РЕЖИМА (v3.2)
    bool calibration_done_;                                
    std::vector<FinalPillar> final_pillars_results_;       


    // --- ПРИВАТНЫЕ МЕТОДЫ ---
    
    void publishPointCloud(const std::vector<Eigen::Vector2d>& points, ros::Publisher& pub, const std::string& frame_id, float r, float g, float b, float a = 1.0)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;           // Создание облака PCL с цветными точками
        cloud.header.frame_id = frame_id;                  // Установка фрейма
        cloud.width = (uint32_t)points.size();
        cloud.height = 1;

        int r_i = (int)(r * 255);
        int g_i = (int)(g * 255);
        int b_i = (int)(b * 255);
        uint32_t rgb = (r_i << 16) | (g_i << 8) | b_i;

        for(const auto& p : points)
        {
            pcl::PointXYZRGB pt;
            pt.x = (float)p.x();
            pt.y = (float)p.y();
            pt.z = 0.0f;
            pt.rgb = *reinterpret_cast<float*>(&rgb);
            cloud.points.push_back(pt);
        }

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        pub.publish(msg);
    }
    
    void publishMethodClusters(const std::vector<Eigen::Vector2d>& m1, 
                               const std::vector<Eigen::Vector2d>& m2, 
                               const std::vector<Eigen::Vector2d>& m3)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.header.frame_id = meta_scan.header.frame_id;
        cloud.width = (uint32_t)(m1.size() + m2.size() + m3.size());
        cloud.height = 1;

        uint32_t c1 = (255 << 16) | (0 << 8) | 0;
        uint32_t c2 = (0 << 16) | (0 << 8) | 255;
        uint32_t c3 = (255 << 16) | (255 << 8) | 0;
        
        auto add_points = [&](const std::vector<Eigen::Vector2d>& points, uint32_t color)
        {
            float color_f = *reinterpret_cast<float*>(&color);
            for(const auto& p : points)
            {
                pcl::PointXYZRGB pt;
                pt.x = (float)p.x();
                pt.y = (float)p.y();
                pt.z = 0.0f;
                pt.rgb = color_f;
                cloud.points.push_back(pt);
            }
        };

        add_points(m1, c1);
        add_points(m2, c2);
        add_points(m3, c3);

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        pub_method_clusters.publish(msg);
    }

    // Метод для публикации маркеров (v3.2)
    void publishFinalMarkers(const std::vector<FinalPillar>& pillars)
    {
        if (pillars.empty()) return;
        
        visualization_msgs::MarkerArray marker_array;
        
        // Используем фрейм лидара для всех визуализаций
        std::string laser_frame_id = meta_scan.header.frame_id; 
        
        for(size_t i=0; i<pillars.size(); ++i)
        {
            const FinalPillar& p = pillars[i];
            
            // Находим эталонную точку по имени для корректного отображения Ref
            Eigen::Vector2d ref = Eigen::Vector2d(0, 0);
            if (p.name == "RB") ref = reference_centers_[0];
            else if (p.name == "RT") ref = reference_centers_[1];
            else if (p.name == "LT") ref = reference_centers_[2];
            else if (p.name == "LB") ref = reference_centers_[3];

            // 1. Маркер-сфера (синяя) - Найденный центр
            visualization_msgs::Marker sphere_marker;
            sphere_marker.header.frame_id = laser_frame_id; // Используем фрейм лидара
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
            
            // 2. Текстовый маркер (белый) - Имя столба
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = laser_frame_id; // Используем фрейм лидара
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
            text_marker.text = p.name + "\nRef:(" + std::to_string((int)(ref.x()*100)) + "," + std::to_string((int)(ref.y()*100)) + ")";
            marker_array.markers.push_back(text_marker);
        }
        
        pub_final_markers.publish(marker_array);
    }
    
    // Callback таймера для периодической публикации (v3.2)
    void publishResultsTimerCallback(const ros::TimerEvent& event)
    {
        if (calibration_done_ && !final_pillars_results_.empty())
        {
            publishFinalMarkers(final_pillars_results_); // Публикация сохраненных результатов
        }
    }


    void processCluster(const std::vector<Eigen::Vector2d>& cluster, int method_id, 
                        std::vector<PillarCandidate>& out, std::vector<Eigen::Vector2d>& out_cluster_points)
    {
        if (cluster.size() < 5) return;
        double width = MathUtils::dist2D(cluster.front(), cluster.back());
        if (width < 0.05 || width > 0.5) return;

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
                
                // Добавление точек в общий массив для RViz
                for (const auto& p : cluster)
                {
                    out_cluster_points.push_back(p);
                }
            }
        }
    }

    // Детекция на основе разрыва/плотности (v3.1)
    std::vector<PillarCandidate> detectGenericClustering(const std::vector<Eigen::Vector2d>& pts, double threshold, int method_id, 
                                                         std::vector<Eigen::Vector2d>& out_cluster_points)
    {
        std::vector<PillarCandidate> results;
        if (pts.empty()) return results;

        std::vector<Eigen::Vector2d> current_cluster;
        current_cluster.push_back(pts[0]);

        for (size_t i = 1; i < pts.size(); ++i)
        {
            double d = MathUtils::dist2D(pts[i], pts[i-1]);
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

    // Детекция на основе локальных минимумов дальности (v3.1)
    std::vector<PillarCandidate> detectLocalMinima(const std::vector<Eigen::Vector2d>& pts, int method_id,
                                                   std::vector<Eigen::Vector2d>& out_cluster_points)
    {
        std::vector<PillarCandidate> results;
        if (pts.size() < 10) return results;

        for (size_t i = 5; i < pts.size() - 5; ++i)
        {
            double r = pts[i].norm();
            bool is_min = true;
            for (int k = -5; k <= 5; ++k)
            {
                if (k==0) continue;
                if (pts[i+k].norm() < r) is_min = false;
            }

            if (is_min)
            {
                std::vector<Eigen::Vector2d> cluster;
                cluster.push_back(pts[i]);
                
                // Расширение кластера
                for (int k = -1; i+k >= 0; --k) {
                    if (pts[i+k].norm() > r + 0.2 || MathUtils::dist2D(pts[i+k], pts[i+k+1]) > 0.1) break;
                    cluster.push_back(pts[i+k]);
                }
                for (size_t k = 1; i+k < pts.size(); ++k) {
                    if (pts[i+k].norm() > r + 0.2 || MathUtils::dist2D(pts[i+k], pts[i+k-1]) > 0.1) break;
                    cluster.push_back(pts[i+k]);
                }
                processCluster(cluster, method_id, results, out_cluster_points);
                i += cluster.size(); // Пропуск уже обработанных точек
            }
        }
        return results;
    }

    // Логика слияния (Fusion)
    std::vector<FinalPillar> fuseCandidates(const std::vector<PillarCandidate>& candidates)
    {
        std::vector<FinalPillar> final_pillars;
        if (candidates.empty()) return final_pillars;

        std::vector<bool> processed(candidates.size(), false);
        std::vector<Eigen::Vector2d> found_centers;

        for (size_t i = 0; i < candidates.size(); ++i)
        {
            if (processed[i]) continue;
            double sum_w = 0;
            Eigen::Vector2d w_center(0, 0);
            
            for (size_t j = i; j < candidates.size(); ++j)
            {
                if (processed[j]) continue;
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

        // Публикация отобранных центров (Зеленый)
        publishPointCloud(found_centers, pub_fused_pillars, meta_scan.header.frame_id, 0.0f, 1.0f, 0.0f);

        if (found_centers.size() != 4)
        {
            logi.log_w("Fusion found %lu pillars. Umeyama calibration requires 4. Skipping.\n", found_centers.size());
            return final_pillars;
        }

        // Временно присваиваем имена для целей калибровки (по углу)
        std::sort(found_centers.begin(), found_centers.end(), 
            [](const Eigen::Vector2d& a, const Eigen::Vector2d& b){
                return atan2(a.y(), a.x()) < atan2(b.y(), b.x());
            });
        
        for(size_t i=0; i<4; ++i)
        {
            FinalPillar fp;
            fp.local = found_centers[i];
            fp.name = "Pillar_" + std::to_string(i); // Временное имя
            final_pillars.push_back(fp);
        }

        return final_pillars;
    }

    // Сохранение результатов в ROS Parameter Server
    void saveResults(const std::vector<FinalPillar>& pillars)
    {
        for (const auto& p : pillars)
        {
            if (p.name.find("Pillar_") != std::string::npos) continue;
            std::string base = "/pb_config/result/" + p.name;
            nh.setParam(base + "/x", p.global.x());
            nh.setParam(base + "/y", p.global.y());
        }
        logi.log_g("Results saved to rosparam.\n");
    }

    // --- ПУБЛИЧНЫЕ МЕТОДЫ ---
public:
    PillarScanNode() : scans_collected(0), calibration_done_(false)
    {
        logi.log("=== PillarScanNode v3.2 Started (Active Mode, 1Hz Publishing) ===\n");
        
        loadParameters();                                  // 1. Загрузка параметров
        if (!ros::ok()) return;

        // --- Инициализация Паблишеров RViz ---
        pub_filtered_scan = nh.advertise<sensor_msgs::PointCloud2>("/rviz/filtered_scan", 1);
        pub_method_clusters = nh.advertise<sensor_msgs::PointCloud2>("/rviz/method_clusters", 1);
        pub_fused_pillars = nh.advertise<sensor_msgs::PointCloud2>("/rviz/fused_pillars", 1);
        pub_final_markers = nh.advertise<visualization_msgs::MarkerArray>("/rviz/final_pillars", 1);
        
        initReferenceSystem();                             // 2. Построение эталона (Q_global)

        // --- ПРОВЕРКА НАЛИЧИЯ ДАННЫХ ЛИДАРА ---
        logi.log("Checking /scan topic availability (timeout 30s)...\n");
        
        sensor_msgs::LaserScan::ConstPtr first_scan =       
            ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(30));
        
        if (!first_scan)
        {
            logi.log_r("Timed out waiting for /scan topic. Is the LiDAR node running? Shutting down.\n");
            ros::shutdown();
            return;
        }
        
        // Инициализация структур
        size_t num_rays = first_scan->ranges.size();
        accumulated_ranges.resize(num_rays);
        meta_scan = *first_scan;
        
        logi.log_b("LiDAR initialized. Rays: %lu. Starting accumulation.\n", num_rays);

        // Запуск подписчика
        scan_sub = nh.subscribe("/scan", 100, &PillarScanNode::scanCallback, this);
        logi.log("Waiting for %d laser scans on /scan topic to complete initial calibration.\n", SCANS_TO_COLLECT);
        
        // Запуск таймера для периодической публикации результатов (1 Гц)
        publish_timer_ = nh.createTimer(ros::Duration(1.0), &PillarScanNode::publishResultsTimerCallback, this);
    }

    // ----------------------------------------------------------------------------------
    // Загрузка параметров
    // ----------------------------------------------------------------------------------
    void loadParameters()
    {
        logi.log("\n--- Loading YAML Parameters ---\n");

        // 1. Геометрия
        nh.param<double>("/pb_config/scan_node/pillar_diametr", pillar_diam_, 0.315);
        logi.log_b("  pillar_diametr: %.4f\n", pillar_diam_);
        pillar_radius_ = pillar_diam_ / 2.0;

        const char* dist_names[] = {
            "pillar_0_1", "pillar_0_2", "pillar_0_3", 
            "pillar_1_2", "pillar_1_3", "pillar_2_3"
        };
        
        bool dist_valid = true;
        for(int i=0; i<6; ++i)
        {
            std::string path = "/pb_config/scan_node/" + std::string(dist_names[i]);
            nh.param<double>(path, d_surf[i], 0.0);
            d_center[i] = d_surf[i] + 2.0 * pillar_radius_;
            
            logi.log_b("  %s (surf): %.4f -> (center): %.4f\n", dist_names[i], d_surf[i], d_center[i]);
            
            if(d_center[i] <= pillar_diam_) dist_valid = false;
        }

        if(!dist_valid)
        {
            logi.log_r("Invalid distances! Check YAML. Shutting down.\n");
            ros::shutdown(); return;
        }

        // 2. Фильтрация
        nh.param<double>("/pb_config/scan_node/filter/min_range_filter", min_range_filter, 0.2);
        nh.param<double>("/pb_config/scan_node/filter/max_range_filter", max_range_filter, 15.0);
        nh.param<double>("/pb_config/scan_node/filter/neighbor_radius_filter", neighbor_radius_filter, 0.3);
        nh.param<int>("/pb_config/scan_node/filter/min_neighbors_filter", min_neighbors_filter, 3);

        // 3. Детекция
        nh.param<double>("/pb_config/scan_node/detection/jump_dist_threshold", jump_dist_threshold, 0.5);
        nh.param<double>("/pb_config/scan_node/detection/cluster_dist_threshold", cluster_dist_threshold, 0.05);

        // 4. Fusion
        nh.param<double>("/pb_config/scan_node/fusion/rmse_max_tolerance", rmse_max_tolerance, 0.01);
        nh.param<int>("/pb_config/scan_node/fusion/n_max_points_norm", n_max_points_norm, 100);
        nh.param<double>("/pb_config/scan_node/fusion/fusion_group_radius", fusion_group_radius, 0.2);

        // 5. Веса
        nh.param<double>("/pb_config/scan_node/weights/w_method_1_jump", w_method[1], 1.0);
        nh.param<double>("/pb_config/scan_node/weights/w_method_2_cluster", w_method[2], 0.9);
        nh.param<double>("/pb_config/scan_node/weights/w_method_3_minima", w_method[3], 0.8);
        
        logi.log_b("  Filter Range: [%.2f, %.2f], KNN: R=%.2f, N=%d\n", 
                 min_range_filter, max_range_filter, neighbor_radius_filter, min_neighbors_filter);
        logi.log_b("  Detection: JumpThresh=%.2f, ClusterThresh=%.2f\n", jump_dist_threshold, cluster_dist_threshold);
        logi.log_b("  Fusion: RMSE_max=%.4f, N_max=%d, GroupR=%.2f\n", rmse_max_tolerance, n_max_points_norm, fusion_group_radius);
        logi.log_b("  Weights: M1=%.2f, M2=%.2f, M3=%.2f\n", w_method[1], w_method[2], w_method[3]);
        
        logi.log_g("--- Parameters Loaded Successfully ---\n");
    }

    // ----------------------------------------------------------------------------------
    // Инициализация эталона
    // ----------------------------------------------------------------------------------
    void initReferenceSystem()
    {
        reference_centers_.resize(4);
        
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
        reference_centers_[2] = Eigen::Vector2d(x2, y2);

        // P3 (LB) -> Левый Нижний: Триангуляция от P0 и P1
        double d03 = d_center[2];
        double d13 = d_center[4];
        
        double x3 = (pow(d01, 2) + pow(d03, 2) - pow(d13, 2)) / (2 * d01);
        double y3 = sqrt(std::max(0.0, pow(d03, 2) - pow(x3, 2)));
        reference_centers_[3] = Eigen::Vector2d(x3, -y3);

        logi.log_g("\nReference System Initialized:\n");
        const char* names[] = {"RB (0)", "RT (1)", "LT (2)", "LB (3)"};
        for(int i=0; i<4; ++i)
            logi.log_g("  %s: [%.3f, %.3f]\n", names[i], reference_centers_[i].x(), reference_centers_[i].y());
    }

    // ----------------------------------------------------------------------------------
    // Callback скана (v3.2)
    // ----------------------------------------------------------------------------------
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        // Игнорируем сканы, если калибровка уже прошла
        if (calibration_done_) return;

        if (scan->ranges.size() != accumulated_ranges.size())
        {
            logi.log_w("Scan size changed! Skipping scan.\n");
            return;
        }

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float r = scan->ranges[i];
            
            // Фильтрация мусора
            if (std::isnan(r) || std::isinf(r)) continue;
            if (r < min_range_filter || r > max_range_filter) continue;

            accumulated_ranges[i].push_back(r);                // Накопление значения
        }

        scans_collected++;
        if (scans_collected % 10 == 0) logi.log("Collecting scans: %d/%d\n", scans_collected, SCANS_TO_COLLECT);

        if (scans_collected == SCANS_TO_COLLECT)
        {
            logi.log("Accumulation complete. Starting processing pipeline...\n");
            processPipeline();
        }
    }
    
    // Основной конвейер обработки (v3.2)
    void processPipeline()
    {
        logi.log("=== Starting Processing Pipeline (Median Filtered Scan) ===\n");
        
        std::vector<Eigen::Vector2d> clean_points;

        // 1. ФОРМИРОВАНИЕ МЕДИАННОГО СКАНА И ПЕРЕВОД В ТОЧКИ 
        for (size_t i = 0; i < accumulated_ranges.size(); ++i)
        {
            if (accumulated_ranges[i].empty()) continue;
            
            std::vector<double> current_ray_data = accumulated_ranges[i]; 
            double median_r = MathUtils::getMedian(current_ray_data);

            double angle = meta_scan.angle_min + i * meta_scan.angle_increment;

            clean_points.emplace_back(median_r * cos(angle), median_r * sin(angle));
        }

        if (clean_points.empty()) 
        {
            logi.log_r("No valid points after filtering. Exiting pipeline.\n");
            return;
        }
        logi.log("Total median filtered points: %lu\n", clean_points.size());

        // 2. Статистическая фильтрация (упрощенный вариант)
        std::vector<Eigen::Vector2d> filtered_points = clean_points; 

        // Публикация отфильтрованного скана (Светло-серый)
        publishPointCloud(filtered_points, pub_filtered_scan, meta_scan.header.frame_id, 0.8f, 0.8f, 0.8f);

        // 3. Детекция (3 метода)
        std::vector<PillarCandidate> all_candidates;
        std::vector<Eigen::Vector2d> clusters_m1;
        std::vector<Eigen::Vector2d> clusters_m2;
        std::vector<Eigen::Vector2d> clusters_m3;
        
        auto c1 = detectGenericClustering(filtered_points, jump_dist_threshold, 1, clusters_m1);
        all_candidates.insert(all_candidates.end(), c1.begin(), c1.end());

        auto c2 = detectGenericClustering(filtered_points, cluster_dist_threshold, 2, clusters_m2);
        all_candidates.insert(all_candidates.end(), c2.begin(), c2.end());

        auto c3 = detectLocalMinima(filtered_points, 3, clusters_m3);
        all_candidates.insert(all_candidates.end(), c3.begin(), c3.end());
        
        publishMethodClusters(clusters_m1, clusters_m2, clusters_m3);

        logi.log("Total candidates found: %lu (M1=%lu, M2=%lu, M3=%lu)\n", 
            all_candidates.size(), c1.size(), c2.size(), c3.size());

        // 4. Fusion
        std::vector<FinalPillar> final_pillars = fuseCandidates(all_candidates);

        // 5. Калибровка (Full Umeyama)
        performCalibration(final_pillars);

        // 6. Сохранение
        saveResults(final_pillars_results_); // Используем сохраненные в performCalibration результаты

        logi.log_g("Calibration successful. Node remains active, publishing results every 1 second.\n");
    }


    // --- 6. Калибровка: Полноценный Umeyama's Algorithm (SVD) (v3.2) ---
    void performCalibration(std::vector<FinalPillar>& pillars)
    {
        if(pillars.size() != 4)
        {
            logi.log_w("Calibration skipped: Need exactly 4 pillars.\n");
            return;
        }

        logi.log("\n--- Performing FULL Umeyama Calibration (SVD) ---\n");
        
        // 1. Идентификация (Код Umeyama остается без изменений)
        int match_index[4] = {-1, -1, -1, -1};
        std::vector<bool> pillar_used(4, false);

        int idx_RB = -1;
        int idx_RT = -1;
        double best_err = 1000.0;
        
        // Идентификация RB (0) и RT (1)
        for(int i=0; i<4; ++i)
        {
            for(int j=0; j<4; ++j)
            {
                if(i==j) continue;
                double d = MathUtils::dist2D(pillars[i].local, pillars[j].local);
                double err = std::abs(d - d_center[0]); 
                if(err < 0.2 && err < best_err) 
                {
                    best_err = err;
                    if (pillars[i].local.x() < pillars[j].local.x()) {
                        idx_RB = i; idx_RT = j;
                    } else {
                        idx_RB = j; idx_RT = i;
                    }
                }
            }
        }
        
        if(idx_RB == -1)
        {
            logi.log_r("Calibration Failed: Could not identify RB-RT pair.\n");
            return;
        }

        match_index[0] = idx_RB; pillar_used[idx_RB] = true;
        match_index[1] = idx_RT; pillar_used[idx_RT] = true;
        pillars[idx_RB].name = "RB";
        pillars[idx_RT].name = "RT";

        // Идентификация LT (2) и LB (3) по Y-координате
        int rem_idx[2];
        int rem_count = 0;
        for(int i=0; i<4; ++i)
        {
            if (!pillar_used[i]) rem_idx[rem_count++] = i;
        }
        
        if (pillars[rem_idx[0]].local.y() > pillars[rem_idx[1]].local.y())
        {
            match_index[2] = rem_idx[0]; match_index[3] = rem_idx[1];
        } 
        else
        {
            match_index[2] = rem_idx[1]; match_index[3] = rem_idx[0];
        }
        
        pillars[match_index[2]].name = "LT";
        pillars[match_index[3]].name = "LB";

        logi.log("Identified: RB(%d), RT(%d), LT(%d), LB(%d)\n", 
            match_index[0], match_index[1], match_index[2], match_index[3]);


        // 2. Подготовка матриц
        Eigen::Matrix<double, 2, 4> X;
        Eigen::Matrix<double, 2, 4> Y;

        for(int i=0; i<4; ++i)
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

        logi.log_g("Umeyama Transform:\n");
        logi.log_g("  Scale (c): %.4f\n", c);
        
        double angle_rad = atan2(R(1,0), R(0,0));
        logi.log_g("  Rotation (deg): %.3f\n", angle_rad * 180.0 / M_PI);
        
        logi.log_g("  Translation (T): [%.3f, %.3f]\n", T.x(), T.y());


        // 9. Применение
        double final_rmse_sum_sq = 0.0;
        for(int i=0; i<4; ++i)
        {
            FinalPillar& p = pillars[match_index[i]];
            
            p.global = c * R * p.local + T;
            
            logi.log_g("Pillar %s -> Global: [%.3f, %.3f]\n", p.name.c_str(), p.global.x(), p.global.y());
            
            Eigen::Vector2d error = p.global - reference_centers_[i]; 
            final_rmse_sum_sq += error.squaredNorm();
        }
        
        double final_rmse = sqrt(final_rmse_sum_sq / 4.0);
        logi.log_g("Final Alignment RMSE: %.5f meters\n", final_rmse);
        
        if (final_rmse > 0.05) {
            logi.log_w("High calibration error (%.1f cm)! Check pillars visibility and geometry.\n", final_rmse*100.0);
        }
        
        // --- НОВЫЕ СТРОКИ ДЛЯ АКТИВНОГО РЕЖИМА (v3.2) ---
        if (pillars.size() == 4)
        {
            final_pillars_results_ = pillars;               // Сохраняем результаты
            calibration_done_ = true;                       // Устанавливаем флаг завершения
            
            // ПЕРВИЧНАЯ ПУБЛИКАЦИЯ: Публикуем сразу после расчета
            publishFinalMarkers(final_pillars_results_); 
        }
        // ----------------------------------------------
    }

};

int main(int argc, char** argv)
{
    // Инициализация ROS
    ros::init(argc, argv, "scan_pillar_node");
    
    // Создание объекта
    PillarScanNode node; 
    
    // Запуск цикла обработки (нода теперь остается активной)
    ros::spin();
    
    // Завершение работы
    logi.log("Node finished execution gracefully.\n");
    return 0;
}

// Число строк кода: 593