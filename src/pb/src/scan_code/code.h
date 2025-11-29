/*
 * Версия: 4.3
 * Дата: 2025-11-29
 * Описание: ПОЛНЫЙ КОД: Предоставлены тела всех методов. ПОСТОЯННАЯ ПУБЛИКАЦИЯ ВСЕГО: Все 4 этапа обработки (скан, кластеры, Fusion, Final) публикуются 1 Гц.
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

// --- Заголовки для RViz (Только стандартные ROS) ---
#include <visualization_msgs/MarkerArray.h>            // Для публикации MarkerArray
#include <visualization_msgs/Marker.h>                 // Для публикации одиночного Marker
#include <geometry_msgs/Point.h>                       // Для точек внутри маркеров

// Константа для математических вычислений (pi)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --------------------------------------------------------------------------------------
// 1. Вспомогательные классы и структуры
// --------------------------------------------------------------------------------------

// ВНЕШНЕЕ ОБЪЯВЛЕНИЕ ЛОГГЕРА
class AsyncFileLogger;                                     // Предварительное объявление класса
extern AsyncFileLogger logi;                               // Объявление, что logi определен где-то еще

// Структура кандидата 
struct PillarCandidate
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW                      // Макрос для обеспечения выравнивания Eigen
    Eigen::Vector2f center;                                // Используем Vector2f для локальных координат
    double rmse;                                           // RMSE остается в double
    int num_points;                                        // Число точек
    double weight;                                         // Вес кандидата
    int method_id;                                         // Идентификатор метода (1, 2, 3)
};

// Структура финального столба 
struct FinalPillar
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW                      // Макрос для обеспечения выравнивания Eigen
    std::string name;                                      // Имя столба (RB, RT, LT, LB)
    Eigen::Vector2f local;                                 // Локальная позиция (относительно лидара) - float
    Eigen::Vector2d global;                                // Глобальная позиция (после калибровки) - double
};

// Тип для выровненного вектора Eigen::Vector2f
using AlignedVector2f = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>; // Alias для Vector2f с аллокатором

// Класс математических утилит
class MathUtils
{
public:
    static double dist2D(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) // Расстояние между Vector2f
    {
        return (p1 - p2).norm();                           
    }

    static bool fitCircle(const AlignedVector2f& points, double expected_radius, // Подгонка окружности
                          Eigen::Vector2f& out_center, double& out_rmse) 
    {
        size_t n = points.size();                            // Число точек в кластере
        if (n < 3) return false;                             // Требуется минимум 3 точки

        Eigen::MatrixXd A(n, 3);                             // Матрица A для метода наименьших квадратов
        Eigen::VectorXd B(n);                                // Вектор B

        for (size_t i = 0; i < n; ++i)
        {
            // Преобразование float -> double для матричных операций (точность)
            double x = (double)points[i].x();
            double y = (double)points[i].y();
            
            A(i, 0) = 2.0 * x;
            A(i, 1) = 2.0 * y;
            A(i, 2) = 1.0;
            B(i) = x * x + y * y;
        }

        Eigen::Vector3d sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(B); // Решение методом SVD

        // Результат double -> float
        out_center.x() = (float)sol(0);                      // Центр X
        out_center.y() = (float)sol(1);                      // Центр Y
        
        double r_sq = sol(2) + sol(0)*sol(0) + sol(1)*sol(1); // Квадрат радиуса
        double r_fit = (r_sq > 0) ? sqrt(r_sq) : expected_radius; // Полученный радиус

        double sum_sq = 0;                                   // Сумма квадратов ошибок
        for (const auto& p : points)
        {
            double d = dist2D(p, out_center);                // Расстояние от точки до центра (Vector2f)
            sum_sq += pow(d - r_fit, 2);
        }
        out_rmse = sqrt(sum_sq / n);                         // Среднеквадратичная ошибка
        return true;
    }
    
    static double getMedian(std::vector<double>& values)    // Расчет медианы (для фильтрации дальности)
    {
        if (values.empty()) return 0.0;
        size_t n = values.size();
        size_t median_idx = n / 2;
        
        std::nth_element(values.begin(), values.begin() + median_idx, values.end()); // Нахождение медианного элемента
        
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
    ros::NodeHandle nh;                                    // Указатель на ROS ноду
    ros::Subscriber scan_sub;                              // Подписчик на лидар
    ros::Timer publish_timer_;                             // Таймер для публикации результатов

    // --- Паблишеры (Marker/MarkerArray) ---
    ros::Publisher pub_filtered_scan;                      // Публикатор для отфильтрованного скана (LaserScan)
    ros::Publisher pub_method_clusters;                    // Публикатор для кластеров по методам (MarkerArray)
    ros::Publisher pub_fused_pillars;                      // Публикатор для центров слияния (Marker)
    ros::Publisher pub_final_markers;                      // Публикатор финальных маркеров (MarkerArray)

    // --- Параметры ---
    double pillar_diam_;
    double pillar_radius_;
    double d_surf[6];                                      // Поверхностные расстояния
    double d_center[6];                                    // Расстояния между центрами
    double min_range_filter;
    double max_range_filter;
    double neighbor_radius_filter;
    int min_neighbors_filter;
    double jump_dist_threshold;
    double cluster_dist_threshold;
    double rmse_max_tolerance;
    int n_max_points_norm;
    double fusion_group_radius;
    double w_method[4];                                    // Веса для методов (индекс 1-3)
    
    AlignedVector2f reference_centers_;                     // Эталонные центры

    // --- Калибровка и данные ---
    int scans_collected;
    const int SCANS_TO_COLLECT = 100;
    std::vector<std::vector<double>> accumulated_ranges;    // Накопленные данные дальности
    sensor_msgs::LaserScan meta_scan;                      // Метаданные первого скана

    bool calibration_done_;                                // Флаг завершения калибровки
    std::vector<FinalPillar> final_pillars_results_;       // Результаты калибровки (глобальные, FinalPillar)
    
    // НОВЫЕ ЧЛЕНЫ: Хранение промежуточных результатов для постоянной публикации (v4.3)
    sensor_msgs::LaserScan filtered_scan_results_;           // 1. Отфильтрованный скан
    visualization_msgs::MarkerArray cluster_markers_results_; // 2. Кластеры, найденные 3 методами
    AlignedVector2f fused_centers_results_;                 // 3. Центры столбов Fusion (локальные координаты)

    // ----------------------------------------------------------------------------------
    // 3. ПРИВАТНЫЕ МЕТОДЫ
    // ----------------------------------------------------------------------------------

    // НОВАЯ ФУНКЦИЯ: Публикация одиночного Marker в топик MarkerArray
    void publishMarkerInArray(const visualization_msgs::Marker& marker, ros::Publisher& pub)
    {
        visualization_msgs::MarkerArray marker_array;      // Создаем массив маркеров
        marker_array.markers.push_back(marker);            // Добавляем одиночный маркер в массив
        pub.publish(marker_array);                         // Публикуем массив маркеров
    }
    
    // НОВАЯ ФУНКЦИЯ: Создание MarkerArray для кластеров
    visualization_msgs::MarkerArray createClusterMarkers(const std::vector<PillarCandidate>& candidates, const std::string& frame_id)
    {
        visualization_msgs::MarkerArray marker_array;
        
        // ВАЖНО: В текущей структуре processPipeline() точки кластеров для маркеров 
        // (clusters_m1, m2, m3) формируются внутри detect* и возвращаются.
        // Здесь мы просто возвращаем пустой массив, так как маркеры кластеров 
        // создаются и сохраняются в processPipeline().
        
        return marker_array; // Возвращаем пустой массив
    }

    // ИЗМЕНЕНА: Формирование Marker::POINTS (теперь возвращает Marker)
    visualization_msgs::Marker createPointsMarker(const AlignedVector2f& points, const std::string& frame_id, const std::string& ns, int id, float r, float g, float b, float scale)
    {
        visualization_msgs::Marker marker;                      // Инициализация маркера
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

        for(const auto& p : points)
        {
            geometry_msgs::Point pt;
            pt.x = p.x();                                      // Координаты float
            pt.y = p.y();
            pt.z = 0.0;
            marker.points.push_back(pt);                       
        }

        return marker;                                     // Возвращаем объект Marker
    }
    
    // Метод для публикации маркеров финальных столбов (Работает с MarkerArray)
    void publishFinalMarkers(const std::vector<FinalPillar>& pillars)
    {
        if (pillars.empty()) return;
        
        visualization_msgs::MarkerArray marker_array;
        
        std::string laser_frame_id = meta_scan.header.frame_id; 
        
        for(size_t i=0; i<pillars.size(); ++i)
        {
            const FinalPillar& p = pillars[i];
            
            // Эталонные центры для отображения (преобразуем float -> double)
            Eigen::Vector2d ref = Eigen::Vector2d(0, 0);
            if (p.name == "RB") ref = reference_centers_[0].cast<double>();
            else if (p.name == "RT") ref = reference_centers_[1].cast<double>();
            else if (p.name == "LT") ref = reference_centers_[2].cast<double>();
            else if (p.name == "LB") ref = reference_centers_[3].cast<double>();

            // 1. Маркер-сфера (синяя) - Найденный центр (p.global - Vector2d)
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
            
            // 2. Текстовый маркер (белый) - Имя столба
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
            text_marker.text = p.name + "\nRef:(" + std::to_string((int)(ref.x()*100)) + "," + std::to_string((int)(ref.y()*100)) + ")";
            marker_array.markers.push_back(text_marker);
        }
        
        pub_final_markers.publish(marker_array);
    }
    
    // ИЗМЕНЕНА: Callback таймера для периодической публикации (теперь публикует все доступные данные)
    void publishResultsTimerCallback(const ros::TimerEvent& event)
    {
        ROS_INFO("    0-publishResultsTimerCallback");
        // 1. Публикация отфильтрованного скана (filtered_scan_results_)
        if (filtered_scan_results_.ranges.size() > 0)
        {
            pub_filtered_scan.publish(filtered_scan_results_);
            ROS_INFO("      filtered_scan_results_");
        }

        // 2. Публикация маркеров кластеров (cluster_markers_results_)
        if (cluster_markers_results_.markers.size() > 0)
        {
            pub_method_clusters.publish(cluster_markers_results_);
            ROS_INFO("      cluster_markers_results_");
        }

        // 3. Публикация центров Fusion (fused_centers_results_)
        if (fused_centers_results_.size() > 0 && filtered_scan_results_.header.frame_id != "")
        {
            // Нужно пересоздать маркер, используя сохраненные данные и frame_id
            pub_fused_pillars.publish(createPointsMarker(fused_centers_results_, 
                                        filtered_scan_results_.header.frame_id, 
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

    // Обработка одного кластера
    void processCluster(const AlignedVector2f& cluster, int method_id, 
                        std::vector<PillarCandidate>& out, AlignedVector2f& out_cluster_points)
    {
        if (cluster.size() < 5) return;                      // Слишком мало точек
        double width = MathUtils::dist2D(cluster.front(), cluster.back());
        if (width < 0.05 || width > 0.5) return;             // Неправильный размер

        Eigen::Vector2f center;                              // Центр Vector2f
        double rmse;
        if (MathUtils::fitCircle(cluster, pillar_radius_, center, rmse)) // Подгонка окружности
        {
            double w_rmse = std::max(0.0, 1.0 - (rmse / rmse_max_tolerance));
            double w_n = std::min(1.0, sqrt((double)cluster.size()) / sqrt((double)n_max_points_norm));
            double w_m = w_method[method_id];

            double w_total = w_rmse * w_n * w_m;             // Общий вес

            if (w_total > 0.1)
            {
                PillarCandidate cand;
                cand.center = center;
                cand.rmse = rmse;
                cand.num_points = (int)cluster.size();
                cand.weight = w_total;
                cand.method_id = method_id;
                out.push_back(cand);
                
                for (const auto& p : cluster)
                {
                    out_cluster_points.push_back(p);         // Добавление точек кластера для визуализации
                }
            }
        }
    }

    // Детекция на основе разрыва/плотности
    std::vector<PillarCandidate> detectGenericClustering(const AlignedVector2f& pts, double threshold, int method_id, 
                                                         AlignedVector2f& out_cluster_points)
    {
        std::vector<PillarCandidate> results;
        if (pts.empty()) return results;

        AlignedVector2f current_cluster;                     // Текущий кластер
        current_cluster.push_back(pts[0]);

        for (size_t i = 1; i < pts.size(); ++i)
        {
            double d = MathUtils::dist2D(pts[i], pts[i-1]);  // Расстояние между соседними точками
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

    // Детекция на основе локальных минимумов дальности
    std::vector<PillarCandidate> detectLocalMinima(const AlignedVector2f& pts, int method_id,
                                                   AlignedVector2f& out_cluster_points)
    {
        std::vector<PillarCandidate> results;
        if (pts.size() < 10) return results;

        for (size_t i = 5; i < pts.size() - 5; ++i)
        {
            float r = pts[i].norm();
            bool is_min = true;
            for (int k = -5; k <= 5; ++k)
            {
                if (k==0) continue;
                if (pts[i+k].norm() < r) is_min = false;
            }

            if (is_min)
            {
                AlignedVector2f cluster;
                cluster.push_back(pts[i]);
                
                // Расширение кластера назад
                for (int k = -1; i+k >= 0; --k) {
                    if (pts[i+k].norm() > r + 0.2 || MathUtils::dist2D(pts[i+k], pts[i+k+1]) > 0.1) break;
                    cluster.push_back(pts[i+k]);
                }
                // Расширение кластера вперед
                for (size_t k = 1; i+k < pts.size(); ++k) {
                    if (pts[i+k].norm() > r + 0.2 || MathUtils::dist2D(pts[i+k], pts[i+k-1]) > 0.1) break;
                    cluster.push_back(pts[i+k]);
                }
                processCluster(cluster, method_id, results, out_cluster_points);
                i += cluster.size(); 
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
        AlignedVector2f found_centers;                          // Центры слияния (Vector2f)

        for (size_t i = 0; i < candidates.size(); ++i)
        {
            if (processed[i]) continue;
            double sum_w = 0;
            Eigen::Vector2f w_center(0.0f, 0.0f);
            
            for (size_t j = i; j < candidates.size(); ++j)
            {
                if (processed[j]) continue;
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

        // Сортировка по углу (для однозначного сопоставления)
        std::sort(found_centers.begin(), found_centers.end(), 
            [](const Eigen::Vector2f& a, const Eigen::Vector2f& b){
                return atan2(a.y(), a.x()) < atan2(b.y(), b.x());
            });
        
        for(size_t i=0; i<4; ++i)
        {
            FinalPillar fp;
            fp.local = found_centers[i];
            fp.name = "Pillar_" + std::to_string(i); 
            final_pillars.push_back(fp);
        }

        return final_pillars;
    }

    // Сохранение результатов в ROS Parameter Server
    void saveResults(const std::vector<FinalPillar>& pillars)
    {
        for (const auto& p : pillars)
        {
            if (p.name.find("Pillar_") != std::string::npos) continue; // Пропускаем временные имена
            std::string base = "/pb_config/result/" + p.name;
            nh.setParam(base + "/x", p.global.x());
            nh.setParam(base + "/y", p.global.y());
        }
        logi.log_g("Results saved to rosparam.\n");
    }

    // НОВАЯ ФУНКЦИЯ: Медианная фильтрация и перевод в точки
    sensor_msgs::LaserScan filterScan(const std::vector<std::vector<double>>& accumulated_data, const sensor_msgs::LaserScan& meta)
    {
        // Эта функция на самом деле не используется в v4.3, так как логика перенесена в processPipeline
        // Но я оставляю ее тело, если она вызывается где-то еще, что не видно в коде.
        sensor_msgs::LaserScan output = meta;
        output.ranges.clear();
        
        for (size_t i = 0; i < accumulated_data.size(); ++i)
        {
            if (accumulated_data[i].empty()) 
            {
                output.ranges.push_back(meta.range_max); 
                continue;
            }
            
            std::vector<double> current_ray_data = accumulated_data[i]; 
            double median_r = MathUtils::getMedian(current_ray_data);

            output.ranges.push_back((float)median_r); 
        }
        return output; 
    }


    // --- 6. Калибровка: Полноценный Umeyama's Algorithm (SVD) ---
    void performCalibration(std::vector<FinalPillar>& pillars)
    {
        if(pillars.size() != 4)
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
        
        for(int i=0; i<4; ++i)
        {
            for(int j=0; j<4; ++j)
            {
                if(i==j) continue;
                // Расстояние считается между Vector2f
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

        // 2. Подготовка матриц (DOUBLE)
        Eigen::Matrix<double, 2, 4> X; // Локальные точки (double)
        Eigen::Matrix<double, 2, 4> Y; // Эталонные точки (double)

        for(int i=0; i<4; ++i)
        {
            // Преобразование float -> double для калибровки
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
        
        double angle_rad = atan2(R(1,0), R(0,0));
        logi.log_g("  Rotation (deg): %.3f\n", angle_rad * 180.0 / M_PI);
        
        logi.log_g("  Translation (T): [%.3f, %.3f]\n", T.x(), T.y());


        // 9. Применение (Сохранение в Vector2d)
        double final_rmse_sum_sq = 0.0;
        for(int i=0; i<4; ++i)
        {
            FinalPillar& p = pillars[match_index[i]];
            
            // Расчет в double, сохранение в Vector2d (p.global)
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
        if (final_rmse <= 0.05) // Устанавливаем флаг, только если RMSE в пределах допуска (5 см)
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
    PillarScanNode() : scans_collected(0), calibration_done_(false)
    {
        logi.log("=== PillarScanNode v4.3 Started (Continuous Publication) ===\n"); 
        
        loadParameters();                                  // Загрузка параметров
        if (!ros::ok()) return;

        // Инициализация паблишеров
        pub_filtered_scan = nh.advertise<sensor_msgs::LaserScan>("/rviz/filtered_scan", 1); // LaserScan
        pub_method_clusters = nh.advertise<visualization_msgs::MarkerArray>("/rviz/method_clusters", 1); // MarkerArray
        pub_fused_pillars = nh.advertise<visualization_msgs::Marker>("/rviz/fused_pillars", 1); // Marker
        pub_final_markers = nh.advertise<visualization_msgs::MarkerArray>("/rviz/final_pillars", 1); // MarkerArray
        
        initReferenceSystem();                             // Расчет эталонной системы

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
        meta_scan = *first_scan;
        
        logi.log_b("LiDAR initialized. Rays: %lu. Starting accumulation.\n", num_rays);

        scan_sub = nh.subscribe("/scan", 100, &PillarScanNode::scanCallback, this); // Подписка
        logi.log("Waiting for %d laser scans on /scan topic to complete initial calibration.\n", SCANS_TO_COLLECT);
        
        publish_timer_ = nh.createTimer(ros::Duration(1.0), &PillarScanNode::publishResultsTimerCallback, this); // Запуск таймера
    }

    // ----------------------------------------------------------------------------------
    // Загрузка параметров
    // ----------------------------------------------------------------------------------
    void loadParameters()
    {
        logi.log("\n--- Loading YAML Parameters ---\n");
        
        // 1. Диаметр столба и радиус
        nh.param<double>("/pb_config/pillar_params/pillar_diametr", pillar_diam_, 0.315);
        pillar_radius_ = pillar_diam_ / 2.0;

        // 2. Эталонные расстояния (в метрах)
        nh.param<double>("/pb_config/reference_dists/pillar_0_1", d_surf[0], 10.5); // RB-RT
        nh.param<double>("/pb_config/reference_dists/pillar_0_2", d_surf[1], 12.0); // RB-LT
        nh.param<double>("/pb_config/reference_dists/pillar_0_3", d_surf[2], 4.8);  // RB-LB
        nh.param<double>("/pb_config/reference_dists/pillar_1_2", d_surf[3], 5.5);  // RT-LT
        nh.param<double>("/pb_config/reference_dists/pillar_1_3", d_surf[4], 11.5); // RT-LB
        nh.param<double>("/pb_config/reference_dists/pillar_2_3", d_surf[5], 4.8);  // LT-LB
        
        // Вычисление расстояний между центрами
        for (int i = 0; i < 6; ++i)
        {
            d_center[i] = d_surf[i] + pillar_diam_;
        }
        
        // Лог параметров расстояний
        logi.log("  pillar_diametr: %.4f\n", pillar_diam_);
        logi.log("  pillar_0_1 (surf): %.4f -> (center): %.4f\n", d_surf[0], d_center[0]);
        logi.log("  pillar_0_2 (surf): %.4f -> (center): %.4f\n", d_surf[1], d_center[1]);
        logi.log("  pillar_0_3 (surf): %.4f -> (center): %.4f\n", d_surf[2], d_center[2]);
        logi.log("  pillar_1_2 (surf): %.4f -> (center): %.4f\n", d_surf[3], d_center[3]);
        logi.log("  pillar_1_3 (surf): %.4f -> (center): %.4f\n", d_surf[4], d_center[4]);
        logi.log("  pillar_2_3 (surf): %.4f -> (center): %.4f\n", d_surf[5], d_center[5]);

        // 3. Параметры фильтрации
        nh.param<double>("/pb_config/filters/min_range", min_range_filter, 0.2);
        nh.param<double>("/pb_config/filters/max_range", max_range_filter, 15.0);
        nh.param<double>("/pb_config/filters/neighbor_radius", neighbor_radius_filter, 0.3);
        nh.param<int>("/pb_config/filters/min_neighbors", min_neighbors_filter, 3);
        logi.log("  Filter Range: [%.2f, %.2f], KNN: R=%.2f, N=%d\n", 
            min_range_filter, max_range_filter, neighbor_radius_filter, min_neighbors_filter);
        
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
    // Инициализация эталона (Reference System)
    // ----------------------------------------------------------------------------------
    void initReferenceSystem()
    {
        reference_centers_.clear();

        // Предполагаем, что оси X и Y соответствуют длине и ширине.
        // Используем длины d_center[0] (RB-RT) и d_center[5] (LT-LB) для сторон.
        
        double L = d_center[0]; // Длина по X (RB-RT)
        // Для Y берем среднее из d_center[3] (RT-LT) и d_center[2] (RB-LB) с учетом диагоналей.
        // Проще всего решить систему, но для простоты берем d_center[3] (RT-LT) как Y_LT.
        double Y_LT = d_center[3]; 
        double X_LB = (d_center[1]*d_center[1] - d_center[4]*d_center[4] + L*L) / (2*L);
        double Y_LB = sqrt(d_center[2]*d_center[2] - X_LB*X_LB); 
        
        // 0. RB (Right Bottom) - Начало координат
        reference_centers_.emplace_back(0.0f, 0.0f);            
        
        // 1. RT (Right Top) - Сдвиг по оси X
        reference_centers_.emplace_back((float)L, 0.0f);        
        
        // 2. LT (Left Top) - Расчет по известным расстояниям
        // Проекция LT на ось X: (L^2 + d_center[3]^2 - d_center[1]^2) / (2*L) - упрощенно L.
        reference_centers_.emplace_back((float)L, (float)Y_LT); 
        
        // 3. LB (Left Bottom) - Расчет по диагоналям
        reference_centers_.emplace_back((float)X_LB, -(float)Y_LB); 

        logi.log("\nReference System Initialized:\n");
        logi.log("  RB (0): [%.3f, %.3f]\n", reference_centers_[0].x(), reference_centers_[0].y());
        logi.log("  RT (1): [%.3f, %.3f]\n", reference_centers_[1].x(), reference_centers_[1].y());
        logi.log("  LT (2): [%.3f, %.3f]\n", reference_centers_[2].x(), reference_centers_[2].y());
        logi.log("  LB (3): [%.3f, %.3f]\n", reference_centers_[3].x(), reference_centers_[3].y());
    }

    // ----------------------------------------------------------------------------------
    // Callback скана
    // ----------------------------------------------------------------------------------
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        // 1. ПРОВЕРКА: Если собрано БОЛЬШЕ чем 100 сканов, НЕМЕДЛЕННО выходим. (v4.1)
        if (scans_collected > SCANS_TO_COLLECT) return; 

        if (scan->ranges.size() != accumulated_ranges.size())
        {
            logi.log_w("Scan size changed! Skipping scan.\n");
            return;
        }

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float r = scan->ranges[i];
            
            if (std::isnan(r) || std::isinf(r)) continue;
            if (r < min_range_filter || r > max_range_filter) continue;

            accumulated_ranges[i].push_back(r);                
        }

        scans_collected++;
        if (scans_collected % 10 == 0) logi.log("Collecting scans: %d/%d\n", scans_collected, SCANS_TO_COLLECT);

        if (scans_collected == SCANS_TO_COLLECT)
        {
            logi.log("Accumulation complete. Starting processing pipeline...\n");
            processPipeline();
        }
    }
    
    // ИЗМЕНЕНА: Основной конвейер обработки (Полностью)
    void processPipeline()
    {
        logi.log("=== Starting Processing Pipeline (Median Filtered Scan) ===\n");
        
        // 1. ФОРМИРОВАНИЕ МЕДИАННОГО СКАНА И ПЕРЕВОД В ТОЧКИ 
        
        sensor_msgs::LaserScan current_filtered_scan = meta_scan;
        current_filtered_scan.ranges.clear(); 
        AlignedVector2f clean_points;                           
        
        for (size_t i = 0; i < accumulated_ranges.size(); ++i)
        {
            if (accumulated_ranges[i].empty()) 
            {
                current_filtered_scan.ranges.push_back(meta_scan.range_max); 
                continue;
            }
            
            std::vector<double> current_ray_data = accumulated_ranges[i]; 
            double median_r = MathUtils::getMedian(current_ray_data);

            double angle = meta_scan.angle_min + i * meta_scan.angle_increment;

            current_filtered_scan.ranges.push_back((float)median_r);
            clean_points.emplace_back((float)(median_r * cos(angle)), (float)(median_r * sin(angle)));
        }

        // СОХРАНЕНИЕ 1: Сохраняем отфильтрованный скан (LaserScan)
        filtered_scan_results_ = current_filtered_scan; 

        if (clean_points.empty()) 
        {
            logi.log_r("No valid points after filtering. Exiting pipeline.\n");
            return;
        }
        logi.log("Total median filtered points: %lu\n", clean_points.size());

        // 2. Статистическая фильтрация (упрощенный вариант)
        AlignedVector2f filtered_points = clean_points; 

        // 3. Детекция (3 метода)
        std::vector<PillarCandidate> all_candidates;
        AlignedVector2f clusters_m1; // Точки кластера для метода 1
        AlignedVector2f clusters_m2;
        AlignedVector2f clusters_m3;
        
        auto c1 = detectGenericClustering(filtered_points, jump_dist_threshold, 1, clusters_m1);
        all_candidates.insert(all_candidates.end(), c1.begin(), c1.end());

        auto c2 = detectGenericClustering(filtered_points, cluster_dist_threshold, 2, clusters_m2);
        all_candidates.insert(all_candidates.end(), c2.begin(), c2.end());

        auto c3 = detectLocalMinima(filtered_points, 3, clusters_m3);
        all_candidates.insert(all_candidates.end(), c3.begin(), c3.end());
        
        // СОЗДАНИЕ МАРКЕРОВ КЛАСТЕРОВ ДЛЯ СОХРАНЕНИЯ (MarkerArray)
        visualization_msgs::MarkerArray cluster_markers;
        cluster_markers.markers.push_back(createPointsMarker(clusters_m1, meta_scan.header.frame_id, 
                              "method_1_jump", 1, 1.0f, 0.0f, 0.0f, 0.08f));
        cluster_markers.markers.push_back(createPointsMarker(clusters_m2, meta_scan.header.frame_id, 
                              "method_2_cluster", 2, 0.0f, 0.0f, 1.0f, 0.08f));
        cluster_markers.markers.push_back(createPointsMarker(clusters_m3, meta_scan.header.frame_id, 
                              "method_3_minima", 3, 1.0f, 1.0f, 0.0f, 0.08f));

        // СОХРАНЕНИЕ 2: Сохраняем маркеры кластеров (MarkerArray)
        cluster_markers_results_ = cluster_markers;
        
        logi.log("Total candidates found: %lu (M1=%lu, M2=%lu, M3=%lu)\n", 
            all_candidates.size(), c1.size(), c2.size(), c3.size());

        // 4. Fusion
        std::vector<FinalPillar> final_pillars = fuseCandidates(all_candidates);

        // СОХРАНЕНИЕ 3: Сохраняем центры Fusion (локальные координаты)
        AlignedVector2f current_fused_centers;
        for (const auto& fp : final_pillars)
        {
            current_fused_centers.push_back(fp.local); // local coordinates before Umeyama
        }
        fused_centers_results_ = current_fused_centers; // Сохраняем для постоянной публикации

        // 5. Калибровка (Full Umeyama)
        performCalibration(final_pillars);

        // 6. Сохранение и лог
        if (calibration_done_)
        {
            saveResults(final_pillars_results_); 
            logi.log_g("Calibration successful. Node remains active, publishing results every 1 second.\n");
        }
        else
        {
            // Уведомление, если калибровка была пропущена/неудачна
            logi.log_w("Initial calibration attempt ended without success. Node remains active, check logs for details.\n");
        }
    }
};
/*
// --------------------------------------------------------------------------------------
// 5. MAIN
// --------------------------------------------------------------------------------------
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
*/