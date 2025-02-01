#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <vector>
#include <mutex>

// Глобальные переменные для обмена данными между коллбэками
std::mutex scan_mutex;
sensor_msgs::LaserScan::ConstPtr last_scan;

// Параметры алгоритма
struct {
    float cluster_threshold = 0.3;
    int min_cluster_size = 7;
    int ransac_iterations = 150;
    float distance_threshold = 0.07;
    float max_distance = 20.0;
    float expected_radius = 0.1575; // 315mm diameter
} params;

// Параметры визуализации
struct {
    float lifetime = 1.1;
    std::vector<float> color = {0.0, 1.0, 0.0};
    float alpha = 0.7;
} viz;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::lock_guard<std::mutex> lock(scan_mutex);
    last_scan = scan;
}

std::vector<geometry_msgs::Point> processScan(const sensor_msgs::LaserScan& scan) {
    // Преобразование в декартовы координаты
    std::vector<geometry_msgs::Point> points;
    for(size_t i = 0; i < scan.ranges.size(); ++i) {
        const float range = scan.ranges[i];
        if(!std::isfinite(range) || range > params.max_distance) continue;
        
        const float angle = scan.angle_min + i * scan.angle_increment;
        geometry_msgs::Point p;
        p.x = range * cos(angle);
        p.y = range * sin(angle);
        points.push_back(p);
    }

    // Кластеризация
    std::vector<std::vector<geometry_msgs::Point>> clusters;
    std::vector<bool> processed(points.size(), false);
    
    for(size_t i = 0; i < points.size(); ++i) {
        if(processed[i]) continue;
        
        std::vector<geometry_msgs::Point> cluster;
        std::vector<size_t> queue = {i};
        processed[i] = true;
        
        while(!queue.empty()) {
            size_t idx = queue.back();
            queue.pop_back();
            cluster.push_back(points[idx]);
            
            for(size_t j = 0; j < points.size(); ++j) {
                if(processed[j]) continue;
                
                float dx = points[idx].x - points[j].x;
                float dy = points[idx].y - points[j].y;
                if(hypotf(dx, dy) < params.cluster_threshold) {
                    processed[j] = true;
                    queue.push_back(j);
                }
            }
        }
        
        if(cluster.size() >= params.min_cluster_size) {
            clusters.push_back(cluster);
        }
    }

    // Поиск столбов
    std::vector<geometry_msgs::Point> pillars;
    for(const auto& cluster : clusters) {
        // Простейший алгоритм поиска центра
        geometry_msgs::Point center;
        float sum_x = 0, sum_y = 0;
        for(const auto& p : cluster) {
            sum_x += p.x;
            sum_y += p.y;
        }
        center.x = sum_x / cluster.size();
        center.y = sum_y / cluster.size();
        
        // Простая проверка радиуса
        float avg_dist = 0;
        for(const auto& p : cluster) {
            avg_dist += hypotf(p.x - center.x, p.y - center.y);
        }
        avg_dist /= cluster.size();
        
        if(fabs(avg_dist - params.expected_radius) < 0.06*params.expected_radius) {
            pillars.push_back(center);
        }
    }
    
    return pillars;
}

void publishMarkers(const std::vector<geometry_msgs::Point>& pillars, 
                   ros::Publisher& pub, 
                   const std::string& frame_id) {
    visualization_msgs::MarkerArray markers;
    
    // Очистка старых маркеров
    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    // Создание новых маркеров
    for(size_t i = 0; i < pillars.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "pillars";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = pillars[i];
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = 2*params.expected_radius;
        marker.scale.z = 1.0;
        marker.color.r = viz.color[0];
        marker.color.g = viz.color[1];
        marker.color.b = viz.color[2];
        marker.color.a = viz.alpha;
        marker.lifetime = ros::Duration(viz.lifetime);
        
        markers.markers.push_back(marker);
    }
    
    pub.publish(markers);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pillar_detector");
    ros::NodeHandle nh("~");
    
    // Загрузка параметров
    nh.param("cluster_threshold", params.cluster_threshold, 0.3f);
    nh.param("min_cluster_size", params.min_cluster_size, 7);
    nh.param("ransac_iterations", params.ransac_iterations, 150);
    nh.param("distance_threshold", params.distance_threshold, 0.07f);
    nh.param("max_distance", params.max_distance, 20.0f);
    nh.param("marker_lifetime", viz.lifetime, 1.1f);
    nh.param("marker_color", viz.color, std::vector<float>{0.0, 1.0, 0.0});
    nh.param("marker_alpha", viz.alpha, 0.7f);

    // Инициализация подписчиков и издателей
    ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scanCallback);
    ros::Publisher markers_pub = nh.advertise<visualization_msgs::MarkerArray>("pillar_markers", 1);
    
    ros::Rate rate(1); // 1 Hz
    
    while(ros::ok()) {
        // Получение данных скана
        sensor_msgs::LaserScan::ConstPtr scan;
        {
            std::lock_guard<std::mutex> lock(scan_mutex);
            scan = last_scan;
        }
        
        if(scan) {
            // Обработка и публикация
            auto pillars = processScan(*scan);
            publishMarkers(pillars, markers_pub, scan->header.frame_id);
        }
        
        // Обработка коллбэков и ожидание
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}