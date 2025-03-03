#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>
#include <algorithm>

// Структура для хранения точки
struct Point {
    double x, y;
};

// Глобальные переменные
ros::Publisher marker_pub;
ros::Publisher cluster_pub;
double COLUMN_RADIUS = 0.1575;  // Ожидаемый радиус столбов (315 мм / 2)
double DIST_THRESHOLD = 0.07; // Порог расстояния для кластеризации
const double MAX_DETECTION_DISTANCE = 20.0; // Максимальная дистанция обнаружения

// Функция преобразования полярных координат в декартовы
std::vector<Point> convertToCartesian(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<Point> points;
    double angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r = scan->ranges[i];
        if (r < scan->range_max && r > scan->range_min && r <= MAX_DETECTION_DISTANCE) { // Фильтруем некорректные данные
            points.push_back({r * cos(angle), r * sin(angle)});
        }
        angle += scan->angle_increment;
    }
    return points;
}

// Функция кластеризации (DBSCAN-подобный алгоритм)
std::vector<std::vector<Point>> clusterize(const std::vector<Point>& points) {
    std::vector<std::vector<Point>> clusters;
    std::vector<bool> visited(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) continue;

        std::vector<Point> cluster;
        cluster.push_back(points[i]);
        visited[i] = true;

        for (size_t j = i + 1; j < points.size(); ++j) {
            if (visited[j]) continue;
            double dist = std::hypot(points[j].x - points[i].x, points[j].y - points[i].y);
            if (dist < DIST_THRESHOLD) {
                cluster.push_back(points[j]);
                visited[j] = true;
            }
        }

        if (cluster.size() > 11 && cluster.size() < 101) // Фильтруем маленькие шумовые кластеры
        {
            clusters.push_back(cluster);
            ROS_INFO("    clusters size %i: ", cluster.size());
        }
    }
    ROS_INFO("ChatGPT Count clusters %i: ", clusters.size());
    return clusters;
}

// Функция подгонки окружности методом наименьших квадратов
bool fitCircle(const std::vector<Point>& cluster, Point& center, double& radius) {
    double sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, sumXY = 0, sumR = 0;
    int n = cluster.size();
    
    if (n < 5) return false;
    
    for (const auto& p : cluster) {
        sumX += p.x;
        sumY += p.y;
        sumX2 += p.x * p.x;
        sumY2 += p.y * p.y;
        sumXY += p.x * p.y;
    }

    double C = n * sumX2 - sumX * sumX;
    double D = n * sumXY - sumX * sumY;
    double E = n * sumY2 - sumY * sumY;
    double G = 0.5 * (n * (sumX2 + sumY2) - (sumX * sumX + sumY * sumY));

    double denom = C * E - D * D;
    if (std::abs(denom) < 1e-6) return false; // Избегаем деления на 0
    
    double a = (G * E - D * sumXY) / denom;
    double b = (C * G - D * sumXY) / denom;
    
    center.x = a;
    center.y = b;
    
    for (const auto& p : cluster) {
        sumR += std::hypot(p.x - a, p.y - b);
    }
    
    radius = sumR / n;
    return std::abs(radius - COLUMN_RADIUS) < COLUMN_RADIUS * 0.1; // Проверяем радиус с допуском 10%
}

// Визуализация найденных столбов
void publishMarkers(const std::vector<Point>& centers) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "columns";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0; // Установка кватерниона

    for (const auto& c : centers) {
        geometry_msgs::Point p;
        p.x = c.x;
        p.y = c.y;
        p.z = 0.0;
        marker.points.push_back(p);
    }

    marker_pub.publish(marker);
}

// Визуализация кластеров
void publishClusters(const std::vector<std::vector<Point>>& clusters) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "clusters";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;

    for (const auto& cluster : clusters) {
        for (const auto& point : cluster) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }
    }
    cluster_pub.publish(marker);
}

// Основная функция обработки данных лидара
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<Point> points = convertToCartesian(scan);
    std::vector<std::vector<Point>> clusters = clusterize(points);

    publishClusters(clusters);
    std::vector<Point> detected_columns;
    for (const auto& cluster : clusters) {
        Point center;
        double radius;
        if (fitCircle(cluster, center, radius)) {
            detected_columns.push_back(center);
            ROS_INFO("Column detected at (%.2f, %.2f) with radius %.2f", center.x, center.y, radius);
        }
    }
    publishMarkers(detected_columns);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_column_detector");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/scan", 1, lidarCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("detected_columns", 1);
    cluster_pub = nh.advertise<visualization_msgs::Marker>("detected_clusters", 1);

    ros::Rate rate(1); // 1 раз в секунду
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
