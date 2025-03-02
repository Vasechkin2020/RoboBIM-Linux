#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <cmath>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <signal.h>

// Параметры задачи
const double PILLAR_RADIUS = 0.1575; // Радиус столба (0,315 м / 2)
const double RADIUS_TOLERANCE = 0.02; // Допуск на радиус (±2 см)
const double MIN_ARC_ANGLE = 0.5236; // Минимальная дуга (30° в радианах)
const double CLUSTER_TOLERANCE = 0.2; // Максимальное расстояние между точками кластера (м)
const int MIN_POINTS_PER_CLUSTER = 5; // Минимальное число точек в кластере

// Структура для представления окружности
struct Circle {
    double x_c, y_c; // Координаты центра
    double radius;   // Радиус
};

// Глобальный флаг для обработки Ctrl+C
volatile sig_atomic_t shutdown_flag = 0;

void signalHandler(int sig) {
    shutdown_flag = 1;
    ros::shutdown();
}

// Класс для обработки данных лидара и детекции столбов
class PillarDetector {
public:
    PillarDetector() : nh_("~") {
        // Подписка на данные лидара
        scan_sub_ = nh_.subscribe("/scan", 1, &PillarDetector::scanCallback, this);
        // Публикация маркеров для визуализации
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pillar_markers", 1);
        // Таймер для визуализации (1 Гц)
        viz_timer_ = nh_.createTimer(ros::Duration(1.0), &PillarDetector::visualizeCallback, this);
        
        ROS_INFO("Pillar Detector initialized. Press Ctrl+C to exit.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher marker_pub_;
    ros::Timer viz_timer_;
    std::vector<Circle> detected_pillars_; // Хранение найденных столбов

    // Callback для обработки данных лидара
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Конвертация LaserScan в облако точек
        pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            if (range >= scan->range_min && range <= scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                pcl::PointXY point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                cloud->push_back(point);
            }
        }

        // Сегментация и детекция столбов
        std::vector<std::vector<pcl::PointXY>> clusters = segmentClusters(cloud);
        detectPillars(clusters);
    }

    // Сегментация облака точек на кластеры
    std::vector<std::vector<pcl::PointXY>> segmentClusters(const pcl::PointCloud<pcl::PointXY>::Ptr& cloud) {
        std::vector<std::vector<pcl::PointXY>> clusters;

        // Создание KD-дерева для кластеризации
        pcl::search::KdTree<pcl::PointXY>::Ptr tree(new pcl::search::KdTree<pcl::PointXY>);
        tree->setInputCloud(cloud);

        // Настройка евклидовой кластеризации
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXY> ec;
        ec.setClusterTolerance(CLUSTER_TOLERANCE);
        ec.setMinClusterSize(MIN_POINTS_PER_CLUSTER);
        ec.setMaxClusterSize(1000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // Извлечение кластеров
        for (const auto& indices : cluster_indices) {
            std::vector<pcl::PointXY> cluster;
            for (const auto& idx : indices.indices) {
                cluster.push_back(cloud->points[idx]);
            }
            clusters.push_back(cluster);
        }

        return clusters;
    }

    // RANSAC для подгонки окружности
    Circle fitCircleRANSAC(const std::vector<pcl::PointXY>& points, int max_iterations = 100) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, points.size() - 1);

        Circle best_circle{0, 0, 0};
        int best_inliers = 0;

        for (int iter = 0; iter < max_iterations; ++iter) {
            int idx1 = dis(gen), idx2 = dis(gen), idx3 = dis(gen);
            if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) continue;

            const auto& p1 = points[idx1];
            const auto& p2 = points[idx2];
            const auto& p3 = points[idx3];

            double d = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
            if (fabs(d) < 1e-6) continue;

            double x_c = ((p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) +
                          (p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) +
                          (p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y)) / d;
            double y_c = ((p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) +
                          (p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) +
                          (p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x)) / d;
            double radius = sqrt((p1.x - x_c) * (p1.x - x_c) + (p1.y - y_c) * (p1.y - y_c));

            int inliers = 0;
            for (const auto& p : points) {
                double dist = fabs(sqrt((p.x - x_c) * (p.x - x_c) + (p.y - y_c) * (p.y - y_c)) - radius);
                if (dist < RADIUS_TOLERANCE) inliers++;
            }

            if (inliers > best_inliers) {
                best_inliers = inliers;
                best_circle = {x_c, y_c, radius};
            }
        }

        return best_circle;
    }

    // Детекция столбов (обновление списка)
    void detectPillars(const std::vector<std::vector<pcl::PointXY>>& clusters) {
        detected_pillars_.clear(); // Очистка предыдущих результатов

        for (const auto& cluster : clusters) {
            // Проверка минимального углового охвата
            double min_theta = atan2(cluster.front().y, cluster.front().x);
            double max_theta = min_theta;
            for (const auto& p : cluster) {
                double theta = atan2(p.y, p.x);
                min_theta = std::min(min_theta, theta);
                max_theta = std::max(max_theta, theta);
            }
            if (max_theta - min_theta < MIN_ARC_ANGLE) continue;

            // Подгонка окружности
            Circle circle = fitCircleRANSAC(cluster);
            if (fabs(circle.radius - PILLAR_RADIUS) > RADIUS_TOLERANCE) continue;

            // Добавление столба в список
            detected_pillars_.push_back(circle);

            // Логирование
            double distance = sqrt(circle.x_c * circle.x_c + circle.y_c * circle.y_c);
            double direction = atan2(circle.y_c, circle.x_c);
            ROS_INFO("Pillar detected: Distance = %.2f m, Direction = %.2f rad (%.1f deg), Radius = %.3f m",
                     distance, direction, direction * 180 / M_PI, circle.radius);
        }
    }

    // Callback для визуализации (1 раз в секунду)
    void visualizeCallback(const ros::TimerEvent&) {
        if (detected_pillars_.empty()) return;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "pillars";
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.315; // Диаметр столба
        marker.scale.y = 0.315;
        marker.scale.z = 0.315;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.id = 0;

        for (const auto& pillar : detected_pillars_) {
            geometry_msgs::Point point;
            point.x = pillar.x_c;
            point.y = pillar.y_c;
            point.z = 0.0;
            marker.points.push_back(point);
        }

        marker_pub_.publish(marker);
        ROS_INFO("Published %lu pillars", detected_pillars_.size());
    }
};

int main(int argc, char** argv) {
    // Настройка обработки Ctrl+C
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "pillar_detector");
    PillarDetector detector;

    // Цикл ROS с обработкой прерывания
    ros::Rate rate(10); // 10 Гц для основного цикла
    while (ros::ok() && !shutdown_flag) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Shutting down Pillar Detector");
    return 0;
}