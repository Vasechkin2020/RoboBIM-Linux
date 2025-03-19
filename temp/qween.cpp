#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cmath>

// Структура для хранения точки
struct Point
{
    double x, y;
    int cluster_id; // Идентификатор кластера (-1 для шума)
    bool visited;

    Point(double x_val, double y_val)
        : x(x_val), y(y_val), cluster_id(-1), visited(false) {}
};

// Функция для вычисления евклидова расстояния между двумя точками
double euclidean_distance(const Point &p1, const Point &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// Функция для поиска ε-окрестности точки
std::vector<int> find_neighbors(const std::vector<Point> &points, int index, double eps)
{
    std::vector<int> neighbors;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (euclidean_distance(points[index], points[i]) <= eps)
        {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

// Рекурсивная функция для расширения кластера
void expand_cluster(std::vector<Point> &points, int index, int cluster_id, double eps, int min_pts)
{
    points[index].cluster_id = cluster_id;
    points[index].visited = true;

    std::vector<int> neighbors = find_neighbors(points, index, eps);

    if (neighbors.size() >= min_pts)
    {
        for (int neighbor_index : neighbors)
        {
            if (!points[neighbor_index].visited)
            {
                points[neighbor_index].visited = true;
                std::vector<int> new_neighbors = find_neighbors(points, neighbor_index, eps);
                if (new_neighbors.size() >= min_pts && new_neighbors.size() < 101)
                {
                    neighbors.insert(neighbors.end(), new_neighbors.begin(), new_neighbors.end());
                }
            }
            if (points[neighbor_index].cluster_id == -1)
            {
                points[neighbor_index].cluster_id = cluster_id;
            }
        }
    }
}

// Основной метод DBSCAN
void dbscan(std::vector<Point> &points, double eps, int min_pts)
{
    int cluster_id = 0;

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (!points[i].visited)
        {
            points[i].visited = true;

            std::vector<int> neighbors = find_neighbors(points, i, eps);

            if (neighbors.size() < min_pts)
            {
                // Точка является шумом
                points[i].cluster_id = -1;
            }
            else
            {
                // Создаем новый кластер
                expand_cluster(points, i, cluster_id, eps, min_pts);
                ++cluster_id;
            }
        }
    }
    ROS_INFO("Qween Count clusters %i: ", cluster_id);
}

// Преобразование данных LaserScan в декартовы координаты
std::vector<Point> convert_to_cartesian(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    std::vector<Point> points;

    double angle_min = scan_msg->angle_min;
    double angle_increment = scan_msg->angle_increment;
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
        double range = scan_msg->ranges[i];
        if (range > scan_msg->range_min && range < scan_msg->range_max)
        { // Пропускаем невалидные значения
            double angle = angle_min + i * angle_increment;
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            points.emplace_back(x, y);
            // if (i<10)
            // ROS_INFO("Qween Point %zu: angle = %6.3f x = %6.3f, y = %6.3f", i, angle, x, y);
        }
    }

    return points;
}

// Публикация маркеров для RViz
void publish_markers(ros::Publisher &marker_pub, const std::vector<Point> &points)
{
    visualization_msgs::MarkerArray marker_array;

    int cluster_id = 0;
    for (const auto &point : points)
    {
        if (point.cluster_id != -1)
        { // Пропускаем шумовые точки
            visualization_msgs::Marker marker;
            marker.header.frame_id = "laser";
            marker.header.stamp = ros::Time::now();
            marker.ns = "clustersQween";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = 0.0;

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // Разные цвета для разных кластеров
            float hue = static_cast<float>(point.cluster_id) / 10.0f;
            marker.color.r = std::fabs(std::cos(hue));
            marker.color.g = std::fabs(std::sin(hue));
            marker.color.b = 0.5f;
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }
    }

    marker_pub.publish(marker_array);
}

sensor_msgs::LaserScan::ConstPtr last_scan_msg;

// Основная функция обработки данных лидара
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
   last_scan_msg = scan;

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_dbscan_node");
    ros::NodeHandle nh;

    // Параметры DBSCAN
    double eps;
    int min_pts;

    // Загрузка параметров из параметрического сервера ROS
    if (!nh.getParam("eps", eps))
    {
        ROS_ERROR("Parameter 'eps' not set. Using default value.");
        eps = 0.07; // Значение по умолчанию
    }

    if (!nh.getParam("min_pts", min_pts))
    {
        ROS_ERROR("Parameter 'min_pts' not set. Using default value.");
        min_pts = 11; // Значение по умолчанию
    }

    // Подписка на топик /scan
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, [](const sensor_msgs::LaserScan::ConstPtr& msg) {
    //     // Сохраняем последнее сообщение
    //     static sensor_msgs::LaserScan::ConstPtr last_scan_msg = msg;
    //     last_scan_msg = msg;
    // });
// ros::Subscriber sub = nh.subscribe("/scan", 1, [](const sensor_msgs::LaserScan::ConstPtr& msg) {
//     static sensor_msgs::LaserScan::ConstPtr last_scan_msg;
//     last_scan_msg = msg;
// });

    ros::Subscriber sub = nh.subscribe("/scan", 1, lidarCallback);

    // Публикатор для маркеров
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);

    // Цикл с фиксированной частотой
    ros::Rate rate(3.0); // 1 Гц
    while (ros::ok())
    {
        printf("qween\n");
        // Получаем последнее сообщение LaserScan
        if (!last_scan_msg)
        {
            ROS_WARN("No LaserScan data received yet.");
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        // Преобразуем данные в декартовы координаты
        std::vector<Point> points = convert_to_cartesian(last_scan_msg);

        // Выполняем DBSCAN
        dbscan(points, eps, min_pts);

        // Публикуем маркеры для RViz
        publish_markers(marker_pub, points);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}