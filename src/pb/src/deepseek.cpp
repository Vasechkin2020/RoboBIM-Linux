#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <vector>
#include <mutex>

// Глобальные переменные для обмена данными между коллбэками
std::mutex scan_mutex;                      // Мьютекс для потокобезопасного доступа к данным сканирования
sensor_msgs::LaserScan::ConstPtr last_scan; // Последние данные лазерного сканирования

// Параметры алгоритма
struct
{
    float cluster_threshold = 0.07;   // Максимальное расстояние между точками для объединения в кластер
    int min_cluster_size = 11;        // Минимальное количество точек в кластере
    int max_cluster_size = 101;        // Максимальное количество точек в кластере
    
    // int ransac_iterations = 150;     // Количество итераций RANSAC (не используется в текущей реализации)
    // float distance_threshold = 0.07; // Пороговое расстояние для RANSAC (не используется)
    float max_distance = 20.0;       // Максимальное расстояние для учета точек сканирования
    float expected_radius = 0.1575;  // Ожидаемый радиус столба (диаметр 315 мм)
} params;

// Параметры визуализации
struct
{
    float lifetime = 1.1;                       // Время жизни маркеров
    std::vector<float> color = {0.0, 1.0, 0.0}; // Цвет маркеров (по умолчанию зеленый)
    float alpha = 0.7;                          // Прозрачность маркеров
} viz;

// Коллбэк для получения данных лазерного сканирования
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    std::lock_guard<std::mutex> lock(scan_mutex); // Блокировка мьютекса
    last_scan = scan;                             // Сохранение данных сканирования
}

// Функция для обработки данных сканирования
std::pair<std::vector<geometry_msgs::Point>, std::vector<std::vector<geometry_msgs::Point>>> processScan(const sensor_msgs::LaserScan &scan)
{
    // Преобразование данных сканирования из полярных координат в декартовы
    std::vector<geometry_msgs::Point> points;
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const float range = scan.ranges[i];                       // Расстояние до точки
        if (!std::isfinite(range) || range > params.max_distance) // Пропуск невалидных точек
            continue;

        const float angle = scan.angle_min + i * scan.angle_increment; // Угол точки
        geometry_msgs::Point p;
        p.x = range * cos(angle); // Преобразование в декартовы координаты
        p.y = range * sin(angle);
        points.push_back(p);
    }

    // Кластеризация точек
    std::vector<std::vector<geometry_msgs::Point>> clusters;
    std::vector<bool> processed(points.size(), false); // Массив для отметки обработанных точек

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (processed[i]) // Если точка уже обработана, пропустить
            continue;

        std::vector<geometry_msgs::Point> cluster; // Новый кластер
        std::vector<size_t> queue = {i};           // Очередь для обработки точек
        processed[i] = true;                       // Отметить точку как обработанную

        while (!queue.empty())
        {
            size_t idx = queue.back(); // Берем последнюю точку из очереди
            queue.pop_back();
            cluster.push_back(points[idx]); // Добавляем точку в кластер

            // Поиск соседних точек
            for (size_t j = 0; j < points.size(); ++j)
            {
                if (processed[j]) // Если точка уже обработана, пропустить
                    continue;

                float dx = points[idx].x - points[j].x;        // Разница по X
                float dy = points[idx].y - points[j].y;        // Разница по Y
                if (hypotf(dx, dy) < params.cluster_threshold) // Если расстояние меньше порога
                {
                    processed[j] = true; // Отметить точку как обработанную
                    queue.push_back(j);  // Добавить точку в очередь
                }
            }
        }

        if (cluster.size() >= params.min_cluster_size && cluster.size() <  params.max_cluster_size) // Если кластер достаточно большой
        {
            clusters.push_back(cluster); // Добавить кластер в список
            ROS_INFO("    clusters size %i: ", cluster.size());
        }
    }
    ROS_INFO("DeepSeek Count clusters %i: ", clusters.size());
    // Поиск столбов в кластерах
    std::vector<geometry_msgs::Point> pillars;
    for (const auto &cluster : clusters)
    {
        // Вычисление центра кластера
        geometry_msgs::Point center;
        float sum_x = 0, sum_y = 0;
        for (const auto &p : cluster)
        {
            sum_x += p.x;
            sum_y += p.y;
        }
        center.x = sum_x / cluster.size(); // Среднее по X
        center.y = sum_y / cluster.size(); // Среднее по Y

        // Проверка радиуса кластера
        float avg_dist = 0;
        for (const auto &p : cluster)
        {
            avg_dist += hypotf(p.x - center.x, p.y - center.y); // Расстояние от центра до точки
        }
        avg_dist /= cluster.size(); // Среднее расстояние

        // Если среднее расстояние близко к ожидаемому радиусу, считаем это столбом
        if (fabs(avg_dist - params.expected_radius) < 0.06 * params.expected_radius)
        {
            pillars.push_back(center); // Добавить центр в список столбов
        }
    }

    return {pillars, clusters}; // Возврат центров столбов и кластеров
}

// Функция для публикации маркеров столбов
void publishMarkers(const std::vector<geometry_msgs::Point> &pillars,
                    ros::Publisher &pub,
                    const std::string &frame_id)
{
    visualization_msgs::MarkerArray markers;

    // Очистка старых маркеров
    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    // Создание маркеров для столбов
    for (size_t i = 0; i < pillars.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;                            // Система координат
        marker.header.stamp = ros::Time::now();                       // Временная метка
        marker.ns = "pillars";                                        // Пространство имен
        marker.id = i;                                                // Уникальный идентификатор
        marker.type = visualization_msgs::Marker::CYLINDER;           // Тип маркера (цилиндр)
        marker.action = visualization_msgs::Marker::ADD;              // Добавление маркера
        marker.pose.position = pillars[i];                            // Позиция маркера
        marker.pose.position.z = 0.5;                                 // Высота цилиндра
        marker.pose.orientation.w = 1.0;                              // Ориентация
        marker.scale.x = marker.scale.y = 2 * params.expected_radius; // Размер цилиндра
        marker.scale.z = 1.0;                                         // Высота цилиндра
        marker.color.r = viz.color[0];                                // Цвет маркера
        marker.color.g = viz.color[1];
        marker.color.b = viz.color[2];
        marker.color.a = viz.alpha;                    // Прозрачность
        marker.lifetime = ros::Duration(viz.lifetime); // Время жизни маркера

        markers.markers.push_back(marker); // Добавление маркера в массив
    }

    pub.publish(markers); // Публикация маркеров
}

// Функция для публикации маркеров кластеров
void publishClusterMarkers(const std::vector<std::vector<geometry_msgs::Point>> &clusters,
                           ros::Publisher &pub,
                           const std::string &frame_id)
{
    visualization_msgs::MarkerArray markers;

    // Очистка старых маркеров
    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    // Создание маркеров для каждого кластера
    for (size_t i = 0; i < clusters.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;                // Система координат
        marker.header.stamp = ros::Time::now();           // Временная метка
        marker.ns = "clusters";                           // Пространство имен
        marker.id = i;                                    // Уникальный идентификатор
        marker.type = visualization_msgs::Marker::POINTS; // Тип маркера (точки)
        marker.action = visualization_msgs::Marker::ADD;  // Добавление маркера
        marker.scale.x = 0.05;                            // Размер точек
        marker.scale.y = 0.05;
        marker.color.r = static_cast<float>(rand()) / RAND_MAX; // Случайный цвет
        marker.color.g = static_cast<float>(rand()) / RAND_MAX;
        marker.color.b = static_cast<float>(rand()) / RAND_MAX;
        marker.color.a = 1.0;                          // Полная непрозрачность
        marker.lifetime = ros::Duration(viz.lifetime); // Время жизни маркера

        // Добавление точек кластера в маркер
        for (const auto &point : clusters[i])
        {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0; // Точки на плоскости
            marker.points.push_back(p);
        }

        markers.markers.push_back(marker); // Добавление маркера в массив
    }

    pub.publish(markers); // Публикация маркеров
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pillar_detector"); // Инициализация ROS-узла
    ros::NodeHandle nh("~");

    // Загрузка параметров
    // nh.param("cluster_threshold", params.cluster_threshold, 0.3f);
    // nh.param("min_cluster_size", params.min_cluster_size, 11);
    // nh.param("distance_threshold", params.distance_threshold, 0.07f);
    // nh.param("max_distance", params.max_distance, 20.0f);
    // nh.param("marker_lifetime", viz.lifetime, 1.1f);
    // nh.param("marker_color", viz.color, std::vector<float>{0.0, 1.0, 0.0});
    // nh.param("marker_alpha", viz.alpha, 0.7f);

    // Инициализация подписчиков и издателей
    ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scanCallback);                                        // Подписка на топик /scan
    ros::Publisher markers_pub = nh.advertise<visualization_msgs::MarkerArray>("pillar_markers", 1);          // Публикация маркеров столбов
    ros::Publisher cluster_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1); // Публикация маркеров кластеров

    ros::Rate rate(1); // Частота работы узла (1 Гц)

    while (ros::ok())
    {
        // Получение данных сканирования
        sensor_msgs::LaserScan::ConstPtr scan;
        {
            std::lock_guard<std::mutex> lock(scan_mutex); // Блокировка мьютекса
            scan = last_scan;
        }

        if (scan)
        {
            // Обработка данных сканирования
            auto [pillars, clusters] = processScan(*scan);

            // Публикация маркеров
            publishMarkers(pillars, markers_pub, scan->header.frame_id);                 // Маркеры столбов
            publishClusterMarkers(clusters, cluster_markers_pub, scan->header.frame_id); // Маркеры кластеров
        }

        // Обработка коллбэков и ожидание
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}