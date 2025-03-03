#include <ros/ros.h>              // Библиотека ROS для работы с узлами
#include <sensor_msgs/LaserScan.h> // Сообщения для данных лидара
#include <visualization_msgs/Marker.h> // Сообщения для визуализации в RViz
#include <vector>                 // Стандартный вектор C++
#include <cmath>                  // Математические функции (sin, cos, sqrt)
#include <signal.h>               // Для обработки Ctrl+C

// Константы для задачи
const double PILLAR_RADIUS = 0.1575;    // Радиус столба (половина диаметра 0,315 м)
const double RADIUS_TOLERANCE = 0.05;   // Допуск на радиус столба (±5 см)
const double MIN_ARC_ANGLE = 0.5236;    // Минимальный угол дуги столба (30° в радианах)

const double CLUSTER_TOLERANCE = 0.07;   // Максимальное расстояние между точками в кластере (20 см)
const int MINPOINTS = 11;                // Минимальное количество точек для кластера
const int MAXPOINTS = 101;                // Минимальное количество точек для кластера

// Структура для точки с координатами x и y
struct PointXY 
{
    double x;  // Координата x
    double y;  // Координата y
    double angle;  // Угол
    double range;  // Дистанция
};

// Структура для окружности (будем искать столбы как окружности)
struct Circle 
{
    double x_center;  // Координата x центра окружности
    double y_center;  // Координата y центра окружности
    double radius;    // Радиус окружности
};

// Переменная для остановки программы по Ctrl+C
int keep_running = 1;

// Функция, которая срабатывает при нажатии Ctrl+C
void stopProgram(int signal)
{
    keep_running = 0;  // Устанавливаем флаг, чтобы остановить цикл
    ros::shutdown();   // Завершаем работу ROS
}

// Класс для поиска и отображения столбов
class PillarDetector 
{
public:
    PillarDetector() 
    {
        // Подписываемся на топик /scan, чтобы получать данные лидара
        scan_subscriber = node.subscribe("/scan", 1, &PillarDetector::scanCallback, this);
        
        // Создаём publisher для отправки маркеров столбов в RViz
        marker_publisher = node.advertise<visualization_msgs::Marker>("/pillar_markers", 1);
        
        // Создаём publisher для отправки маркеров кластеров в RViz
        cluster_publisher = node.advertise<visualization_msgs::Marker>("/cluster_markers", 1);
        
        // Настраиваем таймер, чтобы визуализация столбов происходила раз в секунду
        timer = node.createTimer(ros::Duration(1.0), &PillarDetector::visualizeCallback, this);
        
        ROS_INFO("Program started. Press Ctrl+C to exit.");
    }

private:
    ros::NodeHandle node;           // Узел ROS для работы с топиками
    ros::Subscriber scan_subscriber; // Подписчик на данные лидара
    ros::Publisher marker_publisher; // Издатель для маркеров столбов
    ros::Publisher cluster_publisher; // Издатель для маркеров кластеров
    ros::Timer timer;               // Таймер для визуализации столбов
    std::vector<Circle> pillars;    // Список найденных столбов

    // Функция обработки данных от лидара
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        // Создаём пустой список точек
        std::vector<PointXY> points;

        // Проходим по всем измерениям лидара
        for (int i = 0; i < scan->ranges.size(); i++)
        {
            float range = scan->ranges[i];  // Расстояние до объекта в текущем направлении
            // Проверяем, что расстояние в допустимом диапазоне
            if (range >= scan->range_min && range <= scan->range_max)
            {
                // Вычисляем угол для текущей точки
                float angle = scan->angle_min + i * scan->angle_increment;
                // Переводим полярные координаты (угол, расстояние) в декартовы (x, y)
                PointXY point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                point.angle = angle;
                point.range = range;
                // Добавляем точку в список
                points.push_back(point);
                // if (i<10)
                //     ROS_INFO("Grok Point %zu: angle = %6.3f x = %6.3f, y = %6.3f", i, angle, point.x, point.y);
            }
        }

        // Находим группы точек (кластеры)
        std::vector<std::vector<PointXY>> clusters = findClusters(points);
        // Ищем столбы в этих кластерах
        findPillars(clusters);
        
        // Публикуем кластеры в RViz
        publishClusters(clusters);
    }

    // Функция для поиска кластеров (групп точек)
    std::vector<std::vector<PointXY>> findClusters(std::vector<PointXY> points)
    {
        // Создаём список для хранения кластеров
        std::vector<std::vector<PointXY>> clusters;
        // Создаём список, чтобы отмечать, какие точки уже обработаны (0 - нет, 1 - да)
        std::vector<int> used(points.size(), 0);

        // Проходим по всем точкам
        for (int i = 0; i < points.size(); i++)
        {
            // Если точка уже обработана, пропускаем её
            if (used[i] == 1)
            {
                continue;
            }

            // Создаём новый кластер и добавляем в него текущую точку
            std::vector<PointXY> cluster;
            cluster.push_back(points[i]);
            used[i] = 1;  // Отмечаем точку как обработанную

            // Проверяем все остальные точки
            for (int j = 0; j < points.size(); j++)
            {
                // Пропускаем текущую точку и уже обработанные
                if (i == j || used[j] == 1)
                {
                    continue;
                }

                // Вычисляем расстояние между точками
                double dist = getDistance(points[i], points[j]);
                // Если расстояние меньше порога, добавляем точку в кластер
                if (dist <= CLUSTER_TOLERANCE)
                {
                    cluster.push_back(points[j]);
                    used[j] = 1;  // Отмечаем точку как обработанную

                    // Проверяем соседей новой точки
                    int k = 0;
                    while (k < cluster.size())
                    {
                        for (int m = 0; m < points.size(); m++)
                        {
                            if (used[m] == 1)
                            {
                                continue;
                            }
                            // Если точка близко к текущей точке кластера, добавляем её
                            if (getDistance(cluster[k], points[m]) <= CLUSTER_TOLERANCE)
                            {
                                cluster.push_back(points[m]);
                                used[m] = 1;
                            }
                        }
                        k++;  // Переходим к следующей точке кластера
                    }
                }
            }

            // Если в кластере достаточно точек, сохраняем его
            if (cluster.size() >= MINPOINTS && cluster.size() < MAXPOINTS)
            {
                clusters.push_back(cluster);
                ROS_INFO(" clusters size %i: ", cluster.size());
            }
        }
        ROS_INFO("Grok Count clusters %i: ", clusters.size());
        return clusters;
    }

    // Функция для вычисления расстояния между двумя точками
    double getDistance(PointXY p1, PointXY p2)
    {
        double dx = p1.x - p2.x;  // Разница по x
        double dy = p1.y - p2.y;  // Разница по y
        // Вычисляем расстояние по формуле √(dx² + dy²)
        return sqrt(dx * dx + dy * dy);
    }

    // Функция для поиска окружности в кластере (RANSAC)
    Circle fitCircle(std::vector<PointXY> points)
    {
        // Создаём переменную для лучшей окружности
        Circle best_circle;
        best_circle.x_center = 0;
        best_circle.y_center = 0;
        best_circle.radius = 0;
        int best_inliers = 0;  // Счётчик точек, попавших в лучшую окружность

        // Пробуем 100 раз найти окружность
        for (int iter = 0; iter < 100; iter++)
        {
            // Выбираем 3 случайные точки для построения окружности
            int idx1 = rand() % points.size();
            int idx2 = rand() % points.size();
            int idx3 = rand() % points.size();
            // Проверяем, что точки разные
            if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3)
            {
                continue;
            }

            PointXY p1 = points[idx1];
            PointXY p2 = points[idx2];
            PointXY p3 = points[idx3];

            // Вычисляем центр окружности через 3 точки (математика из геометрии)
            double d = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
            // Если знаменатель слишком мал, пропускаем
            if (fabs(d) < 0.000001)
            {
                continue;
            }

            double x_c = ((p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) +
                          (p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) +
                          (p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y)) / d;
            double y_c = ((p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) +
                          (p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) +
                          (p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x)) / d;
            double radius = sqrt((p1.x - x_c) * (p1.x - x_c) + (p1.y - y_c) * (p1.y - y_c));

            // Считаем, сколько точек попадает в эту окружность
            int inliers = 0;
            for (int i = 0; i < points.size(); i++)
            {
                double dist = fabs(sqrt((points[i].x - x_c) * (points[i].x - x_c) + 
                                       (points[i].y - y_c) * (points[i].y - y_c)) - radius);
                if (dist < RADIUS_TOLERANCE)
                {
                    inliers++;
                }
            }

            // Если эта окружность лучше предыдущей, сохраняем её
            if (inliers > best_inliers)
            {
                best_inliers = inliers;
                best_circle.x_center = x_c;
                best_circle.y_center = y_c;
                best_circle.radius = radius;
            }
        }

        return best_circle;
    }

    // Функция для поиска столбов среди кластеров
    void findPillars(std::vector<std::vector<PointXY>> clusters)
    {
        pillars.clear();  // Очищаем список столбов перед поиском

        // Проходим по всем кластерам
        for (int i = 0; i < clusters.size(); i++)
        {
            // Вычисляем минимальный и максимальный углы дуги кластера
            double min_theta = atan2(clusters[i][0].y, clusters[i][0].x);
            double max_theta = min_theta;
            for (int j = 0; j < clusters[i].size(); j++)
            {
                double theta = atan2(clusters[i][j].y, clusters[i][j].x);
                if (theta < min_theta)
                {
                    min_theta = theta;
                }
                if (theta > max_theta)
                {
                    max_theta = theta;
                }
            }
            // Если дуга слишком мала, это не столб
            if (max_theta - min_theta < MIN_ARC_ANGLE)
            {
                continue;
            }

            // Проверяем, похож ли кластер на окружность
            Circle circle = fitCircle(clusters[i]);
            // Если радиус не соответствует столбу, пропускаем
            if (fabs(circle.radius - PILLAR_RADIUS) > RADIUS_TOLERANCE)
            {
                continue;
            }

            // Добавляем найденный столб в список
            pillars.push_back(circle);

            // Вычисляем расстояние и угол до столба
            double distance = sqrt(circle.x_center * circle.x_center + circle.y_center * circle.y_center);
            double direction = atan2(circle.y_center, circle.x_center);
            // Выводим информацию в консоль
            ROS_INFO("Pillar detected: distance = %.2f m, direction = %.2f rad (%.1f deg), radius = %.3f m",
                     distance, direction, direction * 180 / M_PI, circle.radius);
        }
    }

    // Функция для публикации кластеров в RViz
    void publishClusters(std::vector<std::vector<PointXY>> clusters)
    {
        // Создаём сообщение для RViz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser";  // Система координат лидара
        marker.header.stamp = ros::Time::now(); // Текущая метка времени
        marker.ns = "clustersGrok";                 // Пространство имён для кластеров
        marker.type = visualization_msgs::Marker::POINTS; // Тип маркера - точки
        marker.action = visualization_msgs::Marker::ADD;  // Действие - добавить
        marker.pose.orientation.w = 1.0;        // Ориентация (без вращения)
        marker.scale.x = 0.05;                  // Размер точки по x (5 см)
        marker.scale.y = 0.05;                  // Размер точки по y
        marker.color.r = 0.0;                   // Цвет - красный
        marker.color.g = 1.0;                   // Цвет - зелёный (для отличия от столбов)
        marker.color.b = 0.0;                   // Цвет - синий
        marker.color.a = 1.0;                   // Прозрачность (непрозрачный)
        marker.id = 0;                          // Идентификатор маркера

        // Проходим по всем кластерам
        for (int i = 0; i < clusters.size(); i++)
        {
            // Проходим по всем точкам в текущем кластере
            for (int j = 0; j < clusters[i].size(); j++)
            {
                geometry_msgs::Point point;
                point.x = clusters[i][j].x;  // Координата x точки
                point.y = clusters[i][j].y;  // Координата y точки
                point.z = 0.0;               // Высота (z=0, так как 2D)
                marker.points.push_back(point); // Добавляем точку в список
            }
        }

        // Отправляем маркер в RViz
        cluster_publisher.publish(marker);
        ROS_INFO("Published %d clusters with %d points", (int)clusters.size(), (int)marker.points.size());
    }

    // Функция для визуализации столбов в RViz
    void visualizeCallback(const ros::TimerEvent&)
    {
        // Если столбов нет, ничего не делаем
        if (pillars.size() == 0)
        {
            return;
        }

        // Создаём сообщение для RViz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser_frame";  // Система координат лидара
        marker.header.stamp = ros::Time::now(); // Текущая метка времени
        marker.ns = "pillars";                  // Пространство имён
        marker.type = visualization_msgs::Marker::SPHERE_LIST; // Тип маркера - список сфер
        marker.action = visualization_msgs::Marker::ADD;       // Действие - добавить
        marker.pose.orientation.w = 1.0;        // Ориентация (без вращения)
        marker.scale.x = 0.315;                 // Размер сферы по x (диаметр столба)
        marker.scale.y = 0.315;                 // Размер сферы по y
        marker.scale.z = 0.315;                 // Размер сферы по z
        marker.color.r = 1.0;                   // Цвет - красный
        marker.color.g = 0.0;                   // Цвет - зелёный
        marker.color.b = 0.0;                   // Цвет - синий
        marker.color.a = 1.0;                   // Прозрачность (непрозрачный)
        marker.id = 0;                          // Идентификатор маркера

        // Добавляем все столбы в маркер
        for (int i = 0; i < pillars.size(); i++)
        {
            geometry_msgs::Point point;
            point.x = pillars[i].x_center;  // Координата x центра столба
            point.y = pillars[i].y_center;  // Координата y центра столба
            point.z = 0.0;                  // Высота (z=0, так как 2D)
            marker.points.push_back(point); // Добавляем точку в список
        }

        // Отправляем маркер в RViz
        marker_publisher.publish(marker);
        ROS_INFO("Published %d pillars", (int)pillars.size());
    }
};

// Главная функция программы
int main(int argc, char** argv)
{
    // Настраиваем обработку Ctrl+C
    signal(SIGINT, stopProgram);

    // Инициализируем ROS с именем узла "pillar_detector"
    ros::init(argc, argv, "pillar_detector");
    // Создаём объект детектора столбов
    PillarDetector detector;

    // Создаём цикл с частотой 10 Гц
    ros::Rate loop_rate(10);
    // Пока ROS работает и не нажат Ctrl+C
    while (ros::ok() && keep_running)
    {
        ros::spinOnce();  // Обрабатываем входящие сообщения
        loop_rate.sleep(); // Ждём, чтобы поддерживать частоту 10 Гц
    }

    ROS_INFO("Program stopped");
    return 0;
}