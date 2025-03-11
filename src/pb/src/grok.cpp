#include <ros/ros.h>                   // Библиотека ROS для работы с узлами
#include <sensor_msgs/LaserScan.h>     // Сообщения для данных лидара
#include <visualization_msgs/Marker.h> // Сообщения для визуализации в RViz
#include <vector>                      // Стандартный вектор C++
#include <cmath>                       // Математические функции (sin, cos, sqrt)
#include <signal.h>                    // Для обработки Ctrl+C

// Константы для задачи
const double CLUSTER_TOLERANCE = 0.1;  // Максимальное расстояние между точками в кластере (10 см)
const int MIN_POINTS = 11;             // Минимальное количество точек для кластера
const int MAX_POINTS = 101;            // Максимальное количество точек для кластера
const double MIN_PILLAR_WIDTH = 0.2;   // Минимальная ширина кластера для столба (м)
const double MAX_PILLAR_WIDTH = 0.4;   // Максимальная ширина кластера для столба (м)
const double PILLAR_RADIUS = 0.1575;   // Радиус столба (половина диаметра 0,315 м)
const double MAX_MATCH_DISTANCE = 0.5; // Максимальное расстояние для сопоставления столба (м)

// Заданные координаты четырёх столбов (x, y) в метрах
const std::vector<std::pair<double, double>> KNOWN_PILLARS = {
    {4.0, 0.3},  // Столб 1
    {0.0, 0.5}, // Столб 2
    {0.5, 4.0}, // Столб 3
    {5.0, 4.0} // Столб 4
};

// Структура для точки с координатами x и y
struct PointXY
{
    double x; // Координата x
    double y; // Координата y
};

// Структура для информации о кластере
struct ClusterInfo
{
    std::vector<PointXY> points; // Список точек кластера
    int point_count;             // Количество точек в кластере
    double azimuth;              // Азимут до центра масс кластера в радианах
    double min_distance;         // Минимальное расстояние до кластера от лидара
    double width;                // Ширина кластера (максимальное расстояние между точками)
};

// Структура для столба (координаты центра)
struct Pillar
{
    double x_center; // Координата x центра столба
    double y_center; // Координата y центра столба
};

// Переменная для остановки программы по Ctrl+C
int keep_running = 1;

// Функция, которая срабатывает при нажатии Ctrl+C
void stopProgram(int signal)
{
    keep_running = 0; // Устанавливаем флаг, чтобы остановить цикл
    ros::shutdown();  // Завершаем работу ROS
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

        // Настраиваем таймер, чтобы визуализация происходила раз в секунду
        timer = node.createTimer(ros::Duration(1.0), &PillarDetector::visualizeCallback, this);

        ROS_INFO("Program started. Press Ctrl+C to exit.");
    }

private:
    ros::NodeHandle node;                       // Узел ROS для работы с топиками
    ros::Subscriber scan_subscriber;            // Подписчик на данные лидара
    ros::Publisher marker_publisher;            // Издатель для маркеров столбов
    ros::Publisher cluster_publisher;           // Издатель для маркеров кластеров
    ros::Timer timer;                           // Таймер для визуализации
    std::vector<Pillar> pillars;                // Список найденных столбов
    std::vector<ClusterInfo> cluster_info_list; // Список информации о кластерах

    // Функция обработки данных от лидара
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        // Создаём пустой список точек
        std::vector<PointXY> points;

        // Проходим по всем измерениям лидара
        for (int i = 0; i < scan->ranges.size(); i++)
        {
            float range = scan->ranges[i]; // Расстояние до объекта в текущем направлении
            // Проверяем, что расстояние в допустимом диапазоне
            if (range >= scan->range_min && range <= scan->range_max)
            {
                // Вычисляем угол для текущей точки
                float angle = scan->angle_min + i * scan->angle_increment;
                // Переводим полярные координаты (угол, расстояние) в декартовы (x, y)
                PointXY point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                // Добавляем точку в список
                points.push_back(point);
            }
        }

        // Находим группы точек (кластеры)
        findClusters(points);
        // Ищем столбы в этих кластерах
        findPillars();
        // Сопоставляем столбы с известными координатами
        matchPillars();
    }

    // Функция для поиска кластеров (групп точек)
    void findClusters(std::vector<PointXY> points)
    {
        // Очищаем список кластеров перед новым поиском
        cluster_info_list.clear();

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
            used[i] = 1; // Отмечаем точку как обработанную

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
                    used[j] = 1; // Отмечаем точку как обработанную

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
                        k++; // Переходим к следующей точке кластера
                    }
                }
            }

            // Если в кластере от MIN_POINTS до MAX_POINTS точек, сохраняем его
            if (cluster.size() >= MIN_POINTS && cluster.size() <= MAX_POINTS)
            {
                // Создаём структуру для хранения информации о кластере
                ClusterInfo cluster_info;
                cluster_info.points = cluster;
                cluster_info.point_count = cluster.size();

                // Вычисляем центр масс кластера
                double x_sum = 0;
                double y_sum = 0;
                for (int j = 0; j < cluster.size(); j++)
                {
                    x_sum += cluster[j].x;
                    y_sum += cluster[j].y;
                }
                double x_center_mass = x_sum / cluster.size();
                double y_center_mass = y_sum / cluster.size();

                // Вычисляем азимут до центра масс кластера
                cluster_info.azimuth = atan2(y_center_mass, x_center_mass);

                // Вычисляем минимальное расстояние до кластера от лидара (0, 0)
                double min_dist = 1000.0; // Большое начальное значение
                for (int j = 0; j < cluster.size(); j++)
                {
                    double dist = sqrt(cluster[j].x * cluster[j].x + cluster[j].y * cluster[j].y);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                    }
                }
                cluster_info.min_distance = min_dist;

                // Вычисляем ширину кластера (максимальное расстояние между любыми двумя точками)
                double max_width = 0.0;
                for (int j = 0; j < cluster.size(); j++)
                {
                    for (int k = j + 1; k < cluster.size(); k++)
                    {
                        double dist = getDistance(cluster[j], cluster[k]);
                        if (dist > max_width)
                        {
                            max_width = dist;
                        }
                    }
                }
                cluster_info.width = max_width;

                // Выводим информацию о кластере в консоль
                ROS_INFO("Cluster found: %d points, azimuth = %.2f rad, min_distance = %.2f m, width = %.2f m",
                         cluster_info.point_count, cluster_info.azimuth, cluster_info.min_distance, cluster_info.width);

                // Сохраняем информацию о кластере в список
                cluster_info_list.push_back(cluster_info);
            }
        }

        // Выводим общее количество найденных кластеров
        ROS_INFO("Found %d clusters total", (int)cluster_info_list.size());
    }

    // Функция для вычисления расстояния между двумя точками
    double getDistance(PointXY p1, PointXY p2)
    {
        double dx = p1.x - p2.x; // Разница по x
        double dy = p1.y - p2.y; // Разница по y
        // Вычисляем расстояние по формуле √(dx² + dy²)
        return sqrt(dx * dx + dy * dy);
    }

    // Функция для поиска столбов среди кластеров (по ширине)
    void findPillars()
    {
        pillars.clear(); // Очищаем список столбов перед поиском

        // Проходим по всем кластерам
        for (int i = 0; i < cluster_info_list.size(); i++)
        {
            // Проверяем ширину кластера
            if (cluster_info_list[i].width >= MIN_PILLAR_WIDTH && cluster_info_list[i].width <= MAX_PILLAR_WIDTH)
            {
                // Находим точку с минимальным расстоянием до лидара (0, 0)
                double min_dist = 1000.0;
                PointXY closest_point = {0.0, 0.0};
                for (int j = 0; j < cluster_info_list[i].points.size(); j++)
                {
                    double dist = sqrt(cluster_info_list[i].points[j].x * cluster_info_list[i].points[j].x +
                                       cluster_info_list[i].points[j].y * cluster_info_list[i].points[j].y);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        closest_point = cluster_info_list[i].points[j];
                    }
                }

                // Используем азимут кластера для направления
                double azimuth = cluster_info_list[i].azimuth;

                // Смещаем центр столба на расстояние радиуса от ближайшей точки по азимуту
                Pillar pillar;
                pillar.x_center = closest_point.x + PILLAR_RADIUS * cos(azimuth);
                pillar.y_center = closest_point.y + PILLAR_RADIUS * sin(azimuth);

                // Добавляем найденный столб в список
                pillars.push_back(pillar);

                // Выводим информацию о столбе в консоль, включая координаты, ширину и номер кластера
                ROS_INFO("Pillar detected from cluster %d: x = %.2f m, y = %.2f m, distance = %.2f m, direction = %.2f rad (%.1f deg), width = %.2f m",
                         i,
                         pillar.x_center, pillar.y_center,
                         sqrt(pillar.x_center * pillar.x_center + pillar.y_center * pillar.y_center),
                         atan2(pillar.y_center, pillar.x_center),
                         atan2(pillar.y_center, pillar.x_center) * 180 / M_PI,
                         cluster_info_list[i].width);
            }
        }
    }

    // Функция для сопоставления обнаруженных столбов с известными координатами
    void matchPillars()
    {
        if (pillars.empty())
        {
            ROS_INFO("No pillars detected to match with known coordinates.");
            return;
        }

        std::vector<bool> used(KNOWN_PILLARS.size(), false); // Отмечаем использованные известные столбы

        // Проходим по всем обнаруженным столбам
        for (size_t i = 0; i < pillars.size(); i++)
        {
            double min_dist = 1e10; // Большое начальное значение
            int best_match = -1;    // Индекс ближайшего известного столба

            // Находим ближайший известный столб
            for (size_t j = 0; j < KNOWN_PILLARS.size(); j++)
            {
                if (used[j])
                    continue; // Пропускаем уже сопоставленные столбы

                double dx = pillars[i].x_center - KNOWN_PILLARS[j].first;
                double dy = pillars[i].y_center - KNOWN_PILLARS[j].second;
                double dist = sqrt(dx * dx + dy * dy);

                if (dist < min_dist)
                {
                    min_dist = dist;
                    best_match = j;
                }
            }

            if (best_match != -1 && min_dist <= MAX_MATCH_DISTANCE)
            {
                // Вычисляем разницу в координатах
                double delta_x = pillars[i].x_center - KNOWN_PILLARS[best_match].first;
                double delta_y = pillars[i].y_center - KNOWN_PILLARS[best_match].second;

                // Отмечаем известный столб как использованный
                used[best_match] = true;

                // Выводим результат сопоставления
                ROS_INFO("Pillar %zu matched to known pillar %d (x=%.2f, y=%.2f): delta_x = %.2f m, delta_y = %.2f m, distance = %.2f m",
                         i, best_match,
                         KNOWN_PILLARS[best_match].first, KNOWN_PILLARS[best_match].second,
                         delta_x, delta_y, min_dist);
            }
            else
            {
                // Если расстояние больше порога или нет подходящего столба
                ROS_INFO("Pillar %zu (x=%.2f, y=%.2f) not matched: closest distance %.2f m exceeds threshold %.2f m",
                         i, pillars[i].x_center, pillars[i].y_center, min_dist, MAX_MATCH_DISTANCE);
            }
        }

        // Проверяем, остались ли несопоставленные известные столбы
        for (size_t j = 0; j < KNOWN_PILLARS.size(); j++)
        {
            if (!used[j])
            {
                ROS_INFO("Known pillar %zu (x=%.2f, y=%.2f) not matched to any detected pillar.",
                         j, KNOWN_PILLARS[j].first, KNOWN_PILLARS[j].second);
            }
        }
    }

    // Функция для визуализации кластеров и столбов в RViz (вызывается раз в секунду)
    void visualizeCallback(const ros::TimerEvent &)
    {
        // Создаём маркер для кластеров
        visualization_msgs::Marker cluster_marker;
        cluster_marker.header.frame_id = "laser";                 // Система координат лидара
        cluster_marker.header.stamp = ros::Time::now();           // Текущая метка времени
        cluster_marker.ns = "clusters";                           // Пространство имён для кластеров
        cluster_marker.type = visualization_msgs::Marker::POINTS; // Тип маркера - точки
        cluster_marker.action = visualization_msgs::Marker::ADD;  // Действие - добавить
        cluster_marker.pose.orientation.w = 1.0;                  // Ориентация (без вращения)
        cluster_marker.scale.x = 0.05;                            // Размер точки по x (5 см)
        cluster_marker.scale.y = 0.05;                            // Размер точки по y
        cluster_marker.color.r = 0.0;                             // Цвет - красный
        cluster_marker.color.g = 1.0;                             // Цвет - зелёный (для отличия от столбов)
        cluster_marker.color.b = 0.0;                             // Цвет - синий
        cluster_marker.color.a = 1.0;                             // Прозрачность (непрозрачный)
        cluster_marker.id = 0;                                    // Идентификатор маркера

        // Заполняем маркер кластерами
        for (int i = 0; i < cluster_info_list.size(); i++)
        {
            for (int j = 0; j < cluster_info_list[i].points.size(); j++)
            {
                geometry_msgs::Point point;
                point.x = cluster_info_list[i].points[j].x; // Координата x точки
                point.y = cluster_info_list[i].points[j].y; // Координата y точки
                point.z = 0.0;                              // Высота (z=0, так как 2D)
                cluster_marker.points.push_back(point);     // Добавляем точку в список
            }
        }

        // Создаём маркер для столбов
        visualization_msgs::Marker pillar_marker;
        pillar_marker.header.frame_id = "laser";                      // Система координат лидара
        pillar_marker.header.stamp = ros::Time::now();                // Текущая метка времени
        pillar_marker.ns = "pillars";                                 // Пространство имён
        pillar_marker.type = visualization_msgs::Marker::SPHERE_LIST; // Тип маркера - список сфер
        pillar_marker.action = visualization_msgs::Marker::ADD;       // Действие - добавить
        pillar_marker.pose.orientation.w = 1.0;                       // Ориентация (без вращения)
        pillar_marker.scale.x = 0.315;                                // Размер сферы по x (диаметр столба)
        pillar_marker.scale.y = 0.315;                                // Размер сферы по y
        pillar_marker.scale.z = 0.315;                                // Размер сферы по z
        pillar_marker.color.r = 1.0;                                  // Цвет - красный
        pillar_marker.color.g = 0.0;                                  // Цвет - зелёный
        pillar_marker.color.b = 0.0;                                  // Цвет - синий
        pillar_marker.color.a = 1.0;                                  // Прозрачность (непрозрачный)
        pillar_marker.id = 0;                                         // Идентификатор маркера

        // Заполняем маркер столбами
        for (int i = 0; i < pillars.size(); i++)
        {
            geometry_msgs::Point point;
            point.x = pillars[i].x_center;         // Координата x центра столба
            point.y = pillars[i].y_center;         // Координата y центра столба
            point.z = 0.0;                         // Высота (z=0, так как 2D)
            pillar_marker.points.push_back(point); // Добавляем точку в список
        }

        // Отправляем оба маркера в RViz
        cluster_publisher.publish(cluster_marker);
        marker_publisher.publish(pillar_marker);

        // Выводим информацию о публикации
        ROS_INFO("Published %d clusters with %d points and %d pillars",
                 (int)cluster_info_list.size(), (int)cluster_marker.points.size(), (int)pillars.size());
    }
};

// Главная функция программы
int main(int argc, char **argv)
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
        ros::spinOnce();   // Обрабатываем входящие сообщения
        loop_rate.sleep(); // Ждём, чтобы поддерживать частоту 10 Гц
    }

    ROS_INFO("Program stopped");
    return 0;
}