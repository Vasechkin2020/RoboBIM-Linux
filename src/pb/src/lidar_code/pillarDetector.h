#ifndef PILLAR_DETECTOR_H
#define PILLAR_DETECTOR_H

// Класс для поиска и отображения столбов
class PillarDetector
{
public:
    PillarDetector()
    {
        // Подписываемся на топик /scan, чтобы получать данные лидара
        // scan_subscriber = node.subscribe("/scan", 1, &PillarDetector::scanCallback, this);

        // Создаём publisher для отправки маркеров столбов в RViz
        marker_publisher = node.advertise<visualization_msgs::Marker>("pbRviz/pillar_markers", 1);

        // Создаём publisher для отправки маркеров кластеров в RViz
        cluster_publisher = node.advertise<visualization_msgs::Marker>("pbRviz/cluster_markers", 1);

        // Создаём publisher для отправки маркера лидара в RViz
        lidar_publisher = node.advertise<visualization_msgs::Marker>("pbRviz/lidar_marker", 1);

        // Настраиваем таймер, чтобы визуализация происходила раз в секунду
        timer = node.createTimer(ros::Duration(0.25), &PillarDetector::visualizeCallback, this);

        ROS_INFO("Program started. Press Ctrl+C to exit.");
    }

    // Константы для задачи
    const double CLUSTER_TOLERANCE = 0.33; // Максимальное расстояние между точками в кластере (10 см)
    const int MIN_POINTS = 11;             // Минимальное количество точек для кластера
    const int MAX_POINTS = 101;            // Максимальное количество точек для кластера
    const double MIN_PILLAR_WIDTH = 0.20;  // Минимальная ширина кластера для столба (м)
    const double MAX_PILLAR_WIDTH = 0.40;  // Максимальная ширина кластера для столба (м)
    const double PILLAR_RADIUS = 0.1575;   // Радиус столба (половина диаметра 0,315 м)
    const double MAX_MATCH_DISTANCE = 0.3; // Максимальное расстояние для сопоставления столба (м)

    // Структура для столба (координаты центра)
    struct Pillar
    {
        double x_center;  // Координата x центра столба (локальная)
        double y_center;  // Координата y центра столба (локальная)
        double x_global;  // Глобальная координата x центра столба
        double y_global;  // Глобальная координата y центра столба
        double direction; // Направление
        double distance;  // Дистанция
        bool match;       // Сопоставление
        int count;       // Флаг что есть значение
    };

    Pillar matchPillar[4]; // Сопоставленные столбы по порядку

    // Заданные координаты четырёх столбов (x, y) в метрах в глобальной системе координат
    std::vector<std::pair<double, double>> KNOWN_PILLARS = {
        {0.0, 0.0}, // Столб 1
        {0.0, 0.0}, // Столб 3
        {0.0, 0.0}, // Столб 2
        {0.0, 0.0}  // Столб 4
    };

    // Метод изменения известных столбов
    void changeKnownPillars(int i, float first_, float second_)
    {
        // Изменение координат столба
        KNOWN_PILLARS[i].first = first_;   // Новое значение x
        KNOWN_PILLARS[i].second = second_; // Новое значение y
        ROS_INFO("changeKnownPillars %i x= %.3f y= %.3f", i, KNOWN_PILLARS[i].first, KNOWN_PILLARS[i].second);
    }
    // Метод изменения положения лидара
    void changePoseLidar(float x_, float y_, float th_)
    {
        lidar_x = x_;                             // Координата x лидара в глобальной системе
        lidar_y = y_;                             // Координата y лидара в глобальной системе
        lidar_theta = (th_ + 180) / (180 / M_PI); // Угол ориентации лидара в глобальной системе (рад)
        ROS_INFO("changePoseLidar x= %.3f y= %.3f  th= %.4f (rad) th= %.3f (grad)", lidar_x, lidar_y, lidar_theta, th_);
    }

    // Структура для точки с координатами x и y
    struct PointXY
    {
        double x; // Координата x
        double y; // Координата y
    };

    // Структура для информации о кластере
    struct ClusterInfo
    {
        std::vector<PointXY> points; // Список точек кластера (локальные координаты)
        int point_count;             // Количество точек в кластере
        double azimuth;              // Азимут до центра масс кластера в радианах (локальный)
        double min_distance;         // Минимальное расстояние до кластера от лидара
        double width;                // Ширина кластера (максимальное расстояние между точками)
        double x_global;             // Глобальная координата x центра масс
        double y_global;             // Глобальная координата y центра масс
        double azimuth_global;       // Азимут в глобальной системе
    };

    // Функция обработки данных от лидара
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        start_time = ros::Time::now(); // Записываем начальное время
        ROS_INFO("++++ scanCallback");
        std::vector<PointXY> points; // Создаём пустой список точек
        points.reserve(4096);
        
        for (int i = 0; i < scan->ranges.size(); i++)// Проходим по всем измерениям лидара
        {
            float range = scan->ranges[i];                            // Расстояние до объекта в текущем направлении
            if (range >= scan->range_min && range <= scan->range_max) // Проверяем, что расстояние в допустимом диапазоне
            {
                float angle = scan->angle_min + i * scan->angle_increment; // Вычисляем угол для текущей точки
                PointXY point;
                point.x = range * cos(angle);// Переводим полярные координаты (угол, расстояние) в декартовы (x, y)
                point.y = range * sin(angle);
                points.push_back(point); // Добавляем точку в список
            }
        }
        std::vector<PointXY> pointsInvers(points.rbegin(), points.rend()); // Создаем новый вектор с элементами в обратном порядке
        elapsed_time = ros::Time::now() - start_time;                      // Вычисляем интервал
        // ROS_INFO("    End Scan Elapsed time: %.6f seconds", elapsed_time.toSec());

        std::vector<std::vector<PointXY>> clusters = findClusters(points);          // Находим группы точек (кластеры)
        cluster_info_list = createdClasterList(clusters); // Список информации о кластерах

        // ROS_INFO("    Lidar theta START = %.3f rad (%.3f deg)", lidar_theta, RAD2DEG(lidar_theta));

        findPillars(); // Ищем столбы в этих кластерах
        matchPillars(); // // Сопоставляем столбы с известными координатами и вычисляем позицию и ориентацию лидара
    }

private:
    ros::NodeHandle node;                       // Узел ROS для работы с топиками
    ros::Subscriber scan_subscriber;            // Подписчик на данные лидара
    ros::Publisher marker_publisher;            // Издатель для маркеров столбов
    ros::Publisher cluster_publisher;           // Издатель для маркеров кластеров
    ros::Publisher lidar_publisher;             // Издатель для маркера лидара
    ros::Timer timer;                           // Таймер для визуализации
    std::vector<Pillar> pillars;                // Список найденных столбов
    std::vector<ClusterInfo> cluster_info_list; // Список информации о кластерах

    double lidar_x = 0.0;       // Координата x лидара в глобальной системе
    double lidar_y = 0.0;       // Координата y лидара в глобальной системе
    double lidar_theta = 0.0;   // Угол ориентации лидара в глобальной системе (рад)
    ros::Time start_time;       // Записываем конечное время
    ros::Time end_time;         // Записываем конечное время
    ros::Duration elapsed_time; // Вычисляем интервал

    // Функция для поиска кластеров (групп точек)
    std::vector<std::vector<PointXY>> findClusters(std::vector<PointXY> points)
    {
        start_time = ros::Time::now(); // Записываем начальное время
        ROS_INFO("+++ findClusters");
        std::vector<std::vector<PointXY>> clusters;

        // Новые константы для кластеризации
        const double MAX_STEP_DISTANCE = 0.05; // Максимальное расстояние до предыдущей точки (5 см)
        const int SEARCH_LIMIT = 100;          // Ограничение на количество точек для поиска от первой

        cluster_info_list.clear();               // Очищаем список кластеров перед новым поиском
        std::vector<int> used(points.size(), 0); // Создаём список, чтобы отмечать, какие точки уже обработаны (0 - нет, 1 - да)
        for (int i = 0; i < points.size(); i++)  // Проходим по всем точкам
        {
            if (used[i] == 1) // Если точка уже обработана, пропускаем её
                continue;

            // Создаём новый кластер и добавляем в него первую точку
            std::vector<PointXY> cluster;
            cluster.push_back(points[i]);
            used[i] = 1; // Отмечаем точку как обработанную

            // Ограничиваем поиск ближайших SEARCH_LIMIT точек от текущей
            int search_count = 0;
            for (int j = 0; j < points.size() && search_count < SEARCH_LIMIT; j++)
            {
                // Пропускаем текущую точку и уже обработанные
                if (i == j || used[j] == 1)
                    continue;

                search_count++; // Увеличиваем счётчик проверенных точек

                // Вычисляем расстояние до последней добавленной точки в кластере
                double step_dist = getDistance(cluster.back(), points[j]);
                if (step_dist <= MAX_STEP_DISTANCE)
                {
                    // Проверяем расстояние до всех точек в кластере
                    bool within_tolerance = true;
                    for (const auto &cluster_point : cluster)
                    {
                        double dist = getDistance(cluster_point, points[j]);
                        if (dist > CLUSTER_TOLERANCE)
                        {
                            within_tolerance = false;
                            break;
                        }
                    }

                    if (within_tolerance) // Если точка удовлетворяет обоим условиям, добавляем её в кластер
                    {
                        cluster.push_back(points[j]);
                        used[j] = 1; // Отмечаем точку как обработанную
                    }
                }
            }

            // Если в кластере от MIN_POINTS до MAX_POINTS точек, сохраняем его
            if (cluster.size() >= MIN_POINTS && cluster.size() <= MAX_POINTS)
            {
                clusters.push_back(cluster);
                // ROS_INFO("    clusters size %i: ", cluster.size());
            }
        }
        ROS_INFO("    Found %d clusters total", (int)clusters.size()); // Выводим общее количество найденных кластеров
        elapsed_time = ros::Time::now() - start_time;                  // Вычисляем интервал
        // ROS_INFO("    End findClusters Elapsed time: %.3f seconds", elapsed_time.toSec());
        return clusters;
    }

    std::vector<ClusterInfo> createdClasterList(const std::vector<std::vector<PointXY>> &clusters)
    {
        std::vector<ClusterInfo> cluster_info_list; // Список информации о кластерах

        for (const auto &cluster : clusters)
        {
            // Создаём структуру для хранения информации о кластере
            ClusterInfo cluster_info;
            cluster_info.points = cluster;
            cluster_info.point_count = cluster.size();

            // Вычисляем центр масс кластера (локальные координаты)
            double x_sum = 0;
            double y_sum = 0;
            for (int j = 0; j < cluster.size(); j++)
            {
                x_sum += cluster[j].x;
                y_sum += cluster[j].y;
            }
            double x_center_mass = x_sum / cluster.size();
            double y_center_mass = y_sum / cluster.size();

            // Вычисляем азимут до центра масс кластера (локальный)
            cluster_info.azimuth = atan2(y_center_mass, x_center_mass);

            // Преобразуем локальные координаты центра масс в глобальные
            localToGlobal(x_center_mass, y_center_mass, cluster_info.x_global, cluster_info.y_global);

            // Вычисляем глобальный азимут
            cluster_info.azimuth_global = normalizeAngle(cluster_info.azimuth + lidar_theta);

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

            // Выводим информацию о кластере в консоль в глобальных координатах
            // ROS_INFO("Cluster found: %d points, x_global = %.2f m, y_global = %.2f m, azimuth_global = %.2f rad (%.2f), min_distance = %.2f m, width = %.2f m",
            //          cluster_info.point_count, cluster_info.x_global, cluster_info.y_global,
            //          cluster_info.azimuth_global, cluster_info.azimuth_global*57.17, cluster_info.min_distance, cluster_info.width);

            // Сохраняем информацию о кластере в список
            cluster_info_list.push_back(cluster_info);
        }

        return cluster_info_list;
    }

    // Функция преобразования локальных координат в глобальные
    void localToGlobal(double x_local, double y_local, double &x_global, double &y_global)
    {
        x_global = lidar_x + x_local * cos(lidar_theta) - y_local * sin(lidar_theta);
        y_global = lidar_y + x_local * sin(lidar_theta) + y_local * cos(lidar_theta);
    }

    // Функция для вычисления расстояния между двумя точками
    double getDistance(PointXY p1, PointXY p2)
    {
        double dx = p1.x - p2.x; // Разница по x
        double dy = p1.y - p2.y; // Разница по y
        // Вычисляем расстояние по формуле √(dx² + dy²)
        return sqrt(dx * dx + dy * dy);
    }

    // Функция для нормализации угла в диапазон [-π, π]
    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle <= -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    // Функция для поиска столбов среди кластеров (по ширине)
    void findPillars()
    {
        // ROS_INFO("+++ findPillars");

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

                // Преобразуем локальные координаты столба в глобальные
                localToGlobal(pillar.x_center, pillar.y_center, pillar.x_global, pillar.y_global);

                // Вычисляем расстояние от лидара до столба в глобальной системе
                double dx = pillar.x_global - lidar_x;
                double dy = pillar.y_global - lidar_y;
                double distance = sqrt(dx * dx + dy * dy);
                double direction = atan2(dy, dx);

                pillar.direction = direction;
                pillar.distance = distance - PILLAR_RADIUS; // ОТНИМАЕМ ЧТОБЫ БЫЛО ОДИНАКОВО С ДРУГИМ АЛГОРИТМОМ

                // Добавляем найденный столб в список
                pillars.push_back(pillar);

                // Выводим информацию о столбе в консоль в глобальных координатах
                // ROS_INFO("Pillar detected from cluster %d: x_global = %.2f m, y_global = %.2f m, distance = %.2f m, direction = %.2f rad (%.1f deg), width = %.2f m",
                //          i,
                //          pillar.x_global, pillar.y_global,
                //          distance,
                //          direction,
                //          direction * 180 / M_PI,
                //          cluster_info_list[i].width);
            }
        }
        // ROS_INFO("    Found %d pillars total", (int)pillars.size()); // Выводим общее количество найденных кластеров
    }

    // Функция для сопоставления обнаруженных столбов с известными координатами и вычисления позиции и ориентации лидара
    void matchPillars()
    {
        ROS_INFO("+++ matchPillars");
        if (pillars.empty())
        {
            ROS_INFO("No pillars detected to match with known coordinates.");
            return;
        }

        std::vector<bool> used(KNOWN_PILLARS.size(), false);  // Отмечаем использованные известные столбы
        std::vector<std::pair<double, double>> matched_pairs; // Список пар (локальные, глобальные) координат
        std::vector<double> measured_azimuths;                // Измеренные азимуты от лидара

        for (size_t i = 0; i < 4; i++) // Обнуляем данные
        {
            matchPillar[i].direction = 0;
            matchPillar[i].distance = 0;
            matchPillar[i].x_global = 0;
            matchPillar[i].y_global = 0;
            matchPillar[i].count = 0;
        }
        

        // Проходим по всем обнаруженным столбам
        for (size_t i = 0; i < pillars.size(); i++)
        {
            // ROS_INFO("Pillar %i", i);
            double min_dist = 1e10;   // Большое начальное значение
            int best_match = -1;      // Индекс ближайшего известного столба
            pillars[i].match = false; // С начала он не сопоставленный

            // Находим ближайший известный столб (сравниваем по глобальным координатам)
            for (size_t j = 0; j < KNOWN_PILLARS.size(); j++)
            {
                if (used[j])
                    continue; // Пропускаем уже сопоставленные столбы

                double dx = pillars[i].x_global - KNOWN_PILLARS[j].first;
                double dy = pillars[i].y_global - KNOWN_PILLARS[j].second;
                double dist = sqrt(dx * dx + dy * dy);
                // ROS_INFO("    dist %f", dist);

                if (dist < min_dist)
                {
                    min_dist = dist;
                    best_match = j;
                }
            }

            if (best_match != -1 && min_dist <= MAX_MATCH_DISTANCE)
            {
                // Вычисляем разницу в координатах
                double delta_x = pillars[i].x_global - KNOWN_PILLARS[best_match].first;
                double delta_y = pillars[i].y_global - KNOWN_PILLARS[best_match].second;

                // Отмечаем известный столб как использованный
                used[best_match] = true;

                pillars[i].match = true;

                // Сохраняем пару координат и азимут для вычисления позиции и ориентации лидара
                matched_pairs.push_back({pillars[i].x_global, pillars[i].y_global});
                measured_azimuths.push_back(pillars[i].direction);

                // Выводим результат сопоставления
                ROS_INFO("    Pillar %zu matched pillar %d (x=%.2f, y=%.2f): delta_x = %.3f m, delta_y = %.3f m, gipotenuza = %.3f m, distance = %.3f m, direction = %.3f grad",
                         i, best_match,
                         KNOWN_PILLARS[best_match].first, KNOWN_PILLARS[best_match].second,
                         delta_x, delta_y, min_dist, pillars[i].distance, pillars[i].direction * 180 / M_PI);

                matchPillar[best_match].x_global = KNOWN_PILLARS[best_match].first;
                matchPillar[best_match].y_global = KNOWN_PILLARS[best_match].second;
                matchPillar[best_match].distance = pillars[i].distance;
                
                float direct =  RAD2DEG(pillars[i].direction);
                if (direct < 0) 
                    direct = -direct;
                    else if (direct > 0) 
                    direct = 360 - direct; 
                matchPillar[best_match].direction = direct;
                matchPillar[best_match].count = 1;
            }
            else
            {
                // Если расстояние больше порога или нет подходящего столба
                // ROS_INFO("Pillar %zu (x_global=%.2f, y_global=%.2f) not matched: closest distance %.2f m exceeds threshold %.2f m",
                //          i, pillars[i].x_global, pillars[i].y_global, min_dist, MAX_MATCH_DISTANCE);
            }
        }

        // Проверяем, остались ли несопоставленные известные столбы
        for (size_t j = 0; j < KNOWN_PILLARS.size(); j++)
        {
            if (!used[j])
            {
                ROS_INFO("!!! Known pillar %zu (x=%.2f, y=%.2f) not matched to any detected pillar.",
                         j, KNOWN_PILLARS[j].first, KNOWN_PILLARS[j].second);
            }
        }

        // Вычисляем координаты и ориентацию лидара в глобальной системе
        if (!matched_pairs.empty())
        {
            // ROS_INFO("+++ matched_pairs.empty %i", matched_pairs.size());
            double sum_x = 0.0;
            double sum_y = 0.0;
            double sum_theta = 0.0;
            int count = 0;

            for (size_t i = 0; i < matched_pairs.size(); i++)
            {
                int global_idx = -1;
                for (size_t j = 0; j < KNOWN_PILLARS.size(); j++)
                {
                    double dx = matched_pairs[i].first - KNOWN_PILLARS[j].first;
                    double dy = matched_pairs[i].second - KNOWN_PILLARS[j].second;
                    double dist = sqrt(dx * dx + dy * dy);

                    // ROS_INFO("    j = %i dist= %f ", j, dist);
                    if (dist <= MAX_MATCH_DISTANCE)
                    {
                        global_idx = j;
                        // ROS_INFO("    global_idx = %i", j);
                        break;
                    }
                }

                if (global_idx != -1)
                {
                    sum_x += KNOWN_PILLARS[global_idx].first - matched_pairs[i].first;
                    sum_y += KNOWN_PILLARS[global_idx].second - matched_pairs[i].second;

                    // Вычисляем теоретический азимут
                    double global_azimuth = atan2(KNOWN_PILLARS[global_idx].second - lidar_y,
                                                  KNOWN_PILLARS[global_idx].first - lidar_x);
                    // Разница между измеренным и глобальным азимутом даёт угол поворота лидара
                    double theta_diff = normalizeAngle(measured_azimuths[i] - global_azimuth);
                    // ROS_INFO("    global_azimuth = %.3f, measured_azimuths = %.3f, theta_diff = %.3f", RAD2DEG(global_azimuth), RAD2DEG(measured_azimuths[i]), RAD2DEG(theta_diff));
                    sum_theta += theta_diff;
                    count++;
                }
            }

            if (count > 0)
            {
                float lidar_xX = sum_x / count;
                float lidar_yY = sum_y / count;
                float lidar_theta_T = normalizeAngle(sum_theta / count)+ M_PI;
                ROS_WARN("    MODE3 pose.x= %.3f y= %.3f theta= %.3f ", lidar_x + lidar_xX,lidar_y + lidar_yY, RAD2DEG(lidar_theta_T));
                // ROS_INFO("    Lidar theta END 22 = %.3f rad (%.3f deg)", lidar_theta_T, RAD2DEG(lidar_theta_T));

                // lidar_x += lidar_xX;
                // lidar_y += lidar_yY;
                // lidar_theta = lidar_theta_T;
            }
            else
            {
                ROS_INFO("Could not compute lidar position and orientation: no valid matches.");
            }
        }
        else
        {
            ROS_INFO("Could not compute lidar position and orientation: no pillars matched.");
        }
    }

    // Функция для визуализации кластеров, столбов и лидара в RViz (вызывается раз в секунду)
    void visualizeCallback(const ros::TimerEvent &)
    {
        // Добавляем цвета для каждой точки
        std_msgs::ColorRGBA color, color1, color2, color3;
        color1.r = 1.0;
        color1.g = 0.0;
        color1.b = 0.0;
        color1.a = 1.0; // Красный
        color2.r = 0.0;
        color2.g = 1.0;
        color2.b = 0.0;
        color2.a = 1.0; // Зеленый
        color3.r = 0.0;
        color3.g = 0.0;
        color3.b = 1.0;
        color3.a = 1.0; // Синий

        // Создаём маркер для кластеров
        visualization_msgs::Marker cluster_marker;
        cluster_marker.header.frame_id = "laser";                 // Система координат лидара
        cluster_marker.header.stamp = ros::Time::now();           // Текущая метка времени
        cluster_marker.ns = "clusters";                           // Пространство имён для кластеров
        cluster_marker.type = visualization_msgs::Marker::POINTS; // Тип маркера - точки
        cluster_marker.action = visualization_msgs::Marker::ADD;  // Действие - добавить
        cluster_marker.pose.orientation.w = 1.0;                  // Ориентация (без вращения)
        cluster_marker.scale.x = 0.03;                            // Размер точки по x (5 см)
        cluster_marker.scale.y = 0.03;                            // Размер точки по y
        // cluster_marker.color.r = 0.0;                             // Цвет - красный
        // cluster_marker.color.g = 1.0;                             // Цвет - зелёный (для отличия от столбов)
        // cluster_marker.color.b = 0.0;                             // Цвет - синий
        // cluster_marker.color.a = 1.0;                             // Прозрачность (непрозрачный)
        cluster_marker.id = 0; // Идентификатор маркера
        int aa = 1;

        // Заполняем маркер кластерами (локальные координаты для RViz)
        for (int i = 0; i < cluster_info_list.size(); i++)
        {
            if (aa == 1)
            {
                color = color2;
                aa = 0;
            }
            else
            {
                color = color3;
                aa = 1;
            }
            for (int j = 0; j < cluster_info_list[i].points.size(); j++)
            {
                geometry_msgs::Point point;
                point.x = cluster_info_list[i].points[j].x; // Локальная координата x
                point.y = cluster_info_list[i].points[j].y; // Локальная координата y
                point.z = 0.0;                              // Высота (z=0, так как 2D)
                cluster_marker.points.push_back(point);     // Добавляем точку в список
                cluster_marker.colors.push_back(color);     // Добавляем цвет в список
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
        // pillar_marker.color.r = 1.0;                                  // Цвет - красный
        // pillar_marker.color.g = 0.0;                                  // Цвет - зелёный
        // pillar_marker.color.b = 0.0;                                  // Цвет - синий
        // pillar_marker.color.a = 1.0;                                  // Прозрачность (непрозрачный)
        pillar_marker.id = 0; // Идентификатор маркера

        // Заполняем маркер столбами (локальные координаты для RViz)
        for (int i = 0; i < pillars.size(); i++)
        {
            if (pillars[i].match == true)
                color = color2;
            else
                color = color3;

            geometry_msgs::Point point;
            point.x = pillars[i].x_center;         // Локальная координата x центра столба
            point.y = pillars[i].y_center;         // Локальная координата y центра столба
            point.z = 0.0;                         // Высота (z=0, так как 2D)
            pillar_marker.points.push_back(point); // Добавляем точку в список
            pillar_marker.colors.push_back(color); // Добавляем цвет в список
        }

        // Создаём маркер для лидара с направлением в глобальной системе координат
        visualization_msgs::Marker lidar_marker;
        lidar_marker.header.frame_id = "laser";                // Используем систему laser для простоты
        lidar_marker.header.stamp = ros::Time::now();          // Текущая метка времени
        lidar_marker.ns = "lidar";                             // Пространство имён
        lidar_marker.type = visualization_msgs::Marker::ARROW; // Тип маркера - стрелка
        lidar_marker.action = visualization_msgs::Marker::ADD; // Действие - добавить
        lidar_marker.pose.position.x = lidar_x;                // Координата x лидара (глобальная, но в laser для простоты)
        lidar_marker.pose.position.y = lidar_y;                // Координата y лидара
        lidar_marker.pose.position.z = 0.0;                    // Высота (z=0, так как 2D)
        lidar_marker.pose.orientation.x = 0.0;
        lidar_marker.pose.orientation.y = 0.0;
        lidar_marker.pose.orientation.z = sin(lidar_theta / 2.0); // Кватернион для поворота
        lidar_marker.pose.orientation.w = cos(lidar_theta / 2.0);
        lidar_marker.scale.x = 0.5; // Длина стрелки
        lidar_marker.scale.y = 0.1; // Ширина стрелки
        lidar_marker.scale.z = 0.1; // Высота стрелки
        lidar_marker.color.r = 0.0; // Цвет - красный
        lidar_marker.color.g = 0.0; // Цвет - зелёный
        lidar_marker.color.b = 1.0; // Цвет - синий
        lidar_marker.color.a = 1.0; // Прозрачность (непрозрачный)
        lidar_marker.id = 0;        // Идентификатор маркера

        // Отправляем маркеры в RViz
        cluster_publisher.publish(cluster_marker);
        marker_publisher.publish(pillar_marker);
        lidar_publisher.publish(lidar_marker);

        // Выводим информацию о публикации
        ROS_INFO("    RVIZ pub %d clusters with %d points, %d pillars, and lidar position with orientation",
                 (int)cluster_info_list.size(), (int)cluster_marker.points.size(), (int)pillars.size());
    }
};

#endif