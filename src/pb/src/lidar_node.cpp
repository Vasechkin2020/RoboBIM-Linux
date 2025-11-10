#include <ros/ros.h>                   // Библиотека ROS для работы с узлами
#include <sensor_msgs/LaserScan.h>     // Сообщения для данных лидара
#include <visualization_msgs/Marker.h> // Сообщения для визуализации в RViz
#include <vector>                      // Стандартный вектор C++
#include <cmath>                       // Математические функции (sin, cos, sqrt)
#include <signal.h>                    // Для обработки Ctrl+C

#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
#include "lidar_code/config.h"
#include "lidar_code/code.h"
#include "lidar_code/pillar.h"
#include "lidar_code/pillarDetector.h"
#include "lidar_code/trilaterationSolver.h"
#include "lidar_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате

SPoseLidar g_poseLidar; // Позиции лидара по расчетам Центральная система координат

int main(int argc, char **argv) // Главная функция программы
{
    signal(SIGINT, stopProgram);         // Настраиваем обработку Ctrl+C
    ros::init(argc, argv, "lidar_node"); // Инициализируем ROS с именем узла "lidar_node"

    ros::NodeHandle nh;
    ros::Subscriber subscriber_Lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Lidar); // Подписка на данные лидара
    ros::Duration(1).sleep();                                                                               // Подождем пока все обьявится и инициализируется внутри ROS

    CPillar pillar;          // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов
    PillarDetector detector; // Создаём объект детектора столбов
    CTopic topic;            // Экземпляр класса для всех публикуемых топиков

    SDistDirect distDirect[4] = {
        {0.0, 0.0}, // Столб 1
        {0.0, 0.0}, // Столб 3
        {0.0, 0.0}, // Столб 2
        {0.0, 0.0}};

    SPose startPose;
    SPoint startPillar[4];
    readParam(startPose, startPillar); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

    g_poseLidar.mode = startPose;
    g_poseLidar.mode1 = startPose;
    g_poseLidar.mode2 = startPose;
    pillar.parsingPillar(startPillar); // Разбираем пришедшие данные Заполняем массив правильных координат.

    detector.setPoseLidar(startPose.x, startPose.y, startPose.th);   // Установка начальной позиции
    detector.setKnownPillars(0, startPillar[0].x, startPillar[0].y); // Разбираем пришедшие данные Заполняем массив правильных координат.
    detector.setKnownPillars(1, startPillar[1].x, startPillar[1].y);
    detector.setKnownPillars(2, startPillar[2].x, startPillar[2].y);
    detector.setKnownPillars(3, startPillar[3].x, startPillar[3].y);

    SPoint startPoint;
    startPoint.x = startPose.x;
    startPoint.y = startPose.y;
    TrilaterationSolver solver(startPoint, 0.8); // Инициализация решателя/Задаем начальную точку и вес скоторым будем учитывать расчет по углу. По расстоянияю принято всегда за 1.

    ros::Time timeStart = ros::Time::now(); // Время начала программы
    ros::Time timeLoop = ros::Time::now();  // Время начала текущего цикла

    std::time_t timeSec = static_cast<std::time_t>(timeStart.toSec()); // Преобразуем время в секундах в структуру tm
    std::tm *formattedTime = std::localtime(&timeSec);
    char buffer[80]; // Форматируем вывод
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", formattedTime);
    ROS_INFO("TIME START NODE current time: %s", buffer); // Выводим в консоль

    ros::Rate loop_rate(10);          // Создаём цикл с частотой 10 Гц
    while (ros::ok() && keep_running) // Пока ROS работает и не нажат Ctrl+C
    {
        timeLoop = ros::Time::now(); // Захватываем текущий момент времени начала цикла
        ros::spinOnce();             // Вызываем колбеки

        // Выполняется 10 Hz как ЛИДАР ПРИШЛЕТ ***************************************************************************************************************************************************
        if (flag_msgLidar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.  И знаем где истинные столбы
        {
            ROS_INFO("");
            ROS_ERROR("------------       flag_msgLidar    -------------");
            flag_msgLidar = false;
            // ROS_INFO("=== %.3f %.3f | %.3f %.3f | %.3f %.3f",g_poseLidar.mode1.x, g_poseLidar.mode.x, g_poseLidar.mode1.y, g_poseLidar.mode.y, g_poseLidar.mode1.th, g_poseLidar.mode.th);
            pillar.searchPillars(msg_lidar, g_poseLidar.mode); // Разбираем пришедшие данные и ищем там столбы.
            pillar.comparisonPillar();                         // Сопоставляем столбы
            // topic.publicationPillarAll(pillar);                // Публикуем всю обобщенную информацию по столб

            detector.scanCallback(msg_lidar, g_poseLidar.mode);
            // topic.visualizeClasters(detector.cluster_info_list); // Большой обьем данных. Лучше отключать
            // topic.visualizePillars(detector.pillars); // Визуализация найденых столбов
            // topic.visualizeLidar();

            calcDistDirect(distDirect, pillar, detector); // Обьединение сопоставленных столбов в итоговую таблицу. Дальше по этой таблице все считается

            g_poseLidar.mode1 = pillar.getLocationMode1(distDirect, g_poseLidar.mode); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            g_poseLidar.mode2 = pillar.getLocationMode2(distDirect, g_poseLidar.mode); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

            // if (isnan(g_poseLidar.mode2.x) || isnan(g_poseLidar.mode2.y) || isnan(g_poseLidar.mode2.th))
            // {
            //     ROS_ERROR("STOP MODE 1-2");
            //     exit(0);
            // }
            g_poseLidar.mode.x = g_poseLidar.mode1.x * 0.8 + g_poseLidar.mode2.x * 0.2 + g_poseLidar.mode3.x * 0.0; // Легкая комплементация двух методов расчета. Второй сильно волатильный
            g_poseLidar.mode.y = g_poseLidar.mode1.y * 0.8 + g_poseLidar.mode2.y * 0.2 + g_poseLidar.mode3.y * 0.0;
            // g_poseLidar.mode.th = g_poseLidar.mode1.th * 0.4 + g_poseLidar.mode2.th * 0.3 + g_poseLidar.mode3.th * 0.3;

            try
            {
                solver.clear_circles(); // <<< Очистка данных перед расчетом!

                // SPoint B = {4.0, 0.3}; // Маяк B
                // SPoint C = {0.0, 0.5}; // Маяк C
                // SPoint D = {0.5, 4.0}; // Маяк D
                // SPoint E = {5.0, 4.0}; // Маяк E

                // // Углы из трилатерации: МОДУЛИ УГЛОВ (Берем точные значения из Вашей отладки)
                // double angle_BAC = 94.15;  // Угол <BAC
                // double angle_CAD = 132.27; // Угол <CAD
                // double angle_DAE = 77.47;  // Угол <DAE
                // double angle_EAB = 56.10;  // Угол <EAB

                // solver.add_filtered_circle_from_angle(B, C, angle_BAC);
                // solver.add_filtered_circle_from_angle(C, D, angle_CAD);
                // solver.add_filtered_circle_from_angle(D, E, angle_DAE);
                // solver.add_filtered_circle_from_angle(E, B, angle_EAB);

                for (size_t i = 0; i < 4; i++)
                {
                    // Сбор окружностей 4 по расстоянию
                    SPoint point;
                    point.x = pillar.pillar[i].x_true;
                    point.y = pillar.pillar[i].y_true;
                    // double distance = pillar.pillar[i].distance_lidar + PILLAR_RADIUS;
                    // double distance2 = detector.matchPillar[i].distance + PILLAR_RADIUS;
                    double distanceFused = distDirect[i].distance + PILLAR_RADIUS;

                    solver.add_circle_from_distance(point, distanceFused); // Добавление окружности по дальности AB
                    // solver.add_circle_from_distance(point, distance2); // Добавление окружности по дальности AB
                }

                SPoint_Q AQ_found = solver.find_A_by_mnk(); // Запуск расчета положения
                solver.set_A_prev(AQ_found.A);
                g_poseLidar.mnkDist.x = AQ_found.A.x;
                g_poseLidar.mnkDist.y = AQ_found.A.y;
                g_poseLidar.quality_mknDist = AQ_found.quality;

                int j = 0;
                for (size_t i = 0; i < 4; i++)
                {
                    j++;
                    // Сбор окружностей 4 по углу
                    if (j < 4)
                    {
                        SPoint point1;
                        point1.x = pillar.pillar[i].x_true;
                        point1.y = pillar.pillar[i].y_true;
                        SPoint point2;
                        point2.x = pillar.pillar[j].x_true;
                        point2.y = pillar.pillar[j].y_true;
                        double check_angle = solver.calculate_angle_from_azimuths(pillar.pillar[i].azimuth, pillar.pillar[j].azimuth); // Расчет угла BAC по азимутам
                        solver.add_filtered_circle_from_angle(point1, point2, check_angle);                                            // Добавление окружности по углу BAC
                    }
                    else
                    {
                        SPoint point1;
                        point1.x = pillar.pillar[i].x_true;
                        point1.y = pillar.pillar[i].y_true;
                        SPoint point2;
                        point2.x = pillar.pillar[0].x_true;
                        point2.y = pillar.pillar[0].y_true;
                        double check_angle = solver.calculate_angle_from_azimuths(pillar.pillar[i].azimuth, pillar.pillar[0].azimuth); // Расчет угла BAC по азимутам
                        solver.add_filtered_circle_from_angle(point1, point2, check_angle);                                            // Добавление окружности по углу BAC
                    }
                }
            }
            catch (const std::invalid_argument &e)
            {
                fprintf(stderr, "Error collecting circles: %s\n", e.what()); // Вывод ошибки
                return 1;                                                    // Код ошибки
            }

            SPoint_Q BQ_found = solver.find_A_by_mnk(); // Запуск расчета положения
            g_poseLidar.mnkFused.x = BQ_found.A.x;
            g_poseLidar.mnkFused.y = BQ_found.A.y;
            g_poseLidar.quality_mknFused = BQ_found.quality;

            solver.set_A_prev(BQ_found.A);

            // // Перед комплментацией углы нужно привести к 360 градусов чтобы правильно усреднять

            // float angle1, angle2, angle3, angleSum;
            // (g_poseLidar.mode1.th < 0) ? angle1 = g_poseLidar.mode1.th + 360 : angle1 = g_poseLidar.mode1.th;
            // (g_poseLidar.mode2.th < 0) ? angle2 = g_poseLidar.mode2.th + 360 : angle2 = g_poseLidar.mode2.th;
            // (g_poseLidar.mode3.th < 0) ? angle3 = g_poseLidar.mode3.th + 360 : angle3 = g_poseLidar.mode3.th;

            // ROS_INFO("    th1 = %.3f th2 = %.3f th3 = %.3f | angle1 = %.3f angle2 = %.3f angle3 = %.3f ",
            //          g_poseLidar.mode1.th, g_poseLidar.mode2.th, g_poseLidar.mode3.th, angle1, angle2, angle3);

            // angleSum = angle1 * 0.4 + angle2 * 0.3 + angle3 * 0.3;
            // (angleSum > 180) ? angleSum = angleSum - 360 : angleSum = angleSum;           // Обратно после усреденения превращаем в +-180
            // float angleSum2 = RAD2DEG(detector.normalizeAngle(DEG2RAD(angleSum + 0.66))); // Add offset angle lidar

            // float angleSum4 = RAD2DEG(detector.normalizeAngle(DEG2RAD(angleSum3 + 0.66))); // Add offset angle lidar
            // g_poseLidar.mode.th = angleSum3;
            // ROS_INFO("    angleSum = %.3f angleSum2 = %.3f angleSum3 = %.3f ", angleSum, angleSum2, angleSum3);

            g_poseLidar.mode.th = complementAngle(g_poseLidar.mode1.th, g_poseLidar.mode2.th, g_poseLidar.mode3.th, 0.8, 0.2, 0.0, 0.0); // Функйия комплементации 3 углов с разными весвми и добавление поправки offset по лидару

            ROS_WARN("    g_poseLidar.mode.x = %.3f th = %.3f th = %.3f ", g_poseLidar.mode.x, g_poseLidar.mode.y, g_poseLidar.mode.th);

            // g_poseLidar.mode.th = g_poseLidar.mode.th * COMPLEMENTARN + ((g_poseLidar.mode1.th + g_poseLidar.mode2.th) / 2.0) * (1 - COMPLEMENTARN);

            // g_transformGlobal2Local.x = g_poseLidar.mode.x;
            // g_transformGlobal2Local.y = g_poseLidar.mode.y;
            // g_transformGlobal2Local.th = g_poseLidar.mode.th;

            topic.publicationPoseLidar();    // Публикуем все варианты расчета позиций mode 0.1.2.3.4
            topic.visualPillarPoint(pillar); // Отображение места размещения столбов

            topic.transformLidar();           // Публикуем трансформации систем координат , задаем по какому расчету трансформировать
            topic.visualStartPose(startPose); // Отобращение стрелкой где начало стартовой позиции и куда направлен нос платформы
            // topic.visualPublishOdomMode_1();  // Отобращение стрелкой где начало и куда смотрит в Mode 0 1 2
            // topic.visualPublishOdomMode_2();  // Отобращение стрелкой где начало и куда смотрит в Mode 0 1 2

            topic.visualPoseAngleLaser();
        }

        timeCycle(timeStart, timeLoop); // Выводим справочно время работы цикла и время с начала работы программы
        loop_rate.sleep();              // Ждём, чтобы поддерживать частоту 10 Гц
    }

    ROS_INFO("Program stopped");
    return 0;
}
