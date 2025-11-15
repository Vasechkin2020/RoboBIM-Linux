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

    nh.param<double>("/pb_config/lidar/bias", lidar_bias, 0.0); // Считываем bias, по умолчанию 0
    ROS_INFO("--- Start node with parametrs: /pb_config/lidar/bias = %+8.3f deg", lidar_bias);
    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS

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

    g_poseLidar.modeDist = startPose;
    g_poseLidar.modeAngle = startPose;
    g_poseLidar.modeFused = startPose;
    g_poseLidar.mnkDist = startPose;
    g_poseLidar.mnkAngle = startPose;
    g_poseLidar.mnkFused = startPose;

    pillar.parsingPillar(startPillar); // Разбираем пришедшие данные Заполняем массив правильных координат.

    detector.setPoseLidar(startPose.x, startPose.y, startPose.th);   // Установка начальной позиции
    detector.setKnownPillars(0, startPillar[0].x, startPillar[0].y); // Разбираем пришедшие данные Заполняем массив правильных координат.
    detector.setKnownPillars(1, startPillar[1].x, startPillar[1].y);
    detector.setKnownPillars(2, startPillar[2].x, startPillar[2].y);
    detector.setKnownPillars(3, startPillar[3].x, startPillar[3].y);

    SPoint startPoint;
    startPoint.x = startPose.x;
    startPoint.y = startPose.y;
    TrilaterationSolver solver(startPoint); // Инициализация решателя/Задаем начальную точку и вес скоторым будем учитывать расчет по углу. По расстоянияю принято всегда за 1.

    ros::Time timeStart = ros::Time::now(); // Время начала программы
    ros::Time timeLoop = ros::Time::now();  // Время начала текущего цикла

    std::time_t timeSec = static_cast<std::time_t>(timeStart.toSec()); // Преобразуем время в секундах в структуру tm
    std::tm *formattedTime = std::localtime(&timeSec);
    char buffer[80]; // Форматируем вывод
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", formattedTime);
    ROS_INFO("TIME START NODE current time: %s", buffer); // Выводим в консоль

    ros::Rate loop_rate(100);          // Создаём цикл с частотой 10 Гц
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
            // ROS_INFO("=== %+8.3f %+8.3f | %+8.3f %+8.3f | %+8.3f %+8.3f",g_poseLidar.modeDist.x, g_poseLidar.mode.x, g_poseLidar.modeDist.y, g_poseLidar.mode.y, g_poseLidar.modeDist.th, g_poseLidar.mode.th);
            pillar.searchPillars(msg_lidar, g_poseLidar.mnkFused); // Разбираем пришедшие данные и ищем там столбы.
            pillar.comparisonPillar();                             // Сопоставляем столбы
            // topic.publicationPillarAll(pillar);                // Публикуем всю обобщенную информацию по столб

            detector.scanCallback(msg_lidar, g_poseLidar.mnkFused);
            // topic.visualizeClasters(detector.cluster_info_list); // Большой обьем данных. Лучше отключать
            // topic.visualizePillars(detector.pillars); // Визуализация найденых столбов
            // topic.visualizeLidar();

            calcDistDirect(distDirect, pillar, detector); // Обьединение сопоставленных столбов в итоговую таблицу. Дальше по этой таблице все считается
            // Сначала по углу а потом пересчитываем по растоянию инача ужу входный dist другой становиться
            g_poseLidar.modeAngle = pillar.getLocationmodeAngle(distDirect, g_poseLidar.modeDist, 0.16); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            g_poseLidar.modeDist = pillar.getLocationmodeDist(distDirect, g_poseLidar.modeDist, 0.16);   // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

            // if (isnan(g_poseLidar.modeAngle.x) || isnan(g_poseLidar.modeAngle.y) || isnan(g_poseLidar.modeAngle.th))
            // {
            //     ROS_ERROR("STOP MODE 1-2");
            //     exit(0);
            // }
            g_poseLidar.modeFused.x = g_poseLidar.modeDist.x * 0.8 + g_poseLidar.modeAngle.x * 0.2 + g_poseLidar.modeClaster.x * 0.0; // Легкая комплементация двух методов расчета. Второй сильно волатильный
            g_poseLidar.modeFused.y = g_poseLidar.modeDist.y * 0.8 + g_poseLidar.modeAngle.y * 0.2 + g_poseLidar.modeClaster.y * 0.0;
            // g_poseLidar.mode.th = g_poseLidar.modeDist.th * 0.4 + g_poseLidar.modeAngle.th * 0.3 + g_poseLidar.modeClaster.th * 0.3;

            try
            {
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
                int count_circle = 0;
                SPoint_Q AQ_found;
                SPoint_Q BQ_found;
                SPoint_Q CQ_found;
                SPoint point1;
                SPoint point2;
                /*
                //-----------------------------------------------------------------------------------------------------------------------------
                printf("======================================== 1 ==========================================\n");
                solver.clear_circles(); // <<< Очистка данных перед расчетом!
                for (size_t i = 0; i < 4; i++)
                {
                    // Сбор окружностей 4 по расстоянию
                    SPoint point;
                    if (distDirect[i].count != 0) // Только если столб сопоставленный то добавляем
                    {
                        point.x = distDirect[i].x_true;
                        point.y = distDirect[i].y_true;
                        double distanceFused = distDirect[i].distance + PILLAR_RADIUS;
                        solver.add_circle_from_distance(point, distanceFused); // Добавление окружности по дальности AB
                        count_circle++;                                        // Считаем сколько окружностей добавили
                    }
                }
                if (count_circle > 2) // Если больше 2 окружностей то считаем
                {
                    // SPoint_Q AQ_found = solver.find_A_by_mnk_simple(); // Запуск расчета положения
                    AQ_found = solver.find_A_by_mnk_robust(); // Запуск расчета положения
                    g_poseLidar.mnkDist.x = AQ_found.A.x;
                    g_poseLidar.mnkDist.y = AQ_found.A.y;
                    g_poseLidar.quality_mknDist = AQ_found.quality;
                }
                printf("======================================== 2 ==========================================\n");
                //-----------------------------------------------------------------------------------------------------------------------------
                solver.clear_circles(); // <<< Очистка данных перед расчетом!
                count_circle = 0;           // Обнуляем счетчик
                for (int i = 0; i < 3; i++) //        Перебираем столбы, и для каждой пары формируем окружность
                {
                    for (int j = i + 1; j < 4; j++)
                    {
                        if (distDirect[i].count != 0 && distDirect[j].count != 0) // Только если оба столба хорошие (сопоставленные) то добавляем
                        {
                            point1.x = distDirect[i].x_true; // Формируем пары столбов, получаем угол между ними и если он в пределах от 30 до 150 градусов то считаем окружность которую потом переберем
                            point1.y = distDirect[i].y_true;
                            point2.x = distDirect[j].x_true;
                            point2.y = distDirect[j].y_true;
                            double check_angle = solver.calculate_angle_from_azimuths(distDirect[i].direction, distDirect[j].direction); // Расчет угла BAC по азимутам
                            // printf("check angle = %8.3f = %8.3f => ", check_angle, a1);
                            solver.add_filtered_circle_from_angle(point1, point2, check_angle); // Добавление окружности по углу BAC
                            count_circle++;
                        }
                    }
                }
                if (count_circle > 2) // Если больше 2 окружностей то считаем
                {
                    // SPoint_Q BQ_found = solver.find_A_by_mnk_simple(); // Запуск расчета положения
                    BQ_found = solver.find_A_by_mnk_robust(); // Запуск расчета положения
                    g_poseLidar.mnkAngle.x = BQ_found.A.x;
                    g_poseLidar.mnkAngle.y = BQ_found.A.y;
                    g_poseLidar.quality_mknAngle = BQ_found.quality;
                }
                //-----------------------------------------------------------------------------------------------------------------------------
                */
                printf("======================================== 3 ==========================================\n");
                count_circle = 0;       // Обнуляем счетчик
                solver.clear_circles(); // <<< Очистка данных перед расчетом!
                for (size_t i = 0; i < 4; i++)
                {
                    // Сбор окружностей 4 по расстоянию
                    if (distDirect[i].count != 0) // Только если столб сопоставленный то добавляем
                    {
                        SPoint point;
                        point.x = distDirect[i].x_true;
                        point.y = distDirect[i].y_true;
                        double distanceFused = distDirect[i].distance + PILLAR_RADIUS;
                        solver.add_circle_from_distance(point, distanceFused); // Добавление окружности по дальности AB
                        // solver.add_circle_from_distance(point, distance2); // Добавление окружности по дальности AB
                        count_circle++;
                    }
                }
                for (int i = 0; i < 3; i++) //        Перебираем столбы, и для каждой пары формируем окружность
                {
                    for (int j = i + 1; j < 4; j++)
                    {
                        if (distDirect[i].count != 0 && distDirect[j].count != 0) // Только если оба столба хорошие (сопоставленные) то добавляем
                        {
                            point1.x = distDirect[i].x_true; // Формируем пары столбов, получаем угол между ними и если он в пределах от 30 до 150 градусов то считаем окружность которую потом переберем
                            point1.y = distDirect[i].y_true;
                            point2.x = distDirect[j].x_true;
                            point2.y = distDirect[j].y_true;
                            double check_angle = solver.calculate_angle_from_azimuths(distDirect[i].direction, distDirect[j].direction); // Расчет угла BAC по азимутам
                            solver.add_filtered_circle_from_angle(point1, point2, check_angle);                                          // Добавление окружности по углу BAC
                            count_circle++;
                        }
                    }
                }

                CQ_found = solver.find_A_by_mnk_robust(); // Запуск расчета положения
                solver.set_A_prev(CQ_found.A);            // ТОчка опорная для следующего расчета

                //-----------------------------------------------------------------------------------------------------------------------------
                float k_mnk = 0.5; // Коефициент для фильтра
                g_poseLidar.mnkFused.x = g_poseLidar.mnkFused.x * k_mnk + CQ_found.A.x * (1 - k_mnk);
                g_poseLidar.mnkFused.y = g_poseLidar.mnkFused.y * k_mnk + CQ_found.A.y * (1 - k_mnk);
                g_poseLidar.quality_mknFused = g_poseLidar.quality_mknFused * k_mnk + CQ_found.quality * (1 - k_mnk);
                printf("======================================== 4  ==========================================\n");

                std::vector<SPoint> orientation_beacons; // Вектор координат маяков
                std::vector<double> lidar_angles_deg;    // Вектор измеренных углов

                for (size_t i = 0; i < 4; i++) // ЗАПОЛНЕНИЕ ВЕКТОРОВ ИЗ ПЕРЕМЕННЫХ
                {
                    if (distDirect[i].count != 0) // Только если столб сопоставленный то добавляем
                    {
                        SPoint point;
                        point.x = distDirect[i].x_true;
                        point.y = distDirect[i].y_true;
                        orientation_beacons.push_back(point); // Добавляем маяк

                        // double angle_to_point = normalize_and_invert_sign_deg(distDirect[i].direction); // Преобразуем в нормальный вид +-180. Плюс против часовой

                        double convert;
                        convert = -distDirect[i].direction;
                        if (convert < -180)
                            convert = 360 + convert;

                        lidar_angles_deg.push_back(convert); // Добавляем направление на маяк

                        printf("=== direction = %+8.3f  convert = %+8.3f \n", distDirect[i].direction, convert);
                    }
                }
                // 1. Вызываем метод расчета ориентации, используя найденную позицию
                double calculated_orientation = solver.get_lidar_orientation(
                    CQ_found.A,          // Найденное положение (A_final из Pass 2)
                    orientation_beacons, // Координаты маяков (заполнены из переменных)
                    lidar_angles_deg     // Измеренные углы лидара (заполнены из переменных)
                );
                // calculated_orientation = normalize_and_invert_sign_deg(calculated_orientation); // Исправление что плюс по часовой Подгонка как уж есть
                // 2. Вывод результата
                // printf("\n--- SUMMARY TEST 4 (ORIENTATION) ---\n");                         // Output summary 4
                printf("    SUMMARY TEST 4 (ORIENTATION) Position A =>: (%+8.3f, %+8.3f)\n", CQ_found.A.x, CQ_found.A.y); // Output result A
                printf("    Calculated Orientation Psi: %+8.3f deg\n", calculated_orientation);                           // Output calculated orientation
                // printf("--------------------------------------\n");                         // Separator
                // g_poseLidar.mnkFused.th = g_poseLidar.mnkFused.th * k_mnk + calculated_orientation * (1 - k_mnk);
                g_poseLidar.mnkFused.th = solver.complementary_filter_angle_deg(g_poseLidar.mnkFused.th, calculated_orientation, k_mnk);

                // g_poseLidar.mnkDist.th = solver.get_lidar_orientation(AQ_found.A, orientation_beacons, lidar_angles_deg); // Вывод углов без фильтрации
                // g_poseLidar.mnkAngle.th = solver.get_lidar_orientation(BQ_found.A, orientation_beacons, lidar_angles_deg);

                ROS_WARN("    mnkFused x= %+8.3f y= %+8.3f th= %+8.3f ", g_poseLidar.mnkFused.x, g_poseLidar.mnkFused.y, g_poseLidar.mnkFused.th);
                printf("======================================== END  ==========================================\n");
            }
            catch (const std::invalid_argument &e)
            {
                fprintf(stderr, "Error collecting circles: %s\n", e.what()); // Вывод ошибки
                return 1;                                                    // Код ошибки
            }

            // // Перед комплментацией углы нужно привести к 360 градусов чтобы правильно усреднять

            // float angle1, angle2, angle3, angleSum;
            // (g_poseLidar.modeDist.th < 0) ? angle1 = g_poseLidar.modeDist.th + 360 : angle1 = g_poseLidar.modeDist.th;
            // (g_poseLidar.modeAngle.th < 0) ? angle2 = g_poseLidar.modeAngle.th + 360 : angle2 = g_poseLidar.modeAngle.th;
            // (g_poseLidar.modeClaster.th < 0) ? angle3 = g_poseLidar.modeClaster.th + 360 : angle3 = g_poseLidar.modeClaster.th;

            // ROS_INFO("    th1 = %+8.3f th2 = %+8.3f th3 = %+8.3f | angle1 = %+8.3f angle2 = %+8.3f angle3 = %+8.3f ",
            //          g_poseLidar.modeDist.th, g_poseLidar.modeAngle.th, g_poseLidar.modeClaster.th, angle1, angle2, angle3);

            // angleSum = angle1 * 0.4 + angle2 * 0.3 + angle3 * 0.3;
            // (angleSum > 180) ? angleSum = angleSum - 360 : angleSum = angleSum;           // Обратно после усреденения превращаем в +-180
            // float angleSum2 = RAD2DEG(detector.normalizeAngle(DEG2RAD(angleSum + 0.66))); // Add offset angle lidar

            // float angleSum4 = RAD2DEG(detector.normalizeAngle(DEG2RAD(angleSum3 + 0.66))); // Add offset angle lidar
            // g_poseLidar.mode.th = angleSum3;
            // ROS_INFO("    angleSum = %+8.3f angleSum2 = %+8.3f angleSum3 = %+8.3f ", angleSum, angleSum2, angleSum3);

            // g_poseLidar.modeFused.th = complementAngle(g_poseLidar.modeDist.th, g_poseLidar.modeAngle.th, g_poseLidar.modeClaster.th, 0.8, 0.2, 0.0, 0.0); // Функйия комплементации 3 углов с разными весвми и добавление поправки offset по лидару

            // ROS_WARN("    g_poseLidar.modeFused.x = %+8.3f th = %+8.3f th = %+8.3f ", g_poseLidar.modeFused.x, g_poseLidar.modeFused.y, g_poseLidar.modeFused.th);

            // g_poseLidar.mode.th = g_poseLidar.mode.th * COMPLEMENTARN + ((g_poseLidar.modeDist.th + g_poseLidar.modeAngle.th) / 2.0) * (1 - COMPLEMENTARN);

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
            timeCycle(timeStart, timeLoop); // Выводим справочно время работы цикла и время с начала работы программы
        }

        loop_rate.sleep();              // Ждём, чтобы поддерживать частоту 10 Гц
    }

    ROS_INFO("Program stopped");
    return 0;
}
