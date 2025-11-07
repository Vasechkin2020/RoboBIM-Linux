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
#include "lidar_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате

SPoseLidar g_poseLidar;        // Позиции лидара по расчетам Центральная система координат

int main(int argc, char **argv)// Главная функция программы
{
    signal(SIGINT, stopProgram);         // Настраиваем обработку Ctrl+C
    ros::init(argc, argv, "lidar_node"); // Инициализируем ROS с именем узла "lidar_node"

    ros::NodeHandle nh;
    ros::Subscriber subscriber_Lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Lidar); // Подписка на данные лидара
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

    g_poseLidar.mode = startPose;
    g_poseLidar.mode1 = startPose;
    g_poseLidar.mode2 = startPose;
    pillar.parsingPillar(startPillar); // Разбираем пришедшие данные Заполняем массив правильных координат.

    detector.setPoseLidar(startPose.x, startPose.y, startPose.th); // Установка начальной позиции
    detector.setKnownPillars(0, startPillar[0].x, startPillar[0].y); // Разбираем пришедшие данные Заполняем массив правильных координат.
    detector.setKnownPillars(1, startPillar[1].x, startPillar[1].y);
    detector.setKnownPillars(2, startPillar[2].x, startPillar[2].y);
    detector.setKnownPillars(3, startPillar[3].x, startPillar[3].y);

    ros::Time timeStart = ros::Time::now(); // Время начала программы
    ros::Time timeLoop = ros::Time::now();  // Время начала текущего цикла

    std::time_t timeSec = static_cast<std::time_t>(timeStart.toSec()); // Преобразуем время в секундах в структуру tm
    std::tm *formattedTime = std::localtime(&timeSec);
    char buffer[80]; // Форматируем вывод
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", formattedTime);
    ROS_INFO("TIME START NODE current time: %s", buffer); // Выводим в консоль

    ros::Rate loop_rate(20);          // Создаём цикл с частотой 10 Гц
    while (ros::ok() && keep_running) // Пока ROS работает и не нажат Ctrl+C
    {
        timeLoop = ros::Time::now(); // Захватываем текущий момент времени начала цикла
        ros::spinOnce();             // Вызываем колбеки

        // Выполняется 10 Hz как ЛИДАР ПРИШЛЕТ ***************************************************************************************************************************************************
        if (flag_msgLidar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.  И знаем где истинные столбы
        {
            ROS_INFO("");
            ROS_INFO("------------       flag_msgLidar    -------------");
            flag_msgLidar = false;
            // ROS_INFO("=== %.3f %.3f | %.3f %.3f | %.3f %.3f",g_poseLidar.mode1.x, g_poseLidar.mode.x, g_poseLidar.mode1.y, g_poseLidar.mode.y, g_poseLidar.mode1.th, g_poseLidar.mode.th);
            pillar.searchPillars(msg_lidar, g_poseLidar.mode); // Разбираем пришедшие данные и ищем там столбы.
            pillar.comparisonPillar();                        // Сопоставляем столбы
            // topic.publicationPillarAll(pillar);                // Публикуем всю обобщенную информацию по столб

            detector.scanCallback(msg_lidar, g_poseLidar.mode);
            // topic.visualizeClasters(detector.cluster_info_list); // Большой обьем данных. Лучше отключать
            topic.visualizePillars(detector.pillars); // Визуализация найденых столбов
            topic.visualizeLidar();

            calcDistDirect(distDirect, pillar, detector); // Обьединение сопоставленных столбов в итоговую таблицу. Дальше по этой таблице все считается

            g_poseLidar.mode1 = pillar.getLocationMode1(distDirect, g_poseLidar.mode); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            g_poseLidar.mode2 = pillar.getLocationMode2(distDirect, g_poseLidar.mode); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

            if (isnan(g_poseLidar.mode2.x) || isnan(g_poseLidar.mode2.y) || isnan(g_poseLidar.mode2.th))
            {
                ROS_ERROR("STOP MODE 1-2");
                exit(0);
            }
            g_poseLidar.mode.x = g_poseLidar.mode1.x * 0.8 + g_poseLidar.mode2.x * 0.1 + g_poseLidar.mode3.x * 0.1; // Легкая комплементация двух методов расчета. Второй сильно волатильный
            g_poseLidar.mode.y = g_poseLidar.mode1.y * 0.8 + g_poseLidar.mode2.y * 0.1 + g_poseLidar.mode3.y * 0.1;
            // g_poseLidar.mode.th = g_poseLidar.mode1.th * 0.4 + g_poseLidar.mode2.th * 0.3 + g_poseLidar.mode3.th * 0.3;

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

            g_poseLidar.mode.th = complementAngle(g_poseLidar.mode1.th, g_poseLidar.mode2.th, g_poseLidar.mode3.th, 0.4, 0.3, 0.3, 0.66);// Функйия комплементации 3 углов с разными весвми и добавление поправки offset по лидару

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
