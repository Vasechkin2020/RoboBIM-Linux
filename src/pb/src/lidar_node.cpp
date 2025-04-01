#include <ros/ros.h>                   // Библиотека ROS для работы с узлами
#include <sensor_msgs/LaserScan.h>     // Сообщения для данных лидара
#include <visualization_msgs/Marker.h> // Сообщения для визуализации в RViz
#include <vector>                      // Стандартный вектор C++
#include <cmath>                       // Математические функции (sin, cos, sqrt)
#include <signal.h>                    // Для обработки Ctrl+C

#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
#include "lidar_code/config.h"
#include "lidar_code/pillar.h"
#include "lidar_code/pillarDetector.h"

SPoseLidar g_poseLidar;        // Позиции лидара по расчетам Центральная система координат
SPose g_transformGlobal2Local; // Система трансформации из одной позиции в другую
#define COMPLEMENTARN 0.1

#include "lidar_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате

int keep_running = 1;                       // Переменная для остановки программы по Ctrl+C
sensor_msgs::LaserScan::ConstPtr msg_lidar; // Перемеенная в которую сохраняем данные лидара из сообщения

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР и ФУНКЦИЙ **********************************
void callback_Lidar(sensor_msgs::LaserScan::ConstPtr msg);                             //
void timeCycle(ros::Time timeStart_, ros::Time timeNow_);                              // Выводим справочно время работы цикла
void readParam(SPose &startPose, SPoint *startPillar);                                 // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
static void stopProgram(int signal);                                                   // Функция, которая срабатывает при нажатии Ctrl+C
void calcDistDirect(SDistDirect *distDirect, CPillar pillar, PillarDetector detector); // Обьединение сопоставленных столбов в итоговую таблицу. Дальше по этой таблице все считается

// Главная функция программы
int main(int argc, char **argv)
{
    signal(SIGINT, stopProgram);         // Настраиваем обработку Ctrl+C
    ros::init(argc, argv, "lidar_node"); // Инициализируем ROS с именем узла "lidar_node"

    ros::NodeHandle nh;
    ros::Subscriber subscriber_Lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Lidar);
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

    detector.changePoseLidar(startPose.x, startPose.y, startPose.th);
    detector.changeKnownPillars(0, startPillar[0].x, startPillar[0].y); // Разбираем пришедшие данные Заполняем массив правильных координат.
    detector.changeKnownPillars(1, startPillar[1].x, startPillar[1].y);
    detector.changeKnownPillars(2, startPillar[2].x, startPillar[2].y);
    detector.changeKnownPillars(3, startPillar[3].x, startPillar[3].y);

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
        ros::spinOnce();             // Обрабатываем входящие сообщения

        // Выполняется 10 Hz как ЛИДАР ПРИШЛЕТ ***************************************************************************************************************************************************
        if (flag_msgLidar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.  И знаем где истинные столбы
        {
            ROS_INFO("------------       flag_msgLidar    -------------");
            flag_msgLidar = false;
            // ROS_INFO("=== %.3f %.3f | %.3f %.3f | %.3f %.3f",g_poseLidar.mode1.x, g_poseLidar.mode.x, g_poseLidar.mode1.y, g_poseLidar.mode.y, g_poseLidar.mode1.th, g_poseLidar.mode.th);
            pillar.parsingLidar(msg_lidar, g_poseLidar.mode); // Разбираем пришедшие данные и ищем там столбы.
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
            g_poseLidar.mode.x = g_poseLidar.mode1.x * 0.9 + g_poseLidar.mode2.x * 0.1; // Легкая комплементация двух методов расчета. Второй сильно волатильный
            g_poseLidar.mode.y = g_poseLidar.mode1.y * 0.9 + g_poseLidar.mode2.y * 0.1;
            g_poseLidar.mode.th = g_poseLidar.mode.th * COMPLEMENTARN + ((g_poseLidar.mode1.th + g_poseLidar.mode2.th) / 2.0) * (1 - COMPLEMENTARN);

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
//==================================== РЕАЛИЗАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ =================================================

void callback_Lidar(sensor_msgs::LaserScan::ConstPtr msg)
{
    msg_lidar = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
    flag_msgLidar = true;
}

// Выводим справочно время работы цикла
void timeCycle(ros::Time timeStart_, ros::Time timeLoop_)
{
    ros::Duration durationStart = ros::Time::now() - timeStart_;                                                                                        // Находим разницу между началом и концом
    ros::Duration durationLoop = ros::Time::now() - timeLoop_;                                                                                          // Находим разницу между началом и концом
    ROS_INFO("    dt Start = %.3f sec | dt last cycle = %.3f sec and %d nanoseconds ", durationStart.toSec(), durationLoop.toSec(), durationLoop.nsec); // Время цикла в милисекундах
    // ROS_INFO("Current time: %d seconds and %d nanoseconds", durationLoop.sec, durationLoop.nsec);
    // if (dtEnd > 5)                                      // Если цикл занял бользе 5 милисекунд значит что не уложились в 200 Нz
    //     ROS_INFO("    !!! cycle = %8.3f msec", dtEnd);  // Время цикла в милисекундах
    // else
}

// Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
void readParam(SPose &startPose, SPoint *startPillar)
{
    ROS_INFO("+++ readParam");
    ros::NodeHandle nh_private("~");
    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим

    //<!-- Указываем стартовую позицию робота. В какое место поставили и куда направили-->
    if (!nh_private.getParam("x", startPose.x))
        startPose.x = 0.11;
    if (!nh_private.getParam("y", startPose.y))
        startPose.y = 0.11;
    if (!nh_private.getParam("theta", startPose.th))
        startPose.th = 0.11;

    g_transformGlobal2Local.x = startPose.x;
    g_transformGlobal2Local.y = startPose.y;
    g_transformGlobal2Local.th = startPose.th + 180;

    //<!-- Указываем места расположения столбов на локальной карте -->
    if (!nh_private.getParam("x0", startPillar[0].x))
        startPillar[0].x = 0.11;
    if (!nh_private.getParam("y0", startPillar[0].y))
        startPillar[0].y = 0.11;

    if (!nh_private.getParam("x1", startPillar[1].x))
        startPillar[1].x = 1.11;
    if (!nh_private.getParam("y1", startPillar[1].y))
        startPillar[1].y = 1.11;

    if (!nh_private.getParam("x2", startPillar[2].x))
        startPillar[2].x = 2.11;
    if (!nh_private.getParam("y2", startPillar[2].y))
        startPillar[2].y = 2.11;

    if (!nh_private.getParam("x3", startPillar[3].x))
        startPillar[3].x = 3.11;
    if (!nh_private.getParam("y3", startPillar[3].y))
        startPillar[3].y = 3.11;

    ROS_INFO("    startPose x = %.3f y = %.3f th = %.3f gradus", startPose.x, startPose.y, startPose.th);
    ROS_INFO("    g_transformGlobal2Local x = %.3f y = %.3f th = %.3f gradus", g_transformGlobal2Local.x, g_transformGlobal2Local.y, g_transformGlobal2Local.th);
    ROS_INFO("    start PillarPose");
    ROS_INFO("    x0= %.3f y0 = %.3f", startPillar[0].x, startPillar[0].y);
    ROS_INFO("    x1= %.3f y1 = %.3f", startPillar[1].x, startPillar[1].y);
    ROS_INFO("    x2= %.3f y2 = %.3f", startPillar[2].x, startPillar[2].y);
    ROS_INFO("    x3= %.3f y3 = %.3f", startPillar[3].x, startPillar[3].y);
    ROS_INFO("--- readParam");
}
// Функция, которая срабатывает при нажатии Ctrl+C
static void stopProgram(int signal)
{
    keep_running = 0; // Устанавливаем флаг, чтобы остановить цикл
    ros::shutdown();  // Завершаем работу ROS
}

// Обьединение сопоставленных столбов в итоговую таблицу. Дальше по этой таблице все считается
void calcDistDirect(SDistDirect *distDirect, CPillar pillar, PillarDetector detector)
{
    int sum = 0;
    static int flagFirst = 0;
    for (size_t i = 0; i < 4; i++)
    {
        distDirect[i].distance = 0;
        distDirect[i].direction = 0;
        distDirect[i].count = 0;
        // ROS_INFO("x_true = %.3f x_true = %.3f  | y_true = %.3f y_true = %.3f  | distance = %.5f distance = %.5f  | direction = %.3f direction = %.3f",
        //          pillar.pillar[i].x_true, detector.matchPillar[i].x_global,
        //          pillar.pillar[i].y_true, detector.matchPillar[i].y_global,
        //          pillar.pillar[i].distance_lidar, detector.matchPillar[i].distance,
        //          pillar.pillar[i].azimuth, detector.matchPillar[i].direction);
        if (pillar.pillar[i].status) // Теперь в зависимости от статуса заполняем табличку
        {
            distDirect[i].x_true = pillar.pillar[i].x_true;
            distDirect[i].y_true = pillar.pillar[i].y_true;
            distDirect[i].distance = pillar.pillar[i].distance_lidar;
            distDirect[i].direction = pillar.pillar[i].azimuth;
            distDirect[i].count = 1;
        }
        float count = distDirect[i].count + detector.matchPillar[i].count;
        sum += count;

        if (count > 0)
        {
            distDirect[i].distance = (distDirect[i].distance + detector.matchPillar[i].distance) / count;
            distDirect[i].direction = (distDirect[i].direction + detector.matchPillar[i].direction) / count;
        }
        // ROS_INFO("x_true = %.3f y_true = %.3f distance = %.5f direction = %.3f", distDirect[i].x_true,distDirect[i].y_true, distDirect[i].distance, distDirect[i].direction);

        // g_poseLidar.azimut[i] = distDirect[i].direction; // Индивидуальные углы наведения
        if (flagFirst == 0)
        {
            g_poseLidar.azimut[i] = distDirect[i].direction;
        }
        else
            g_poseLidar.azimut[i] = g_poseLidar.azimut[i] * COMPLEMENTARN + distDirect[i].direction * (1 - COMPLEMENTARN);
    }
    flagFirst = 1;
    ROS_INFO("    sum = %i", sum);
    g_poseLidar.countMatchPillar = sum;
}