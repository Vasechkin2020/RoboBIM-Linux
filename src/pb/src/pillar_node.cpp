#include <ros/ros.h>                   // Библиотека ROS для работы с узлами
#include <sensor_msgs/LaserScan.h>     // Сообщения для данных лидара
#include <visualization_msgs/Marker.h> // Сообщения для визуализации в RViz
#include <vector>                      // Стандартный вектор C++
#include <cmath>                       // Математические функции (sin, cos, sqrt)
#include <signal.h>                    // Для обработки Ctrl+C

#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
#include "pillar_code/config.h"
#include "pillar_code/pillar.h"
#include "pillar_code/pillarDetector.h"

#include "pillar_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате


int keep_running = 1;                       // Переменная для остановки программы по Ctrl+C
sensor_msgs::LaserScan::ConstPtr msg_lidar; // Перемеенная в которую сохраняем данные лидара из сообщения

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР и ФУНКЦИЙ **********************************
void callback_Lidar(sensor_msgs::LaserScan::ConstPtr msg); //
void timeCycle(ros::Time timeStart_, ros::Time timeNow_);  // Выводим справочно время работы цикла
void readParam(SPose &startPose, SPoint *startPillar);     // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
static void stopProgram(int signal);                       // Функция, которая срабатывает при нажатии Ctrl+C

// Главная функция программы
int main(int argc, char **argv)
{
    signal(SIGINT, stopProgram);          // Настраиваем обработку Ctrl+C
    ros::init(argc, argv, "pillar_node"); // Инициализируем ROS с именем узла "pillar_node"

    ros::NodeHandle nh;
    ros::Subscriber subscriber_Lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Lidar);

    SPoseLidar g_poseLidar;  // Позиции лидара по расчетам Центральная система координат
    CPillar pillar;          // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов
    PillarDetector detector; // Создаём объект детектора столбов

    SPose startPose;
    SPoint startPillar[4];
    readParam(startPose, startPillar); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

    g_poseLidar.mode1 = startPose;
    pillar.parsingPillar(startPillar); // Разбираем пришедшие данные Заполняем массив правильных координат.

    detector.changePoseLidar(startPose.x, startPose.y, startPose.th);
    detector.changeKnownPillars(0, startPillar[0].x, startPillar[0].y); // Разбираем пришедшие данные Заполняем массив правильных координат.
    detector.changeKnownPillars(1, startPillar[1].x, startPillar[1].y);
    detector.changeKnownPillars(2, startPillar[2].x, startPillar[2].y);
    detector.changeKnownPillars(3, startPillar[3].x, startPillar[3].y);


    ros::Rate loop_rate(5);           // Создаём цикл с частотой 10 Гц
    while (ros::ok() && keep_running) // Пока ROS работает и не нажат Ctrl+C
    {
        ros::spinOnce(); // Обрабатываем входящие сообщения

        // Выполняется 10 Hz как ЛИДАР ПРИШЛЕТ ***************************************************************************************************************************************************
        if (flag_msgLidar) // Если пришло сообщение в топик от лидара и мы уже разобрали данные по координатам машинки, а значит можем грубо посчитать где стоят столбы.  И знаем где истинные столбы
        {
            ROS_INFO("--------------------------------------- flag_msgLidar ***");
            flag_msgLidar = false;

            detector.scanCallback(msg_lidar);

            pillar.parsingLidar(msg_lidar, g_poseLidar.mode1); // Разбираем пришедшие данные и ищем там столбы.
            pillar.comparisonPillar();                         // Сопоставляем столбы
            // topic.publicationPillarAll(pillar);                // Публикуем всю обобщенную информацию по столб

            pillar.getLocationMode1(g_poseLidar.mode1, g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара
            pillar.getLocationMode2(g_poseLidar.mode2, g_poseLidar.mode1); // Считаем текущие координаты по столбам На вход старая позиция лидара, на выходе новая позиция лидара

            // topic.visualPoseLidarMode_1_2();                                // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            // topic.visualPublishOdomMode_1(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
            // topic.visualPublishOdomMode_2(); // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2

            // if (isnan(g_poseLidar.mode2.x) || isnan(g_poseLidar.mode2.y) || isnan(g_poseLidar.mode2.th))
            // {
            //     ROS_ERROR("STOP MODE 1-2");
            //     exit(0);
            // }
        }
        loop_rate.sleep(); // Ждём, чтобы поддерживать частоту 10 Гц
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
void timeCycle(ros::Time timeStart_, ros::Time timeNow_)
{
    ros::Time timeEnd = ros::Time::now();               // Захватываем конечный момент времени
    ros::Duration durationEnd = timeEnd - timeNow_;     // Находим разницу между началом и концом
    ros::Duration durationStart = timeEnd - timeStart_; // Находим разницу между началом и концом
    double dtEnd = durationEnd.toSec() * 1000;          // Получаем количество милисекунд
    double dtStart = durationStart.toSec();             // Получаем количество секунд
    if (dtEnd > 5)                                      // Если цикл занял бользе 5 милисекунд значит что не уложились в 200 Нz
        ROS_INFO("    !!! cycle = %8.3f msec", dtEnd);  // Время цикла в милисекундах
    else
        ROS_INFO_THROTTLE(1, "    dtStart = %7.0f sec | last cycle = %8.3f msec", dtStart, dtEnd); // Время цикла в милисекундах
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

    ROS_INFO("    startPose x = %.3f y = %.3f th = %.3f", startPose.x, startPose.y, startPose.th);
    ROS_INFO("    start PillarPose");
    ROS_INFO("    x0= %.3f y0 = %.3f", startPillar[0].x, startPillar[0].y);
    ROS_INFO("    x1= %.3f y1 = %.3f", startPillar[1].x, startPillar[1].y);
    ROS_INFO("    x2= %.3f y2 = %.3f", startPillar[2].x, startPillar[2].y);
    ROS_INFO("    x3= %.3f y3 = %.3f", startPillar[3].x, startPillar[3].y);
    ROS_INFO("--- readParam");
}

static void stopProgram(int signal) // Функция, которая срабатывает при нажатии Ctrl+C
{
    keep_running = 0; // Устанавливаем флаг, чтобы остановить цикл
    ros::shutdown();  // Завершаем работу ROS
}