#ifndef CODE_H
#define CODE_H

#include "config.h"
#include "pillar.h"
#include "pillarDetector.h"
#include "topic.h" // Файл для функций для формирования топиков в нужном виде и формате

sensor_msgs::LaserScan::ConstPtr msg_lidar; // Перемеенная в которую сохраняем данные лидара из сообщения
int keep_running = 1;                       // Переменная для остановки программы по Ctrl+C
SPose g_transformGlobal2Local; // Система трансформации из одной позиции в другую


//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР и ФУНКЦИЙ **********************************
void callback_Lidar(sensor_msgs::LaserScan::ConstPtr msg);                             //
void timeCycle(ros::Time timeStart_, ros::Time timeNow_);                              // Выводим справочно время работы цикла
void readParam(SPose &startPose, SPoint *startPillar);                                 // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
static void stopProgram(int signal);                                                   // Функция, которая срабатывает при нажатии Ctrl+C
void calcDistDirect(SDistDirect *distDirect, CPillar pillar, PillarDetector detector); // Обьединение сопоставленных столбов в итоговую таблицу. Дальше по этой таблице все считается
double complementAngle(double angle1, double angle2, double angle3, float weight1, float weight2, float weight3, float popravka_) ;                 // Функция для вычисления среднего угла



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
    // ros::NodeHandle nh_private("~");
    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
    ros::NodeHandle nh_global; // <--- Используется для доступа к /pb_config/ // Создаем ГЛОБАЛЬНЫЙ обработчик, который ищет параметры, начиная с корня (/).

    //<!-- Указываем стартовую позицию робота. В какое место поставили и куда направили-->
    // if (!nh_private.getParam("x", startPose.x))
    //     startPose.x = 0.11;
    // if (!nh_private.getParam("y", startPose.y))
    //     startPose.y = 0.11;
    // if (!nh_private.getParam("theta", startPose.th))
    //     startPose.th = 0.11;

	nh_global.param<double>("/pb_config/start_pose/x", startPose.x, 0.0);
	nh_global.param<double>("/pb_config/start_pose/y", startPose.y, 0.0);
	nh_global.param<double>("/pb_config/start_pose/th", startPose.th, 0.0);

    g_transformGlobal2Local.x = startPose.x;
    g_transformGlobal2Local.y = startPose.y;
    g_transformGlobal2Local.th = startPose.th + 180;

    //<!-- Указываем места расположения столбов на локальной карте -->
    // if (!nh_private.getParam("x0", startPillar[0].x))
    //     startPillar[0].x = 0.11;
    // if (!nh_private.getParam("y0", startPillar[0].y))
    //     startPillar[0].y = 0.11;

    // if (!nh_private.getParam("x1", startPillar[1].x))
    //     startPillar[1].x = 1.11;
    // if (!nh_private.getParam("y1", startPillar[1].y))
    //     startPillar[1].y = 1.11;

    // if (!nh_private.getParam("x2", startPillar[2].x))
    //     startPillar[2].x = 2.11;
    // if (!nh_private.getParam("y2", startPillar[2].y))
    //     startPillar[2].y = 2.11;

    // if (!nh_private.getParam("x3", startPillar[3].x))
    //     startPillar[3].x = 3.11;
    // if (!nh_private.getParam("y3", startPillar[3].y))
    //     startPillar[3].y = 3.11;



	// printf("\n--- Считывание смещений массива лазеров ---\n"); // Разделитель секции...
	nh_global.param<double>("/pb_config/pillars/pillar_0_x", startPillar[0].x, 0.11); // Если не найдено, laser_b0 = -0.0001
	nh_global.param<double>("/pb_config/pillars/pillar_0_y", startPillar[0].y, 0.11); // Если не найдено, laser_b0 = -0.0001
	nh_global.param<double>("/pb_config/pillars/pillar_1_x", startPillar[1].x, 1.11); // Если не найдено, laser_b0 = -0.0001
	nh_global.param<double>("/pb_config/pillars/pillar_1_y", startPillar[1].y, 1.11); // Если не найдено, laser_b0 = -0.0001
	nh_global.param<double>("/pb_config/pillars/pillar_2_x", startPillar[2].x, 2.11); // Если не найдено, laser_b0 = -0.0001
	nh_global.param<double>("/pb_config/pillars/pillar_2_y", startPillar[2].y, 2.11); // Если не найдено, laser_b0 = -0.0001
	nh_global.param<double>("/pb_config/pillars/pillar_3_x", startPillar[3].x, 3.11); // Если не найдено, laser_b0 = -0.0001
	nh_global.param<double>("/pb_config/pillars/pillar_3_y", startPillar[3].y, 3.11); // Если не найдено, laser_b0 = -0.0001



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

// Обьединение сопоставленных столбов в итоговую таблицу. Дальше по этой таблице все считается. Вроде тут я усредняю расстояние до столбов
void calcDistDirect(SDistDirect *distDirect, CPillar pillar, PillarDetector detector)
{
    #define COMPLEMENTARN 0.1
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
            // ROS_INFO("    distDirect[i].direction = %.3f ||| detector.matchPillar[i].direction = %.3f count = %.1f", distDirect[i].direction, detector.matchPillar[i].direction,count);
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
    ROS_INFO("    calcDistDirect -> sum = %i", sum);
    g_poseLidar.countMatchPillar = sum;
}



// Функция для вычисления среднего угла
double complementAngle(double angle1, double angle2, double angle3, float weight1, float weight2, float weight3, float popravka_)
{

    double rad1 = DEG2RAD(angle1); // Переводим углы в радианы
    double rad2 = DEG2RAD(angle2);
    double rad3 = DEG2RAD(angle3);

    double x = weight1 * cos(rad1) + weight2 * cos(rad2) + weight3 * cos(rad3); // Вычисляем сумму векторов (x, y)
    double y = weight1 * sin(rad1) + weight2 * sin(rad2) + weight3 * sin(rad3);

    double avgRadians = atan2(y, x);         // Находим средний угол через atan2
    double avgDegrees = RAD2DEG(avgRadians); // Переводим результат обратно в градусы
    avgDegrees += popravka_;                      // Добавляем поправку на угол лидара

    if (avgDegrees > 180) // Приводим угол к диапазону [-180, 180]
    {
        avgDegrees -= 360;
    }
    else if (avgDegrees < -180)
    {
        avgDegrees += 360;
    }
    return avgDegrees;
}
















#endif
