#ifndef CONFIG_H
#define CONFIG_H

#include <string> // Подключение библиотеки для работы со строками

#include <ros/ros.h>
#include <log4cxx/mdc.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <wiringPi.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
// #include <pb_msgs/point.h>

#include <pb_msgs/Struct_Data2Driver.h>
#include <pb_msgs/Struct_Data2Modul.h>
#include <pb_msgs/Struct_Data2Print.h>

#include <pb_msgs/Struct_Driver2Data.h>
#include <pb_msgs/Struct_Modul2Data.h>

#include <pb_msgs/PillarOut.h>
#include <pb_msgs/pillar.h>
#include <pb_msgs/topicPillar.h>
#include <pb_msgs/SAngleLaserLidar.h>
#include <pb_msgs/SAngleLL.h>
#include <pb_msgs/Struct_PoseLidar.h>
#include <pb_msgs/Struct_PoseRotation.h>
#include <pb_msgs/SLinAngVel.h>

#include <pb_msgs/SEncoder.h>
#include <pb_msgs/SMpu.h>
#include <pb_msgs/SSensor.h>
#include <pb_msgs/SMotor.h>

#include <pb_msgs/SSetSpeed.h>

// #include <data/pointA.h>

#define RATE 100                                 // Частота шага
#define RATE_LASER 3                            // Частота измерения лазерного датчика при калибровке для расчета
#define STEP_LASER_MOTOR (360.0 / 400.0 / 16.0) // Шаг поаорота мотора с лазерным датчиком 360 ГРАДУСОВ /400 шагов мотор /16 шагов драйвер

#define OFFSET_LAZER 0.042 // Оффсет лоя лазера с учетом напечатанного крепления.Если крепление дургое то и офсет надо перезамерить....

// #define DISTANCE_LAZER 0.39 // Середина стандартного дипазона датчика лазерного в стандартном положении
#define DIAPAZON 0.04 // Диапазон +- лазерного датчика

#define DISTANCE_WHEELS 0.38 // Растояние между колесами робота. подобрал экспериментально Влияет на правильность круга

#define MAX_SPEED 1.5      // Максимальная скорость робота
#define MAX_RADIUS 0.5     // Максимальный радиус поворота робота
#define SPEED_STANDART 0.5 // Стандартная скорость робота
#define SPEED_ROTATION 0.2 // Скорость вращения робота
// #define MAX_ACCELERATION_UP 0.4                         // Максимальное ускорение робота
// const float step_accel_up = MAX_ACCELERATION_UP / RATE; // Максимальное ускорение робота с учтом частоты шага (цикл) работы программы

// #define MAX_ACCELERATION_DOWN 0.7                           // Максимальное замедление робота
// const float step_accel_down = MAX_ACCELERATION_DOWN / RATE; // Максимальное замедление робота с учтом частоты шага (цикл) работы программы

#define RAD2DEG(x) ((x) * 180. / M_PI) // Первод из радиан в градусы
#define DEG2RAD(x) ((x) * M_PI / 180.) // Первод из градусов в радианы

float offsetAngle = 0.7; // Ошибка для азимута. Прибавляем к углу что получилди с лидара

//--------------------------------- ПОДПИСКА НА ТОПИКИ -------------------------------------------------
sensor_msgs::LaserScan::ConstPtr msg_lidar; // Перемеенная в которую сохраняем данные лидара из сообщения
pb_msgs::topicPillar msg_pillar;            // Перемеенная в которую сохраняем данные по столбам из сообщения
geometry_msgs::Pose2D msg_startPose2d;      // Перемеенная в которую сохраняем данные о координатах машинки начальных из сообщения

pb_msgs::Struct_Driver2Data msg_Driver2Data; // Сообщение которое считываем из топика
pb_msgs::Struct_Modul2Data msg_Modul2Data;   // Сообщение которое считываем из топика
pb_msgs::SSetSpeed msg_Speed;                // Сообщение которое считываем из топика
//-------------------------------------------------------------------------------------------------------

bool flag_msgPillar = false;    // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgLidar = false;     // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgStartPose = false; // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgDriver = false;    // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgModul = false;     // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgSpeed = false;     // Флаг что пришло сообщение в топик и можно его парсить

bool flag_dataPillar = false; // Флаг что разобрали данные по координатам столбов и можно обсчитывать дальше
bool flag_startPose = false;  // Флаг что разобрали данные по координатам машины и можно обсчитывать дальше
bool flag_dataLidar = false;  // Флаг что разобрали данные по лидару и можно сопоставлять столбы

pb_msgs::Struct_Data2Driver Data2Driver;      // Структура с командами которую публикуем и которую потом Driver исполняет
pb_msgs::Struct_Data2Driver Data2Driver_prev; // Структура с командами которую публикуем и которую потом Driver исполняет предыдущее состоние

float linearOffsetX[128] = {0};
float linearOffsetY[128] = {0};
float linearOffsetZ[128] = {0};

bool modeColibrovka = false;
SPose transformLidar2Rotation; // данные для трансформации из одной системы координат в другую из Lidar в Rotation
std::string log_name = "pos_node";

// float gTheta = 0; // Угол куда смотрит робот theta

// struct SEuler
// {
//   float roll = 0;  // Крен в право  влево
//   float pitch = 0; // Тангаж вверх или вних
//   float yaw = 0;   // Поворот по часовой мом против часовой
// };

// // Структура для углов наклонов
// struct STwist
// {
//   double vx = 0;  // Линейная скорость движения робота по оси X
//   double vy = 0;  // Линейная скорость движения робота по оси Y
//   double vth = 0; // Угловая скорость вращения робота
// };
// // Структура для одометрии
// struct SOdom
// {
//   SPose pose;
//   STwist twist;
// };

struct SEuler
{
    float roll = 0;  // Крен в право  влево
    float pitch = 0; // Тангаж вверх или вних
    float yaw = 0;   // Поворот по часовой мом против часовой
};

// Структура для углов наклонов
struct STwist
{
    float vx = 0;  // Линейная скорость движения робота по оси X
    float vy = 0;  // Линейная скорость движения робота по оси Y
    float vth = 0; // Угловая скорость вращения робота
};
struct STwistDt
{
    STwist twist;
    double dt = 0.0;
};
// STwistDt mpuTwistDt;    // Скорости полученные по mpu и интревал который прошел с предыдущего измерения
// STwistDt wheelTwistDt;  // Скорости полученные по ecoder и интревал который прошел с предыдущего измерения
// STwistDt unitedTwistDt; // Обьединенные комплементратный способом Скорости и интервал

struct SLinAngVel
{
    STwistDt mpu;    // Скорости полученные по mpu и интревал который прошел с предыдущего измерения
    STwistDt wheel;  // Скорости полученные по ecoder и интревал который прошел с предыдущего измерения
    STwistDt united; // Обьединенные комплементратный способом Скорости и интервал
};

// Структура для одометрии
struct SPoseRotation
{
    SPose mode0;     // Расчет чисто по одометрии колес без комплементации с чем-либо
    SPose mode10;    // Комплементация ИТОГОВАЯ ПОЗИЦИЯ ИСПОЛЬЗУЕМАЯ ДАЛЕЕ КАК ОСНОВНАЯ
    SPose mode11;    // Комплементация по Калману с mode1 Lidar Distance
    SPose mode12;    // Комплементация по Калману с mode2 Lidar Angle
    SPose mode13;    // Комплементация по Калману с mode3 Laser
    SPose mode14;    // Комплементация по Калману с mode4 IMU
    // float theta = 0; // Единый угол направления. Получается комплементаций угла с датчика BNO055 и поправками с других датчиков от уплывания с очень маленьким коефициентом. Так как с BNO055 очень точно все!!!
    // float angleAccelGyro = 0; // Угол посчитанный по аксельрометру и гироскопу отдельным алгоритмом
};

struct SPoseLidar // Варианты расчетов координат лидара
{
    SPose mode10; // // Комплементация ИТОГОВАЯ ПОЗИЦИЯ ИСПОЛЬЗУЕМАЯ для расчета как входящая позиция
    SPose mode1;  // Для лидара по растоянию
    SPose mode2;  // Для лидара по углу
    SPose mode3;  // Для лазерных датчиков

    // SPose mode4;  // Для IMU
    // SPose mode0;  // Для одометрии по колесам
    // SPose mode123; // Для скомплементированного положения Mode0 + Mode1 + Mode3
    // SPose mode11; // Для комплементации по Калману + Mode1
    // SPose mode12; // Для комплементации по Калману + Mode2
    // SPose mode13; // Для комплементации по Калману + Mode3
};

// SOdom odomMode0;  // Высчитанная одометрия по колесам по полученным из топика скоростям
// SOdom odomMpu;    // Высчитанная одомтрия по датчику mpu cкорректированная
// SOdom odomUnited; // Высчитанная одометрия по энкодеру скорректированная
struct SCircle // Окружность
{
    float x = 0; //
    float y = 0; //
    float r = 0; // Радиус окружности
};
struct SCircle2 // Две окружности
{
    SCircle c1;
    SCircle c2;
};

//************************************** ОБЬЯВЛЯЕМ ФУНКЦИИ **********************************
void normalizeVector(double &x, double &y, double &z);                  // Нормализация вектора
SPoint povorotSystemCoordinate(float xloc_, float yloc_, float theta_); // Формулы поворота системы координат. Угол задавать отрицательный если поворачиваем против часовой к нулю который вверх
SPoint povorotSmechenie(SPoint point_, SPose pose_);                    // Формулы поворота системы координат и смещения. На вход координаты точки и положение системы координат в которой мы хотим получить координаты этой заданной точки. На выходе координаты заданной точки, но уже в новой системе координат
float angleThetaFromPoint(SPoint point_);                               // Получение угла между вектором и осью Y //Задаем координаты точки и получаем угол между У и линией на точку (это гипотенуза)
SPoint pointGlobal2Local(SPoint pointGlobal_, SPose poseLocal_);        // Преобразование координат точки из Глобальной системы координат в Локальную
SPoint pointLocal2Global(SPoint pointLocal_, SPose poseLocal_);         // Преобразование координат точки из Локальной системы координат в Глобальную

//*******************************************************************************************

// Формулы поворота системы координат. Угол задавать отрицательный если поворачиваем против часовой к нулю который вверх
SPoint povorotSystemCoordinate(float xloc_, float yloc_, float theta_)
{
    SPoint ret;
    theta_ = DEG2RAD(theta_); // Превращаем в радианы из градусов
    ret.x = xloc_ * cos(theta_) - yloc_ * sin(theta_);
    ret.y = xloc_ * sin(theta_) + yloc_ * cos(theta_);
    return ret;
}

// Формулы поворота системы координат и смещения.
// На вход координаты точки и положение системы координат в которой мы хотим получить координаты этой заданной точки. На выходе координаты заданной точки, но уже в новой системе координат
SPoint povorotSmechenie(SPoint point_, SPose pose_)
{
    SPoint ret;
    float theta = DEG2RAD(pose_.th); // Превращаем в радианы из градусов
    float xloc = point_.x;
    float yloc = point_.y;
    float xnew = xloc * cos(theta) - yloc * sin(theta); // Поворачиваем по формулам поворота системы координат
    float ynew = xloc * sin(theta) + yloc * cos(theta);
    ret.x = xnew + pose_.x; // Добавляем смещение
    ret.y = ynew + pose_.y;
    return ret;
}
// Преобразование координат точки из Глобальной системы координат в Локальную
// Задаем координаты точки в Глобальной системе и задаем позицию Локальной системы в координатах Глобальной системы
// Получаем координаты точки в Локальной системе
SPoint pointGlobal2Local(SPoint pointGlobal_, SPose poseLocal_)
{
    SPoint ret;
    float theta = DEG2RAD(poseLocal_.th); // Превращаем в радианы из градусов
    float x = pointGlobal_.x - poseLocal_.x;
    float y = pointGlobal_.y - poseLocal_.y;
    float xnew = x * cos(theta) - y * sin(theta); // Поворачиваем по формулам поворота системы координат
    float ynew = x * sin(theta) + y * cos(theta);
    ret.x = xnew; // Добавляем смещение
    ret.y = ynew;
    return ret;
}
SPoint pointGlobal2LocalRos(SPoint pointGlobal_, SPose poseLocal_) // Вариант поворота по стандарту РОС что вращение против часовой
{
    SPoint ret;
    float theta = DEG2RAD(-poseLocal_.th); // Превращаем в радианы из градусов
    float x = pointGlobal_.x - poseLocal_.x;
    float y = pointGlobal_.y - poseLocal_.y;
    float xnew = x * cos(theta) - y * sin(theta); // Поворачиваем по формулам поворота системы координат
    float ynew = x * sin(theta) + y * cos(theta);
    ret.x = xnew; // Добавляем смещение
    ret.y = ynew;
    return ret;
}
// Преобразование координат точки из Локальной системы координат в Глобальную
// Задаем координаты точки в Локальной системе и задаем позицию самой Локальной системы в координатах Глобальной системы
// Получаем координаты точки в Глобальной системе
SPoint pointLocal2Global(SPoint pointLocal_, SPose poseLocal_)
{
    SPoint ret;
    float theta = DEG2RAD(poseLocal_.th); // Превращаем в радианы из градусов
    float x = pointLocal_.x;
    float y = pointLocal_.y;
    float xnew = x * cos(theta) + y * sin(theta); // Поворачиваем по формулам поворота системы координат
    float ynew = -x * sin(theta) + y * cos(theta);
    ret.x = xnew + poseLocal_.x; // Добавляем смещение
    ret.y = ynew + poseLocal_.y;
    return ret;
}
SPoint pointLocal2GlobalRos(SPoint pointLocal_, SPose poseLocal_)
{
    SPoint ret;
    float theta = DEG2RAD(-poseLocal_.th); // Превращаем в радианы из градусов
    float x = pointLocal_.x;
    float y = pointLocal_.y;
    float xnew = x * cos(theta) + y * sin(theta); // Поворачиваем по формулам поворота системы координат
    float ynew = -x * sin(theta) + y * cos(theta);
    ret.x = xnew + poseLocal_.x; // Добавляем смещение
    ret.y = ynew + poseLocal_.y;
    return ret;
}
SPoint pointLocal2GlobalRosRAD(SPoint pointLocal_, SPose poseLocal_)
{
    // printf("IN poseLocal_.x= % .3f poseLocal_.y= % .3f \n", poseLocal_.x, poseLocal_.y);
    SPoint ret;
    float theta = -(poseLocal_.th); //
    // theta = 0; //
    float x = pointLocal_.x;
    float y = pointLocal_.y;

    // printf("1 pointLocal_.x= % .3f pointLocal_.y= % .3f th= % .3f ",pointLocal_.x,pointLocal_.y, theta);
    float xnew = x * cos(theta) + y * sin(theta); // Поворачиваем по формулам поворота системы координат
    float ynew = -x * sin(theta) + y * cos(theta);
    // printf("xnew= % .3f ynew= % .3f \n", xnew, ynew);

    // theta = -theta;
    // printf("2 pointLocal_.x= % .3f pointLocal_.y= % .3f th= % .3f ",pointLocal_.x,pointLocal_.y, theta);
    // xnew = x * cos(theta) + y * sin(theta); // Поворачиваем по формулам поворота системы координат
    // ynew = -x * sin(theta) + y * cos(theta);
    // printf("xnew= % .3f ynew= % .3f \n", xnew, ynew);

    ret.x = xnew + poseLocal_.x; // Добавляем смещение
    ret.y = ynew + poseLocal_.y;

    // printf("ret.x= % .3f ret.y= % .3f \n", ret.x, ret.y);

    return ret;
}

// Получение угла между вектором и осью Y
// Задаем координаты точки и получаем угол между У и линией на точку (это гипотенуза)
float angleThetaFromPoint(SPoint point_) // Возвращает от -180 до +180
{
    float ret = RAD2DEG(atan2(point_.x, point_.y));
    return ret;
}
float angleThetaFromPointRos(SPoint point_) // Возвращает от -180 до +180 Родгонял под првила ROS по осям
{
    float ret = RAD2DEG(atan2(point_.y, point_.x));
    return ret;
}
// Получение координат точки по углу и расстоянию
// Задаем угол на точку от оси У вверх которая и расстояние до точки
SPoint pointFromTetha(float angle_, float distance_) // Возвращает от -180 до +180
{
    // printf(" pointFromTetha angle_= %f distance_ = %f ", angle_, distance_);
    SPoint ret;
    angle_ = -DEG2RAD(angle_); // Превращаем в радианы, так как приходит в градусах МИнут так как положительные углы вправо по часовой
    if (abs(angle_) < 90)
    {
        ret.x = cos(angle_) * distance_;
        ret.y = sin(angle_) * distance_;
    }
    else
    {
        angle_ = angle_ - 90;
        ret.x = -sin(angle_) * distance_;
        ret.y = cos(angle_) * distance_;
    }
    // printf("x= %f y = %f \n", ret.x, ret.y);
    return ret;
}

// my_msgs::Control Control_msg; // Топик полученный из ноды Control
// my_msgs::Command Command_msg; // Топик отправляемый в ноду Body как команда к исполнению
// my_msgs::Body Body_msg;       // Топик полученный из ноды Body

// Структура  данных где указаны координаты  текущей позиции
// struct pos_struct
// {
//     double x = 0;  // Координата по Х
//     double y = 0;  // Координата по У
//     double th = 0; // Направление
// };

// // Структура для координат и направления
// struct stru_xy
// {
//     float x = 0;
//     float y = 0;
// };

// // Структура для фактического значения и заданного
// struct stru_ft
// {
//     float fact = 0;
//     float target = 0;
// };
// // Структура характеризующая робота на верхнем уровне
// struct stru_position
// {
//     stru_xy position;
//     stru_xy direction;
//     stru_ft speed;
//     float radius;
// };
// // Структура характеризующая точку цели в которой мы должны оказаться
// struct stru_target
// {
//     stru_xy position;
//     stru_xy direction;
//     stru_ft speed;
// };

// stru_position g_my_position;   // позиция робота
// stru_target g_target_position; // позиция цели

//********************************** Вывод на печать отладочной информации

// #define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_WARN  // Если поставить WARN и раскомментировать то не будут выводиться сооющения ROS уровня INFO
// #define LEVEL_SEVERITY YES              // Если раскомментировать то мои метки не будут выводиться

#define RED "\x1b[31;40m"
#define GREEN "\x1b[32;40m"
#define YELLOW "\x1b[33;40m"
#define BLUE "\x1b[34;40m"
#define MAGENTA "\x1b[35;40m"
#define CYAN "\x1b[36;40m"
#define NORM "\x1b[0m"

// #define NN "\x1b[32;40m pos_node"
#define NN "\x1b[32;40m"

void my_printInfo()
{
#ifndef LEVEL_SEVERITY
    printf("%s %s [ INFO]", GREEN, NN);
#endif
}
void my_printWarn()
{
#ifndef LEVEL_SEVERITY
    fprintf(stderr, "%s %s [ WARN]", YELLOW, NN);
#endif
}
void my_printErr()
{
#ifndef LEVEL_SEVERITY
    fprintf(stderr, "%s %s [ ERR ]", RED, NN);
#endif
}

#define INFO my_printInfo();
#define WARN my_printWarn();
#define ERROR my_printErr();

#endif