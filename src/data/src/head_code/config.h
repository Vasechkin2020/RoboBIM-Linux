#ifndef CONFIG_H
#define CONFIG_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

// #include <data/point.h>
#include <data/SControlDriver.h>
#include <data/SDriver2Data.h>
#include <data/PillarOut.h>
#include <data/pillar.h>
#include <data/topicPillar.h>
#include <data/SAngleLaserLidar.h>
#include <data/SAngleLL.h>
#include <data/Struct_PoseLidar.h>
// #include <data/pointA.h>

#define RATE 2 // Частота шага

#define DISTANCE_LAZER 0.39 // Середина стандартного дипазона датчика лазерного в стандартном положении
#define DIAPAZON 0.04       // Диапазон +- лазерного датчика

#define MAX_SPEED 1.5                                   // Максимальная скорость робота
#define MAX_RADIUS 0.5                                  // Максимальный радиус поворота робота
#define SPEED_STANDART 0.5                              // Стандартная скорость робота
#define SPEED_ROTATION 0.2                              // Скорость вращения робота
#define MAX_ACCELERATION_UP 0.4                         // Максимальное ускорение робота
const float step_accel_up = MAX_ACCELERATION_UP / RATE; // Максимальное ускорение робота с учтом частоты шага (цикл) работы программы

#define MAX_ACCELERATION_DOWN 0.7                           // Максимальное замедление робота
const float step_accel_down = MAX_ACCELERATION_DOWN / RATE; // Максимальное замедление робота с учтом частоты шага (цикл) работы программы

#define RAD2DEG(x) ((x) * 180. / M_PI) // Первод из радиан в градусы
#define DEG2RAD(x) ((x) * M_PI / 180.) // Первод из градусов в радианы

//--------------------------------- ПОДПИСКА НА ТОПИКИ -------------------------------------------------
sensor_msgs::LaserScan::ConstPtr msg_lidar; // Перемеенная в которую сохраняем данные лидара из сообщения
data::topicPillar msg_pillar;               // Перемеенная в которую сохраняем данные по столбам из сообщения
geometry_msgs::Pose2D msg_startPose2d;      // Перемеенная в которую сохраняем данные о координатах машинки начальных из сообщения
data::SDriver2Data msg_Driver2Data;         // Сообщение которое считываем из топика
//-------------------------------------------------------------------------------------------------------

bool flag_msgPillar = false;  // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgLidar = false;   // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgCar = false;     // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgDriver = false;  // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgModul = false;   // Флаг что пришло сообщение в топик и можно его парсить
bool flag_dataPillar = false; // Флаг что разобрали данные по координатам столбов и можно обсчитывать дальше
bool flag_dataCar = false;    // Флаг что разобрали данные по координатам машины и можно обсчитывать дальше
bool flag_dataLidar = false;  // Флаг что разобрали данные по лидару и можно сопоставлять столбы

data::SControlDriver Data2Driver;      // Структура с командами которую публикуем и которую потом Driver исполняет
data::SControlDriver Data2Driver_prev; // Структура с командами которую публикуем и которую потом Driver исполняет предыдущее состоние

struct SPoint // Точка
{
    float x = 0; //
    float y = 0; //
};

struct SPose
{
    float x = 0;  // Координата по Х
    float y = 0;  // Координата по Y
    float th = 0; // Направление носа
};
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
// Структура для одометрии
struct SOdom
{
    SPose pose;
    STwist twist;
};
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

struct SPoseLidar // Варианты расчетов координат лидара
{
    SPose mode1;
    SPose mode2;
    SPose mode3;
};

//************************************** ОБЬЯВЛЯЕМ ФУНКЦИИ **********************************
float sqr(float x_);                                                    // Функция возведния в квадрат
float ctan(float x_);                                                   // Функция котангенса угла
float vectorLen(SPoint point1, SPoint point2);                          // Функция нахождения длинны вектора
SPoint povorotSystemCoordinate(float xloc_, float yloc_, float theta_); // Формулы поворота системы координат. Угол задавать отрицательный если поворачиваем против часовой к нулю который вверх
SPoint povorotSmechenie(SPoint point_, SPose pose_);                    // Формулы поворота системы координат и смещения. На вход координаты точки и положение системы координат в которой мы хотим получить координаты этой заданной точки. На выходе координаты заданной точки, но уже в новой системе координат
float angleThetaFromPoint(SPoint point_);                               // Получение угла между вектором и осью Y //Задаем координаты точки и получаем угол между У и линией на точку (это гипотенуза)

//*******************************************************************************************

float vectorLen(SPoint point1, SPoint point2) // Функция возвращает длинну вектора, фактически растояние по прямой между двумя точкам
{
    float dx = abs(point2.x - point1.x); // Находим разницу в координатах
    float dy = abs(point2.y - point1.y); //
    float len = sqrt(sqr(dx) + sqr(dy)); // Теорема пифагора
    return len;
}
// Функция возверения в квадрат
float sqr(float x_)
{
    return x_ * x_;
}
// Функция котангенса
float ctan(float x_)
{
    return 1 / tan(x_);
}

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

// Получение угла между вектором и осью Y
// Задаем координаты точки и получаем угол между У и линией на точку (это гипотенуза)
float angleThetaFromPoint(SPoint point_) // Возвращает от -180 до +180
{
    float ret = RAD2DEG(atan2(point_.x, point_.y));
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

#define NN "\x1b[32;40m Head_node"

void my_printInfo()
{
#ifndef LEVEL_SEVERITY
    printf("%s %s [ INFO]", NORM, NN);
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