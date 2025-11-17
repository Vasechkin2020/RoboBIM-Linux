#ifndef CONFIG_H
#define CONFIG_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <wiringPi.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
// #include <pb_msgs/point.h>

#include <pb_msgs/Struct_Data2Driver.h>

#include <sensor_msgs/Joy.h>
#include <pb_msgs/SJoy.h>

#include <pb_msgs/Struct_Driver2Data.h>
#include <pb_msgs/Struct_PoseRotation.h>

#include <pb_msgs/SEncoder.h>
#include <pb_msgs/SMpu.h>
#include <pb_msgs/SSensor.h>
#include <pb_msgs/SMotor.h>

#include <pb_msgs/SSetSpeed.h>
#include <list>

pb_msgs::Struct_Data2Driver controlSpeed;
sensor_msgs::Joy msg_joy; // Переменная в которую записываем пришедшее сообщение а колбеке

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

//--------------------------------- ПОДПИСКА НА ТОПИКИ -------------------------------------------------
pb_msgs::Struct_Driver2Data msg_Driver2Data; // Сообщение которое считываем из топика
pb_msgs::SSetSpeed msg_Speed;                // Сообщение которое считываем из топика
pb_msgs::Struct_PoseRotation msg_Pose;       // Сообщение которое считываем из топика
//-------------------------------------------------------------------------------------------------------
bool flag_msgJoy = false;    // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgDriver = false; // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgSpeed = false;  // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgPose = false;   // Флаг что пришло сообщение в топик и можно его парсить

int verComand = 0; // Параметр какой массив команд загружать

bool flagCommand = true;     // Флаг можно исполнять каманду
bool flagAngle = false;      // Флаг отслеживания угла из топика
bool flagAngleFirst = true;  // Флаг отслеживания первого запуска алгоритма отслеживания
bool flagVector = false;     // Флаг отслеживания длины вектора точки из топика
bool flagVectorFirst = true; // Флаг отслеживания первого запуска алгоритма отслеживания
// static SPoint vectorStart;
static SPoint point_A;
static SPoint point_B;
static SPoint point_C;
static SPoint point_D;

pb_msgs::Struct_Data2Driver Data2Driver;      // Структура с командами которую публикуем и которую потом Driver исполняет
pb_msgs::Struct_Data2Driver Data2Driver_prev; // Структура с командами которую публикуем и которую потом Driver исполняет предыдущее состоние

// struct SCommand
// {
//     int mode = 0;       // Вид команды

//     int duration = 0;   // Длительность действия команды в милисекундах
//     float velL = 0;     // Скорость колеса
//     float velR = 0;     // Скорость колеса

//     float angle = 0;    // Угол в который должны повернуться
//     float velAngle = 0; // Скорость угловая поворота

//     float len = 0;      // Длина вектора который должны проехать
//     float velLen = 0;   // Скорость линейная движения

// };

// ===================================================================================
// SCommand: расширенная структура с координатами до/после
// ===================================================================================
struct SCommand
{
    int mode;
    float duration;   // мс
    float velL, velR; // м/с
    float angle;      // градусы
    float velAngle;   // м/с
    float len;        // метры
    float velLen;     // м/с
    // НОВОЕ: координаты до и после выполнения команды
    float point_A_x, point_A_y, point_A_a; // состояние ДО
    float point_B_x, point_B_y, point_B_a; // состояние ПОСЛЕ

    SCommand()
        : mode(0), duration(0.0f), velL(0.0f), velR(0.0f),
          angle(0.0f), velAngle(0.0f), len(0.0f), velLen(0.0f),
          point_A_x(0.0f), point_A_y(0.0f), point_A_a(0.0f),
          point_B_x(0.0f), point_B_y(0.0f), point_B_a(0.0f)
    {
    }
};

// SCommand commandArray[48]; // Массив команд
std::vector<SCommand> commandArray; // Динамический Массив команд

// //************************************** ОБЬЯВЛЯЕМ ФУНКЦИИ **********************************
// float sqr(float x_);  // Функция возведния в квадрат
// float ctan(float x_); // Функция котангенса угла

// //*******************************************************************************************

// // Функция возверения в квадрат
// float sqr(float x_)
// {
//     return x_ * x_;
// }
// // Функция котангенса
// float ctan(float x_)
// {
//     return 1 / tan(x_);
// }

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

#define NN "\x1b[32;40m control_node"

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