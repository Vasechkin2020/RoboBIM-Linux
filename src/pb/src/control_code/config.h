#ifndef CONFIG_H
#define CONFIG_H


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>


#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
// #include <pb_msgs/point.h>

#include <pb_msgs/Struct_Data2Driver.h>

#include <pb_msgs/Struct_Data2Print.h>
#include <pb_msgs/Struct_Print2Data.h>

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

bool adv_log = 1; // Выводить ли расширенные логи

// #include <wiringPi.h>
// --- ЗАМЕНА WIRINGPI TIME ---  Возвращает миллисекунды (как в Arduino/WiringPi), но на основе времени ROS
static ros::Time g_start_time_offset(0); // Переменная для хранения времени старта (инициализируется нулем)

inline uint64_t millis() // Возвращает миллисекунды от старта программы (на базе ROS времени)
{
    // Если это первый вызов (или после сброса) — запоминаем время старта
    if (g_start_time_offset.isZero()) {
        g_start_time_offset = ros::Time::now();
    }
    ros::Duration duration = ros::Time::now() - g_start_time_offset;    // Считаем разницу между "сейчас" и "стартом"
    return (uint64_t)(duration.toNSec() / 1000000);    // Возвращаем миллисекунды (наносекунды / 1 000 000)
}

inline uint64_t micros() // Возвращает микросекунды от старта программы (для расчета dt)
{
    if (g_start_time_offset.isZero()) {
        g_start_time_offset = ros::Time::now();
    }
    ros::Duration duration = ros::Time::now() - g_start_time_offset;
    return (uint64_t)(duration.toNSec() / 1000);     // Возвращаем микросекунды (наносекунды / 1 000)
}



pb_msgs::Struct_Data2Driver controlSpeed; // Переменная в которую записываем данные из массива команд после gcode
pb_msgs::Struct_Data2Print controlPrint; // Переменная в которую записываем данные из массива команд после gcode

sensor_msgs::Joy msg_joy; // Переменная в которую записываем пришедшее сообщение а колбеке

#define MAX_SPEED 1.5      // Максимальная скорость робота
#define MAX_RADIUS 0.5     // Максимальный радиус поворота робота
#define SPEED_STANDART 0.5 // Стандартная скорость робота
#define SPEED_ROTATION 0.2 // Скорость вращения робота

#define RAD2DEG(x) ((x) * 180. / M_PI) // Первод из радиан в градусы
#define DEG2RAD(x) ((x) * M_PI / 180.) // Первод из градусов в радианы

//--------------------------------- ПОДПИСКА НА ТОПИКИ -------------------------------------------------
pb_msgs::Struct_Driver2Data msg_Driver2Data; // Сообщение которое считываем из топика
pb_msgs::SSetSpeed msg_Speed;                // Сообщение которое считываем из топика
pb_msgs::Struct_PoseRotation msg_PoseRotation;       // Сообщение которое считываем из топика
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


SPose g_poseC; // Тут храним текущую позицию от которой все считаем в зависимости от режима controlMode

pb_msgs::Struct_Data2Driver Data2Driver;      // Структура с командами которую публикуем и которую потом Driver исполняет
pb_msgs::Struct_Data2Driver Data2Driver_prev; // Структура с командами которую публикуем и которую потом Driver исполняет предыдущее состоние

// ===================================================================================
// SCommand: расширенная структура с координатами до/после
// ===================================================================================
struct SCommand
{
    int mode;
    bool use_model_logic; // <--- ДОБАВИТЬ ЭТОТ ФЛАГ (true = ехать по Model+Offset, false = по Est)
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

std::vector<SCommand> commandArray; // Динамический Массив команд


#endif