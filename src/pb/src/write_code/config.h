#ifndef CONFIG_H
#define CONFIG_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

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

#include <pb_msgs/Struct_Data2Print.h>
#include <pb_msgs/Struct_Print2Data.h>

#include <pb_msgs/SSetSpeed.h>
#include <list>

#define RATE 10 // Частота шага

//--------------------------------- ПОДПИСКА НА ТОПИКИ -------------------------------------------------
// pb_msgs::Struct_Driver2Data msg_Driver2Data; // Сообщение которое считываем из топика
// pb_msgs::SSetSpeed msg_Speed;                // Сообщение которое считываем из топика
//-------------------------------------------------------------------------------------------------------

bool flag_msgPrint = false; // Флаг что пришло сообщение в топик и можно его парсить
// bool flag_msgSpeed = false;  // Флаг что пришло сообщение в топик и можно его парсить

pb_msgs::Struct_Data2Print Write2Data;      // Структура с командами которую публикуем и которую потом Data отправляет на Print  для выполнения

struct SCommand
{
  uint32_t mode = 0;      // Текущий режим работы какеи сопла печатют
  uint32_t status = 0;    // Текущий режим работы 0 - не печатем, 1 печатаем
  float position;         // Позиция задаваемая по CAN
  float velocity;         // Скорость задаваемая по CAN
  float torque;           // Момент задаваемая по CAN
  uint32_t duration = 0; // Длительность действия команды
};

SCommand commandArray[16]; // Массив команд

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