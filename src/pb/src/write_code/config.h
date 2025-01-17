#ifndef CONFIG_H
#define CONFIG_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <wiringPi.h>

#include <pb_msgs/Struct_Data2Print.h>

#include <pb_msgs/SSetSpeed.h>
#include <list>

#define RATE 10 // Частота шага

//--------------------------------- ПОДПИСКА НА ТОПИКИ -------------------------------------------------
// pb_msgs::Struct_Driver2Data msg_Driver2Data; // Сообщение которое считываем из топика
// pb_msgs::SSetSpeed msg_Speed;                // Сообщение которое считываем из топика
//-------------------------------------------------------------------------------------------------------

bool flag_msgPrint = false; // Флаг что пришло сообщение в топик и можно его парсить
// bool flag_msgSpeed = false;  // Флаг что пришло сообщение в топик и можно его парсить

pb_msgs::Struct_Data2Print Data2Print;      // Структура с командами которую публикуем и которую потом Driver исполняет

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