#ifndef CONFIG_H
#define CONFIG_H

#define RATE 1 // Частота шага

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