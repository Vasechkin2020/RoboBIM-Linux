#ifndef CONFIG_H
#define CONFIG_H


#define RATE 20 // Частота шага

#define DISTANCE_LAZER 0.39 // Середина стандартного дипазона датчика лазерного в стандартном положении
#define DIAPAZON 0.04       // Диапазон +- лазерного датчика

#define MAX_SPEED 0.5                                   // Максимальная скорость робота
#define MAX_RADIUS 0.5                                  // Максимальный радиус поворота робота
#define SPEED_STANDART 0.5                              // Стандартная скорость робота
#define SPEED_ROTATION 0.2                              // Скорость вращения робота
#define MAX_ACCELERATION_UP 0.4                         // Максимальное ускорение робота
const float step_accel_up = MAX_ACCELERATION_UP / RATE; // Максимальное ускорение робота с учтом частоты шага (цикл) работы программы

#define MAX_ACCELERATION_DOWN 0.7                           // Максимальное замедление робота
const float step_accel_down = MAX_ACCELERATION_DOWN / RATE; // Максимальное замедление робота с учтом частоты шага (цикл) работы программы

#define DISTANCE_WHEELS 0.186 // Растояние между колесами робота. подобрал экспериментально Влияет на правильность круга

my_msgs::Control Control_msg; // Топик полученный из ноды Control
my_msgs::Command Command_msg; // Топик отправляемый в ноду Body как команда к исполнению
my_msgs::Body Body_msg;       // Топик полученный из ноды Body

//Структура для координат и направления
struct stru_xy
{
    float x = 0;
    float y = 0;
};

//Структура для фактического значения и заданного
struct stru_ft
{
    float fact = 0;
    float target = 0;
};
//Структура характеризующая робота на верхнем уровне
struct stru_position
{
    stru_xy position;
    stru_xy direction;
    stru_ft speed;
    float radius;
};
//Структура характеризующая точку цели в которой мы должны оказаться
struct stru_target
{
    stru_xy position;
    stru_xy direction;
    stru_ft speed;
};

stru_position g_my_position;   //позиция робота
stru_target g_target_position; //позиция цели


//********************************** Вывод на печать отладочной информации

//#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_WARN  // Если поставить WARN и раскомментировать то не будут выводиться сооющения ROS уровня INFO
//#define LEVEL_SEVERITY YES              // Если раскомментировать то мои метки не будут выводиться

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