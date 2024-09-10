#ifndef CONFIG_H
#define CONFIG_H

#include </opt/ros/melodic/include/ros/ros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Joy.h>
#include <pb_msgs/SJoy.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <pb_msgs/SControl.h>
#include <pb_msgs/SLed.h>

// #include <pb_msgs/SControlModul.h>
// #include <pb_msgs/SControlPrint.h>
#include <pb_msgs/Struct_Data2Print.h>
#include <pb_msgs/Struct_Data2Modul.h>
#include <pb_msgs/Struct_Data2Driver.h>
// #include <pb_msgs/SControlDriver.h>

#include <pb_msgs/SEncoder.h>
#include <pb_msgs/SMpu.h>
#include <pb_msgs/Struct_Odom.h>
#include <pb_msgs/SSensor.h>
#include <pb_msgs/SMotor.h>

#include <pb_msgs/Struct_Driver2Data.h>
#include <pb_msgs/Struct_Modul2Data.h>

#include <pb_msgs/Struct_Info_SPI.h>

u_int64_t timeSpiModul = 0;  // Время когда пришла команда по топикам
u_int64_t timeSpiDriver = 0; // Время когда пришла команда по топикам
u_int64_t timeSpiPrint = 0;  // Время когда пришла команда по топикам

pb_msgs::Struct_Data2Driver msg_ControlDriver; // Полученное сообщение из топика Head в Data
pb_msgs::Struct_Data2Modul msg_ControlModul;   // Полученное сообщение из топика
pb_msgs::Struct_Data2Print msg_ControlPrint;   // Полученное сообщение из топика
sensor_msgs::Joy msg_joy;                      // Переменная в которую записываем пришедшее сообщение а колбеке

#define RATE 10 // Частота обмена с нижним уровнев в Герц
//---------------------------------------------------------------------------------------
// Посмотреть пины командой <gpio readall> Пины имеют соответсвие между BMC и wiringpi
#define PIN_LED_BLUE 4 // Мигает в цикле что работает
// #define PIN_LED_GREEN 22   // Ошибки при передаче или еще где

#define PIN_SPI_MODUL 21  // Пин на котором сидит чипселекс от SPI к Modul
#define PIN_SPI_DRIVER 23 // Пин на котором сидит чипселекс от SPI к Modul
#define PIN_SPI_PRINT 26  // Пин на котором сидит чипселекс от SPI к Modul
//---------------------------------------------------------------------------------------
#define SPI_CHANNAL_0 0   // Какой из двух каналов инициализируем
#define SPI_CHANNAL_1 1   // Какой из двух каналов инициализируем
#define SPI_SPEED 1000000 // Скорость работы шины SPI
//---------------------------------------------------------------------------------------
// ПЕРЕДЕЛАТЬ НА БУФЕРЫ РАЗНОГО РАЗМЕРА!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define SIZE_BUFF 192            // Размер буфера, стараться делать кратно 32
unsigned char buffer[SIZE_BUFF]; // Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт
//---------------------------------------------------------------------------------------

#define DISTANCE_WHEELS 0.38 // Растояние между колесами робота. подобрал экспериментально Влияет на правильность круга
#define DIAMETR 0.151        // Влияет на правильность длинны через расчет скорости
#define RADIUS (DIAMETR / 2)
#define PERIMETR (DIAMETR * M_PI)
#define RAD2DEG(x) ((x) * 180. / M_PI) // Перевод из радиан в градусы
#define DEG2RAD(x) ((x) * M_PI / 180.) // Перевод из градусов в радианы
#define ACCELERATION 1.0               // Оборота за секунду в квадрате rps
bool rezModul = false;
bool rezPrint = false;
bool rezData = false;

float offSetLaser[4] {0.019, 0.01, 0.01, 0.017}; // Поправочные значения для датчиков расстояния на модуле

int data_driver_all = 0;
int data_driver_bed = 0;
int data_modul_all = 0;
int data_modul_bed = 0;
int data_print_all = 0;
int data_print_bed = 0;

bool flag_msgJoy = false;           // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgControlDriver = false; // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgControlModul = false;  // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgControlPrint = false;  // Флаг что пришло сообщение в топик и можно его парсить

float linearOffsetX[128];
float linearOffsetY[128];

//============================================================================================================================================================
Struct_Data2Driver Data2Driver; // Экземпляр структуры отправлемых данных
Struct_Data2Modul Data2Modul;   // Экземпляр структуры отправлемых данных
Struct_Data2Print Data2Print;   // Экземпляр структуры отправлемых данных
//============================================================================================================================================================
Struct_Driver2Data Driver2Data; // Тело робота. тут все переменные его характеризующие на низком уровне
Struct_Modul2Data Modul2Data;
Struct_Print2Data Print2Data; // Экземпляр структуры принимаемых данных

const uint32_t size_stucturs = sizeof(Struct_Driver2Data);

SSpi spi; // Переменная где все данные по обмену

struct SEuler
{
  float roll = 0;  // Крен в право  влево
  float pitch = 0; // Тангаж вверх или вних
  float yaw = 0;   // Поворот по часовой мом против часовой
};

// Структура для углов наклонов
struct STwist
{
  double vx = 0;  // Линейная скорость движения робота по оси X
  double vy = 0;  // Линейная скорость движения робота по оси Y
  double vth = 0; // Угловая скорость вращения робота
};
// Структура для одометрии
struct SOdom
{
  SPose pose;
  STwist twist;
};
SOdom odomWheel;  // Высчитанная одометрия по энкодеру без корректировок
SOdom odomMpu;    // Высчитанная одомтрия по датчику mpu cкорректированная
SOdom odomUnited; // Высчитанная одометрия по энкодеру скорректированная

struct STwistDt
{
  STwist twist;
  double dt = 0.0;
};
STwistDt mpuTwistDt;    // Скорости полученные по mpu и интревал который прошел с предыдущего измерения
STwistDt wheelTwistDt;  // Скорости полученные по ecoder и интревал который прошел с предыдущего измерения
STwistDt unitedTwistDt; // Обьединенные комплементратный способом Скорости и интервал

// ************************************************************* Struct_Data2Driver *********************
struct SPoseTrue
{
  float x;           // Правильная позиция на которую надо заменить имеющиеся данные
  float y;           // Правильная позиция на которую надо заменить имеющиеся данные
  float th;          // Правильная позиция на которую надо заменить имеющиеся данные
  uint32_t flag = 0; // Флаг нужно ли брать и использовать данные или они повторные и мы их игнорируем
};
//****************************************************************************************************************************************************

SControl g_dreamSpeed; // Желаемая скорость
SPose g_poseControl;   // Позиция с верхнего уровня на которую надо подменить текущую позицию
//****************************************************************************************************************************************************

//****************************************************************************************************************************************************
// ============================================================
// Функция возвращает контрольную сумму структуры без последних 4 байтов, оформленна как шаблон. Может разные структуры обсчитывать, как разные типы данных входных
template <typename T>
uint32_t measureCheksum(const T &structura_)
{
  uint32_t ret = 0;
  unsigned char *adr_structura = (unsigned char *)(&structura_); // Запоминаем адрес начала структуры. Используем для побайтной передачи
  for (int i = 0; i < sizeof(structura_) - 4; i++)
  {
    ret += adr_structura[i]; // Побайтно складываем все байты структуры кроме последних 4 в которых переменная в которую запишем результат
  }
  return ret;
}

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
#define NN "\x1b[33;40m Data_node:"

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