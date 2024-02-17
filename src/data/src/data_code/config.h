#ifndef CONFIG_H
#define CONFIG_H



#include </opt/ros/melodic/include/ros/ros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Joy.h>
#include <data/SJoy.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <data/SControl.h>
#include <data/SLed.h>
#include <data/SControlModul.h>

#include <data/SEncoder.h>
#include <data/SMpu.h>
#include <data/Struct_Odom.h>
#include <data/SSensor.h>

#include <data/SControlDriver.h>
#include <data/SDriver2Data.h>

#include <data/Struct_ModulMotor.h>
#include <data/Struct_ModulLidar.h>
#include <data/Struct_ModulMicric.h>
#include <data/Struct_Info_SPI.h>

data::SControlDriver msg_ControlDriver; // Полученное сообщение из топика Head в Data
data::SControlModul msg_ControlModul;   // Полученное сообщение из топика
sensor_msgs::Joy msg_joy;                   // Переменная в которую записываем пришедшее сообщение а колбеке

#define RATE 20 // Частота обмена с нижним уровнев в Герц
//---------------------------------------------------------------------------------------
// Посмотреть пины командой <gpio readall> Пины имеют соответсвие между BMC и wiringpi
#define PIN_LED_BLUE 4 // Мигает в цикле что работает
// #define PIN_LED_GREEN 22   // Ошибки при передаче или еще где

// #define PIN_PWM 26
//---------------------------------------------------------------------------------------
#define SPI_CHANNAL_0 0   // Какой из двух каналов инициализируем
#define SPI_CHANNAL_1 1   // Какой из двух каналов инициализируем
#define SPI_SPEED 1000000 // Скорость работы шины SPI
// #define PIN_SPI_LINE 2 // Пин обратной связи с ведомым
// #define PIN_SPI_LINE 3 // Пин обратной связи с ведомым
//---------------------------------------------------------------------------------------

#define SIZE_BUFF 160            // Размер буфера, стараться делать кратно 32
unsigned char buffer[SIZE_BUFF]; // Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт
//---------------------------------------------------------------------------------------
// #define DEVICE_ID_STM 0x1A         // Адрес I2C платы STM как slave устройства

#define DISTANCE_WHEELS 0.38 // Растояние между колесами робота. подобрал экспериментально Влияет на правильность круга
#define DIAMETR 0.150 // Влияет на правильность длинны через расчет скорости
#define RADIUS (DIAMETR / 2)
#define PERIMETR (DIAMETR * M_PI)
#define RAD2DEG(x) ((x) * 180. / M_PI) // Первод из радиан в градусы
#define DEG2RAD(x) ((x) * M_PI / 180.) // Первод из градусов в радианы

bool rez_data = false;

int data_driver_all = 0;
int data_driver_bed = 0;
int data_modul_all = 0;
int data_modul_bed = 0;

bool flag_msgJoy = false;    // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgControlDriver = false;    // Флаг что пришло сообщение в топик и можно его парсить
bool flag_dt = true;    


//============================================================================================================================================================
// Структура передаваемых данных из Data к Modul
struct Struct_Data2Modul
{
  uint32_t id = 0;      // Номер команды по порядку

  uint32_t command = 0; // Текущая команда к выполенению 0 - режим колибровки концевиков 1 - обычное управление моторами по углу
  float angle[4];       // Углы в которые нужно повернультя в глобальной системе

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};
Struct_Data2Modul Data2Modul; // Экземпляр структуры отправлемых данных
//---------------------------------------------------------------------------
struct motorStructSend // Структура
{
  int32_t status = 0;    // Передаются импульсы на мотор или нет в данный момент, вращается или нет
  float position = 0;    // Текущая позиция в градусах
  float destination = 0; // Цель назначение в позиции в градусах
};
// Структура по состоянию лидаров
struct lidarStructSend
{
  uint32_t status = 0; // Статус датчика или ошибки
  float distance = 0;  // Последнее измерение
  float angle = 0;     // Положение при последнем измерении
};
// Структура по обратной связи по обмену по шине SPI
struct SSpi
{
  uint32_t all = 0;
  uint32_t bed = 0;
};

// Структура в которой все собранные данные передаются из Modul к Data
struct Struct_Modul2Data
{
  uint32_t id = 0; // id команды

  uint32_t pinMotorEn = 0;  // Стутус пина управления драйвером моторов, включен драйвер или нет
  motorStructSend motor[4]; // Структура по состоянию моторов
  lidarStructSend lidar[4]; // Структура по состоянию лидаров
  uint32_t micric[4];       // Структура по состоянию концевиков
  SSpi spi;                 // Структура по состоянию обмена по шине SPI

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Modul2Data Modul2Data;

//============================================================================================================================================================
// Структура сосдержит всю информацию по мотору на основании данных энкодера
struct SMotor
{
  uint32_t statusDriver = 0; // Статус драйвера.Включен или выключен.Удерживаются колосеса или свободно катаются
  float rpsEncodL = 0;       // Реальная скорость вращения по енкодерам( обороты в секунду)
  float rpsEncodR = 0;       // Реальная скорость вращения по енкодерам( обороты в секунду)
};


struct SEuler
{
  float roll = 0;  // Крен в право  влево
  float pitch = 0; // Тангаж вверх или вних
  float yaw = 0;   // Поворот по часовой мом против часовой
};
struct SXyz // 
{
  float x = 0;  
  float y = 0;  
  float z = 0; 
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

struct SMpu // Структура с данными со всех датчиков, отправляем наверх
{
  int32_t status = 0; // статус состояния
  SXyz angleEuler;
  SXyz linear;
};

// Структура состояния датчика расстония
struct SSensor
{
  int32_t status = 0; // статус состояния
  float distance = 0; // расстояние до препятствия
};

// Структура принимаемых данных от контроллера Driver в Data
struct Struct_Driver2Data
{
  uint32_t id = 0; // id команды
  SMotor motor;
  SMpu bno055; // Данные с датчика BNO055
  SSensor laserL;
  SSensor laserR;
  SSensor uzi;
  SSpi spi;             // Структура по состоянию обмена по шине SPI
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Driver2Data Driver2Data; // Тело робота. тут все переменные его характеризующие на низком уровне
const uint32_t size_stucturs = sizeof(Struct_Driver2Data);

SOdom encoder; // Высчитанная одометрия по энкодеру
SOdom mpu; // Высчитанная одомтрия по датчику mpu bno055

// ************************************************************* Struct_Data2Driver *********************
// Структура управления движением
struct SControl
{
  float speedL = 0;  // Скорость с которой нужно двигаться в оборотах/секунда
  float speedR = 0;  // Скорость с которой нужно двигаться в оборотах/секунда
};
// Структура управления Светодиодами
struct SLed
{
  int32_t num_program = 0; // Номер программы для светодиодов мигания
};
// Структура отправляемых данных от Data к контроллеру Driver
struct SData2Driver
{
  uint32_t id = 0;      // Номер команды по порядку
  SControl control;     // Структура управления машиной
  SLed led;             // Управление светодиодами
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

SData2Driver Data2Driver; // Экземпляр структуры отправлемых данных
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