#ifndef GENSTRUCT_H
#define GENSTRUCT_H
#include </opt/ros/melodic/include/ros/ros.h>
#include </opt/ros/melodic/include/ros/time.h>
#include <log4cxx/mdc.h>
#include <ros/ros.h>                   // Библиотека ROS для работы с узлами
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>     // Сообщения для данных лидара
#include <visualization_msgs/Marker.h> // Сообщения для визуализации в RViz
#include <vector>                      // Стандартный вектор C++
#include <cmath>                       // Математические функции (sin, cos, sqrt)
#include <signal.h>                    // Для обработки Ctrl+C


static const double THROTTLE_PERIOD_1 = 1.0; // секунды 
static const double THROTTLE_PERIOD_3 = 0.33; // секунды 
static const double THROTTLE_PERIOD_5 = 0.20; // секунды 
static const double THROTTLE_PERIOD_10 = 0.1; // секунды 

//============================================================================================================================================================
// Структура передаваемых данных из Data к Modul
struct SControlLaser
{
  uint32_t mode = 0; // Текущий режим работы 0 - отключить датчики 1 - режим одновременного измерения
};
struct SControlMotor
{
  uint32_t mode = 0;                    // Текущий режим работы 0 - режим колибровки концевиков 1 - обычное управление моторами по углу
  float angle[4];                       // Углы в которые нужно повернультя в локальной системе
  int32_t numPillar[4]{-1, -1, -1, -1}; // Номер столба до которого измеряем расстояние
};

struct STest
{
  uint8_t byte0 = 0x00;
  uint8_t byte1 = 0x01;
  uint8_t byte2 = 0x02;
  uint8_t byte3 = 0x03;
  float fff = 3.1415;
};
// Структура получаемых данных от Data к контроллеру Modul
struct Struct_Data2Modul
{
  uint32_t id = 0;            // Номер команды по порядку
  SControlMotor controlMotor; // Управление моторами
  SControlLaser controlLaser; // Управление лазерами
  uint32_t cheksum = 0;       // Контрольная сумма данных в структуре
};
//============================================================================================================================================================
struct SFirmware
{
  uint8_t gen;   // Поколение
  uint8_t ver;   // Версия
  uint8_t laser; // Вариант использования лазеров
  uint8_t debug; // Вариант использования отладки для вывода в printf
  float motor;   // Вариант использования моторов сколько шагов на оборот 200 или 400 (1,8 градуса или 0,9 градусов на шаг)
};

struct SLaserSend
{
  uint32_t status;        // Статус датчика или ошибки
  float distance;         // Последнее измерение
  uint32_t signalQuality; // Качество сигнала
  float angle;            // Положение при последнем измерении
  uint32_t time;          // Время измерения от начала запуска программы
  float rate;             // Частота работы датчика
  int32_t numPillar;      // Номер столба до которого измерили расстояние
};

struct SMotorSend // Структура которая передается на верхний уровень
{
  int32_t status = 0;    // Передаются импульсы на мотор или нет в данный момент, вращается или нет
  float position = 0;    // Текущая позиция в градусах
  float destination = 0; // Цель назначение в позиции в градусах
};
// Структура по обратной связи по обмену по шине SPI
struct SSpi
{
  uint32_t all = 0;
  uint32_t bed = 0;
};
struct SXyz //
{
  float x = 0;
  float y = 0;
  float z = 0;
};
struct SMpu // Структура с данными со всех датчиков, отправляем наверх
{
  int32_t status; // статус состояния
  float rate;     // частота работы датчика
  struct SXyz angleEuler;
  struct SXyz linear;
  struct SXyz accel;
  struct SXyz gyro;
  struct SXyz mag;
};

// Структура в которой все главные переменные передаюся на высокий уровень от Modul к Data
struct Struct_Modul2Data
{
  uint32_t id;                // id команды
  struct SFirmware firmware;  // Версия прошики и использованного оборудования
  uint32_t pinMotorEn;        // Стутус пина управления драйвером моторов, включен драйвер или нет
  struct SMotorSend motor[4]; // Структура по состоянию моторов
  struct SLaserSend laser[4]; // Структура по состоянию лазеров
  uint32_t micric[4];         // Структура по состоянию концевиков
  struct SMpu bno055;         // Данные с датчика BNO055
  struct SSpi spi;            // Структура по состоянию обмена по шине SPI

  uint32_t cheksum; // Контрольная сумма данных в структуре
};

//============================================================================================================================================================
// Структура сосдержит всю информацию по мотору на основании данных энкодера
struct SMotor
{
  uint32_t status = 0; // Статус драйвера.Включен или выключен.Удерживаются колосеса или свободно катаются
  float rpsEncodL = 0; // Реальная скорость вращения по енкодерам( обороты в секунду)
  float rpsEncodR = 0; // Реальная скорость вращения по енкодерам( обороты в секунду)
};

// Структура состояния датчика расстония
struct SSensor
{
  int32_t status = 0;   // статус состояния
  float distance = 0.0; // расстояние до препятствия
};

// Структура в которой все главные переменные передаюся на высокий уровень
struct Struct_Driver2Data
{
  uint32_t id = 0; // id команды
  SMotor motor;
  // SMpu bno055; // Данные с датчика BNO055
  SSensor laserL;
  SSensor laserR;
  SSensor uzi;
  SSpi spi;             // Структура по состоянию обмена по шине SPI
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};
//============================================================================================================================================================
// Структура управления движением
struct SControl
{
  float speedL = 0; // Скорость с которой нужно двигаться в оборотах/секунда
  float speedR = 0; // Скорость с которой нужно двигаться в оборотах/секунда
};

#define NUM_LEDS 43 // Колличество светодиодов всего
// Структура управления Светодиодами
struct SLed
{
  uint8_t led[NUM_LEDS]; // Маасив через который управляем светодиодами
};
// Структура отправляемых данных от Data к контроллеру Driver
struct Struct_Data2Driver
{
  uint32_t id = 0;      // Номер команды по порядку
  SControl control;     // Структура управления машиной
  SLed led;             // Управление светодиодами
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};
//============================================================================================================================================================
// Структура отправляемых данных от Data к контроллеру Print
struct SControlPrint
{
  uint32_t mode = 0;      // Текущий режим работы какеи сопла печатют
  uint32_t status = 0;    // Текущий режим работы 0 - не печатем, 1 печатаем
  uint32_t intensity = 2; // ИНтенсивность печати. Сколько раз прыскаем на 1 мм Значения по умолчанию используются если нет управления из топиков
  float speed = 0.5;      // Текущая скорость движения при которой надо печатать. От нее зависит интервал между выпрыскиванием чернил Значения по умолчанию используются если нет управления из топиков
  float position;         // Позиция задаваемая по CAN
  float velocity;         // Скорость задаваемая по CAN
  float torque;           // Момент задаваемая по CAN
};
// Структура получаемых данных из Data к Print
struct Struct_Data2Print
{
  uint32_t id = 0;            // Id команды
  SControlPrint controlPrint; // Режим печати
  uint32_t cheksum = 0;       // Контрольная сумма данных в структуре
};

//============================================================================================================================================================
// Структура по состоянию лидаров которая передается на верхний уровень

struct SFirmwarePrint
{
  uint8_t gen;   // Поколение
  uint8_t ver;   // Версия
  uint8_t debug; // Вариант использования отладки для вывода в printf
  uint8_t test;  // Вариант использования отладки для вывода в printf
};

// Структура состояния мотора GIM4305 4310
struct SGim43
{
  float position;
  float velocity;
  float torque;
};

// Структура в которой все главные переменные передаюся на высокий уровень от Print к Data
struct Struct_Print2Data
{
  uint32_t id;                    // id команды
  struct SFirmwarePrint firmware; // Версия прошики и использованного оборудования
  struct SGim43 gim43;
  struct SSpi spi; // Структура по состоянию обмена по шине SPI

  uint32_t cheksum; // Контрольная сумма данных в структуре
};
//============================================================================================================================================================

struct SPoint // Точка
{
  double x = 0; //
  double y = 0; //
};
struct SPose
{
  double x = 0;  // Координата по Х
  double y = 0;  // Координата по Y
  double th = 0; // Направление носа
};

//---
// // Структура сосдержит всю информацию по мотору на основании данных энкодера
// struct SEncoder
// {
//     float rpsSet = 0;   // Текущая скорость вращения ( обороты в секунду)
//     float rpsEncod = 0; // Текущая скорость вращения ( обороты в секунду)
// };

// struct SMpu // Структура с данными с mpu bno055
// {
//     SPose pose;
//     STwist twist;
//     SEuler angleEuler;
// };

// // Структура принимаемых данных от контроллера Driver в Data
// struct SDriver2Data
// {
//     uint32_t id = 0; // id команды
//     SEncoder motorLeft;
//     SEncoder motorRight;
//     double dtEncoder; // Время за которое данные с энкодера
//     SMpu bno055;      // Данные с датчика BNO055
//     SSensor laserL;
//     SSensor laserR;
//     SSensor uzi;
// };

//************************************** ОБЬЯВЛЯЕМ ФУНКЦИИ **********************************
float sqr(float x_);                                   // Функция возведния в квадрат
float ctan(float x_);                                  // Функция котангенса угла
float vectorLen(SPoint point1, SPoint point2);         // Функция нахождения длинны вектора
void normalizeVector(double &x, double &y, double &z); // Нормализация вектора

// Функция возведения в квадрат
float sqr(float x_)
{
  return x_ * x_;
}
// Функция котангенса
float ctan(float x_)
{
  return 1 / tan(x_);
}
float vectorLen(SPoint point1, SPoint point2) // Функция возвращает длинну вектора, фактически растояние по прямой между двумя точкам
{
  float dx = abs(point2.x - point1.x); // Находим разницу в координатах
  float dy = abs(point2.y - point1.y); //
  float len = sqrt(sqr(dx) + sqr(dy)); // Теорема пифагора
  return len;
}

void normalizeVector(double &x, double &y, double &z)
{
  double norm = sqrt(x * x + y * y + z * z);

  x /= norm;
  y /= norm;
  z /= norm;
}

#endif