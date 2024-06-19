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

//#include <pb_msgs/SControlModul.h>
//#include <pb_msgs/SControlPrint.h>
#include <pb_msgs/Struct_Data2Print.h>
#include <pb_msgs/Struct_Data2Modul.h>
#include <pb_msgs/Struct_Data2Driver.h>
//#include <pb_msgs/SControlDriver.h>

#include <pb_msgs/SEncoder.h>
#include <pb_msgs/SMpu.h>
#include <pb_msgs/Struct_Odom.h>
#include <pb_msgs/SSensor.h>
#include <pb_msgs/SMotor.h>


#include <pb_msgs/Struct_Driver2Data.h>
#include <pb_msgs/Struct_Modul2Data.h>

#include <pb_msgs/Struct_Info_SPI.h>

pb_msgs::Struct_Data2Driver msg_ControlDriver; // Полученное сообщение из топика Head в Data
pb_msgs::Struct_Data2Modul msg_ControlModul;   // Полученное сообщение из топика
pb_msgs::Struct_Data2Print msg_ControlPrint;   // Полученное сообщение из топика
sensor_msgs::Joy msg_joy;                  // Переменная в которую записываем пришедшее сообщение а колбеке

#define RATE 100 // Частота обмена с нижним уровнев в Герц
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

#define SIZE_BUFF 160            // Размер буфера, стараться делать кратно 32
unsigned char buffer[SIZE_BUFF]; // Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт
//---------------------------------------------------------------------------------------

#define DISTANCE_WHEELS 0.38 // Растояние между колесами робота. подобрал экспериментально Влияет на правильность круга
#define DIAMETR 0.151        // Влияет на правильность длинны через расчет скорости
#define RADIUS (DIAMETR / 2)
#define PERIMETR (DIAMETR * M_PI)
#define RAD2DEG(x) ((x) * 180. / M_PI) // Первод из радиан в градусы
#define DEG2RAD(x) ((x) * M_PI / 180.) // Первод из градусов в радианы
#define ACCELERATION 1.0               // Оборота за секунду в квадрате rps
bool rezModul = false;
bool rezPrint = false;
bool rezData = false;

#define NUM_LEDS 43 // Колличество светодиодов всего

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
// Структура передаваемых данных из Data к Modul
struct SControlLaser
{
  uint32_t mode = 0; // Текущий режим работы 0 - отключить датчики 1 - режим одновременного измерения
};
struct SControlMotor
{
  uint32_t mode = 0; // Текущий режим работы 0 - режим колибровки концевиков 1 - обычное управление моторами по углу
  float angle[4];    // Углы в которые нужно повернультя в локальной системе
};

// Структура получаемых данных от Data к контроллеру Modul
struct Struct_Data2Modul
{
  uint32_t id = 0;            // Номер команды по порядку
  SControlMotor controlMotor; // Управление моторами
  SControlLaser controlLaser; // Управление лазерами
  uint32_t cheksum = 0;       // Контрольная сумма данных в структуре
};
Struct_Data2Modul Data2Modul; // Экземпляр структуры отправлемых данных
//============================================================================================================================================================
struct SLaserSend
{
  uint32_t status = 0;        // Статус датчика или ошибки
  uint32_t distance = 0;      // Последнее измерение
  uint32_t signalQuality = 0; // Качество сигнала
  float angle = 0;            // Положение при последнем измерении
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
SSpi spi; // Переменная где все данные по обмену

// Структура в которой все главные переменные передаюся на высокий уровень от Modul к Data
struct Struct_Modul2Data
{
  uint32_t id = 0; // id команды

  uint32_t pinMotorEn = 0;      // Стутус пина управления драйвером моторов, включен драйвер или нет
  SMotorSend motor[4];          // Структура по состоянию моторов
  SLaserSend laser[4];          // Структура по состоянию лазеров
  uint32_t statusDataLaser = 0; // Статус обновления данных с лазерных датчиков
  uint32_t micric[4];           // Структура по состоянию концевиков
  SSpi spi;                     // Структура по состоянию обмена по шине SPI

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Modul2Data Modul2Data;

//============================================================================================================================================================
// Структура сосдержит всю информацию по мотору на основании данных энкодера
struct SMotor
{
  uint32_t status = 0; // Статус драйвера.Включен или выключен.Удерживаются колосеса или свободно катаются
  float rpsEncodL = 0; // Реальная скорость вращения по енкодерам( обороты в секунду)
  float rpsEncodR = 0; // Реальная скорость вращения по енкодерам( обороты в секунду)
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

struct SMpu // Структура с данными со всех датчиков, отправляем наверх
{
  int32_t status = 0; // статус состояния
  SXyz angleEuler;
  SXyz linear;
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
  SMpu bno055; // Данные с датчика BNO055
  SSensor laserL;
  SSensor laserR;
  SSensor uzi;
  SSpi spi;             // Структура по состоянию обмена по шине SPI
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Driver2Data Driver2Data; // Тело робота. тут все переменные его характеризующие на низком уровне
const uint32_t size_stucturs = sizeof(Struct_Driver2Data);

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
// Структура управления движением
struct SControl
{
  float speedL = 0; // Скорость с которой нужно двигаться в оборотах/секунда
  float speedR = 0; // Скорость с которой нужно двигаться в оборотах/секунда
};
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
Struct_Data2Driver Data2Driver; // Экземпляр структуры отправлемых данных

SControl g_dreamSpeed; // Желаемая скорость
SPose g_poseControl;   // Позиция с верхнего уровня на которую надо подменить текущую позицию
//****************************************************************************************************************************************************
// Структура отправляемых данных от Data к контроллеру Print

struct SControlPrint
{
  uint32_t status = 0;    // Текущий режим работы 0 - не печатем, 1 печатаем
  uint32_t mode = 0;      // Текущий режим работы какеи сопла печатют
  uint32_t intensity = 2; // ИНтенсивность печати. Сколько раз прыскаем на 1 мм Значения по умолчанию используются если нет управления из топиков
  float speed = 0.5;      // Текущая скорость движения при которой надо печатать. От нее зависит интервал между выпрыскиванием чернил Значения по умолчанию используются если нет управления из топиков
};
// Структура получаемых данных из Data к Print
struct Struct_Data2Print
{
  uint32_t id = 0;            // Id команды
  SControlPrint controlPrint; // Режим печати
  uint32_t cheksum = 0;       // Контрольная сумма данных в структуре
};
Struct_Data2Print Data2Print; // Экземпляр структуры отправлемых данных

// Структура в которой все главные переменные передаюся на высокий уровень
struct Struct_Print2Data
{
  uint32_t id = 0;      // id команды
  SSpi spi;             // Структура по состоянию обмена по шине SPI
  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Print2Data Print2Data; // Экземпляр структуры принимаемых данных

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