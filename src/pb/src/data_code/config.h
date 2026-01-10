#ifndef CONFIG_H
#define CONFIG_H

#include </opt/ros/melodic/include/ros/ros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <pb_msgs/SControl.h>
#include <pb_msgs/SLed.h>

#include <pb_msgs/SXyz.h>

#include <pb_msgs/Struct_Data2Print.h>
#include <pb_msgs/Struct_Data2Modul.h>
#include <pb_msgs/Struct_Data2Driver.h>

#include <pb_msgs/Struct_Driver2Data.h>
#include <pb_msgs/Struct_Modul2Data.h>
#include <pb_msgs/Struct_Print2Data.h>

#include <pb_msgs/Struct_Info_SPI.h>
#include <pb_msgs/SSetSpeed.h>

u_int64_t timeSpiModul = 0;  // Время когда пришла команда по топикам
u_int64_t timeSpiDriver = 0; // Время когда пришла команда по топикам
u_int64_t timeSpiPrint = 0;  // Время когда пришла команда по топикам

pb_msgs::Struct_Data2Driver msg_ControlDriver; // Полученное сообщение из топика Head в Data
pb_msgs::Struct_Data2Modul msg_ControlModul;   // Полученное сообщение из топика
pb_msgs::Struct_Data2Print msg_ControlPrint;   // Полученное сообщение из топика


#define PRINTBIM23
// #define PRINTBIM25

#define RATE 4 // Частота обмена с нижним уровнев в Герц  

//---------------------------------------------------------------------------------------
// Посмотреть пины командой <gpio readall> Пины имеют соответсвие между BMC и wiringpi
#define PIN_LED_BLUE 25 // Мигает в цикле что работает
// #define PIN_LED_GREEN 22   // Ошибки при передаче или еще где

#define PIN_SPI_DRIVER 23 // Пин на котором сидит чипселекс от SPI к Modul на плате это 13
#define PIN_SPI_MODUL 26  // Пин на котором сидит чипселекс от SPI к Modul на плате это 12
#define PIN_SPI_PRINT 21  // Пин на котором сидит чипселекс от SPI к Modul на плате это 5
//---------------------------------------------------------------------------------------
#define SPI_CHANNAL_0 0   // Какой из двух каналов инициализируем
#define SPI_CHANNAL_1 1   // Какой из двух каналов инициализируем
#define SPI_SPEED 4000000 // Скорость работы шины SPI
//---------------------------------------------------------------------------------------
#define PIN_MODUL_MOSI_2G 7  // Пин на котором сидит чипселекс от SPI к Modul
#define PIN_MODUL_MISO_3G 1  // Пин на котором сидит чипселекс от SPI к Modul
#define PIN_MODUL_CLK_4G 0  // Пин на котором сидит чипселекс от SPI к Modul

#define PIN_PRINT_MOSI_1G 6  // Пин на котором сидит чипселекс от SPI к Modul
#define PIN_PRINT_MISO_4G 30  // Пин на котором сидит чипселекс от SPI к Modul
#define PIN_PRINT_CLK_3G 31  // Пин на котором сидит чипселекс от SPI к Modul




#define ACCELERATION 0.5 // Метры в секунду в квадрате м/с

#define DIAMETR 0.150 // Влияет на правильность длинны через расчет скорости
#define RADIUS (DIAMETR / 2)
#define PERIMETR (DIAMETR * M_PI)
#define KOEF_ODOM 1.0000; // Коефициент для одометрии. подбираем экспериментально, что-бы было точно движение и расчет

bool rezModul = false;
bool rezPrint = false;
bool rezData = false;

// #ifdef PRINTBIM23
// c // Поправочные значения для датчиков расстояния на модуле в метрах
// #endif
// #ifdef PRINTBIM25
// float offSetLaser[4]{0.011, 0.005, 0.005, 0.006}; // Поправочные значения для датчиков расстояния на модуле
// #endif

int data_driver_all = 0;
int data_driver_bed = 0;
int data_modul_all = 0;
int data_modul_bed = 0;
int data_print_all = 0;
int data_print_bed = 0;


bool flag_msgControlDriver = false; // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgControlModul = false;  // Флаг что пришло сообщение в топик и можно его парсить
bool flag_msgControlPrint = false;  // Флаг что пришло сообщение в топик и можно его парсить

//============================================================================================================================================================
Struct_Data2Driver Data2Driver; // Экземпляр структуры отправлемых данных
Struct_Data2Modul Data2Modul;   // Экземпляр структуры отправлемых данных
Struct_Data2Print Data2Print;   // Экземпляр структуры отправлемых данных
//============================================================================================================================================================
Struct_Driver2Data Driver2Data; // Тело робота. тут все переменные его характеризующие на низком уровне
Struct_Modul2Data Modul2Data;
Struct_Print2Data Print2Data; // Экземпляр структуры принимаемых данных

const uint16_t size_structura_Data2Driver = sizeof(Data2Driver);                                                                                      // Размер структуры с данными которые получаем
const uint16_t size_structura_Driver2Data = sizeof(Driver2Data);                                                                                      // Размер структуры с данными которые получаем
const uint16_t max_size_stuct1 = (size_structura_Data2Driver < size_structura_Driver2Data) ? size_structura_Driver2Data : size_structura_Data2Driver; // Какая из структур больше 80

const uint16_t size_structura_Data2Modul = sizeof(Data2Modul);                                                                                    // Размер структуры с данными которые получаем
const uint16_t size_structura_Modul2Data = sizeof(Modul2Data);                                                                                    // Размер структуры с данными которые получаем
const uint16_t max_size_stuct2 = (size_structura_Data2Modul < size_structura_Modul2Data) ? size_structura_Modul2Data : size_structura_Data2Modul; // Какая из структур больше 236

const uint16_t size_structura_Data2Print = sizeof(Data2Print);                                                                                    // Размер структуры с данными которые получаем
const uint16_t size_structura_Print2Data = sizeof(Print2Data);                                                                                    // Размер структуры с данными которые получаем
const uint16_t max_size_stuct3 = (size_structura_Data2Print < size_structura_Print2Data) ? size_structura_Print2Data : size_structura_Data2Print; // Какая из структур больше 24

// ПЕРЕДЕЛАТЬ НА БУФЕРЫ РАЗНОГО РАЗМЕРА - СДЕЛАЛ !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// #define SIZE_BUFF 192            // Размер буфера, стараться делать кратно 32
#define SIZE_BUFF_DRIVER 96                   // Размер буфера, стараться делать кратно 32
unsigned char bufferDriver[SIZE_BUFF_DRIVER]; // Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт

#define SIZE_BUFF_PRINT 36                  // Размер буфера, стараться делать кратно 32
unsigned char bufferPrint[SIZE_BUFF_PRINT]; // Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт

#define SIZE_BUFF_MODUL 352                 // Размер буфера, стараться делать кратно 32 для ESP, для STM непонятно пока
unsigned char bufferModul[SIZE_BUFF_MODUL]; // Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт
                                            //---------------------------------------------------------------------------------------

	double offSetUzi = 0;
	double offSetLaserL = 0;
	double offSetLaserR = 0;
  
  double dist_offsets[4];        // Офсеты для дальномеров (0-3)
  double angle_offsets[4];       // Массив для калибровочных офсетов (из yaml)

int modeModul = 0;

int unitDriver = false; // Переменные для определения с какими модулями работаем
int unitModul = false;
int unitPrint = false;

STest Modul2Test;
STest Test2Modul;

const uint32_t size_stucturs = sizeof(Struct_Driver2Data);

SSpi spi; // Переменная где все данные по обмену

// ************************************************************* Struct_Data2Driver *********************
struct SPoseTrue
{
  float x;           // Правильная позиция на которую надо заменить имеющиеся данные
  float y;           // Правильная позиция на которую надо заменить имеющиеся данные
  float th;          // Правильная позиция на которую надо заменить имеющиеся данные
  uint32_t flag = 0; // Флаг нужно ли брать и использовать данные или они повторные и мы их игнорируем
};
//****************************************************************************************************************************************************

SControl g_desiredSpeed; // Желаемая скорость desired
SControl g_factSpeed;  // Фактически ранее установленная скорость переданная на моторы и в топик в метрах за секунду
SPose g_poseControl;   // Позиция с верхнего уровня на которую надо подменить текущую позицию

//****************************************************************************************************************************************************
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