#ifndef CONFIG_H
#define CONFIG_H

#define RATE 10 // Частота обмена с нижним уровнев в Герц
//---------------------------------------------------------------------------------------
// Посмотреть пины командой <gpio readall> Пины имеют соответсвие между BMC и wiringpi
#define PIN_LED_BLUE 4 // Мигает в цикле что работает
// #define PIN_LED_GREEN 22   // Ошибки при передаче или еще где

// #define PIN_PWM 26
//---------------------------------------------------------------------------------------
#define SPI_CHANNAL_0 0	  // Какой из двух каналов инициализируем
#define SPI_CHANNAL_1 1	  // Какой из двух каналов инициализируем
#define SPI_SPEED 1000000 // Скорость работы шины SPI
// #define PIN_SPI_LINE 2 // Пин обратной связи с ведомым
// #define PIN_SPI_LINE 3 // Пин обратной связи с ведомым
//---------------------------------------------------------------------------------------

#define SIZE_BUFF 160			 // Размер буфера, стараться делать кратно 32
unsigned char buffer[SIZE_BUFF]; // Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт
//---------------------------------------------------------------------------------------
// #define DEVICE_ID_STM 0x1A         // Адрес I2C платы STM как slave устройства

bool rez_data = false;

int data_driver_all = 0;
int data_driver_bed = 0;
int data_modul_all = 0;
int data_modul_bed = 0;

float a1, a2;
float u1, u2;

//============================================================================================================================================================
// Структура передаваемых данных из Data к Modul
struct Struct_Data2Modul
{
	uint32_t id = 0;	  // Номер команды по порядку
	uint32_t command = 0; // Текущая команда к выполенению 0 - режим колибровки концевиков 1 - обычное управление моторами по углу
	float angle[4];		  // Углы в которые нужно повернультя в глобальной системе

	uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};
Struct_Data2Modul Data2Modul; // Экземпляр структуры отправлемых данных
//---------------------------------------------------------------------------
struct motorStructSend // Структура 
{
	int32_t status = 0;	   // Передаются импульсы на мотор или нет в данный момент, вращается или нет
	float position = 0;	   // Текущая позиция в градусах
	float destination = 0; // Цель назначение в позиции в градусах
};
// Структура по состоянию лидаров 
struct lidarStructSend
{
	uint32_t status = 0; // Статус датчика или ошибки
	float distance = 0;	 // Последнее измерение
	float angle = 0;	 // Положение при последнем измерении
};
//Структура по обратной связи по обмену по шине SPI
struct  spiStruct
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
	uint32_t micric[4];		  // Структура по состоянию концевиков
	spiStruct spi;            // Структура по состоянию обмена по шине SPI

	uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Modul2Data Modul2Data;



//============================================================================================================================================================
// Структура сосдержит всю информацию по мотору на основании данных энкодера
struct Struct_Encoder
{
	float way = 0;		// Пройденный путь колесом с учетом направления вращения
	float rpsSet = 0;	// Текущая скорость вращения ( обороты в секунду)
	float rpsEncod = 0; // Текущая скорость вращения ( обороты в секунду)
};
// Структура содержит данные по статусу Driver
struct Struct_StatusDriver
{
	uint32_t countCommand = 0;	  // Сколько всего пришло команд с момента запуска
	uint32_t countBedCommand = 0; // Сколько из них плохих команд
	uint32_t timeStart = 0;		  // Сколько миллисекунд с моента запуска драйвера
};

// Структура сосдержит всю информацию по мотору на основании данных энкодера
struct Struct_Car
{
	float speedSet = 0;	   // Скорость которую задали в функции (метры в секунду)
	float speedEncod = 0;  // Текущая скорость движения (метры в секунду)
	float radiusSet = 0;   // Радиус который задали в функции в метрах
	float radiusEncod = 0; // Текущий радиус движения в метрах
	float way = 0;		   // Пройденный путь в метрах
};
// Структура для углов наклонов
struct Struct_IMU
{
	float roll = 0;	  // Крен в право  влево
	float pitch = 0;  // Тангаж вверх или вних
	float yaw = 0;	  // Поворот по часовой мом против часовой
	float x = 0;	  // Координата по Х
	float y = 0;	  // Координата по Y
	float th = 0;	  // Направление носа
	float vel_x = 0;  // Линейная скорость движения робота по оси X
	float vel_y = 0;  // Линейная скорость движения робота по оси Y
	float vel_th = 0; // Угловая скорость вращения робота

	Struct_IMU &operator=(const Struct_IMU &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
	{
		roll = source.roll;
		pitch = source.pitch;
		yaw = source.yaw;
		x = source.x;
		y = source.y;
		th = source.th;
		vel_x = source.vel_x;
		vel_y = source.vel_y;
		vel_th = source.vel_th;
		return *this;
	}
};

// Структура состояния сервомотора
struct Struct_ServoOut
{
	int32_t position; // Позиция текущая сервомотра
};

// Структура состояния датчика расстония
struct Struct_Sensor
{
	float distance; // расстояние до препятствия
};

// Структура одометрии
struct Struct_Odom
{
	float x = 0;	  // Координата по Х
	float y = 0;	  // Координата по Y
	float th = 0;	  // Направление носа
	float vel_x = 0;  // Линейная скорость движения робота по оси X
	float vel_y = 0;  // Линейная скорость движения робота по оси Y
	float vel_th = 0; // Угловая скорость вращения робота

	Struct_Odom &operator=(const Struct_Odom &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
	{
		x = source.x;
		y = source.y;
		th = source.th;
		vel_x = source.vel_x;
		vel_y = source.vel_y;
		vel_th = source.vel_th;
		return *this;
	}
};

// Структура принимаемых данных от контроллера Driver в Data
struct Struct_Driver2Data
{
	uint32_t id = 0; // id команды
	Struct_StatusDriver status;
	Struct_Car car;
	Struct_Encoder motorLeft;
	Struct_Encoder motorRight;
	Struct_Odom odom_enc; // Одометрия по энкодерам
	Struct_IMU bno055;	  // Данные с датчика BNO055
	Struct_ServoOut servo1;
	Struct_ServoOut servo2;
	Struct_Sensor lazer1;
	Struct_Sensor lazer2;
	Struct_Sensor uzi1;
	Struct_Sensor uzi2;
	uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Driver2Data Driver2Data; // Тело робота. тут все переменные его характеризующие на низком уровне
const uint32_t size_stucturs = sizeof(Struct_Driver2Data);
// ************************************************************* Struct_Data2Driver *********************
// Структура управления движением
struct Struct_Control
{
	uint32_t startStop = 0; // Стоим или двигаемся
	float radius = 0;		// Радиус по которому нужно двигаться
	float speed = 0;		// Скорость с которой нужно двигаться`
	uint32_t command1 = 0;	// Дополнительная команда
	uint32_t command2 = 0;	// Дополнительная команда
};
// Структура управления сервомотором
struct Struct_Servo
{
	int32_t time = 0;	  // Время за которое мотор должен прити в задаваемую позицию
	int32_t position = 0; // Позиция левого сервомотра
};
// Структура управления Светодиодами
struct Struct_Led
{
	int32_t num_program = 0; // Номер программы для светодиодов мигания
};
// Структура отправляемых данных от Data к контроллеру Driver
struct Struct_Data2Driver
{
	uint32_t id = 0;		// Номер команды по порядку
	Struct_Control control; // Структура управления машиной
	Struct_Servo servo1;	// Управление сервомотором
	Struct_Servo servo2;	// Управление сервомотором
	Struct_Led led;			// Управление светодиодами
	uint32_t cheksum = 0;	// Контрольная сумма данных в структуре
};

Struct_Data2Driver Data2Driver; // Экземпляр структуры отправлемых данных
// ************************************************************* Struct_Data2Driver *********************

//============================================================================================================================================================

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