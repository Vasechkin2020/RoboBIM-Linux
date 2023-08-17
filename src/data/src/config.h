#ifndef CONFIG_H
#define CONFIG_H

#define RATE 2 // Частота обмена с нижним уровнев в Герц
//---------------------------------------------------------------------------------------
// Посмотреть пины командой <gpio readall> Пины имеют соответсвие между BMC и wiringpi
#define PIN_LED_BLUE 4  // Мигает в цикле что работает 
//#define PIN_LED_GREEN 22   // Ошибки при передаче или еще где

//#define PIN_PWM 26
//---------------------------------------------------------------------------------------
#define SPI_CHANNAL_0 0   //Какой из двух каналов инициализируем
#define SPI_CHANNAL_1 1   //Какой из двух каналов инициализируем
#define SPI_SPEED 4000000 // Скорость работы шины SPI
//#define PIN_SPI_LINE 2 // Пин обратной связи с ведомым
//#define PIN_SPI_LINE 3 // Пин обратной связи с ведомым
//---------------------------------------------------------------------------------------

#define SIZE_BUFF 160         // Размер буфера, стараться делать кратно 32
unsigned char buffer[SIZE_BUFF]; //Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт
//---------------------------------------------------------------------------------------
// #define DEVICE_ID_STM 0x1A         // Адрес I2C платы STM как slave устройства


bool rez_data = false;

int data_driver_all = 0;
int data_driver_bed = 0;
int data_Iot_all = 0;
int data_Iot_bed = 0;

float a1, a2;
float u1, u2;
MyKalman laser;
MyKalman uzi;
MyKalman mpu9250_x;
MyKalman mpu9250_y;



//Функция возвращает контрольную сумму структуры без последних 4 байтов
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

//#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_WARN  // Если поставить WARN и раскомментировать то не будут выводиться сооющения ROS уровня INFO
//#define LEVEL_SEVERITY YES              // Если раскомментировать то мои метки не будут выводиться

#define RED "\x1b[31;40m"
#define GREEN "\x1b[32;40m"
#define YELLOW "\x1b[33;40m"
#define BLUE "\x1b[34;40m"
#define MAGENTA "\x1b[35;40m"
#define CYAN "\x1b[36;40m"
#define NORM "\x1b[0m"
#define NN "\x1b[33;40m Data_node"

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