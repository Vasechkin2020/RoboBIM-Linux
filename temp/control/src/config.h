#ifndef CONFIG_H
#define CONFIG_H
//---------------------------------------------------------------------------------------
#define PIN_LED_BLUE 28
#define PIN_LED_RED 29
#define PIN_LED_GREEN 25
//---------------------------------------------------------------------------------------
#define SPI_CHANNAL 0	  //Какой из двух каналов инициализируем
#define SPI_SPEED 4000000 // Скорость работы шины SPI
//#define PIN_SPI_LINE 2 // Пин обратной связи с ведомым
//---------------------------------------------------------------------------------------
#define RATE 2
my_msgs::Control msg;

int ok = 0;
int bed = 0;

struct stru_INA219
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  stru_INA219 &operator=(const stru_INA219 &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
  {
    shuntvoltage = source.shuntvoltage;
    busvoltage = source.busvoltage;
    current_mA = source.current_mA;
    loadvoltage = source.loadvoltage;
    power_mW = source.power_mW;
    return *this;
  }
};

//Структура в которой все главные переменные которые передаюся на высокий уровень
struct Struct_Iot2Data
{
	uint32_t id = 0;		   // Id команды
	float radius = 0;		   // =0..100 положение слайдера
	uint32_t startStop = 0;	   // =0 если переключатель в положении A, =1 если в положении B, =2 если в положении C, ...
	uint32_t connect_flag = 0; // =1 if wire connected, else =0
	stru_INA219 INA219;		   // Данные с датчика INA219
	uint32_t cheksum = 0;	   // Контрольная сумма данных в структуре

	Struct_Iot2Data &operator=(const Struct_Iot2Data &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
	{
		id = source.id;
		radius = source.radius;
		startStop = source.startStop;
		connect_flag = source.connect_flag;
		INA219 = source.INA219;
		cheksum = source.cheksum;
		return *this;
	}
};

// //-----------
//Структура для температурного датчика BMP280
struct Struct_BMP280
{
	float temperature = 33.3;
	float pressure = 44.4;
	float humidity = 55.5;
	// float bufff[25];

	Struct_BMP280 &operator=(const Struct_BMP280 &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
	{
		temperature = source.temperature;
		pressure = source.pressure;
		humidity = source.humidity;
		return *this;
	}
};

struct Struct_DataHL_Control // Структура получаемых данных
{
	//uint8_t hlam = 0;        // Ошибочный байт
	uint32_t id = 0;		// Id команды
	float gaz1_data = 3.14; // Данные с первого датчика газа
	float gaz2_data = 6.99; // Данные со второго датчика газа
	Struct_BMP280 bmp280;	// Данные с датчика BMP280
	uint32_t cheksum = 0;	// Контрольная сумма данных в структуре

	Struct_DataHL_Control &operator=(const Struct_DataHL_Control &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
	{
		id = source.id;
		gaz1_data = source.gaz1_data;
		gaz2_data = source.gaz2_data;
		bmp280 = source.bmp280;
		cheksum = source.cheksum;
		return *this;
	}
};

//========================================================
Struct_Iot2Data Control;				  //Тело робота. тут все переменные его характеризующие на низком уровне
Struct_Iot2Data Copy_Control;		  //Тело робота. тут все переменные его характеризующие на низком уровне
Struct_DataHL_Control DataHL_Control; // Экземпляр структуры отправлемых данных

#endif