#ifndef CONFIG_H
#define CONFIG_H

#define RATE 2 // Частота обмена с нижним уровнев в Герц
//---------------------------------------------------------------------------------------
#define PIN_LED_BLUE 28
#define PIN_LED_RED 29
#define PIN_LED_GREEN 25
//---------------------------------------------------------------------------------------
#define SPI_CHANNAL 1	  //Какой из двух каналов инициализируем
#define SPI_SPEED 4000000 // Скорость работы шины SPI
#define PIN_SPI_LINE 3 // Пин обратной связи с ведомым
//---------------------------------------------------------------------------------------

my_msgs::Command msg_head_receive; // Полученное сообщение из топика
my_msgs::Body msg_body_send;	   // Сообщение которое публикуем в Топик

//Структура для углов Эллера
struct Struct_XYZ
{
	float x = 0;
	float y = 0;
	float z = 0;

	Struct_XYZ &operator=(const Struct_XYZ &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
	{
		x = source.x;
		y = source.y;
		z = source.z;
		return *this;
	}
};

//Структура для углов BMP280
struct Struct_BMP280
{
	float temperature = 0;
	float pressure = 0;
	float humidity = 0;

	Struct_BMP280 &operator=(const Struct_BMP280 &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
	{
		temperature = source.temperature;
		pressure = source.pressure;
		humidity = source.humidity;
		return *this;
	}
};

//Структура в которой все главные переменные которые передаюся на высокий уровень
struct Struct_Body_to_Head
{
	uint32_t id = 0;			 // id команды
	// float odom_l = 0;			 // Пройденный телом путь
	// float odom_r = 0;			 // Пройденный телом путь
	// float distance_uzi = 0;		 // Данные ультразвукового датчика
	// float distance_lazer = 0;	 // Данные лазерного датчика
	float gaz1_data = 0;		 // Данные с первого датчика газа
	float gaz2_data = 0;		 // Данные со второго датчика газа
	// float motor_video_angle = 0; // Теоретическое положение шагового мотора который поворачивает камеру
	// Struct_XYZ mpu9250;			 // Данные с датчика MPU9250
	// Struct_XYZ bno055;			 // Данные с датчика BNO055
	// float magnetrometr;			 // Данные с датчика магнетрометра AK8963
	// Struct_BMP280 bmp280;		 // Данные с датчика BMP280

	Struct_Body_to_Head &operator=(const Struct_Body_to_Head &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
	{
		// odom_l = source.odom_l;
		// odom_r = source.odom_r;
		// distance_uzi = source.distance_uzi;
		// distance_lazer = source.distance_lazer;
		gaz1_data = source.gaz1_data;
		gaz2_data = source.gaz2_data;
		// motor_video_angle = source.motor_video_angle;
		// bno055 = source.bno055;
		// mpu9250 = source.mpu9250;
		// bmp280 = source.bmp280;
		// magnetrometr = source.magnetrometr;
		return *this;
	}
};

// Структура отправляемых данных
struct Struct_Head_to_Body
{
	uint32_t id = 0;	 // Номер команды по порядку
	int32_t command = 0; // Команда для выполнения
	//int32_t napravlenie = 0; // Направление движения Прямо, По часовой, Против часовой
	float speed = 0;  // Скорость которую нужно установить
	float radius = 0; // Радиус по которому нужно двигаться
	//uint32_t time = 0;		 // Время в течении которого нужно двигаться
	//float way = 0;			 // Путь который нужно проехать
	float cam_angle;	   // Угол для шагового мотора камеры
	uint32_t ventil_speed; // Скорость вращения вентилятора от 0 до 255

	Struct_Head_to_Body &operator=(const Struct_Head_to_Body &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
	{
		id = source.id;
		command = source.command;
		//napravlenie = source.napravlenie;
		speed = source.speed;
		radius = source.radius;
		//time = source.time;
		//way = source.way;
		cam_angle = source.cam_angle;
		ventil_speed = source.ventil_speed;
		return *this;
	}
};

//========================================================
Struct_Body_to_Head stru_body_receive; //Тело робота. тут все переменные его характеризующие на низком уровне
Struct_Head_to_Body stru_body_send;	   // Экземпляр структуры получаемых данных

float a1,a2;
float u1,u2;
MyKalman laser;
MyKalman uzi;
MyKalman mpu9250_x;
MyKalman mpu9250_y;

#endif