#ifndef DATA2DRIVER_H
#define DATA2DRIVER_H

// Структура сосдержит всю информацию по мотору на основании данных энкодера
struct Struct_Encoder
{
	float way = 0; // Пройденный путь колесом с учетом направления вращения
	float rps = 0; // Текущая скорость вращения ( обороты в секунду)
};
// Структура сосдержит всю информацию по мотору на основании данных энкодера
struct Struct_Car
{
	float speed = 0;  // Текущая скорость движения (метры в секунду)
	float radius = 0; // Текущий радиус движения в метрах
	float way = 0;	  // Пройденный путь в метрах
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

// Копирование полученных данных в структуру для публикации в топике
void processing_Driver2Data()
{
	// Копируем полученные по SPI данные в сообщение которое потом опубликуем
	msg_Driver2Data.id = Driver2Data.id;

	msg_Driver2Data.car.radius = Driver2Data.car.radius;
	msg_Driver2Data.car.speed = Driver2Data.car.speed;
	msg_Driver2Data.car.way = Driver2Data.car.way;
	
	msg_Driver2Data.motorLeft.rps = Driver2Data.motorLeft.rps;
	msg_Driver2Data.motorLeft.way = Driver2Data.motorLeft.way;
	
	msg_Driver2Data.motorRight.rps = Driver2Data.motorRight.rps;
	msg_Driver2Data.motorRight.way = Driver2Data.motorRight.way;

	msg_Driver2Data.odom_enc.x = Driver2Data.odom_enc.x;
	msg_Driver2Data.odom_enc.y = Driver2Data.odom_enc.y;
	msg_Driver2Data.odom_enc.th = Driver2Data.odom_enc.th;
	msg_Driver2Data.odom_enc.vel_x = Driver2Data.odom_enc.vel_x;
	msg_Driver2Data.odom_enc.vel_y = Driver2Data.odom_enc.vel_y;
	msg_Driver2Data.odom_enc.vel_th = Driver2Data.odom_enc.vel_th;
	
	msg_Driver2Data.bno055.x = Driver2Data.bno055.x;
	msg_Driver2Data.bno055.y = Driver2Data.bno055.y;
	msg_Driver2Data.bno055.th = Driver2Data.bno055.th;
	msg_Driver2Data.bno055.vel_x = Driver2Data.bno055.vel_x;
	msg_Driver2Data.bno055.vel_y = Driver2Data.bno055.vel_y;
	msg_Driver2Data.bno055.vel_th = Driver2Data.bno055.vel_th;
	msg_Driver2Data.bno055.roll = Driver2Data.bno055.roll;
	msg_Driver2Data.bno055.pitch = Driver2Data.bno055.pitch;
	msg_Driver2Data.bno055.yaw = Driver2Data.bno055.yaw;
	
	msg_Driver2Data.servo1.position = Driver2Data.servo1.position;
	msg_Driver2Data.servo2.position = Driver2Data.servo2.position;

	msg_Driver2Data.lazer1.distance = Driver2Data.lazer1.distance;
	msg_Driver2Data.lazer2.distance = Driver2Data.lazer2.distance;

	msg_Driver2Data.uzi1.distance = Driver2Data.uzi1.distance;
	msg_Driver2Data.uzi2.distance = Driver2Data.uzi2.distance;
}

// Копирование данных из сообщения в топике в структуру для передачи по SPI
void collect_Data2Driver() // Данные для передачи с Data на Driver
{
	Data2Driver.id++; //= 0x1F1F1F1F;

	Data2Driver.control.startStop = msg_Head2Data.control.startStop;
	Data2Driver.control.radius = msg_Head2Data.control.radius;
	Data2Driver.control.speed = msg_Head2Data.control.speed;
	
	Data2Driver.servo1.time = msg_Head2Data.servo1.time;
	Data2Driver.servo1.position = msg_Head2Data.servo1.position;

	Data2Driver.servo2.time = msg_Head2Data.servo2.time;
	Data2Driver.servo2.position = msg_Head2Data.servo2.position;
	
	Data2Driver.led.num_program = msg_Head2Data.led.num_program;
	//Data2Driver.led.num_program = Data2Driver.led.num_program + 3;

	Data2Driver.cheksum = measureCheksum(Data2Driver); // Считаем контрольную сумму отправляемой структуры
													   // printf("Отправляем: Id %i, чек= %i  ", Data2Driver.id, Data2Driver.cheksum);
}

// Выводим на экран данные команды которую отправляем
void printData_To_Body()
{
	printf(" SEND id = %i", Data2Driver.id);
	// printf(" command = %i", stru_body_send.command);
	// printf(" napravlenie = %i", msg_head_receive.napravlenie);
	// printf(" radius = %f", stru_body_send.radius);
	// printf(" speed = %f", stru_body_send.speed);
	// printf(" angle_cam = %f", stru_body_send.cam_angle);
	// printf(" ventil_speed = %i", stru_body_send.ventil_speed);
	// printf("  /  ");
}
// Выводим на экран данные которые получили
void printData_From_Driver()
{
	printf(" RECEIVE id = %i", Driver2Data.id);
	printf(" Driver2Data.car.radius = %f", Driver2Data.car.radius);
	// printf(" gaz.x = %f", Driver2Data.gaz_data);
	// printf(" gaz2.x = %f", stru_body_receive.gaz2_data);
	//  printf(" bno055.x = %f", stru_body_receive.bno055.x);
	//  printf(" bno055.y = %f", stru_body_receive.bno055.y);
	//  printf(" bno055.z = %f", stru_body_receive.bno055.z);
	// printf(" temperature = %f", stru_body_receive.bmp280.temperature);
	// printf(" pressure = %f", stru_body_receive.bmp280.pressure);
	// printf(" humidity = %f", stru_body_receive.bmp280.humidity);
	//  printf(" distance_lazer = %.2f", stru_body_receive.distance_lazer);
	//	printf(" a1 = %.2f", a1);
	//  printf(" a2 = %.2f", a2);
	//  printf(" distance_uzi = %.2f", stru_body_receive.distance_uzi);
	//	printf(" u1 = %.2f", u1);
	//  printf(" u2 = %.2f", u2);
	//  printf(" X_comp = %.2f", msg_body_send.mpu9250.x);
	//  printf(" Y_comp = %.2f", msg_body_send.mpu9250.y);
	//  printf(" Z_comp = %.2f", msg_body_send.mpu9250.z);
	printf("  / data_driver_all= %i data_driver_bed= %i /", data_driver_all, data_driver_bed);
	printf("\n");
}

// Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Driver(int channel_, Struct_Driver2Data &structura_receive_, Struct_Data2Driver &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(buffer, 0, sizeof(buffer));																							   // Очищаем буфер передачи

	// Заполняем буфер данными структуры для передачи
	Struct_Data2Driver *buffer_send = (Struct_Data2Driver *)buffer; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;									// Переписываем по этому адресу данные в буфер

	rez = wiringPiSPIDataRW(channel_, buffer, sizeof(buffer)); // Передаем и одновременно получаем данные

	// Извлекаем из буфера данные в формате структуры и копирум данные
	Struct_Driver2Data *copy_buf_master_receive = (Struct_Driver2Data *)buffer; // Создаем переменную в которую пишем адрес буфера в нужном формате
	structura_receive_ = *copy_buf_master_receive;								// Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_);				// Считаем контрольную сумму пришедшей структуры

	if (cheksum_receive != structura_receive_.cheksum || structura_receive_.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		return false;
	}
	else // Все хорошо возвращаем Ок
	{
		return true;
	}
}

#endif