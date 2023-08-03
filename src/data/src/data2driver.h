#ifndef DATA2DRIVER_H
#define DATA2DRIVER_H

// Структура принимаемых данных от контроллера Driver в Data
struct Struct_Driver2Data
{
  uint32_t id = 0;      // id команды
  Struct_Odom odom_enc; // Одометрия по энкодерам
  Struct_Odom odom_imu; // Одометрия по гироскопу и аксельрометру
  float odom_L = 0;     // Пройденный путь левым колесом
  float odom_R = 0;     // Пройденный путь правым колесом
  float speed_L = 0;    // Скорость левого колеса
  float speed_R = 0;    // Скорость правого колеса
  Struct_RPY bno055;    // Данные с датчика BNO055
  Struct_INA ina;       // Данные с датчика INA219

  uint32_t connect_flag; // Флаг связи с пультом ручного управления
  uint32_t startStop;    // Стоим или двигаемся
  float radius = 0;      // Радиус по которому нужно двигаться
  float speed = 0;       // Скорость с которой нужно двигаться
  float angle_camera;    // Угол камеры для шагового мотора камеры
  uint32_t led;          // Номер программы для светодиодов мигания
  uint32_t servo;        // Номер программы для сервомоторов движения рук
  int32_t posServoL;        // Позиция левого сервомотра
  int32_t posServoR;        // ПИзиция правого сервомотра

  uint32_t status_wifi; // Статус запущен ли вайфай или выключен

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

Struct_Driver2Data Driver2Data;           //Тело робота. тут все переменные его характеризующие на низком уровне
const uint32_t size_stucturs = sizeof(Struct_Driver2Data);

// Структура отправляемых данных от Data к контроллеру Driver
struct Struct_Data2Driver
{
  uint32_t id = 0;       // Номер команды по порядку
  uint32_t startStop;    // Стоим или двигаемся
  float radius = 0;      // Радиус по которому нужно двигаться
  float speed = 0;       // Скорость с которой нужно двигаться`
  float angle_camera;    // Угол камеры для шагового мотора камеры
  uint32_t led;          // Номер программы для светодиодов мигания
  uint32_t servo;        // Номер программы для сервомоторов движения рук
  int32_t posServoL;        // Позиция левого сервомотра
  int32_t posServoR;        // ПИзиция правого сервомотра
  int32_t timeServoL;       // Время за которое левый мотор должен прити в задаваемую позицию
  int32_t timeServoR;       // Время за которое правый мотор должен прити в задаваемую позицию
  uint32_t cheksum = 0;  // Контрольная сумма данных в структуре
};

//========================================================

Struct_Data2Driver Data2Driver; // Экземпляр структуры отправлемых данных


//Обработка полученных данных и копирование их куда надо
void dataProcessing_Body()
{
	//Копируем полученные по SPI данные в сообщение которое потом опубликуем
	msg_driver_odometr_send.id = Driver2Data.id;

	msg_driver_odometr_send.odom_enc.x = Driver2Data.odom_enc.x;
	msg_driver_odometr_send.odom_enc.y = Driver2Data.odom_enc.y;
	msg_driver_odometr_send.odom_enc.th = Driver2Data.odom_enc.th;
	msg_driver_odometr_send.odom_enc.vel_x = Driver2Data.odom_enc.vel_x;
	msg_driver_odometr_send.odom_enc.vel_y = Driver2Data.odom_enc.vel_y;
	msg_driver_odometr_send.odom_enc.vel_th = Driver2Data.odom_enc.vel_th;

	msg_driver_odometr_send.odom_imu.x = Driver2Data.odom_imu.x;
	msg_driver_odometr_send.odom_imu.y = Driver2Data.odom_imu.y;
	msg_driver_odometr_send.odom_imu.th = Driver2Data.odom_imu.th;
	msg_driver_odometr_send.odom_imu.vel_x = Driver2Data.odom_imu.vel_x;
	msg_driver_odometr_send.odom_imu.vel_y = Driver2Data.odom_imu.vel_y;
	msg_driver_odometr_send.odom_imu.vel_th = Driver2Data.odom_imu.vel_th;

	//Данные по одометрии принимаем как есть
	msg_driver_odometr_send.odom_L = Driver2Data.odom_L;
	msg_driver_odometr_send.odom_R = Driver2Data.odom_R;

	msg_driver_odometr_send.speed_L = Driver2Data.speed_L;
	msg_driver_odometr_send.speed_R = Driver2Data.speed_R;

	msg_driver_info_send.id = Driver2Data.id;
	//Данные по датчику BNO055 принимаем как есть
	msg_driver_info_send.bno055.roll = Driver2Data.bno055.roll;
	msg_driver_info_send.bno055.pitch = Driver2Data.bno055.pitch;
	msg_driver_info_send.bno055.yaw = Driver2Data.bno055.yaw;

	msg_driver_info_send.ina.busVoltage_V = Driver2Data.ina.busVoltage_V;
	msg_driver_info_send.ina.shuntVoltage_mV = Driver2Data.ina.shuntVoltage_mV;
	msg_driver_info_send.ina.current_mA = Driver2Data.ina.current_mA;
	msg_driver_info_send.ina.power_mW = Driver2Data.ina.power_mW;

	msg_driver_info_send.obmen_all = data_driver_all;
	msg_driver_info_send.obmen_bed = data_driver_bed;
	
	msg_driver_control_send.id = Driver2Data.id;
	msg_driver_control_send.status_wifi = Driver2Data.status_wifi;
	msg_driver_control_send.connect_flag = Driver2Data.connect_flag;
	msg_driver_control_send.startStop = Driver2Data.startStop;
	msg_driver_control_send.radius = Driver2Data.radius;
	msg_driver_control_send.speed = Driver2Data.speed;
	msg_driver_control_send.angle_camera = Driver2Data.angle_camera;
	msg_driver_control_send.led = Driver2Data.led;
	msg_driver_control_send.servo = Driver2Data.servo;
	msg_driver_control_send.posServoL = Driver2Data.posServoL;
	msg_driver_control_send.posServoR = Driver2Data.posServoR;
}

//Копирование данных из сообщения в топике в структуру для передачи по SPI
void Collect_Data2Driver() // Данные для передачи на низкий уровень
{
	Data2Driver.id++; //= 0x1F1F1F1F;
	Data2Driver.startStop = 0;
	Data2Driver.radius = 0;
	Data2Driver.speed = 0;
	Data2Driver.angle_camera = 0;
	Data2Driver.led = 0;
	Data2Driver.servo = 0;
	Data2Driver.posServoL = 0;
	Data2Driver.posServoR = 0;
	Data2Driver.timeServoL = 0;
	Data2Driver.timeServoR = 0;
	Data2Driver.cheksum = measureCheksum(Data2Driver); // Считаем контрольную сумму отправляемой структуры
	//printf("Отправляем: Id %i, чек= %i  ", Data2Driver.id, Data2Driver.cheksum);
}
 

// Выводим на экран данные команды которую отправляем
void printData_To_Body()
{
	printf(" SEND id = %i", Data2Driver.id);
	// printf(" command = %i", stru_body_send.command);
	//printf(" napravlenie = %i", msg_head_receive.napravlenie);
	//printf(" radius = %f", stru_body_send.radius);
	// printf(" speed = %f", stru_body_send.speed);
	// printf(" angle_cam = %f", stru_body_send.cam_angle);
	//printf(" ventil_speed = %i", stru_body_send.ventil_speed);
	//printf("  /  ");
}
// Выводим на экран данные которые получили
void printData_From_Body()
{
	printf(" RECEIVE id = %i", Driver2Data.id);
	//printf(" gaz.x = %f", Driver2Data.gaz_data);
	//printf(" gaz2.x = %f", stru_body_receive.gaz2_data);
	// printf(" bno055.x = %f", stru_body_receive.bno055.x);
	// printf(" bno055.y = %f", stru_body_receive.bno055.y);
	// printf(" bno055.z = %f", stru_body_receive.bno055.z);
	//printf(" temperature = %f", stru_body_receive.bmp280.temperature);
	//printf(" pressure = %f", stru_body_receive.bmp280.pressure);
	//printf(" humidity = %f", stru_body_receive.bmp280.humidity);
	// printf(" distance_lazer = %.2f", stru_body_receive.distance_lazer);
	//	printf(" a1 = %.2f", a1);
	// printf(" a2 = %.2f", a2);
	// printf(" distance_uzi = %.2f", stru_body_receive.distance_uzi);
	//	printf(" u1 = %.2f", u1);
	// printf(" u2 = %.2f", u2);
	// printf(" X_comp = %.2f", msg_body_send.mpu9250.x);
	// printf(" Y_comp = %.2f", msg_body_send.mpu9250.y);
	// printf(" Z_comp = %.2f", msg_body_send.mpu9250.z);
	printf("  / data_driver_all= %i data_driver_bed= %i /", data_driver_all, data_driver_bed);
	printf("\n");
}

//Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Driver(int channel_, Struct_Driver2Data &structura_receive_, Struct_Data2Driver &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(buffer, 0, sizeof(buffer));																							   // Очищаем буфер передачи

	//Заполняем буфер данными структуры для передачи
	Struct_Data2Driver *buffer_send = (Struct_Data2Driver *)buffer; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;								// Переписываем по этому адресу данные в буфер

	rez = wiringPiSPIDataRW(channel_, buffer, sizeof(buffer)); //Передаем и одновременно получаем данные

	//Извлекаем из буфера данные в формате структуры и копирум данные
	Struct_Driver2Data *copy_buf_master_receive = (Struct_Driver2Data *)buffer;  // Создаем переменную в которую пишем адрес буфера в нужном формате
	structura_receive_ = *copy_buf_master_receive;				   // Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_); // Считаем контрольную сумму пришедшей структуры

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