#ifndef DATA2IOT_H
#define DATA2IOT_H

struct DateTime {
    uint8_t second; 
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint16_t year;
};

// Структура передаваемых данных из Data к Iot
struct Struct_Data2Iot 
{
  uint32_t id = 0;            // Id команды
  float odom_L = 0;     // Пройденный путь левым колесом
  float odom_R = 0;     // Пройденный путь правым колесом
  float speed_L = 0;    // Скорость левого колеса
  float speed_R = 0;    // Скорость правого колеса
  Struct_RPY bno055;    // Данные с датчика BNO055
  Struct_INA ina_driver;       // Данные с датчика INA219

  uint32_t connect_flag; // Флаг связи с пультом ручного управления
  uint32_t startStop;    // Стоим или двигаемся
  float radius = 0;      // Радиус по которому нужно двигаться
  float speed = 0;       // Скорость с которой нужно двигаться
  float angle_camera;    // Угол камеры для шагового мотора камеры
  uint32_t led;          // Номер программы для светодиодов мигания
  uint32_t servo;        // Номер программы для сервомоторов движения рук
  int32_t posServoL;        // Позиция левого сервомотра
  int32_t posServoR;        // ПИзиция правого сервомотра

  uint32_t status_wifi_driver; // Статус запущен ли вайфай или выключен

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре

//   float distance_lazer_L = 0; // Данные лазерного датчика
//   float distance_lazer_R = 0; // Данные лазерного датчика
//   float distance_uzi = 0;     // Данные ультразвукового датчика
//   Struct_BME bme;             // Данные с датчика BME
//   float gaz_data = 0;         // Данные с первого датчика газа
//   Struct_INA ina;             // Данные с датчика INA219
//   int32_t command_body = 0;    // Команда для выполнения
//   float radius = 0;            // Радиус по которому нужно двигаться
//   float speed = 0;             // Скорость которую нужно установить
//   float motor_video_angle = 0; // Теоретическое положение шагового мотора который поворачивает камеру
//   uint32_t program_led = 0;    // Номер программы которая выполняется на светодиодах

};

//Структура в которой все собранные данные передаются из Iot к Data
struct Struct_Iot2Data
{
  uint32_t id = 0;            // id команды
  float distance_lazer_L = 0; // Данные лазерного датчика
  float distance_lazer_R = 0; // Данные лазерного датчика
  float distance_uzi = 0;     // Данные ультразвукового датчика
  Struct_BME bme;             // Данные с датчика BME
  Struct_INA ina_iot;         // Данные с датчика INA219 Iot
  float gaz_data = 0;         // Данные с первого датчика газа
  float lux = 0;              // Данные с дачика освещенности
  DateTime timeIot;       // Дата время с датчика]
  uint32_t status_wifi_Iot = 0;   // Статус сети вайфай
  uint32_t status_movement = 0;  // Статус Движения
  char ssid_wifi[16]{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  char password_wifi[16]{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  uint32_t cheksum = 0;       // Контрольная сумма данных в структуре

//   int32_t radius = 0;           // =0..100 положение слайдера
//   uint32_t startStop = 0;     // =0 если переключатель в положении A, =1 если в положении B, =2 если в положении C, ...
//   uint32_t connect_flag = 0;  // =1 if wire connected, else =0
//   uint32_t led_program = 0;   // Номер программы которая выполняется на светодиодах
//   uint32_t camera = 0;        // положение видеокамеры 
//   uint32_t crwl0516 = 0;      // Данные с датчика движения

};


Struct_Iot2Data Iot2Data;           //
Struct_Data2Iot Data2Iot; // Экземпляр структуры отправлемых данных



//Обработка полученных данных и копирование их для публикации в топике
void dataProcessing_Control()
{
	//----------------------  msg_iot_info_send ----------------------
	msg_iot_info_send.id = Iot2Data.id;
	
	msg_iot_info_send.bme.temperature = Iot2Data.bme.temperature;
	msg_iot_info_send.bme.pressure = Iot2Data.bme.pressure;
	msg_iot_info_send.bme.humidity = Iot2Data.bme.humidity;
	msg_iot_info_send.bme.loc = Iot2Data.bme.loc;

	msg_iot_info_send.ina_iot.busVoltage_V = Iot2Data.ina_iot.busVoltage_V;
	msg_iot_info_send.ina_iot.current_mA = Iot2Data.ina_iot.current_mA;
	msg_iot_info_send.ina_iot.shuntVoltage_mV = Iot2Data.ina_iot.shuntVoltage_mV;
	msg_iot_info_send.ina_iot.power_mW = Iot2Data.ina_iot.power_mW;

	msg_iot_info_send.timeIot.year = Iot2Data.timeIot.year;
	msg_iot_info_send.timeIot.month = Iot2Data.timeIot.month;
	msg_iot_info_send.timeIot.date = Iot2Data.timeIot.date;
	msg_iot_info_send.timeIot.day = Iot2Data.timeIot.day;
	msg_iot_info_send.timeIot.hour = Iot2Data.timeIot.hour;
	msg_iot_info_send.timeIot.minute = Iot2Data.timeIot.minute;
	msg_iot_info_send.timeIot.second = Iot2Data.timeIot.second;

	msg_iot_info_send.lux = Iot2Data.lux;
	msg_iot_info_send.gaz_data = Iot2Data.gaz_data;
	msg_iot_info_send.status_wifi_Iot = Iot2Data.status_wifi_Iot;
	msg_iot_info_send.status_movement = Iot2Data.status_movement;
	
	msg_iot_info_send.ssid_wifi = Iot2Data.ssid_wifi;
	msg_iot_info_send.password_wifi = Iot2Data.password_wifi;
	
	msg_iot_info_send.obmen_all = data_Iot_all;
	msg_iot_info_send.obmen_bed = data_Iot_bed;

	//----------------------  msg_iot_distance_send ----------------------
	msg_iot_distance_send.id = Iot2Data.id;
	msg_iot_distance_send.distance_lazer_L = Iot2Data.distance_lazer_L;
	msg_iot_distance_send.distance_lazer_R = Iot2Data.distance_lazer_R;
	msg_iot_distance_send.distance_uzi = Iot2Data.distance_uzi;

	// msg_control_send.radius = Control.radius;
	// msg_control_send.startStop = Control.startStop;
	// msg_control_send.connect_flag = Control.connect_flag;

	// msg_control_send.led_program = Control.led_program;
	// msg_control_send.camera = Control.camera;
	// msg_control_send.crwl0516 = Control.crwl0516;


}


//Копирование рабочих данных в структуру для передачи
void Collect_Data2Iot() // Данные для передачи на низкий уровень
{
	// uint8_t *adr_DataHL_Control = (uint8_t *)(&DataHL_Control);		// Запоминаем адрес начала структуры.
	// memset(adr_DataHL_Control, 0, sizeof(DataHL_Control));             // очистка блока памяти
	Data2Iot.id++; //= 0x1F1F1F1F;
	// Data2Iot.odom_L = Driver2Data.odom_L;
	// Data2Iot.odom_R = Driver2Data.odom_R;
	// Data2Iot.speed_L = Driver2Data.speed_L;
	// Data2Iot.speed_R = Driver2Data.speed_R;
	// Data2Iot.distance_lazer_L = Driver2Data.distance_lazer_L;
	// Data2Iot.distance_lazer_R = Driver2Data.distance_lazer_R;
	// Data2Iot.distance_uzi = Driver2Data.distance_uzi;
	// Data2Iot.bno055.roll = Driver2Data.bno055.roll;
	// Data2Iot.bno055.pitch = Driver2Data.bno055.pitch;
	// Data2Iot.bno055.yaw = Driver2Data.bno055.yaw;
	// Data2Iot.bme.temperature = Driver2Data.bme.temperature;
	// Data2Iot.bme.pressure = Driver2Data.bme.pressure;
	// Data2Iot.bme.humidity = Driver2Data.bme.humidity;
	// Data2Iot.bme.loc = Driver2Data.bme.loc;
	// Data2Iot.gaz_data = Driver2Data.gaz_data;
	// То что передаем из топика головы
	// Data2Iot.command_body = msg_head_receive.command_body;
	Data2Iot.radius = 3.1415;
	// Data2Iot.radius = msg_head_receive.radius;
	// Data2Iot.speed = msg_head_receive.speed;
	// Data2Iot.motor_video_angle = msg_head_receive.motor_video_angle;
	// Data2Iot.program_led = msg_head_receive.program_led;

	//тут нужно посчитать контрольную смму структуры
	Data2Iot.cheksum = measureCheksum(Data2Iot); // Считаем контрольную сумму отправляемой структуры
														 //printf("Отправляем: Id %i, чек= %i  ", Data2Iot.id, Data2Iot.cheksum);
}

//Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Iot(int channel_, Struct_Iot2Data &structura_receive_, Struct_Data2Iot &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	//uint16_t max_size_stuct = getMax_size_Struct(size_structura_receive, size_structura_send); // Какая из структур больше
	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(buffer, 0, sizeof(buffer));																							   // Очищаем буфер передачи

	//Заполняем буфер данными структуры для передачи
	Struct_Data2Iot *buffer_send = (Struct_Data2Iot *)buffer; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;									  // Переписываем по этому адресу данные в буфер

	// for (int i = 0; i < 16; i++)
	// {
	// 	printf("%x ",adr_structura_send[i]);
	// }
	// 	printf("\n");

	// for (int i = 0; i < 16; i++)
	// {
	// 	printf("%x ",buffer[i]);
	// }
	// 	printf("\n");

	//data_Iot_all++;
	// int aa = micros();
	rez = wiringPiSPIDataRW(channel_, buffer, sizeof(buffer)); //Передаем и одновременно получаем данные
	// int time_transfer = micros() - aa;
	// float time_transfer_sec = time_transfer / 1000000.0;

	// Serial.print(String(micros()) + " Delta time= " + (ed - st) + " Size= " + Size_structura_send + " ");
	// float time_sec = (ed - st) / 1000000.0; // Перводим в секунды
	// Serial.println(" Speed= " + String((Size_structura_send / 1024.0) / time_sec) + "Kbyte/sec");

	// printf("---\n");
	// for (int i = 0; i < 40; i++)
	// {
	// 	printf("%x ",buffer[i]);
	// }
	// 	printf("\n");

	//Извлекаем из буфера данные в формате структуры и копирум данные
	Struct_Iot2Data *copy_buf_master_receive = (Struct_Iot2Data *)buffer; // Создаем переменную в которую пишем адрес буфера в нужном формате
	structura_receive_ = *copy_buf_master_receive;						// Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_);		// Считаем контрольную сумму пришедшей структуры

	//Struct_Iot2Data copy_Control_receive = *copy_buf_master_receive;		// Копируем из этой перемнной данные в мою структуру
	//uint8_t *adr_copy_Control_receive = (uint8_t *)(&copy_Control_receive); // Запоминаем адрес начала структуры. Используем для побайтной передачи
	// for (int i = 0; i < 20; i++)
	// {
	// 	printf("%x ",adr_copy_Control_receive[i]);
	// }
	// 	printf("\n");

	uint32_t cheksum_buf = measureCheksum(buffer);				   // Считаем контрольную сумму пришедшей структуры

	//printf(" Получили: Id %i, cheksum %i", structura_receive_.id, structura_receive_.cheksum);
	// printf(" capacity_real %f, capacity_percent %f", structura_receive_.capacity_real, structura_receive_.capacity_percent);
	// printf(" inVoltage %f, current_mA %f", structura_receive_.INA219.inVoltage, structura_receive_.INA219.current_mA);
	// printf(" radius %f, startStop %i", structura_receive_.radius, structura_receive_.startStop);
	//printf(" чексумма %i, myChek= %i, buf_chek= %i", structura_receive_.cheksum, cheksum_receive, cheksum_buf);

	//printf(" ok= %i, bed= %i, time= %i, speed Kbyte/sec= %f \n", ok, bed, time_transfer, (sizeof(buffer)/1024.0) / time_transfer_sec);
	//printf(" /  ok= %i, bed= %i  / ", data_Iot_all, data_Iot_bed);

	//Serial.println(String(micros()) + " copy_DataHL_Control_receive id= " + copy_DataHL_Control_receive.id);
	//Serial.println(String(micros()) + " copy_DataHL_Control_receive bmp280.pressure= " + copy_DataHL_Control_receive.bmp280.pressure);
	// Serial.println(String(micros()) + " copy_DataHL_Control_receive.cheksum_receive= " + copy_DataHL_Control_receive.cheksum);
	//printf(" measureCheksum= %i \n", cheksum_receive);
	if (cheksum_receive != structura_receive_.cheksum || structura_receive_.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		data_Iot_bed++;
		return false;
	}
	else // Все хорошо возвращаем Ок
	{
		return true;
	}
}



// Выводим на экран данные которые отправляем в Control
void printData_To_Control()
{
	// printf(" Data2Iot id = %i", Data2Iot.id);
	// printf(" distance_uzi = %f", Data2Iot.distance_uzi);
}

// Выводим на экран данные которые получили
void printDataFrom_Control()
{
	// printf(" Получили id = %i", Iot2Data.id);
	// printf(" radius = %.2f", Iot2Data.radius);
	// printf(" cheksum = %i", Iot2Data.cheksum);
	// printf("\n");
}






#endif