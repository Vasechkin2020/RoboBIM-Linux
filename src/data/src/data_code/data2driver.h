#ifndef DATA2DRIVER_H
#define DATA2DRIVER_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
bool sendData2Driver(int channel_, Struct_Driver2Data &structura_receive_, Struct_Data2Driver &structura_send_); // Указываем на каком пине устройство и с какого регистра нужно прочитать данные // Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
void processing_Driver2Data();																					 // Копирование полученных данных в структуру для публикации в топике
void collect_Data2Driver();																						 // Данные для передачи с Data на Driver // Копирование данных из сообщения в топике в структуру для передачи по SPI
void printData2Driver();																						 // Выводим на экран данные команды которую отправляем
void printData_From_Driver();																					 // Выводим на экран данные которые получили

//***********************************************************************************
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
	Struct_Driver2Data structura_receive_temp;									// Временная структура, проверить правильные ли пришли данные
	structura_receive_temp = *copy_buf_master_receive;							// Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_temp);			// Считаем контрольную сумму пришедшей структуры

	if (cheksum_receive != structura_receive_temp.cheksum || structura_receive_temp.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		data_driver_bed++;
		return false;
	}
	else // Все хорошо возвращаем Ок
	{
		structura_receive_ = structura_receive_temp; // Копируем хорошие данные уже в итоговую структуру, если плохие то они просто пропадают и не портят прошлые
		return true;
	}
}

// Копирование полученных данных в структуру для публикации в топике
void processing_Driver2Data()
{
	// Копируем полученные по SPI данные в сообщение которое потом опубликуем
	Driver2Data_msg.id = Driver2Data.id;

	// Driver2Data_msg.status.timeStart = Driver2Data.status.timeStart;
	// Driver2Data_msg.status.countCommand = Driver2Data.status.countCommand;
	// Driver2Data_msg.status.countBedCommand = Driver2Data.status.countBedCommand;

	// Driver2Data_msg.car.speedSet = Driver2Data.car.speedSet;
	Driver2Data_msg.car.speedEncod = Driver2Data.car.speedEncod;
	Driver2Data_msg.car.way = Driver2Data.car.way;

	Driver2Data_msg.motorLeft.rpsSet = Driver2Data.motorLeft.rpsSet;
	Driver2Data_msg.motorLeft.rpsEncod = Driver2Data.motorLeft.rpsEncod;

	Driver2Data_msg.motorRight.rpsSet = Driver2Data.motorRight.rpsSet;
	Driver2Data_msg.motorRight.rpsEncod = Driver2Data.motorRight.rpsEncod;

	Driver2Data_msg.bno055.x = Driver2Data.bno055.x;
	Driver2Data_msg.bno055.y = Driver2Data.bno055.y;
	Driver2Data_msg.bno055.th = Driver2Data.bno055.th;
	Driver2Data_msg.bno055.vel_x = Driver2Data.bno055.vel_x;
	Driver2Data_msg.bno055.vel_y = Driver2Data.bno055.vel_y;
	Driver2Data_msg.bno055.vel_th = Driver2Data.bno055.vel_th;
	Driver2Data_msg.bno055.roll = Driver2Data.bno055.roll;
	Driver2Data_msg.bno055.pitch = Driver2Data.bno055.pitch;
	Driver2Data_msg.bno055.yaw = Driver2Data.bno055.yaw;

	Driver2Data_msg.lazer1.distance = Driver2Data.lazer1.distance;
	Driver2Data_msg.lazer2.distance = Driver2Data.lazer2.distance;

	Driver2Data_msg.uzi1.distance = Driver2Data.uzi1.distance;
}

// Копирование данных из сообщения в топике в структуру для передачи по SPI
void collect_Data2Driver() // Данные для передачи с Data на Driver
{
	Data2Driver.id++; //= 0x1F1F1F1F;
	Data2Driver.control.speedL = msg_ControlDriver.control.speedL;
	Data2Driver.control.speedR = msg_ControlDriver.control.speedR;
	Data2Driver.led.num_program = msg_ControlDriver.led.num_program;
	Data2Driver.cheksum = measureCheksum(Data2Driver); // Считаем контрольную сумму отправляемой структу

	// printf("Отправляем: Id %i, чек= %i  ", Data2Driver.id, Data2Driver.cheksum);
}

// Выводим на экран данные команды которую отправляем
void printData2Driver()
{
	printf(" SEND id = %i", Data2Driver.id);
	// printf(" startStop = %i", Data2Driver.control.startStop);
	// printf(" speed = %f", Data2Driver.control.speed);
	// printf(" radius = %f", Data2Driver.control.radius);
	printf(" led.num_program = %i", Data2Driver.led.num_program);
	printf("  \n");
}
// Выводим на экран данные которые получили
void printData_From_Driver()
{
	printf(" RECEIVE id = %i", Driver2Data.id);
	// printf(" Driver2Data.car.radius = %f", Driver2Data.car.radiusSet);
	//  printf(" bno055.x = %f", stru_body_receive.bno055.x);
	//  printf(" bno055.y = %f", stru_body_receive.bno055.y);
	//  printf(" bno055.z = %f", stru_body_receive.bno055.z);
	//  printf(" distance_lazer = %.2f", stru_body_receive.distance_lazer);
	//  printf(" distance_uzi = %.2f", stru_body_receive.distance_uzi);
	printf("  / data_driver_all= %i data_driver_bed= %i \n", data_driver_all, data_driver_bed);
}

#endif