#ifndef DATA2DRIVER_H
#define DATA2DRIVER_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
void collect_Data2Driver(); // Данные для передачи с Data на Driver // Копирование данных из сообщения в топике в структуру для передачи по SPI
// void processing_Driver2Data();																			   // Копирование полученных данных в структуру для публикации в топике
bool sendData2Driver(int channel_, Struct_Driver2Data &structura_receive_, Struct_Data2Driver &structura_send_); // Указываем на каком пине устройство и с какого регистра нужно прочитать данные // Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI

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

	data_driver_all++;
	digitalWrite(PIN_SPI_DRIVER, 0);
	delayMicroseconds(3);
	rez = wiringPiSPIDataRW(channel_, buffer, sizeof(buffer)); // Передаем и одновременно получаем данные
	delayMicroseconds(3);
	digitalWrite(PIN_SPI_DRIVER, 1);

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
// Копирование данных из сообщения в топике в структуру для использования
void collect_Data2Driver()
{
	for (int i = 0; i < 24; i++)
	{
		Data2Driver.led.led[i] = msg_ControlDriver.led.led[i];
	}

	/*
	if (msg_ControlDriver.pose.flag == true) // Если флаг что новые корректирующие данные пришли то меняем данные
	{
		odomUnited.pose.x = 0;
		odomUnited.pose.y = 0;
		odomUnited.pose.th = 0;
		// odomUnited.pose.x = msg_ControlDriver.pose.x;
		// odomUnited.pose.y = msg_ControlDriver.pose.y;
		// odomUnited.pose.th = msg_ControlDriver.pose.th;
		ROS_INFO("Falg true.");
	}
	*/

	g_dreamSpeed.speedL = msg_ControlDriver.control.speedL;
	g_dreamSpeed.speedR = msg_ControlDriver.control.speedR;
	// Data2Driver.control.speedL = 0.1;
	// Data2Driver.control.speedR = 0.1;

	// Data2Driver.led.num_program = msg_ControlDriver.led.num_program;
}

#endif