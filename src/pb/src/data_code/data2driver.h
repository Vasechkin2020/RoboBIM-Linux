#ifndef DATA2DRIVER_H
#define DATA2DRIVER_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
void collect_Data2Driver(int data_); // Данные для передачи с Data на Driver // Копирование данных из сообщения в топике в структуру для передачи по SPI
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
	memset(bufferDriver, 0, sizeof(bufferDriver));																							   // Очищаем буфер передачи

	// Заполняем буфер данными структуры для передачи
	Struct_Data2Driver *buffer_send = (Struct_Data2Driver *)bufferDriver; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;									// Переписываем по этому адресу данные в буфер

	data_driver_all++;
	// printf("   data_driver_all= %i \n",data_driver_all);

	// digitalWrite(PIN_SPI_DRIVER, 0);
	// delayMicroseconds(3);
	// rez = wiringPiSPIDataRW(channel_, bufferDriver, sizeof(bufferDriver)); // Передаем и одновременно получаем данные
	// delayMicroseconds(3);
	// digitalWrite(PIN_SPI_DRIVER, 1);

	// Используем новый метод с защитой     // PIN_SPI_DRIVER - номер пина CS    // bufferDriver - буфер данных    // 3 - задержка в микросекундах (как у тебя было)
    if (spi_drv.transferWithCS(PIN_SPI_DRIVER, bufferDriver, sizeof(bufferDriver), 1)) 
	{
        rez = true;
    } else 
	{
        rez = false;
        ROS_ERROR("SPI Transfer to Driver failed");
    }

	// Извлекаем из буфера данные в формате структуры и копирум данные
	Struct_Driver2Data *copy_buf_master_receive = (Struct_Driver2Data *)bufferDriver; // Создаем переменную в которую пишем адрес буфера в нужном формате
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
void collect_Data2Driver(int data_)
{
	if (data_ == 1) // Данные из топика
	{
		//printf("collect_Data2Driver in... \n");
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

		g_desiredSpeed.speedL = msg_ControlDriver.control.speedL;
		g_desiredSpeed.speedR = msg_ControlDriver.control.speedR;
		// Data2Driver.control.speedL = 0.1;
		// Data2Driver.control.speedR = 0.1;

		// Data2Driver.led.num_program = msg_ControlDriver.led.num_program;
	}
	else // Двнные если нет топика сколько-то времени
	{
		g_desiredSpeed.speedL = 0;
		g_desiredSpeed.speedR = 0;

	}
	// logi.log("msg_ControlDriver L = %+6.3f R= %+6.3f \n", msg_ControlDriver.control.speedL, msg_ControlDriver.control.speedR);
}

#endif