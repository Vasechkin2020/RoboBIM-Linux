#ifndef DATA2MODUL_H
#define DATA2MODUL_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************

void collect_Data2Modul(int data_);																			  // Данные для передачи на низкий уровень //Копирование рабочих данных в структуру для передачи
bool sendData2Modul(int channel_, Struct_Modul2Data &structura_receive_, Struct_Data2Modul &structura_send_); // Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI

//***********************************************************************************

void collect_Data2Modul(int data_) // Данные для передачи на низкий уровень
{
	if (data_ == 1)
	{
		// printf("сollect_Data2Modul in... \n");
		Data2Modul.controlLaser.mode = msg_ControlModul.controlLaser.mode;
		Data2Modul.controlMotor.mode = msg_ControlModul.controlMotor.mode; //
		for (int i = 0; i < 4; i++)
		{
			Data2Modul.controlMotor.angle[i] = msg_ControlModul.controlMotor.angle[i] + angle_offsets[i]; // Прибавляем оффсет чтобы компенсировать ошибку по углу между углом что лазер посчитал и углом мотора в модуле
			Data2Modul.controlMotor.numPillar[i] = msg_ControlModul.controlMotor.numPillar[i];			  //
		}
	}
	else
	{
		Data2Modul.controlLaser.mode = 0;
		Data2Modul.controlMotor.mode = 0; //
		for (int i = 0; i < 4; i++)
		{
			Data2Modul.controlMotor.angle[i] = 0;	   //
			Data2Modul.controlMotor.numPillar[i] = -1; //
		}
	}
}

// Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Modul(int channel_, Struct_Modul2Data &structura_receive_, Struct_Data2Modul &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
// bool sendData2Modul(int channel_, STest &structura_receive_, STest &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	static uint32_t prevId = 0; // Значение предыдущего ID

	// uint16_t max_size_stuct = getMax_size_Struct(size_structura_receive, size_structura_send); // Какая из структур больше
	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(bufferModul, 0, sizeof(bufferModul));																				   // Очищаем буфер передачи

	// Заполняем буфер данными структуры для передачи
	Struct_Data2Modul *buffer_send = (Struct_Data2Modul *)bufferModul; // Создаем переменную в которую записываем адрес буфера в нужном формате
	// STest *buffer_send = (STest *)buffer; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_; // Переписываем по этому адресу данные в буфер
	data_modul_all++;
	// printf("   data_modul_all= %i \n",data_modul_all);

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

	// data_Modul_all++;
	//  int aa = micros();
	
	// digitalWrite(PIN_SPI_MODUL, 0);
	// delayMicroseconds(3);
	// rez = wiringPiSPIDataRW(channel_, bufferModul, sizeof(bufferModul)); // Передаем и одновременно получаем данные
	// delayMicroseconds(3);
	// digitalWrite(PIN_SPI_MODUL, 1);

	if (spi_drv.transferWithCS(PIN_SPI_MODUL, bufferModul, sizeof(bufferModul), 1)) 
	{
        rez = true;
    } else 
	{
        rez = false;
        ROS_ERROR("SPI Transfer to Modul failed");
    }


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

	// Извлекаем из буфера данные в формате структуры и копирум данные

	Struct_Modul2Data *copy_buf_master_receive = (Struct_Modul2Data *)bufferModul; // Создаем переменную в которую пишем адрес буфера в нужном формате
	// STest *copy_buf_master_receive = (STest *)buffer; // Создаем переменную в которую пишем адрес буфера в нужном формате
	//  structura_receive_ = *copy_buf_master_receive;						  // Копируем из этой перемнной данные в мою структуру
	//  uint32_t cheksum_receive = measureCheksum(structura_receive_);		  // Считаем контрольную сумму пришедшей структуры
	Struct_Modul2Data structura_receive_temp; // Временная структура, проверить правильные ли пришли данные
	// STest structura_receive_temp;						   // Временная структура, проверить правильные ли пришли данные
	structura_receive_temp = *copy_buf_master_receive;				   // Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_temp); // Считаем контрольную сумму пришедшей структуры

	// Struct_Modul2Data copy_Control_receive = *copy_buf_master_receive;		// Копируем из этой перемнной данные в мою структуру
	// uint8_t *adr_copy_Control_receive = (uint8_t *)(&copy_Control_receive); // Запоминаем адрес начала структуры. Используем для побайтной передачи
	//  for (int i = 0; i < 20; i++)
	//  {
	//  	printf("%x ",adr_copy_Control_receive[i]);
	//  }
	//  	printf("\n");

	uint32_t cheksum_buf = measureCheksum(bufferModul); // Считаем контрольную сумму пришедшей структуры

	// printf(" Получили: Id %0#6lX, cheksum %0#6lX cheksum_receive %0#6lX ", structura_receive_temp.id, structura_receive_temp.cheksum, cheksum_receive);

	//  printf(" capacity_real %f, capacity_percent %f", structura_receive_.capacity_real, structura_receive_.capacity_percent);
	//  printf(" inVoltage %f, current_mA %f", structura_receive_.INA219.inVoltage, structura_receive_.INA219.current_mA);
	//  printf(" radius %f, startStop %i", structura_receive_.radius, structura_receive_.startStop);
	// printf(" чексумма %i, myChek= %i, buf_chek= %i", structura_receive_.cheksum, cheksum_receive, cheksum_buf);

	// printf(" ok= %i, bed= %i, time= %i, speed Kbyte/sec= %f \n", ok, bed, time_transfer, (sizeof(buffer)/1024.0) / time_transfer_sec);
	// printf(" /  ok= %i, bed= %i  / ", data_Modul_all, data_Modul_bed);

	// Serial.println(String(micros()) + " copy_DataHL_Control_receive id= " + copy_DataHL_Control_receive.id);
	// Serial.println(String(micros()) + " copy_DataHL_Control_receive bmp280.pressure= " + copy_DataHL_Control_receive.bmp280.pressure);
	//  Serial.println(String(micros()) + " copy_DataHL_Control_receive.cheksum_receive= " + copy_DataHL_Control_receive.cheksum);
	// printf(" measureCheksum= %i \n", cheksum_receive);

	if (prevId == structura_receive_temp.id) // Если ID не изменился значит модуль завис и шлет повторно одни и теже данные
	{
		ROS_ERROR("ID Modul NOT CHANGE = %lu", structura_receive_temp.id);
	}
	prevId == structura_receive_temp.id; // Запоминаем ID

	if (cheksum_receive != structura_receive_temp.cheksum || structura_receive_temp.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		data_modul_bed++;
		ROS_ERROR("sendData2Modul Data Err in chek= %#x local chek = %#x", structura_receive_temp.cheksum, cheksum_receive);
		return false;
	}
	else // Все хорошо возвращаем Ок
	{
		structura_receive_ = structura_receive_temp; // Копируем хорошие данные уже в итоговую структуру, если плохие то они просто пропадают и не портят прошлые
		// printf("Data OK \n");
		return true;
	}
	return false;
}

#endif