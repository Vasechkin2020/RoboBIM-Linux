#ifndef DATA2MODUL_H
#define DATA2MODUL_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************


void dataProcessing_Modul();																				  // Обработка полученных данных и копирование их для публикации в топике
void Collect_Data2Modul();																					  // Данные для передачи на низкий уровень //Копирование рабочих данных в структуру для передачи
void printData_To_Control();																				  // Выводим на экран данные которые отправляем в Control
void printDataFrom_Control();																				  // Выводим на экран данные которые получили
bool sendData2Modul(int channel_, Struct_Modul2Data &structura_receive_, Struct_Data2Modul &structura_send_); // Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI

//***********************************************************************************

// Копирование рабочих данных в структуру для передачи
void Collect_Data2Modul() // Данные для передачи на низкий уровень
{
	Data2Modul.id++;							   //= 0x1F1F1F1F;
	Data2Modul.command = msg_ControlModul.command;   //
	Data2Modul.angle[0] = msg_ControlModul.angle[0]; //
	Data2Modul.angle[1] = msg_ControlModul.angle[1]; //
	Data2Modul.angle[2] = msg_ControlModul.angle[2]; //
	Data2Modul.angle[3] = msg_ControlModul.angle[3]; //

	// тут нужно посчитать контрольную сумму структуры
	Data2Modul.cheksum = measureCheksum(Data2Modul); // Считаем контрольную сумму отправляемой структуры
													 // printf("Отправляем: Id %i, чек= %i  ", Data2Modul.id, Data2Modul.cheksum);
}



// Обработка полученных данных и копирование их для публикации в топике
void dataProcessing_Modul()
{
	//----------------------  msg_Modul_info_send ----------------------
	modul_motor_msg.id = Modul2Data.id;
	modul_lidar_msg.id = Modul2Data.id;
	modul_micric_msg.id = Modul2Data.id;

	modul_motor_msg.id = Modul2Data.pinMotorEn; // Стутус пина управления драйвером моторов, включен драйвер или нет
	for (int i = 0; i < 4; i++)
	{
		modul_motor_msg.motor[i].status = Modul2Data.motor[i].status;			//
		modul_motor_msg.motor[i].position = Modul2Data.motor[i].position;		//
		modul_motor_msg.motor[i].destination = Modul2Data.motor[i].destination; //

		modul_lidar_msg.lidar[i].status = Modul2Data.lidar[i].status;	  //
		modul_lidar_msg.lidar[i].distance = Modul2Data.lidar[i].distance; //
		modul_lidar_msg.lidar[i].angle = Modul2Data.lidar[i].angle;		  //

		modul_micric_msg.micric[i] = Modul2Data.micric[i]; // Состояние концевиков
	}
}

// Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Modul(int channel_, Struct_Modul2Data &structura_receive_, Struct_Data2Modul &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	// uint16_t max_size_stuct = getMax_size_Struct(size_structura_receive, size_structura_send); // Какая из структур больше
	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(buffer, 0, sizeof(buffer));																							   // Очищаем буфер передачи

	// Заполняем буфер данными структуры для передачи
	Struct_Data2Modul *buffer_send = (Struct_Data2Modul *)buffer; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;								  // Переписываем по этому адресу данные в буфер

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
	rez = wiringPiSPIDataRW(channel_, buffer, sizeof(buffer)); // Передаем и одновременно получаем данные
	delayMicroseconds(10);
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
	
	Struct_Modul2Data *copy_buf_master_receive = (Struct_Modul2Data *)buffer; // Создаем переменную в которую пишем адрес буфера в нужном формате
	// structura_receive_ = *copy_buf_master_receive;						  // Копируем из этой перемнной данные в мою структуру
	// uint32_t cheksum_receive = measureCheksum(structura_receive_);		  // Считаем контрольную сумму пришедшей структуры
	Struct_Modul2Data structura_receive_temp;								  // Временная структура, проверить правильные ли пришли данные
	structura_receive_temp = *copy_buf_master_receive;						  // Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_temp);		  // Считаем контрольную сумму пришедшей структуры

	// Struct_Modul2Data copy_Control_receive = *copy_buf_master_receive;		// Копируем из этой перемнной данные в мою структуру
	// uint8_t *adr_copy_Control_receive = (uint8_t *)(&copy_Control_receive); // Запоминаем адрес начала структуры. Используем для побайтной передачи
	//  for (int i = 0; i < 20; i++)
	//  {
	//  	printf("%x ",adr_copy_Control_receive[i]);
	//  }
	//  	printf("\n");

	uint32_t cheksum_buf = measureCheksum(buffer); // Считаем контрольную сумму пришедшей структуры

	// printf(" Получили: Id %i, cheksum %i", structura_receive_.id, structura_receive_.cheksum);
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
	if (cheksum_receive != structura_receive_temp.cheksum || structura_receive_temp.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		data_modul_bed++;
		return false;
	}
	else // Все хорошо возвращаем Ок
	{
		structura_receive_= structura_receive_temp; // Копируем хорошие данные уже в итоговую структуру, если плохие то они просто пропадают и не портят прошлые
		return true;
	}
}

// Выводим на экран данные которые отправляем в Control
void printData_To_Control()
{
	// printf(" Data2Modul id = %i", Data2Modul.id);
	// printf(" distance_uzi = %f", Data2Modul.distance_uzi);
}

// Выводим на экран данные которые получили
void printDataFrom_Control()
{
	// printf(" Получили id = %i", Modul2Data.id);
	// printf(" radius = %.2f", Modul2Data.radius);
	// printf(" cheksum = %i", Modul2Data.cheksum);
	// printf("\n");
}

#endif