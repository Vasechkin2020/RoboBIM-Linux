#ifndef DATA2PRINT_H
#define DATA2PRINT_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
void collect_Data2Print(int data_);																			  // Данные для передачи с Data на Print // Копирование данных из сообщения в топике в структуру для передачи по SPI
bool sendData2Print(int channel_, Struct_Print2Data &structura_receive_, Struct_Data2Print &structura_send_); // Указываем на каком пине устройство и с какого регистра нужно прочитать данные // Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI

//***********************************************************************************
// Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Print(int channel_, Struct_Print2Data &structura_receive_, Struct_Data2Print &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(bufferPrint, 0, sizeof(bufferPrint));																				   // Очищаем буфер передачи

	// Заполняем буфер данными структуры для передачи
	Struct_Data2Print *buffer_send = (Struct_Data2Print *)bufferPrint; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;									   // Переписываем по этому адресу данные в буфер

	data_print_all++;
	digitalWrite(PIN_SPI_PRINT, 0);
	delayMicroseconds(3);
	rez = wiringPiSPIDataRW(channel_, bufferPrint, sizeof(bufferPrint)); // Передаем и одновременно получаем данные
	delayMicroseconds(3);
	digitalWrite(PIN_SPI_PRINT, 1);

	// Извлекаем из буфера данные в формате структуры и копирум данные
	Struct_Print2Data *copy_buf_master_receive = (Struct_Print2Data *)bufferPrint; // Создаем переменную в которую пишем адрес буфера в нужном формате
	Struct_Print2Data structura_receive_temp;									   // Временная структура, проверить правильные ли пришли данные
	structura_receive_temp = *copy_buf_master_receive;							   // Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_temp);			   // Считаем контрольную сумму пришедшей структуры

	// for (int i = 0; i < sizeof(bufferPrint); i++)
	// {
	// 	printf(" %#x", bufferPrint[i]);
	// }

	if (cheksum_receive != structura_receive_temp.cheksum || structura_receive_temp.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		printf(" in chek= %#x local chek = %#x\n", structura_receive_temp.cheksum, cheksum_receive);
		data_print_bed++;
		return false;
	}
	else // Все хорошо возвращаем Ок
	{
		structura_receive_ = structura_receive_temp; // Копируем хорошие данные уже в итоговую структуру, если плохие то они просто пропадают и не портят прошлые
		return true;
	}
}
// Копирование данных из сообщения в топике в структуру для использования
void collect_Data2Print(int data_)
{
	if (data_ == 1)
	{
		// printf("collect_Data2Print in... \n");
		Data2Print.controlPrint.status = msg_ControlPrint.controlPrint.status;		 //
		Data2Print.controlPrint.mode = msg_ControlPrint.controlPrint.mode;			 //
		Data2Print.controlPrint.intensity = msg_ControlPrint.controlPrint.intensity; //
		Data2Print.controlPrint.speed = msg_ControlPrint.controlPrint.speed;		 //
	}
	else
	{
		Data2Print.controlPrint.status = 0;	   //
		Data2Print.controlPrint.mode = 0;	   //
		Data2Print.controlPrint.intensity = 0; //
		Data2Print.controlPrint.speed = 0;
	}
}

#endif