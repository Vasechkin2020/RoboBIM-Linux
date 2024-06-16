#ifndef DATA2PRINT_H
#define DATA2PRINT_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
void collect_Data2Print(); // Данные для передачи с Data на Print // Копирование данных из сообщения в топике в структуру для передачи по SPI
// void processing_Print2Data();																			   // Копирование полученных данных в структуру для публикации в топике
bool sendData2Print(int channel_, SPrint2Data &structura_receive_, SData2Print &structura_send_); // Указываем на каком пине устройство и с какого регистра нужно прочитать данные // Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI

//***********************************************************************************
// Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Print(int channel_, SPrint2Data &structura_receive_, SData2Print &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(buffer, 0, sizeof(buffer));																							   // Очищаем буфер передачи

	// Заполняем буфер данными структуры для передачи
	SData2Print *buffer_send = (SData2Print *)buffer; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;					  // Переписываем по этому адресу данные в буфер
	
	data_print_all++;
	rez = wiringPiSPIDataRW(channel_, buffer, sizeof(buffer)); // Передаем и одновременно получаем данные

	// Извлекаем из буфера данные в формате структуры и копирум данные
	SPrint2Data *copy_buf_master_receive = (SPrint2Data *)buffer;	   // Создаем переменную в которую пишем адрес буфера в нужном формате
	SPrint2Data structura_receive_temp;								   // Временная структура, проверить правильные ли пришли данные
	structura_receive_temp = *copy_buf_master_receive;				   // Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_temp); // Считаем контрольную сумму пришедшей структуры

	if (cheksum_receive != structura_receive_temp.cheksum || structura_receive_temp.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
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
void collect_Data2Print()
{
	Data2Print.id++;				 //= 0x1F1F1F1F;
	Data2Print.controlPrint.status = 1; //
	Data2Print.controlPrint.mode = 1; //
	Data2Print.controlPrint.intensity = 2; //
	Data2Print.controlPrint.speed = 0.5; //

	// тут нужно посчитать контрольную сумму структуры
	Data2Print.cheksum = measureCheksum(Data2Print); // Считаем контрольную сумму отправляемой структуры

	// printf("Отправляем: Id %i, чек= %i  ", Data2Modul.id, Data2Modul.cheksum);
}

#endif