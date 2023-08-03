#ifndef CODE_H
#define CODE_H

//Функция возращает максимальный размер из 2 структур
uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_)
{
	uint16_t ret = 0;

	if (stru1_ > stru2_)
	{
		ret = stru1_;
	}
	else
	{
		ret = stru2_;
	}
	return ret += 1; // + 1 байт Для контрольной суммы
}

//Настройка светодиодов
void set_PIN_Led()
{
	pinMode(PIN_LED_RED, OUTPUT);	// Зеленый светодиод
	pinMode(PIN_LED_BLUE, OUTPUT);	// Красный светодиод
	pinMode(PIN_LED_GREEN, OUTPUT); // Красный светодиод
	digitalWrite(PIN_LED_RED, 0);
	digitalWrite(PIN_LED_BLUE, 0);
	digitalWrite(PIN_LED_GREEN, 0);
}

//Функция мигания светодиодом в осномном цикле что программа не зависла и работает
void Led_Blink(int led_, unsigned long time_)
{
	static unsigned long led_time = 0;
	static bool led_status = 0;
	if ((millis() - led_time) > time_)
	{
		led_status = 1 - led_status;
		digitalWrite(led_, led_status);
		led_time = millis();
	}
}

//Обработка полученных данных и копирование их куда надо
void dataProcessing_Control()
{
	//Делаем копию для себя тут
	Copy_Control = Control;

	//msg.speed = Control.speed;
	//msg.radius = Control.radius;
	//msg.napravl = Control.napravl;
	// msg.startStop = Control.startStop;
	// msg.connect_flag = Control.connect_flag;
}

//Копирование рабочих данных в структуру для передачи
void Collect_DataHL_Control() // Данные для передачи на низкий уровень
{
	// DataHL_Body.command = Copy_Control.startStop;
	// DataHL_Body.napravlenie = Copy_Control.napravl;
	// DataHL_Body.radius = Copy_Control.radius;
	// DataHL_Body.speed = Copy_Control.speed;
	// DataHL_Body.time = 5000;
	// DataHL_Body.way = 1.1;
}

//Инициализация канала шины SPI
void init_SPI(int channel_, int speed_)
{
	uint8_t errSpi = 0; //Ошибка при инициализации шины SPI
	printf("Init SPI start... ");

	if ((errSpi = wiringPiSPISetup(channel_, speed_)) < 0) //Инициализация канало 0 это чип селект 0
	{
		printf("Can't open the SPI bus 0: %s\n", strerror(errno));
		printf("errSpi: %s\n", errSpi);
		exit(EXIT_FAILURE);
	}
	else
	{
		printf(" SPI ok! \n");
	}
}

// Выводим на экран данные которые получили
void printData()
{
	printf(" id = %i", Control.id);
	printf(" radius = %.2f", Control.radius);
	printf(" cheksum = %i", Control.cheksum);

	printf("\n");
}

//Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool SendData_in_Control(int channel_, volatile Struct_Iot2Data &structura_receive_, volatile Struct_DataHL_Control &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = 0;	  // Результат выполнения функции
	unsigned char buffer; //Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт
	//byte *p = (byte *)(&DataHL_Body); // Запоминаем адрес начала структуры. Используем для побайтной передачи
	const int pausa = 20; // тут задаем задержку между пакетами. Нужно учитывать не только сколько на том конце обрабатываем пришедшие данные и готовим к отправке новые,
						  // но и задержку вызова самого прерывания после переданного байта

	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	uint8_t *adr_structura_receive = (uint8_t *)(&structura_receive_);	// Запоминаем адрес начала структуры. Используем для побайтной передачи

	const uint16_t size_structura_send = sizeof(structura_send_); // Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);  // Запоминаем адрес начала структуры. Используем для побайтной передачи

	uint16_t max_size_stuct = getMax_size_Struct(size_structura_receive, size_structura_send); // Какая из структур больше
	//uint16_t Max_size_stuct = 0;	 //Определяем какая структура больше и сюда пишем размер болшей + 1 байт на чексумм

	volatile uint8_t data_out = 0;
	volatile uint8_t data_in = 0;
	volatile uint16_t count_in = 0;		// Счетчик входящих байт полученных по протоколу SPI
	volatile uint16_t count_out = 0;	// Счетчик исходящих байт переданных по протоколу SPI
	volatile uint8_t chek_sum_in = 0;	// Байт контрольной суммы получаемых данных
	volatile uint8_t chek_sum_out = 0;	// Байт контрольной суммы отправляемых данных
	volatile uint8_t chek_sum_data = 0; // Байт контрольной суммы пришедший от источника

	// ROS_INFO("Send Data SPI \n");
	// служебные идентификационные байты ими начинается пакет передачи
	buffer = 0x1A;
	rez = wiringPiSPIDataRW(channel_, &buffer, 1);
	delayMicroseconds(pausa);
	buffer = 0x1B;
	rez = wiringPiSPIDataRW(channel_, &buffer, 1);
	delayMicroseconds(pausa);

	for (uint8_t i = 0; i <= max_size_stuct; i++) //Нам нужно отправить байт в размер максимальнйо структуры и 1 байт чек суммы
	{
		// Serial.print("> ");
		// Serial.print(Count_out);
		// Serial.print(" ");

		// Тут готовим байт для передачи. Берем его из структуры
		if (count_out < size_structura_send)
		{
			data_out = adr_structura_send[count_out]; // Считываем байт данных
			chek_sum_out += data_out;				  // считаем контрольную сумму
		}
		else // Передаем контрольную сумму
		{
			if (count_out == size_structura_send)
			{
				data_out = chek_sum_out;
				//Serial.print("CS> ");
			}
		}

		buffer = data_out;
		rez = wiringPiSPIDataRW(channel_, &buffer, 1); //Передаем и одовременно получаем данные
		data_in = buffer;

		count_out++;
		//Serial.println(data_out, HEX);
		delayMicroseconds(pausa); //Звдержка искуственная, чтобы на другом контроллере успели обработать пришедшие данные и записали данные для передачи

		//Прием данных
		if (i != 0) // Перый байт прием данных пропускаем, так как нам данные пойдут только со второго отправленного байта, в ответ на первый
		{
			if (count_in < size_structura_receive) //пока мешьше размера структуры принимаем данные
			{
				adr_structura_receive[count_in] = data_in; //Записываем пришедший байт по адрессу структуры начиная с нулевого
				// Serial.println(data_in, HEX);
				chek_sum_in += data_in; // Суммирем все байты данных при перевышении остается младший байт его и сравниваем
			}
			else //Зачит пришел последний байт с контрольной суммой
			{
				if (count_in == size_structura_receive)
				{
					chek_sum_data = data_in; //Сохраняем для последущей проверки пришедшую контрольную сумму
											 // Serial.print("cs> ");
											 // Serial.println(Chek_sum_data, HEX);
				}
			}
			count_in++;
		}
	}
	if (chek_sum_in == chek_sum_data) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		return true;
		//Serial.println("chek_sum Ok");
	}
	else
	{
		return false;
		//Serial.print("chek_sum BED");
	}
}

//Функция возвращает контрольную сумму структуры без последних 4 байтов
template <typename T>
uint32_t measureCheksum(const T &structura_)
{
	uint32_t ret = 0;
	unsigned char *adr_structura = (unsigned char *)(&structura_); // Запоминаем адрес начала структуры. Используем для побайтной передачи
	for (int i = 0; i < sizeof(structura_) - 4; i++)
	{
		ret += adr_structura[i]; // Побайтно складываем все байты структуры кроме последних 4 в которых переменная в которую запишем результат
	}
	return ret;
}
#define SIZE_BUFF 16
unsigned char buffer[SIZE_BUFF]; //Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт

//Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Iot(int channel_, Struct_Iot2Data &structura_receive_, Struct_DataHL_Control &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	//uint16_t max_size_stuct = getMax_size_Struct(size_structura_receive, size_structura_send); // Какая из структур больше
	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(buffer, 0, sizeof(buffer));																							   // Очищаем буфер передачи

	//тут нужнопосчитать контрольную смму структуры
	structura_send_.cheksum = measureCheksum(structura_send_); // Считаем контрольную сумму отправляемой структуры
	printf("Отправляем: Id %i, чек= %i  ", structura_send_.id, structura_send_.cheksum);

	//Заполняем буфер данными структуры для передачи
	Struct_DataHL_Control *buffer_send = (Struct_DataHL_Control *)buffer; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;										  // Переписываем по этому адресу данные в буфер

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
	ok++;
	int aa = micros();
	rez = wiringPiSPIDataRW(channel_, buffer, sizeof(buffer)); //Передаем и одновременно получаем данные
	int time_transfer = micros() - aa;
	float time_transfer_sec = time_transfer / 1000000.0;

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
	Struct_Iot2Data *copy_buf_master_receive = (Struct_Iot2Data *)buffer;		// Создаем переменную в которую пишем адрес буфера в нужном формате
	Struct_Iot2Data copy_Control_receive = *copy_buf_master_receive;			// Копируем из этой перемнной данные в мою структуру
	structura_receive_ = *copy_buf_master_receive;							// Копируем из этой перемнной данные в мою структуру
	uint8_t *adr_copy_Control_receive = (uint8_t *)(&copy_Control_receive); // Запоминаем адрес начала структуры. Используем для побайтной передачи

	// for (int i = 0; i < 20; i++)
	// {
	// 	printf("%x ",adr_copy_Control_receive[i]);
	// }
	// 	printf("\n");

	uint32_t cheksum_receive = measureCheksum(structura_receive_); // Считаем контрольную сумму пришедшей структуры
	uint32_t cheksum_buf = measureCheksum(buffer); // Считаем контрольную сумму пришедшей структуры

	printf(" Получили: Id %i, radius %f", structura_receive_.id, structura_receive_.radius);
	//printf(" busvoltage %f, current_mA %f", structura_receive_.INA219.busvoltage, structura_receive_.INA219.current_mA);
	printf(" чексумма %i, myChek= %i, buf_chek= %i", structura_receive_.cheksum, cheksum_receive, cheksum_buf);

	//printf(" ok= %i, bed= %i, time= %i, speed Kbyte/sec= %f \n", ok, bed, time_transfer, (sizeof(buffer)/1024.0) / time_transfer_sec);
	printf(" ok= %i, bed= %i \n", ok, bed);

	//Serial.println(String(micros()) + " copy_DataHL_Control_receive id= " + copy_DataHL_Control_receive.id);
	//Serial.println(String(micros()) + " copy_DataHL_Control_receive bmp280.pressure= " + copy_DataHL_Control_receive.bmp280.pressure);
	// Serial.println(String(micros()) + " copy_DataHL_Control_receive.cheksum_receive= " + copy_DataHL_Control_receive.cheksum);
	//printf(" measureCheksum= %i \n", cheksum_receive);
	if (cheksum_receive != structura_receive_.cheksum || structura_receive_.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		printf("%i", micros());
		printf(" measureCheksum BED !!! ");
		bed++;
		return false;
	}
	else // Все хорошо возвращаем Ок
	{
		return true;
		//Serial.print("chek_sum Ok");
	}
}

#endif