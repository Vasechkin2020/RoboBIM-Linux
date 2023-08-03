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
	//delay(2000000);
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
void dataProcessing_Body()
{
	//Копируем полученные по SPI данные в сообщение которое потом опубликуем
	msg_body_send.id = stru_body_receive.id;
	// Данные по лазерам
	/*
	msg_body_send.distance_lazer = stru_body_receive.distance_lazer;
	a1 = laser.filterVar1(msg_body_send.distance_lazer);
	a2 = laser.filterVar2(msg_body_send.distance_lazer);

	msg_body_send.distance_uzi = stru_body_receive.distance_uzi;
	u1 = uzi.filterVar1(msg_body_send.distance_uzi);
	u2 = uzi.filterVar2(msg_body_send.distance_uzi);

	//Данные по одометрии принимаем как есть
	msg_body_send.odom_l = stru_body_receive.odom_l;
	msg_body_send.odom_r = stru_body_receive.odom_r;
	//Данные по магнетрометру принимаем как есть
	msg_body_send.magnetrometr = stru_body_receive.magnetrometr;
	//Данные по датчику BME280 принимаем как есть
	msg_body_send.bmp280.humidity = stru_body_receive.bmp280.humidity;
	msg_body_send.bmp280.pressure = stru_body_receive.bmp280.pressure;
	msg_body_send.bmp280.temperature = stru_body_receive.bmp280.temperature;
	//Данные по датчику BNO055 принимаем как есть
	msg_body_send.bno055.x = stru_body_receive.bno055.x;
	msg_body_send.bno055.y = stru_body_receive.bno055.y;
	msg_body_send.bno055.z = stru_body_receive.bno055.z;
	//Данные по датчику MPU 9250
	msg_body_send.mpu9250.x = stru_body_receive.mpu9250.x;
	msg_body_send.mpu9250.y = stru_body_receive.mpu9250.y;
	msg_body_send.mpu9250.z = stru_body_receive.mpu9250.z;
	*/
	//Данные по датчикам газа
	msg_body_send.gaz1_data = stru_body_receive.gaz1_data;
	msg_body_send.gaz2_data = stru_body_receive.gaz2_data;

	//
}

//Копирование данных из сообщения в топике в структуру для передачи по SPI
void Collect_Command() // Данные для передачи на низкий уровень
{
	//stru_body_send.id = msg_head_receive.id;
	stru_body_send.id += 1;
	stru_body_send.command = msg_head_receive.command;
	//stru_body_send.napravlenie = msg_head_receive.napravlenie;
	//stru_body_send.radius = msg_head_receive.radius;
	stru_body_send.radius += 3.14;
	stru_body_send.speed = msg_head_receive.speed;
	//stru_body_send.time = 333;
	//stru_body_send.way = 3.14;
	stru_body_send.cam_angle = msg_head_receive.cam_angle;
	//stru_body_send.ventil_speed = msg_head_receive.ventil_speed;
	stru_body_send.ventil_speed += 3;
}
// Выводим на экран данные команды которую получили
void printCommand()
{
	printf(" SEND id = %i", stru_body_send.id);
	// printf(" command = %i", stru_body_send.command);
	//printf(" napravlenie = %i", msg_head_receive.napravlenie);
	printf(" radius = %f", stru_body_send.radius);
	// printf(" speed = %f", stru_body_send.speed);
	// printf(" angle_cam = %f", stru_body_send.cam_angle);
	printf(" ventil_speed = %i", stru_body_send.ventil_speed);
	printf("  /  ");
}
// Выводим на экран данные которые получили
void printData()
{
	printf(" RECEIVE id = %i", stru_body_receive.id);
	printf(" gaz1.x = %f", stru_body_receive.gaz1_data);
	printf(" gaz2.x = %f", stru_body_receive.gaz2_data);
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
	printf("\n");
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
//Функция которая проверяет линию обратной связи с ведомым и ждет пока там будет 1 или истечет время
void chekLineSPI()
{
	uint32_t time = micros();
	//while ( (digitalRead(PIN_SPI_LINE) != 1) || ((micros() - time) < 50) )
	while ( digitalRead(PIN_SPI_LINE) != 1 || micros() - time < 50 )
	{
	} // Пока ведомый не поднял линию что готов к новому приему байта или не прошло 50 микросекунд
}

//Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool SendData_in_Body(int channel_, Struct_Body_to_Head &structura_receive_, Struct_Head_to_Body &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
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
	chekLineSPI();
	// delayMicroseconds(pausa);
	buffer = 0x1B;
	rez = wiringPiSPIDataRW(channel_, &buffer, 1);
	chekLineSPI();
	//delayMicroseconds(pausa);

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
		chekLineSPI();
		//delayMicroseconds(pausa); //Задержка искуственная, чтобы на другом контроллере успели обработать пришедшие данные и записали данные для передачи

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
		//printf(" %i chek_sum_ok = %i", millis(), chek_sum_in);
		return true;
	}
	else
	{
		//printf(" %i  chek_sum_in = %i ",millis(), chek_sum_in);
		//printf(" chek_sum_data = %i ", chek_sum_data);
		// for (int i = 0; i < size_structura_receive; i++)
		// {
		// 	printf("%x ", adr_structura_receive[i]);
		// }
		// printf("\n");
		return false;
	}
}

// Обратный вызов при опросе топика Control
void message_callback_Command(const my_msgs::Command &msg)
{
	msg_head_receive = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							// ROS_INFO("message_callback_Command.");
}

#endif