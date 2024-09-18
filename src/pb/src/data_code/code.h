#ifndef CODE_H
#define CODE_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_);	 // Функция возращает максимальный размер из 2 структур
void set_PIN_Led();												 // Настройка светодиодов
void Led_Blink(int led_, unsigned long time_);					 // Функция мигания светодиодом в осномном цикле что программа не зависла и работает
void init_SPI(int channel_, int speed_);						 // Инициализация канала шины SPI

void callback_ControlDriver(const pb_msgs::Struct_Data2Driver &msg); // Обратный вызов при опросе топика Driver
void callback_ControlModul(const pb_msgs::Struct_Data2Modul &msg);	 // Обратный вызов при опросе топика Modul
void callback_ControlPrint(const pb_msgs::Struct_Data2Print &msg);	 // Обратный вызов при опросе топика Print
void callback_Joy(sensor_msgs::Joy msg);						 // Функция обраьтного вызова по подпичке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
void controlAcc(SControl dreamSpeed_); // Функция контроля ускорения

SControl speedToRps(SControl speed_); // Конвертация скорости из метров в секунду в обороты в секунду для передачи на нижний уровень

// **********************************************************************************
// Функция возращает максимальный размер из 2 структур
uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_)
{
	uint16_t ret = 0;
	stru1_ > stru2_ ? ret = stru1_ : ret = stru2_;
	return ret += 1; // + 1 байт Для контрольной суммы
}
// Настройка светодиодов
void set_PIN_Led()
{
	pinMode(PIN_LED_BLUE, OUTPUT); // Красный светодиод
}
// Функция мигания светодиодом в осномном цикле что программа не зависла и работает
void Led_Blink(int led_, unsigned long time_)
{
	static unsigned long led_time = 0;
	static bool led_status = 0;
	if ((millis() - led_time) > time_)
	{
		led_status = 1 - led_status;
		digitalWrite(led_, led_status);
		led_time = millis();
		// ROS_INFO("%s led_time = %i.", NN, led_time);
		printf("+ \n");
	}
}
// Инициализация канала шины SPI
void init_SPI(int channel_, int speed_)
{
	uint8_t errSpi = 0; // Ошибка при инициализации шины SPI
	ROS_INFO("%s Init SPI start... ", NN);

	if ((errSpi = wiringPiSPISetup(channel_, speed_)) < 0) // Инициализация канало 0 это чип селект 0
	{
		ROS_ERROR("%s Can't open the SPI bus 0: %s\n", NN, strerror(errno));
		ROS_ERROR("%s errSpi: %s\n", NN, errSpi);
		exit(EXIT_FAILURE);
	}
	else
	{
		ROS_INFO("%s SPI ok! \n", NN);
	}
}
// Функция обраьтного вызова по подписке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
void callback_Joy(sensor_msgs::Joy msg)
{
	flag_msgJoy = true;
	msg_joy = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
}
// Обратный вызов при опросе топика
void callback_ControlDriver(const pb_msgs::Struct_Data2Driver &msg)
{
	flag_msgControlDriver = true;
	timeSpiDriver = millis();
	msg_ControlDriver = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							 // ROS_INFO("message_callback_Command.");
}
// Обратный вызов при опросе топика
void callback_ControlModul(const pb_msgs::Struct_Data2Modul &msg)
{
	flag_msgControlModul = true;
	timeSpiModul = millis();
	msg_ControlModul = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							// ROS_INFO("message_callback_Command.");
} 
// Обратный вызов при опросе топика
void callback_ControlPrint(const pb_msgs::Struct_Data2Print &msg)
{
	flag_msgControlPrint = true;
	timeSpiPrint = millis();
	msg_ControlPrint = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							// ROS_INFO("message_callback_Command.");
}
// Функция контроля ускорения
void controlAcc(SControl dreamSpeed_) 
{
	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;
	float accel = ACCELERATION * dt; // Ускорение
	//printf("dreamSpeed_ % .3f % .3f accel= % .5f dt= % .5f", dreamSpeed_.speedL, dreamSpeed_.speedR, accel, dt);
	if (dreamSpeed_.speedL != g_factSpeed.speedL) // Если скорость с которой хотим крутиться не равна тому что была ранее установлена, то меняем с учетом ускорения
	{
		// printf("dreamSpeed_ % f : % f : acc= % f | ", dreamSpeed_.speedL, dreamSpeed_.speedR, accel);
		if (g_factSpeed.speedL < dreamSpeed_.speedL) // Если меньше чем надо то прибавим оборотов
		{
			g_factSpeed.speedL = g_factSpeed.speedL + accel; // К старой скорости прибавляем ускорение за этот промежуток
			if (g_factSpeed.speedL > dreamSpeed_.speedL)	 // Если стала больше то ровняем
				g_factSpeed.speedL = dreamSpeed_.speedL;
		}
		if (g_factSpeed.speedL > dreamSpeed_.speedL) // Если меньше чем надо то прибавим оборотов
		{
			g_factSpeed.speedL = g_factSpeed.speedL - accel; // К старой скорости прибавляем ускорение за этот промежуток
			if (g_factSpeed.speedL < dreamSpeed_.speedL)	 // Если стала меньше нужной то далаем какая должна быть
				g_factSpeed.speedL = dreamSpeed_.speedL;
		}
	}
	if (dreamSpeed_.speedR != g_factSpeed.speedR) // Если скорость с которой хотим крутиться не равна тому что была ранее установлена, то меняем с учетом ускорения
	{
		if (g_factSpeed.speedR < dreamSpeed_.speedR) // Если меньше чем надо то прибавим оборотов
		{
			g_factSpeed.speedR = g_factSpeed.speedR + accel; // К старой скорости прибавляем ускорение за этот промежуток
			if (g_factSpeed.speedR > dreamSpeed_.speedR)	 // Если стала больше то ровняем
				g_factSpeed.speedR = dreamSpeed_.speedR;
		}
		if (g_factSpeed.speedR > dreamSpeed_.speedR) // Если меньше чем надо то прибавим оборотов
		{
			g_factSpeed.speedR = g_factSpeed.speedR - accel; // К старой скорости прибавляем ускорение за этот промежуток
			if (g_factSpeed.speedR < dreamSpeed_.speedR)	 // Если стала меньше нужной то далаем какая должна быть
				g_factSpeed.speedR = dreamSpeed_.speedR;
		}
	}
	//printf("g_factSpeed % f : % f : acc= % f \n ", g_factSpeed.speedL, g_factSpeed.speedR);
	//printf(" |g_factSpeed % .3f % .3f \n", g_factSpeed.speedL, g_factSpeed.speedR);
}
// Конвертация скорости из метров в секунду в обороты в секунду для передачи на нижний уровень
SControl speedToRps(SControl speed_)
{
	speed_.speedL = speed_.speedL / PERIMETR * KOEF_ODOM; //Делим скорость вметрах на периметр колеса и умножаем на дополнительный коеффициент который вручную подобрал что-бы одометрия соответствовала реальности
	speed_.speedR = speed_.speedR / PERIMETR * KOEF_ODOM; //Делим скорость вметрах на периметр колеса и умножаем на дополнительный коеффициент который вручную подобрал что-бы одометрия соответствовала реальности
	return speed_;
}

// Функция управления несколькими светодиодами которые отведены для прямого управления нодой data
void controlLed()
{
	static unsigned long led_time = 0;
	static int color = 0;
	if ((millis() - led_time) > 250)
	{
		color = 1 - color;
		led_time = millis();
	}
	for (int i = 24; i < 36; i++)
	{
		Data2Driver.led.led[i] = color;
	}
}

#endif