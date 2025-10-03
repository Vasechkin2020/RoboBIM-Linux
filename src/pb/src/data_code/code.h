#ifndef CODE_H
#define CODE_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_); // Функция возращает максимальный размер из 2 структур
void set_PIN_Led();											   // Настройка светодиодов
void Led_Blink(int led_, unsigned long time_);				   // Функция мигания светодиодом в осномном цикле что программа не зависла и работает
void init_SPI(int channel_, int speed_);					   // Инициализация канала шины SPI
void init_Gpio();											   // Настройка пинов
void setModeModul();										   // Установка режима работы - колибровки модуля на основании переменной из лаунч файла
void readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

void callback_ControlDriver(const pb_msgs::Struct_Data2Driver &msg); // Обратный вызов при опросе топика Driver
void callback_ControlModul(const pb_msgs::Struct_Data2Modul &msg);	 // Обратный вызов при опросе топика Modul
void callback_ControlPrint(const pb_msgs::Struct_Data2Print &msg);	 // Обратный вызов при опросе топика Print
void callback_Joy(sensor_msgs::Joy msg);							 // Функция обраьтного вызова по подпичке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
void controlAcc(SControl dreamSpeed_);								 // Функция контроля ускорения

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

// Настройка пинов
void init_Gpio()
{
	int rez = wiringPiSetup(); // Инициализация библиотеки
	// //rez = wiringPiSetupGpio(); // При такой инициализациипины имеют другие номера, как изначально в распбери ПИ.
	pinMode(PIN_SPI_MODUL, OUTPUT);	 //
	pinMode(PIN_SPI_DRIVER, OUTPUT); //
	pinMode(PIN_SPI_PRINT, OUTPUT);	 //

	digitalWrite(PIN_SPI_MODUL, 1);
	digitalWrite(PIN_SPI_DRIVER, 1);
	digitalWrite(PIN_SPI_PRINT, 1);

	// pinMode(PIN_MODUL_MOSI_2G, OUTPUT);  //
	// digitalWrite(PIN_MODUL_MOSI_2G, 1);
	// pinMode(PIN_MODUL_MISO_3G, OUTPUT);  //
	// digitalWrite(PIN_MODUL_MISO_3G, 1);
	// pinMode(PIN_MODUL_CLK_4G, OUTPUT);  //
	// digitalWrite(PIN_MODUL_CLK_4G, 1);

	// pinMode(PIN_PRINT_MOSI_1G, OUTPUT);  //
	// digitalWrite(PIN_PRINT_MOSI_1G, 1);
	// pinMode(PIN_PRINT_MISO_4G, OUTPUT);  //
	// digitalWrite(PIN_PRINT_MISO_4G, 1);
	// pinMode(PIN_PRINT_CLK_3G, OUTPUT);  //
	// digitalWrite(PIN_PRINT_CLK_3G, 1);
}

// Инициализация канала шины SPI
void init_SPI(int channel_, int speed_)
{
	uint8_t errSpi = 0; // Ошибка при инициализации шины SPI
	ROS_INFO("Init SPI start... ");

	if ((errSpi = wiringPiSPISetup(channel_, speed_)) < 0) // Инициализация канало 0 это чип селект 0
	{
		ROS_ERROR("%s Can't open the SPI bus 0: %s\n", NN, strerror(errno));
		ROS_ERROR("%s errSpi: %s\n", NN, errSpi);
		exit(EXIT_FAILURE);
	}
	else
	{
		ROS_INFO("SPI ok!");
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
	// printf("dreamSpeed_ % .3f % .3f accel= % .5f dt= % .5f", dreamSpeed_.speedL, dreamSpeed_.speedR, accel, dt);
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
	// printf("g_factSpeed % f : % f : acc= % f \n ", g_factSpeed.speedL, g_factSpeed.speedR);
	// printf(" |g_factSpeed % .3f % .3f \n", g_factSpeed.speedL, g_factSpeed.speedR);
}
// Конвертация скорости из метров в секунду в обороты в секунду для передачи на нижний уровень
SControl speedToRps(SControl speed_)
{
	speed_.speedL = speed_.speedL / PERIMETR * KOEF_ODOM; // Делим скорость вметрах на периметр колеса и умножаем на дополнительный коеффициент который вручную подобрал что-бы одометрия соответствовала реальности
	speed_.speedR = speed_.speedR / PERIMETR * KOEF_ODOM; // Делим скорость вметрах на периметр колеса и умножаем на дополнительный коеффициент который вручную подобрал что-бы одометрия соответствовала реальности
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
// Установка режима работы - колибровки модуля на основании переменной из лаунч файла
void setModeModul()
{
	switch (modeModul)
	{
	case 0:
		// printf("modeModul = 0 \n");
		break;
	case 1:
		// printf("modeModul = 1 \n");
		Data2Modul.controlMotor.mode = 1;		// Ручной вариант проверка
		Data2Modul.controlLaser.mode = 1;		// Ручной вариант проверка
		Data2Modul.controlMotor.angle[0] = 45;	//
		Data2Modul.controlMotor.angle[1] = 135; //
		Data2Modul.controlMotor.angle[2] = 45;	//
		Data2Modul.controlMotor.angle[3] = 135; //
		Data2Modul.controlMotor.numPillar[0] = 0;
		Data2Modul.controlMotor.numPillar[1] = 1;
		Data2Modul.controlMotor.numPillar[2] = 2;
		Data2Modul.controlMotor.numPillar[3] = 3;
		break;
	case 2:
		// printf("modeModul = 2 \n");
		Data2Modul.controlMotor.mode = 1;		// Ручной вариант проверка
		Data2Modul.controlLaser.mode = 1;		// Ручной вариант проверка
		Data2Modul.controlMotor.angle[0] = 135; //
		Data2Modul.controlMotor.angle[1] = 45;	//
		Data2Modul.controlMotor.angle[2] = 135; //
		Data2Modul.controlMotor.angle[3] = 45;	//
		Data2Modul.controlMotor.numPillar[0] = 0;
		Data2Modul.controlMotor.numPillar[1] = 1;
		Data2Modul.controlMotor.numPillar[2] = 2;
		Data2Modul.controlMotor.numPillar[3] = 3;
		break;
	case 3:
		// printf("modeModul = 3 \n");
		Data2Modul.controlMotor.mode = 1;		  // Ручной вариант проверка
		Data2Modul.controlLaser.mode = 1;		  // Ручной вариант проверка
		Data2Modul.controlMotor.angle[0] = 67.6;  //
		Data2Modul.controlMotor.angle[1] = 34.6;  //
		Data2Modul.controlMotor.angle[2] = 143.6; //
		Data2Modul.controlMotor.angle[3] = 105.7; //
		Data2Modul.controlMotor.numPillar[0] = 0;
		Data2Modul.controlMotor.numPillar[1] = 1;
		Data2Modul.controlMotor.numPillar[2] = 2;
		Data2Modul.controlMotor.numPillar[3] = 3;
		break;
	case 4:
		// printf("modeModul = 4 \n");
		Data2Modul.controlMotor.mode = 1;		  // Ручной вариант проверка
		Data2Modul.controlLaser.mode = 1;		  // Ручной вариант проверка
		Data2Modul.controlMotor.angle[0] = 74.13;  //
		Data2Modul.controlMotor.angle[1] = 78.1;  //
		Data2Modul.controlMotor.angle[2] = 122.3; //
		Data2Modul.controlMotor.angle[3] = 108.81; //
		Data2Modul.controlMotor.numPillar[0] = 0;
		Data2Modul.controlMotor.numPillar[1] = 1;
		Data2Modul.controlMotor.numPillar[2] = 2;
		Data2Modul.controlMotor.numPillar[3] = 3;
		break;
	}

	// Data2Modul.controlMotor.mode = 1; // Ручной вариант проверка
	// Data2Modul.controlLaser.mode = 1; // Ручной вариант проверка

	// Data2Modul.controlMotor.angle[0] = 67.6;  //
	// Data2Modul.controlMotor.angle[1] = 34.6;  //
	// Data2Modul.controlMotor.angle[2] = 143.6; //
	// Data2Modul.controlMotor.angle[3] = 105.7; //

	// Data2Modul.controlMotor.angle[1] = 42.5;     //

	// Data2Modul.controlMotor.angle[0] = 45;  //
	// Data2Modul.controlMotor.angle[1] = 135; //
	// Data2Modul.controlMotor.angle[2] = 45;  //
	// Data2Modul.controlMotor.angle[3] = 135; //

	// Data2Modul.controlMotor.angle[0] = 135; //
	// Data2Modul.controlMotor.angle[1] = 45; //
	// Data2Modul.controlMotor.angle[2] = 135; //
	// Data2Modul.controlMotor.angle[3] = 45; //
}

void readParam() // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
{
	ros::NodeHandle nh_private("~");
    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
    nh_private.getParam("laser0", offSetLaser[0]);
    nh_private.getParam("laser1", offSetLaser[1]);
    nh_private.getParam("laser2", offSetLaser[2]);
    nh_private.getParam("laser3", offSetLaser[3]);

    nh_private.getParam("laserL", offSetLaserL);
    nh_private.getParam("uzi", offSetUzi);
    nh_private.getParam("laserR", offSetLaserR);

    nh_private.getParam("modeModul", modeModul);

    ROS_INFO("--- Start node with parametrs:");
    ROS_INFO("offSetLaser0 = %.3f offSetLaser1 = %.3f offSetLaser2 = %.3f offSetLaser3 = %.3f",offSetLaser[0],offSetLaser[1],offSetLaser[2],offSetLaser[3]);
    ROS_INFO("offSetLaserL = %.3f offSetLaserR = %.3f",offSetLaserL,offSetLaserR);
    ROS_INFO("offSetUZI = %.3f",offSetUzi);
    ROS_INFO("modeModul = %i",modeModul);
    ROS_INFO("---");
}
#endif