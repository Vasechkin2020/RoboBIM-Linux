#ifndef CODE_H
#define CODE_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_); // Функция возращает максимальный размер из 2 структур
void set_PIN_Led();											   // Настройка светодиодов
void Led_Blink(int led_, unsigned long time_);				   // Функция мигания светодиодом в осномном цикле что программа не зависла и работает
void init_SPI(int channel_, int speed_);					   // Инициализация канала шины SPI
void init_Gpio();											   // Настройка пинов
void setModeModul();										   // Установка режима работы - колибровки модуля на основании переменной из лаунч файла
void readParam();											   // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

void callback_ControlDriver(const pb_msgs::Struct_Data2Driver &msg); // Обратный вызов при опросе топика Driver
void callback_ControlModul(const pb_msgs::Struct_Data2Modul &msg);	 // Обратный вызов при опросе топика Modul
void callback_ControlPrint(const pb_msgs::Struct_Data2Print &msg);	 // Обратный вызов при опросе топика Print

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
	logi.log_b("+++ Init SPI start... \n");

	if ((errSpi = wiringPiSPISetup(channel_, speed_)) < 0) // Инициализация канало 0 это чип селект 0
	{
		logi.log_r("    Can't open the SPI bus 0: %s\n", NN, strerror(errno));
		logi.log_r("    errSpi: %s\n", errSpi);
		exit(EXIT_FAILURE);
	}
	else
	{
		logi.log("    SPI ok!\n");
	}
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
		Data2Modul.controlMotor.mode = 1;		   // Ручной вариант проверка
		Data2Modul.controlLaser.mode = 1;		   // Ручной вариант проверка
		Data2Modul.controlMotor.angle[0] = 74.13;  //
		Data2Modul.controlMotor.angle[1] = 78.1;   //
		Data2Modul.controlMotor.angle[2] = 122.3;  //
		Data2Modul.controlMotor.angle[3] = 108.81; //
		Data2Modul.controlMotor.numPillar[0] = 0;
		Data2Modul.controlMotor.numPillar[1] = 1;
		Data2Modul.controlMotor.numPillar[2] = 2;
		Data2Modul.controlMotor.numPillar[3] = 3;
		break;
	case 5: // ВСЕ ОТКЛЮЧЕНО
		// printf("modeModul = 5 \n");
		Data2Modul.controlMotor.mode = 0;		// Ручной вариант проверка
		Data2Modul.controlLaser.mode = 0;		// Ручной вариант проверка
		Data2Modul.controlMotor.angle[0] = 0.0; //
		Data2Modul.controlMotor.angle[1] = 0.0; //
		Data2Modul.controlMotor.angle[2] = 0.0; //
		Data2Modul.controlMotor.angle[3] = 0.0; //
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
	ros::NodeHandle nh_global; // <--- Используется для доступа к /pb_config/ // Создаем ГЛОБАЛЬНЫЙ обработчик, который ищет параметры, начиная с корня (/).

	// printf("\n--- Считывание смещений датчиков ---\n"); // Разделитель секции...
	nh_global.param<double>("/pb_config/uzi_bias", offSetUzi, -0.01);
	nh_global.param<double>("/pb_config/laser_L_bias", offSetLaserL, -0.01);
	nh_global.param<double>("/pb_config/laser_R_bias", offSetLaserR, -0.01);

	nh_global.param<int>("/pb_config/modul/mode", modeModul, 5 );

	nh_global.param<int>("/pb_config/unit_driver", unitDriver, 0);
	nh_global.param<int>("/pb_config/unit_modul", unitModul, 0);
	nh_global.param<int>("/pb_config/unit_print", unitPrint, 0);

	logi.log_b("+++ =========================================");
	logi.log("    Start node with parametrs:\n");
	logi.log("    offSetLaserL = %+8.3f offSetLaserR = %+8.3f \n", offSetLaserL, offSetLaserR);
	logi.log("    offSetUZI = %+8.3f \n", offSetUzi);
	logi.log("    unit_driver = %i unit_modul = %i unit_print = %i \n", unitDriver, unitModul, unitPrint);
	logi.log("    modeModul = %i \n", modeModul);
	logi.log_b("+++ =========================================");
}
#endif