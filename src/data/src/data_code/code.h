#ifndef CODE_H
#define CODE_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
void processingSPI();																						  // Сбор данных по результатам обмена по шине SPI по обоим контроллерам
uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_); // Функция возращает максимальный размер из 2 структур
void set_PIN_Led();											   // Настройка светодиодов
void Led_Blink(int led_, unsigned long time_);				   // Функция мигания светодиодом в осномном цикле что программа не зависла и работает
void init_SPI(int channel_, int speed_);																	// Инициализация канала шины SPI
void callback_ControlDriver(const data::SControlDriver &msg); // Обратный вызов при опросе топика Head2Data
void callback_ControlModul(const data::SControlModul &msg); // Обратный вызов при опросе топика Angle
// void setOdomToTf(ros::NodeHandle nh_, tf::TransformBroadcaster odom_broadcaster_, ros::Time current_time_); // Функция записи в нужные места данных одометрии в tf и в odom
// void transOdom();											  // Перенес то что было на главном файле

// **********************************************************************************

// Функция возращает максимальный размер из 2 структур
uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_)
{
	uint16_t ret = 0;
	stru1_ > stru2_ ? ret = stru1_ : ret = stru2_;
	return ret += 1; // + 1 байт Для контрольной суммы
}
// Сбор данных по результатам обмена по шине SPI по обоим контроллерам
void processingSPI()
{
	spi_msg.ModulData.all = Modul2Data.spi.all;// Собираем для публикации данные о результатах обмена полученных из Modul о том как он принял по SPI данные отправленные Data
	spi_msg.ModulData.bed = Modul2Data.spi.bed;

	spi_msg.DataModul.all = data_modul_all;// Собираем для публикации данные о результатах обмена из Data о том как он принял по SPI данные отправленные Modul
	spi_msg.DataModul.bed = data_modul_bed;
//-------------------------------------------------------------------------------------
	spi_msg.DriverData.all = Driver2Data.spi.all;// Собираем для публикации данные о результатах обмена полученных из Modul о том как он принял по SPI данные отправленные Data
	spi_msg.DriverData.bed = Driver2Data.spi.bed;

	spi_msg.DataDriver.all = data_driver_all;// Собираем для публикации данные о результатах обмена из Data о том как он принял по SPI данные отправленные Modul
	spi_msg.DataDriver.bed = data_driver_bed;

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

// Обратный вызов при опросе топика Head2Data
void callback_ControlDriver(const data::SControlDriver &msg)
{
	msg_ControlDriver = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
						 // ROS_INFO("message_callback_Command.");
}
// Обратный вызов при опросе топика Angle
void callback_ControlModul(const data::SControlModul &msg)
{
	msg_ControlModul = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
						  // ROS_INFO("message_callback_Command.");
}



#endif