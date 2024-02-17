#ifndef CODE_H
#define CODE_H

//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************

uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_); // Функция возращает максимальный размер из 2 структур
void set_PIN_Led();											   // Настройка светодиодов
void Led_Blink(int led_, unsigned long time_);				   // Функция мигания светодиодом в осномном цикле что программа не зависла и работает
void init_SPI(int channel_, int speed_);					   // Инициализация канала шины SPI
void callback_ControlDriver(const data::SControlDriver &msg);  // Обратный вызов при опросе топика Head2Data
void callback_ControlModul(const data::SControlModul &msg);	   // Обратный вызов при опросе топика Angle
void callback_Joy(sensor_msgs::Joy msg);					   // Функция обраьтного вызова по подпичке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
void calculateOdometryFromEncoder();						   // Обработка пришедших данных.Обсчитываем одометрию по энкодеру
void calculateOdometryFromMpu();							   // Обработка пришедших данных.Обсчитываем одометрию по энкодеру
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
// Обратный вызов при опросе топика Head2Data
void callback_ControlDriver(const data::SControlDriver &msg)
{
	flag_msgControlDriver = true;
	msg_ControlDriver = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							 // ROS_INFO("message_callback_Command.");
}
// Обратный вызов при опросе топика Angle
void callback_ControlModul(const data::SControlModul &msg)
{
	msg_ControlModul = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							// ROS_INFO("message_callback_Command.");
}

// Обработка пришедших данных.Обсчитываем одометрию по энкодеру
void calculateOdometryFromEncoder()
{
	double radius = 0;
	double theta = 0;
	double lenArc = 0;
	static SPose pose;
	STwist twist;

	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;

	// int start = clock(); // засекаем время старта
	// int end = clock(); // засекаем время окончания
	// int t = (end - start) / CLOCKS_PER_SEC;// команда CLOCKS_PER_SEC нужна для перевода результата функции clock в секунды

	double speedL = PERIMETR * Driver2Data.motor.rpsEncodL; // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
	double speedR = PERIMETR * Driver2Data.motor.rpsEncodR; // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
	printf("speed= %.4f speedR = %.4f / ", speedL, speedR);
	double sumSpeed = speedL + speedR;
	double deltaSpeed = speedL - speedR;
	double speed = (speedR + speedL) / 2.0;	 // Находим скорость всего обьекта.
	double w = deltaSpeed / DISTANCE_WHEELS; // Находим уголовую скорость движения по радиусу. Плюс по часовой минус против часовой
											 // ROS_INFO("speedL= %.4f speedR= %.4f speed= %.4f w = %.4f ///  ", speedL, speedR, speed, RAD2DEG(w));
	if (speedL == 0 && speedR == 0)			 // Стоим на месте. Скорости равны нулю
	{
		radius = 0;
		speed = 0;
		theta = 0;
		// ROS_INFO("0 STOIM NA MESTE radius = %.4f theta gradus = %.4f ", radius, RAD2DEG(theta));
	}
	else
	{
		if (abs(sumSpeed) < 0.01 && speedL != 0 && speedR != 0) // Если сумма скоростей очень маленькая или ноль значит крутимся на месте или стоим на месте и тогда совсем иной расчет чем если движемся// Крутимся на месте
		{
			radius = 0.5 * DISTANCE_WHEELS; // Радиус в таком случае это половина между колесами
			if (speedL > speedR)			// Значит крутимся по часовой и знак угловой скорсти плюс
			{
				lenArc = speedL; // меняем скорость со среднй на скорость одного колеса, наружнего // Находим путь какой проехали. Это длинна дуги.
			}
			else
			{
				lenArc = -speedR; //  Значит крутимся против часовой и знак минус будет у угловой скорости // Находим путь какой проехали. Это длинна дуги.
			}
			speed = 0;				 // Обнуляем скорость чтобы дальше позиция не сдвигалась, мы же на месте.
			theta = lenArc / radius; // Отношение улинны дуги окружночти к радиусу дает угол в радианах. Нахождение центрального угла по дуге и радиусу.
									 // ROS_INFO("1 KRUTIMSA NA MESTE radius = %.4f theta gradus = %.4f lenArc = %.4f speedL = %.4f speedR = %.4f ", radius, RAD2DEG(theta), lenArc, speedL, speedR);
		}
		else // Тут нормальный расчет что мы движемся или по прямой или по радиусу
		{
			lenArc = speed;				// Находим путь какой проехали за время в течении которого энкодер собирал данные. Это длинна дуги.
			if (abs(deltaSpeed) < 0.01) // Если раздница скоростей незначительна то считаем что едем прямо вперед или назад
			{
				radius = 0; // Едем прямо или назад и все по нулям
				theta = 0;	// Если едем прямо то угол поворота отклонения от оси равен 0
						   // ROS_INFO("2 EDEM PRIAMO radius = %.4f theta gradus = %.4f ", radius, RAD2DEG(theta));
			}
			else // Едем по радиусу и надо все считать
			{
				radius = (0.5 * DISTANCE_WHEELS) * (sumSpeed / deltaSpeed); // Находим радиус движения
				theta = lenArc / radius;									// Отношение улинны дуги окружночти к радиусу дает угол в радианах. Нахождение центрального угла по дуге и радиусу.
																			// ROS_INFO("3 EDEM RADIUS radius = %.4f theta gradus = %.4f ", radius, RAD2DEG(theta));
			}
		}
	}

	// ROS_INFO("theta = %.3f", theta);
	// Находим линейные скорости из моего вектора скорости. Я задаю общую скорость движения (длинна вектора), ее надо разложить на проекции по осям x y. Это будут линейные скорости
	// speed = 0.26;
	twist.vth = theta * dt;		   // Угловая скорость в радианах за интрвал измерения. Это угол на который провернулись на интервал Знак минус так как у меня Вращение ПЛЮС по часовой, а у всех против часовой
	twist.vx = speed * sin(twist.vth); // Проекция моей скорости на ось Y получаем линейную скорость по оси
	twist.vy = speed * cos(twist.vth); // Проекция моей скорости на ось X получаем линейную скорость по оси
	// printf("speed= %.4f twist.vth = %.4f / sin(twist.vth )= %.4f cos(twist.vth ) = %.4f / ", speed, RAD2DEG(twist.vth), sin(twist.vth ), cos(twist.vth ));
	printf("speed= %.4f twist.vth = %.4f / ", speed, RAD2DEG(twist.vth));
	// ROS_INFO("SPEED= %.3f Linear speed twist.vx = %.3f twist.vy = %.3f Angular speed twist.vth = %.3f for sec.", speed, twist.vx, twist.vy, RAD2DEG(twist.vth));
	//==============================================================================================================================
	double vx = twist.vx * dt; // Находим проекции скорсти на оси за интревал времени
	double vy = twist.vy * dt;
	printf("vx= %.8f vy= %.8f / ", vx, vy);

	// printf("DO pose.x= %.3f pose.y= %.3f pose.th= %.3f / ", pose.x, pose.y, RAD2DEG(pose.th));
	//  Находим смещние по осям матрица координаты точки из локальной системы координат в глобальной
	printf(" pose.th= %.6f / vx*cos(pose.th)= %.6f vy*sin(pose.th)= %.6f / ",pose.th, vx * cos(pose.th), vy * sin(pose.th));
	double delta_x = vx * cos(pose.th) - vy * sin(pose.th);
	double delta_y = vx * sin(pose.th) + vy * cos(pose.th);
	double delta_th = twist.vth;
	// printf(" / pose.th = %.6f cos= %.6f sin= %.6f / ",pose.th, cos(pose.th), sin(pose.th));

	printf("delta_x= %.6f delta_y= %.6f delta_th= %.6f ", delta_x, delta_y, delta_th);

	// Меняем координаты и угол на основе вычислений
	pose.x += delta_x;	 // Вычисляем координаты
	pose.y += delta_y;	 // Вычисляем координаты
	pose.th += delta_th; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот в ГРАДУСАХ
	(pose.th > (2 * M_PI)) ? (pose.th -= (2 * M_PI)) : (pose.th = pose.th);
	(pose.th < 0) ? (pose.th += (2 * M_PI)) : (pose.th = pose.th);
	printf(" /// POSLE pose.x= %.6f pose.y= %.6f pose.th= %.6f \n", pose.x, pose.y, pose.th);

	encoder.pose = pose;
	encoder.twist = twist;

	//**************************************************************** СТАРЫЙ ПРИМЕР РАСЧЕТА С НИЖНЕГО УРОВНЯ ******************************************
	// float angle_pov_sum = 0;
	// printf("odom_impuls_L= %i rps_L= %f \n", odom_impuls_L, rps_L);
	// printf("odom_impuls_R= %i rps_R= %f \n", odom_impuls_R, rps_R);
	// printf("dt= %f odom_impuls_L= %i impuls for sec= %f rps= %f rps2= %f \n", dt, odom_impuls_L, odom_impuls_L /dt, (gradusL /dt) / 360, rps_L);
	// printf("dt= %f odom_impuls_R= %i impuls for sec= %f rps= %f rps2= %f \n", dt, odom_impuls_R, odom_impuls_R /dt, (gradusR /dt) / 360, rps_R);

	// float way_L = ((PI * (RADIUS / 1000.0)) / 180.0) * gradusL; // По формуле длинны дуги находим пройденный путь колесом Радиус приводим в метры так как он указан в мм Это и есть скорость за секунду
	// float way_R = ((PI * (RADIUS / 1000.0)) / 180.0) * gradusR; // По формуле длинны дуги находим пройденный путь колесом
	// float way_C = (way_L + way_R) / 2.0;                        // Путь средней точки Center
	// motorLeft.way += way_L;                                     // суммируем
	// motorRight.way += way_R;                                    // суммируем
	// car.way += way_C; // суммируем

	// car.speedEncod = way_C / dt; // Скорость с какой фактически едем это длинна дуги за секунду

	// // Радиус фактический высчитываем из раздницы скоростей колес https://present5.com/presentation/111019976_277103894/image-5.jpg
	// float VV = (way_L + way_R); // Путь это и есть скорость за время
	// float V_V = (way_L - way_R);

	// if (V_V == 0 && V_V < 0.001) // Чтобы не делить на ноль когда скорости равны или очень маленькая разница
	// {
	//     V_V = 0.001;
	// }
	// car.radiusEncod = 0.5 * DISTANCE_WHEELS * (VV / V_V); //
	// if (car.radiusEncod > 0)
	// {
	//     car.radiusEncod = car.radiusEncod + DISTANCE_WHEELS / 2;
	// }
	// else
	// {
	//     car.radiusEncod = car.radiusEncod - DISTANCE_WHEELS / 2;
	// }
	// if (abs(car.radiusEncod) > 5) // Если едем прямо то большой радиус и его превращаем в ноль, чтобы ясно было что едем прямо
	// {
	//     car.radiusEncod = 0;
	// }

	//****************************************** РАСЧЕТ ОДОМЕТРИИ **************************************************************************************************************************
	// printf("dt= %f odom_way_L= %f  odom_way_R= %f odom_way_C= %f \n", dt, motorLeft.way, motorRight.way, car.way);

	// float angle_pov;
	// Находим угловую скорость вращения (поворота) робота
	// if (car.radiusSet == 0) // Если двигаемся прямо то
	// {
	//     angle_pov = 0; // Угол поворота равен нулю
	// }
	// else
	// {
	//     angle_pov = way_C / -car.radiusSet; // Делим пройденный по дуге путь на радиус движения получаем угол поворота в радианах
	//     angle_pov_sum += angle_pov * 180.0 / PI;
	// }
	// Находим угловую скорость поворота в радианах в секунду
	// odom_enc.vel_th = angle_pov / dt; //  Вычисляем радианы в секунду получаем угловую скорость
	// //---------------------------------------
	// // Находим линейные скорости из моего вектора скорости. Я задаю общую скорость движения (длинна вектора), ее надо разложить на проекции по осям x y. Это будут линейные скорости
	// odom_enc.vel_x = car.speedEncod * cos(odom_enc.th); // Проекция моей скорости на ось X получаем линейную скорость по оси
	// odom_enc.vel_y = car.speedEncod * sin(odom_enc.th); // Проекция моей скорости на ось Y получаем линейную скорость по оси

	// printf("car.speedEncod= %.2f  g_radius= %.2f angle_pov= %f  angle_pov_sum= %f vel_x= %.2f  vel_y= %.2f  vel_th= %f ", car.speedEncod, g_radius, angle_pov, angle_pov_sum, g_odom_enc.vel_x, g_odom_enc.vel_y, g_odom_enc.vel_th);

	// Находим смещение по глобальной карте по осям Х и Y c помощью матрицы вращения. Она вычисляет смешения по осям на нужный угол
	// float delta_x = (odom_enc.vel_x * cos(odom_enc.th) - odom_enc.vel_y * sin(odom_enc.th)) * dt;
	// float delta_y = (odom_enc.vel_x * sin(odom_enc.th) + odom_enc.vel_y * cos(odom_enc.th)) * dt;

	// Меняем координаты и угол на основе вычислений
	// odom_enc.x += delta_x;   // Вычисляем координаты
	// odom_enc.y += delta_y;   // Вычисляем координаты
	// odom_enc.th += delta_th; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот

	// printf("x= %.2f y= %.2f th= %.3f  time= %u \n", g_odom_enc.x, g_odom_enc.y, g_odom_enc.th, millis());
}
// Обработка пришедших данных.Обсчитываем одометрию по энкодеру
void calculateOdometryFromMpu()
{
	static bool firstStart = 1;
	static SMpu mpu_pred; // Предыдущее значение mpu

	// Замеряем интервалы по времени между запросами данных
	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;

	double delta_th = (Driver2Data.bno055.angleEuler.z - mpu_pred.angleEuler.z); // считаем величину изменения угла
	// (delta_th > 180) ? (delta_th = delta_th - 360) : (delta_th = delta_th);		 // Если
	// (delta_th < -180) ? (delta_th = delta_th + 360) : (delta_th = delta_th);	 // Если

	mpu_pred = Driver2Data.bno055; // Запоминаем для следующего раза

	/* Примечание. Сигнал линейного ускорения обычно не может быть интегрирован для восстановления скорости или дважды
	интегрирован для восстановления положения. Ошибка обычно становится больше сигнала менее чем за 1 секунду, если для
	компенсации этой ошибки интегрирования не используются другие источники датчиков.*/
	mpu.twist.vx += Driver2Data.bno055.linear.x * dt;
	mpu.twist.vy += Driver2Data.bno055.linear.y * dt;
	mpu.twist.vth = delta_th / dt; // Скорость изменнения угла в градусы за секунду. Угловая скорость вращения робота

	// if (firstStart) // При первом запуске функции обнуляем первое получение угловых скоростей, так как считает неправильно так как не знает предыдущего расчета
	// {
	// 	mpu.twist.vx = 0;
	// 	mpu.twist.vy = 0;
	// 	mpu.twist.vth = 0;
	// 	mpu.pose.x = 0;
	// 	mpu.pose.y = 0;
	// 	mpu.pose.th = Driver2Data.bno055.angleEuler.z;
	// 	firstStart = 0;
	// }
	// printf("twist.x= %.4f y= %.4f th= %.4f gradus ", bno055.twist.vx, bno055.twist.vy, bno055.twist.vth);

	// compute odometry in a typical way given the twistocities of the robot
	// ПОСКОЛЬКУ ЛИНЕЙНУЮ СКРОСТЬ ДАТЧИК ВЫДАЕТ ОТНОСИТЕЛЬНО СВОЕЙ СИСТЕМЫ ОТСЧЕТА, А МЫ ДВИЖЕМСЯ В СИСТЕМЕ ОТСЧЕТА ОТНОСИТЕЛЬНО СТАРТОВОЙ ТО ВСЕГДА НУЖНО ПРЕОБРАЗОВЫВАТЬ СИСТЕМОЙ ПОВОРОТА МАТРИЦЕЙ ПОВОРОТА В СТАРТОВУЮ СИСТЕМУ КООРДИНАТ
	double thRad = DEG2RAD(mpu.pose.th); // Превращаем в радианы для синусови косинусов
	double delta_x = (mpu.twist.vx * cos(thRad) - mpu.twist.vy * sin(thRad)) * dt;
	double delta_y = (mpu.twist.vx * sin(thRad) + mpu.twist.vy * cos(thRad)) * dt;
	delta_th = mpu.twist.vth * dt;

	// Меняем координаты и угол на основе вычислений
	mpu.pose.x += delta_x;	 // Вычисляем координаты
	mpu.pose.y += delta_y;	 // Вычисляем координаты
	mpu.pose.th += delta_th; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот относительно момента включения В РАДИАНАХ
	(mpu.pose.th > 360) ? (mpu.pose.th -= 360) : (mpu.pose.th = mpu.pose.th);
	(mpu.pose.th < 0) ? (mpu.pose.th += 360) : (mpu.pose.th = mpu.pose.th);

	// ПРИМЕР ИЗ ПОМОЩИ ПО РОС ОДОМЕТРИИ КАК ПРАВИЛЬНО СЧИТАТЬ
	//  double dt = (current_time - last_time).toSec();
	//  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	//  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
	//  double delta_th = vth * dt;
	//  x += delta_x;
	//  y += delta_y;
	//  th += delta_th;

	// printf(" /  pose.x= %.4f y= %.4f th= %.4f gradus \n", bno055.pose.x, bno055.pose.y, bno055.pose.th);

	// НАСТРОЙКИ ПОД МОЕ ПОЛОЖЕНИЕ ДАТЧИКА !!!!!!!!!!!!!!!!!!!!!!!!!!!
	// BNO055_EulerAngles.x = -BNO055_EulerAngles.x;
	// if (BNO055_EulerAngles.y > 0)
	//{
	//	BNO055_EulerAngles.y = BNO055_EulerAngles.y - 180;
	//}
	// else
	//{
	//	BNO055_EulerAngles.y = BNO055_EulerAngles.y + 180;
	//}
	// BNO055_EulerAngles.z = BNO055_EulerAngles.z - 60 + 180;
	// if (BNO055_EulerAngles.z < 0) { BNO055_EulerAngles.z = BNO055_EulerAngles.z + 360; }
	// if (BNO055_EulerAngles.z > 360) { BNO055_EulerAngles.z = BNO055_EulerAngles.z - 360; }
}

#endif