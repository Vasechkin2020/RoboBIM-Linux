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

STwistDt calcTwistFromWheel(SControl control_);						// Обработка пришедших данных.Обсчитываем одометрию по энкодеру
STwistDt calcTwistFromMpu(SMpu mpu_, float koef_);					// Обработка пришедших данных.Обсчитываем угловые скорости по энкодеру
STwistDt calcTwistUnited(STwistDt wheelTwist_, STwistDt mpuTwist_); // Функция комплементации угловых скоростей полученных с колес и с датчика MPU и угла поворота

void calcNewOdom(SOdom &odom_, STwistDt data_); // На вход подаются старая одометрия и новые угловая угловая скорость. Возвращается новая позиция по данным угловым скоростям // Обработка пришедших данных.Обсчитываем одометрию по энкодеру

// void calculateOdometryFromMpu(SMpu mpu_);					   // Обработка пришедших данных.Обсчитываем одометрию по энкодеру
void controlAcc(SControl &control_, SControl g_dreamSpeed); // Функция контроля ускорения
float autoOffsetX(float data_);								// Функция считаем скользящее среднее из 128 элементов как офсет значений при стоянии на месте
float autoOffsetY(float data_);								// Функция считаем скользящее среднее из 128 элементов как офсет значений при стоянии на месте

float filtrComplem(float koef_, float oldData_, float newData_); // функция фильтрации, берем старое значение с некоторым весом

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
	msg_ControlDriver = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							 // ROS_INFO("message_callback_Command.");
}
// Обратный вызов при опросе топика
void callback_ControlModul(const pb_msgs::Struct_Data2Modul &msg)
{
	flag_msgControlModul = true;
	msg_ControlModul = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							// ROS_INFO("message_callback_Command.");
}
// Обратный вызов при опросе топика
void callback_ControlPrint(const pb_msgs::Struct_Data2Print &msg)
{
	flag_msgControlPrint = true;
	msg_ControlPrint = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							// ROS_INFO("message_callback_Command.");
}
void initArray()
{
	for (int i = 0; i < 128; i++)
	{
		linearOffsetX[i] = 0;
		linearOffsetY[i] = 0;
	}
}

float autoOffsetX(float data_)
{
	static uint16_t i = 0;
	static uint16_t k = 0;
	static float sum = 0;
	if (k < 128)
		k++;
	else
		sum = sum - linearOffsetX[i]; // Убираем из среднего прежнее значение
	linearOffsetX[i] = data_;		  // Меняем значение в массиве
	sum = sum + linearOffsetX[i];	  // Добавляем в среднее новое значение
	i++;
	if (i >= 128)
		i = 0;
	// printf(" sumX= % .3f ", sum);
	return sum / k;
}
float autoOffsetY(float data_)
{
	static uint16_t i = 0;
	static uint16_t k = 0;
	static float sum = 0;
	if (k < 128)
		k++;
	else
		sum = sum - linearOffsetY[i]; // Убираем из среднего прежнее значение
	linearOffsetY[i] = data_;		  // Меняем значение в массиве
	sum = sum + linearOffsetY[i];	  // Добавляем в среднее новое значение
	i++;
	if (i >= 128)
		i = 0;
	// printf(" sumX= % .3f ", sum);
	return sum / k;
}

// функция фильтрации, берем старое значение с некоторым весом
float filtrComplem(float koef_, float oldData_, float newData_)
{
	return (1 - koef_) * oldData_ + (koef_ * newData_);
}

// Обработка пришедших данных.Обсчитываем одометрию по энкодеру
STwistDt calcTwistFromWheel(SControl control_)
{
	STwistDt ret;
	double radius = 0;
	double theta = 0;
	double lenArc = 0;
	static SPose pose;
	STwist twist;

	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;
	if (dt < 0.005) // При первом запуске просто выходим из функции
	{
		printf("calcTwistFromWheel dt< 0.005 !!!! \n");
		return ret;
	}
	// double speedL = PERIMETR * Driver2Data.motor.rpsEncodL; // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
	// double speedR = PERIMETR * Driver2Data.motor.rpsEncodR; // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
	double speedL = PERIMETR * control_.speedL;
	double speedR = PERIMETR * control_.speedR;
	double sumSpeed = speedL + speedR;
	double deltaSpeed = speedL - speedR;
	double speed = (speedR + speedL) / 2.0; // Находим скорость всего обьекта.
	// printf("speed= %.6f / ", speed);
	double w = deltaSpeed / DISTANCE_WHEELS; // Находим уголовую скорость движения по радиусу. Плюс по часовой минус против часовой
											 // ROS_INFO("speedL= %.4f speedR= %.4f speed= %.4f w = %.4f ///  ", speedL, speedR, speed, RAD2DEG(w));
											 // if (dt > (1.0 / RATE * 0.90))			 //
											 // {
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
	twist.vx = speed * sin(theta * dt); // Проекция моей скорости на ось Y получаем линейную скорость по оси за секунуду
	twist.vy = speed * cos(theta * dt); // Проекция моей скорости на ось X получаем линейную скорость по оси за секунуду
	twist.vth = theta;					// Угловая скорость в радианах.

	printf("% 6lu |Wheel % .3f % .3f \n", millis(), twist.vx, twist.vy);
	// printf("vy= % .4f", twist.vy);
	// printf("speed= %.4f twist.vth = %.4f / sin(twist.vth )= %.4f cos(twist.vth ) = %.4f / ", speed, RAD2DEG(twist.vth), sin(twist.vth ), cos(twist.vth ));
	// printf("speed= %.4f twist.vth = %.8f / ", speed, RAD2DEG(twist.vth));
	// ROS_INFO("SPEED= %.3f Linear speed twist.vx = %.3f twist.vy = %.3f Angular speed twist.vth = %.3f for sec.", speed, twist.vx, twist.vy, RAD2DEG(twist.vth));
	// //==============================================================================================================================
	ret.twist = twist;
	ret.dt = dt;

	// }
	return ret;
}
/*
		double vx = twist.vx * dt; // Находим проекции скорсти на оси за интревал времени это коокрдинаты нашей точки в локальной системе координат
		double vy = twist.vy * dt;
		// vx = 0.00291409;
		// vy = 0.02337962;
		// twist.vth = DEG2RAD(71.24713973);

		// printf("vx= % .3f vy= % .f vth= % .3f | ", vx, vy, twist.vth);

		// // printf("DO pose.x= %.3f pose.y= %.3f pose.th= %.3f / ", pose.x, pose.y, RAD2DEG(pose.th));
		// // Находим смещние по осям матрица координаты точки из локальной системы координат в глобальной
		// double delta_x = vx * cos(pose.th) + vy * sin(pose.th);
		// double delta_y = -vx * sin(pose.th) + vy * cos(pose.th);

		// // Меняем координаты и угол на основе вычислений
		// pose.x += delta_x; // Вычисляем координаты
		// pose.y += delta_y; // Вычисляем координаты

		SPoint pointLoc;
		pointLoc.x = twist.vx * dt;
		pointLoc.y = twist.vy * dt;
		SPoint newPoint = pointLocal2Global(pointLoc, pose);
		pose.x = newPoint.x;
		pose.y = newPoint.y;

		// printf("pose.th= %.8f cos(pose.th)= %.8f sin(pose.th)= %.8f / ", RAD2DEG(pose.th), cos(pose.th), sin(pose.th));
		// printf("vx*cos(pose.th)= %.8f vy*sin(pose.th)= %.8f =>delta_x= %.8f \n", vx * cos(pose.th), vy * sin(pose.th), delta_x);
		// printf("vx*sin(pose.th)= %.8f vy*cos(pose.th)= %.8f =>delta_y= %.8f \n", vx * sin(pose.th), vy * cos(pose.th), delta_y);

		pose.th += twist.vth * dt; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот
		(pose.th > (2 * M_PI)) ? (pose.th -= (2 * M_PI)) : (pose.th = pose.th);
		(pose.th < 0) ? (pose.th += (2 * M_PI)) : (pose.th = pose.th);

		// printf("pose.x= % .3f y= % .3f th= % .3f ", pose.x, pose.y, RAD2DEG(pose.th));

		odomWheel.pose = pose;
		odomWheel.twist = twist;
	}
}
*/

// Обработка пришедших данных.Обсчитываем угловые скорости по энкодеру
STwistDt calcTwistFromMpu(SMpu mpu_, float koef_)
{
	static STwistDt ret;
	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;
	static double predAngleZ = 0;
	if (dt < 0.005) // При первом запуске просто выходим из функции
	{
		printf("calcTwistFromMpu dt< 0.005 !!!! \n");
		predAngleZ = mpu_.angleEuler.z;
		return ret;
	}
	static float offsetX = 0;
	static float offsetY = 0;
	static float complX = 0; // Значение после комплементарного фильтра
	static float complY = 0; // Значение после комплементарного фильтра

	printf("% 6lu |Mpu   % .3f % .3f | % .3f ", millis(), mpu_.linear.x, mpu_.linear.y, dt);

	if (Data2Driver.control.speedL == 0 && Data2Driver.control.speedR == 0) // Если стоим на месте, то считаем офсет. Как только тронемся, его и будем применять до следующей остановки
	{
		offsetX = autoOffsetX(mpu_.linear.x);
		offsetY = autoOffsetY(mpu_.linear.y);
	}
	// printf(" |offset % .4f % .4f | ", offsetX, offsetY);
	printf(" |offset x= % .4f y= % .4f ", offsetX, offsetY);

	mpu_.linear.x = mpu_.linear.x - offsetX;
	mpu_.linear.y = mpu_.linear.y - offsetY;

	printf(" |Average  % .3f % .3f ", mpu_.linear.x, mpu_.linear.y);
	// printf(" |Average % .3f | ", mpu_.linear.y);

	complX = filtrComplem(koef_, complX, mpu_.linear.x);
	complY = filtrComplem(koef_, complY, mpu_.linear.y);
	printf(" |Compl x= % .3f y= % .3f ", complX, complY);

	/* Примечание. Сигнал линейного ускорения обычно не может быть интегрирован для восстановления скорости или дважды интегрирован для восстановления положения.
	Ошибка обычно становится больше сигнала менее чем за 1 секунду, если для компенсации этой ошибки интегрирования не используются другие источники датчиков.*/
	ret.twist.vx += complX * dt; // Линейное ускорение по оси метры за секунуду умножаем на интервал, получаем ускорение за интервал и суммируем в скорость линейную по оси
	ret.twist.vy += complY * dt; // Линейное ускорение по оси метры за секунуду

	double delta_th = (mpu_.angleEuler.z - predAngleZ); // считаем величину изменения угла, тут она в градусах
	predAngleZ = mpu_.angleEuler.z;
	if (delta_th > 180)
		(delta_th = delta_th - 360); // Если
	if (delta_th < -180)
		(delta_th = delta_th + 360);		// Если
	ret.twist.vth = DEG2RAD(delta_th) / dt; // превращаем в радианы в секунды Угловая скорость вращения
	ret.dt = dt;

	if (Data2Driver.control.speedL == 0 && Data2Driver.control.speedR == 0) // Если обе скорости равны нулю то обнуляем расчеты по mpu. Так как стоим на мете и никаких линейных скоростей быть не может. Стои на месте.
	{
		ret.twist.vx = ret.twist.vy = 0;
	}

	// printf(" ||| LinearSpeed vx= % .3f vy=  % .3f vth= % .6f | ", ret.twist.vx, ret.twist.vy, ret.twist.vth);
	// printf(" |Vel= % .3f % .3f % .3f\n", ret.twist.vx, ret.twist.vy, ret.twist.vth);
	printf(" | Twist vx = % .3f vy= % .3f th= % .3f || \n", ret.twist.vx, ret.twist.vy, RAD2DEG(ret.twist.vth));
	return ret;
}
// Функция комплементации угловых скоростей полученных с колес и с датчика MPU и угла поворота
STwistDt calcTwistUnited(STwistDt wheelTwist_, STwistDt mpuTwist_)
{
	STwistDt ret;
	if (wheelTwist_.dt < 0.005) // При первом запуске просто выходим из функции
	{
		printf("calcTwistUnited dt< 0.005 !!!! \n");
		return ret;
	}
	float koef = 0.5;	// Коефициант по умолчанию.Пополам.
	float koefTh = 0.5; // Коефициант по умолчанию.Пополам.

	ret.dt = wheelTwist_.dt * 0.5 + mpuTwist_.dt * 0.5;

	ret.twist.vx = wheelTwist_.twist.vx * (1 - koef) + mpuTwist_.twist.vx * koef;
	ret.twist.vy = wheelTwist_.twist.vy * (1 - koef) + mpuTwist_.twist.vy * koef;
	ret.twist.vth = wheelTwist_.twist.vth * (1 - koefTh) + mpuTwist_.twist.vth * koefTh;
	printf("% 6lu |United Wheel | % .3f % .3f | % .3f % .3f || % .3f % .3f || \n", millis(), wheelTwist_.twist.vx, wheelTwist_.twist.vy, mpuTwist_.twist.vx, mpuTwist_.twist.vy, ret.twist.vx, ret.twist.vy, ret.twist.vth);

	return ret;
}
// Обработка пришедших данных.Обсчитываем одометрию по энкодеру
void calcNewOdom(SOdom &odom_, STwistDt data_) // На вход подаются старая одометрия и новые угловая угловая скорость. Возвращается новая позиция по данным угловым скоростям
{
	if (data_.dt < 0.005) // Если пришли данные с нулевой дельтой то сразу выходим и ничего не считаем
	{
		printf("calcNewOdom dt< 0.005 !!!! \n");
		return;
	}
	odom_.twist = data_.twist; // Ничего не меняем в угловой скорости
	SPoint pointLoc;
	pointLoc.x = data_.twist.vx * data_.dt; // Находим проекции скорсти на оси за интревал времени это коокрдинаты нашей точки в локальной системе координат
	pointLoc.y = data_.twist.vy * data_.dt;
	printf(" pointLoc.x= % .3f y= % .3f dt= % .3f th= % .3f | ", pointLoc.x, pointLoc.y, data_.dt, RAD2DEG(odom_.pose.th));

	// printf("DO pose.x= %.3f pose.y= %.3f pose.th= %.3f / ", odom_.pose.x, odom_.pose.y, RAD2DEG(odom_.pose.th));
	// Находим смещние по осям матрица координаты точки из локальной системы координат в глобальной
	double delta_x = pointLoc.x * cos(odom_.pose.th) + pointLoc.y * sin(odom_.pose.th);
	double delta_y = -pointLoc.x * sin(odom_.pose.th) + pointLoc.y * cos(odom_.pose.th);
	printf("delta.x= % .3f y= % .3f | ", delta_x, delta_y);

	// Меняем координаты и угол на основе вычислений
	odom_.pose.x += delta_x; // Вычисляем координаты
	odom_.pose.y += delta_y; // Вычисляем координаты

	// printf("twist.x= %.4f y= %.4f th= %.4f gradus ", bno055.twist.vx, bno055.twist.vy, bno055.twist.vth);
	// Меняем координаты и угол на основе вычислений
	odom_.pose.th += data_.twist.vth * data_.dt; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот
	if (odom_.pose.th > (2 * M_PI))
		(odom_.pose.th -= (2 * M_PI));
	if (odom_.pose.th < 0)
		(odom_.pose.th += (2 * M_PI));
	printf(" =pose.x= % .3f y= % .3f th= % .3f \n", odom_.pose.x, odom_.pose.y, RAD2DEG(odom_.pose.th));
}

void controlAcc(SControl &control_, SControl g_dreamSpeed) // Функция контроля ускорения
{
	static SControl factControl;				 // Фактически ранее установленная скорость переданная на моторы
	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;
	float accel = ACCELERATION * dt; // Ускорение
	// printf("g_dreamSpeed % .3f % .3f ", g_dreamSpeed.speedL, g_dreamSpeed.speedR);
	if (g_dreamSpeed.speedL != factControl.speedL) // Если скорость с которой хотим крутиться не равна тому что была ранее установлена, то меняем с учетом ускорения
	{
		// printf("g_dreamSpeed % f : % f : acc= % f | ", g_dreamSpeed.speedL, g_dreamSpeed.speedR, accel);
		if (factControl.speedL < g_dreamSpeed.speedL) // Если меньше чем надо то прибавим оборотов
		{
			factControl.speedL = factControl.speedL + accel; // К старой скорости прибавляем ускорение за этот промежуток
			if (factControl.speedL > g_dreamSpeed.speedL)	 // Если стала больше то ровняем
				factControl.speedL = g_dreamSpeed.speedL;
		}
		if (factControl.speedL > g_dreamSpeed.speedL) // Если меньше чем надо то прибавим оборотов
		{
			factControl.speedL = factControl.speedL - accel; // К старой скорости прибавляем ускорение за этот промежуток
			if (factControl.speedL < g_dreamSpeed.speedL)	 // Если стала меньше нужной то далаем какая должна быть
				factControl.speedL = g_dreamSpeed.speedL;
		}
	}
	if (g_dreamSpeed.speedR != factControl.speedR) // Если скорость с которой хотим крутиться не равна тому что была ранее установлена, то меняем с учетом ускорения
	{
		if (factControl.speedR < g_dreamSpeed.speedR) // Если меньше чем надо то прибавим оборотов
		{
			factControl.speedR = factControl.speedR + accel; // К старой скорости прибавляем ускорение за этот промежуток
			if (factControl.speedR > g_dreamSpeed.speedR)	 // Если стала больше то ровняем
				factControl.speedR = g_dreamSpeed.speedR;
		}
		if (factControl.speedR > g_dreamSpeed.speedR) // Если меньше чем надо то прибавим оборотов
		{
			factControl.speedR = factControl.speedR - accel; // К старой скорости прибавляем ускорение за этот промежуток
			if (factControl.speedR < g_dreamSpeed.speedR)	 // Если стала меньше нужной то далаем какая должна быть
				factControl.speedR = g_dreamSpeed.speedR;
		}
	}
	// printf("factControl % f : % f : acc= % f \n ", factControl.speedL, factControl.speedR);
	// printf(" |factControl % .3f % .3f \n", factControl.speedL, factControl.speedR);
	control_ = factControl; // Передаем для дальнейшего испонения
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

/*   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ПРИМЕР ОТ ВАДИМА КАК НУЖНО СЧИТАТЬ одометрию!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	#define  R2G(val) (val*57.29577951308233)
	#define  G2R(val) (val/57.29577951308233)
	// НАЧАЛЬНАЯ Позиция в мировой СК
	double pos_x = 0.0;
	double pos_y = 0.0;

	double speed = 0.3; //  скорость движения
	double vth = G2R(6); // угловая скорость рад/сек (положительное значение - против часовой стрелки)
	double dt = 0.4; // пусть один шаг по времени это 0.1 сек

	// ЭТО НАЧАЛЬНОЕ СТАРТОВОЕ НАПРАВЛЕНИЕ СРАЗУ СПЕЦИАЛЬНО ЗАДАЕМ ЕДИНИЧНОЙ ДЛИННЫ. Вектор нарпавления движения в мировой СК (вперед вдоль Y)
	double course_x = 0.0;
	double course_y = 1.0;
	//double len = sqrt(course_x * course_x + course_y * course_y); // длина вектора направления должна быть 1. Это проверка вектора если подставлять другие цифры.

	for (int i = 0; i < 150; i++) // считаем несколько шагов вперед
	{
	   // Считаем на сколько повернулись за последний интервал времени
	   double delta_th = vth * dt;
	   //if (i == 0)delta_th /= 2; // первый шаго делаем на половину угла Это для идеальной окружности. В реальности неприменимо.

	   // Поворачиваем вектор направления на этот угол КРУТИМ ЕДИНИЧНЫЙ ВЕКТОР,МЕНЯЕМ НАПРАВЛЕНИЕ КУДА МАШИНКА СМОТРИТ, ЭТО МЕНЯЕТ ТОЛЬКО НАПРВЛЕНИЕ КУДА СМОТРИМ. НАПРВЛЕНИЕ КУДА СМОТРИМ ЗАДАЕМ ЧЕРЕЗ ЕДИНИЧНЫЙ ВЕКТОР, А НЕ УГЛОМ ОТ ОСИ Y
	   double course_x_new = course_x * cos(delta_th) - course_y * sin(delta_th);
	   double course_y_new = course_x * sin(delta_th) + course_y * cos(delta_th);
	   course_x = course_x_new;
	   course_y = course_y_new;

	   //len = sqrt(course_x * course_x + course_y * course_y); // Проверка для себя что после поворота длина вектора осталась равна 1 ПОСМОТРЕТЬ В ОТЛАДЧИКЕ ИЛИ ВЫВЕСТИ ПРИНТОМ

	   // Считаем на какое расстояние проехали за последний интервал времени
	   double len = speed * dt;

	   pos_x += course_x * len;
	   pos_y += course_y * len;

	   printf("x= % .6f y= % .6f\n", pos_x, pos_y);
	}
*/

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

//**************************************************************** СТАРЫЙ ПРИМЕР РАСЧЕТА С НИЖНЕГО УРОВНЯ НЕВЕРНЫЙ ******************************************
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
#endif