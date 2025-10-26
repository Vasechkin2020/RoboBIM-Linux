#ifndef CODE_H
#define CODE_H

// #include "pillar.h"
//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************

void readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

void initCommandArray(int verCommand_); // Заполнение маасива команд

void workAngle(float angle_, u_int64_t &time_, float velAngle_);				   // Тут отрабатываем алгоритм отслеживания угла при повороте
void workVector(float len_, SPoint vectorStart_, u_int64_t &time_, float velLen_); // Тут отрабатываем алгоритм отслеживания длины вектора при движении прямо
float calculate_max_safe_speed(float distance_to_stop, float max_deceleration); // Вычисляет максимально допустимую скорость для остановки.
float calculate_max_safe_angular_speed_degrees(    float angular_error_deg,     float max_angular_acceleration_degs2); //Вычисляет максимально допустимую угловую скорость для поворота на месте, используя ГРАДУСЫ.
float convert_angular_speed_to_linear_wheel_speed(float angular_speed_degs, float wheel_base_m); //Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с). 

// pb_msgs::SControlDriver speedCorrect(pb_msgs::SDriver2Data Driver2Data_msg_, pb_msgs::SControlDriver Data2Driver_); // Корректировка скорости движения в зависимости от датчиков растояния перед
// void collectCommand(); // //Функция формирования команды для нижнего уровня на основе всех полученных данных, датчиков и анализа ситуации

// **********************************************************************************
// float filtrComplem(float koef_, float oldData_, float newData_); // функция фильтрации, берем старое значение с некоторым весом

// // функция фильтрации, берем старое значение с некоторым весом
// float filtrComplem(float koef_, float oldData_, float newData_)
// {
// 	return (1 - koef_) * oldData_ + (koef_ * newData_);
// }

// void callback_Driver(pb_msgs::Struct_Driver2Data msg)
// {
// 	msg_Driver2Data = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
// 	flag_msgDriver = true;
// }

void callback_Speed(pb_msgs::SSetSpeed msg)
{
	msg_Speed = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgSpeed = true;
}
void callback_Pose(pb_msgs::Struct_PoseRotation msg)
{
	msg_Pose = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgPose = true;
}

// Находим минимальную дистанцию из 3 датчиков
float minDistance(float laserL_, float laserR_, float uzi1_)
{
	float min = laserL_;
	if (laserR_ < min)
	{
		min = laserR_;
	}
	if (uzi1_ < min)
	{
		min = uzi1_;
	}
	return min;
}
// Заполнение маасива команд
void initCommandArray(int verCommand_)
{
	if (verCommand_ == 1)
	{
		commandArray[0].mode = 3;
		commandArray[0].len = 0.5;
		commandArray[0].velLen = 0.1;

		commandArray[1].mode = 1;
		commandArray[1].duration = 5000;
		commandArray[1].velL = 0.0;
		commandArray[1].velR = 0.0;

		commandArray[2].mode = 2;
		commandArray[2].angle = 130;
		commandArray[2].velAngle = 0.02;

		commandArray[3].mode = 1;
		commandArray[3].duration = 5000;
		commandArray[3].velL = 0.0;
		commandArray[3].velR = 0.0;

		commandArray[4].mode = 3;
		commandArray[4].len = 0.5;
		commandArray[4].velLen = 0.1;

		commandArray[5].mode = 1;
		commandArray[5].duration = 5000;
		commandArray[5].velL = 0.0;
		commandArray[5].velR = 0.0;

		commandArray[6].mode = 2;
		commandArray[6].angle = 0.0;
		commandArray[6].velAngle = 0.02;

		commandArray[7].mode = 1;
		commandArray[7].duration = 5000;
		commandArray[7].velL = 0.0;
		commandArray[7].velR = 0.0;

		commandArray[8].mode = 3;
		commandArray[8].len = 0.5;
		commandArray[8].velLen = 0.1;

		commandArray[9].mode = 1;
		commandArray[9].duration = 5000;
		commandArray[9].velL = 0.0;
		commandArray[9].velR = 0.0;

		commandArray[10].mode = 2;
		commandArray[10].angle = -130;
		commandArray[10].velAngle = 0.02;

		commandArray[11].mode = 1;
		commandArray[11].duration = 5000;
		commandArray[11].velL = 0.0;
		commandArray[11].velR = 0.0;

		commandArray[12].mode = 3;
		commandArray[12].len = 0.5;
		commandArray[12].velLen = 0.1;

		commandArray[13].mode = 1;
		commandArray[13].duration = 5000;
		commandArray[13].velL = 0.0;
		commandArray[13].velR = 0.0;

		commandArray[14].mode = 2;
		commandArray[14].angle = 0;
		commandArray[14].velAngle = 0.02;

		commandArray[15].mode = 1;
		commandArray[15].duration = 5000;
		commandArray[15].velL = 0.0;
		commandArray[15].velR = 0.0;

		commandArray[16].mode = 3;
		commandArray[16].len = 0.5;
		commandArray[16].velLen = 0.1;

		commandArray[17].mode = 1;
		commandArray[17].duration = 500000;
		commandArray[17].velL = 0.0;
		commandArray[17].velR = 0.0;

		commandArray[18].mode = 9;
	}
	if (verCommand_ == 2)
	{
		commandArray[0].mode = 3;
		commandArray[0].len = 1.0;
		commandArray[0].velLen = 0.1;

		commandArray[1].mode = 1;
		commandArray[1].duration = 5000;
		commandArray[1].velL = 0.0;
		commandArray[1].velR = 0.0;

		commandArray[2].mode = 3;
		commandArray[2].len = 1.0;
		commandArray[2].velLen = 0.2;

		commandArray[3].mode = 1;
		commandArray[3].duration = 5000;
		commandArray[3].velL = 0.0;
		commandArray[3].velR = 0.0;

		commandArray[4].mode = 3;
		commandArray[4].len = 1.0;
		commandArray[4].velLen = 0.1;

		commandArray[5].mode = 1;
		commandArray[5].duration = 10000;
		commandArray[5].velL = 0.0;
		commandArray[5].velR = 0.0;
		//***************************************************

		commandArray[6].mode = 3;
		commandArray[6].len = -1.0;
		commandArray[6].velLen = 0.1;

		commandArray[7].mode = 1;
		commandArray[7].duration = 5000;
		commandArray[7].velL = 0.0;
		commandArray[7].velR = 0.0;

		commandArray[8].mode = 3;
		commandArray[8].len = -1.0;
		commandArray[8].velLen = 0.2;

		commandArray[9].mode = 1;
		commandArray[9].duration = 5000;
		commandArray[9].velL = 0.0;
		commandArray[9].velR = 0.0;

		commandArray[10].mode = 3;
		commandArray[10].len = -1.0;
		commandArray[10].velLen = 0.1;

		commandArray[11].mode = 1;
		commandArray[11].duration = 20000;
		commandArray[11].velL = 0.0;
		commandArray[11].velR = 0.0;

		commandArray[12].mode = 9;
	}
	if (verCommand_ == 3)
	{
		commandArray[0].mode = 1;
		commandArray[0].duration = 20000;
		commandArray[0].velL = 0.02;
		commandArray[0].velR = -0.02;

		commandArray[1].mode = 1;
		commandArray[1].duration = 5000;
		commandArray[1].velL = 0.0;
		commandArray[1].velR = 0.0;

		commandArray[2].mode = 1;
		commandArray[2].duration = 20000;
		commandArray[2].velL = -0.02;
		commandArray[2].velR = 0.02;

		commandArray[3].mode = 1;
		commandArray[3].duration = 5000;
		commandArray[3].velL = 0.0;
		commandArray[3].velR = 0.0;

		commandArray[4].mode = 1;
		commandArray[4].duration = 20000;
		commandArray[4].velL = -0.02;
		commandArray[4].velR = 0.02;

		commandArray[5].mode = 1;
		commandArray[5].duration = 5000;
		commandArray[5].velL = 0.0;
		commandArray[5].velR = 0.0;

		commandArray[6].mode = 1;
		commandArray[6].duration = 20000;
		commandArray[6].velL = 0.02;
		commandArray[6].velR = -0.02;

		commandArray[7].mode = 1;
		commandArray[7].duration = 20000;
		commandArray[7].velL = 0.0;
		commandArray[7].velR = 0.0;

		commandArray[8].mode = 9;
	}
	if (verCommand_ == 4)
	{
		commandArray[0].mode = 3; // Прямо 1 метр
		commandArray[0].len = 0.7;

		commandArray[1].mode = 1; // Задержка на месте
		commandArray[1].duration = 1000;
		commandArray[1].velL = 0.0;
		commandArray[1].velR = 0.0;

		commandArray[2].mode = 3; // Прямо 113 метр
		commandArray[2].len = 0.113;

		commandArray[3].mode = 1; // Задержка на месте
		commandArray[3].duration = 1000;
		commandArray[3].velL = 0.0;
		commandArray[3].velR = 0.0;

		commandArray[4].mode = 2; // По часовой на 90 градусов
		commandArray[4].angle = -90;

		commandArray[5].mode = 1; // Задержка на месте
		commandArray[5].duration = 1000;
		commandArray[5].velL = 0.0;
		commandArray[5].velR = 0.0;

		commandArray[6].mode = 3; // Прямо 113 метр
		commandArray[6].len = -0.113;

		commandArray[7].mode = 1; // Задержка на месте
		commandArray[7].duration = 1000;
		commandArray[7].velL = 0.0;
		commandArray[7].velR = 0.0;
		//------------------------------------------------------------------------
		commandArray[8].mode = 3; // Прямо 1 метр
		commandArray[8].len = 0.7;

		commandArray[9].mode = 1; // Задержка на месте
		commandArray[9].duration = 1000;
		commandArray[9].velL = 0.0;
		commandArray[9].velR = 0.0;

		commandArray[10].mode = 3; // Прямо 113 метр
		commandArray[10].len = 0.113;

		commandArray[11].mode = 1; // Задержка на месте
		commandArray[11].duration = 1000;
		commandArray[11].velL = 0.0;
		commandArray[11].velR = 0.0;

		commandArray[12].mode = 2; // По часовой на 90 градусов
		commandArray[12].angle = -180;

		commandArray[13].mode = 1; // Задержка на месте
		commandArray[13].duration = 1000;
		commandArray[13].velL = 0.0;
		commandArray[13].velR = 0.0;

		commandArray[14].mode = 3; // Прямо 113 метр
		commandArray[14].len = -0.113;

		commandArray[15].mode = 1; // Задержка на месте
		commandArray[15].duration = 1000;
		commandArray[15].velL = 0.0;
		commandArray[15].velR = 0.0;
		//------------------------------------------------------------------------
		commandArray[16].mode = 3; // Прямо 1 метр
		commandArray[16].len = 0.7;

		commandArray[17].mode = 1; // Задержка на месте
		commandArray[17].duration = 1000;
		commandArray[17].velL = 0.0;
		commandArray[17].velR = 0.0;

		commandArray[18].mode = 3; // Прямо 113 метр
		commandArray[18].len = 0.113;

		commandArray[19].mode = 1; // Задержка на месте
		commandArray[19].duration = 1000;
		commandArray[19].velL = 0.0;
		commandArray[19].velR = 0.0;

		commandArray[20].mode = 2; // По часовой на 90 градусов
		commandArray[20].angle = 90;

		commandArray[21].mode = 1; // Задержка на месте
		commandArray[21].duration = 1000;
		commandArray[21].velL = 0.0;
		commandArray[21].velR = 0.0;

		commandArray[22].mode = 3; // Прямо 113 метр
		commandArray[22].len = -0.113;

		commandArray[23].mode = 1; // Задержка на месте
		commandArray[23].duration = 1000;
		commandArray[23].velL = 0.0;
		commandArray[23].velR = 0.0;
		//------------------------------------------------------------------------
		commandArray[24].mode = 3; // Прямо 1 метр
		commandArray[24].len = 0.7;

		commandArray[25].mode = 1; // Задержка на месте
		commandArray[25].duration = 1000;
		commandArray[25].velL = 0.0;
		commandArray[25].velR = 0.0;

		commandArray[26].mode = 3; // Прямо 113 метр
		commandArray[26].len = 0.113;

		commandArray[27].mode = 1; // Задержка на месте
		commandArray[27].duration = 1000;
		commandArray[27].velL = 0.0;
		commandArray[27].velR = 0.0;

		commandArray[28].mode = 2; // По часовой на 90 градусов
		commandArray[28].angle = 0;

		commandArray[29].mode = 1; // Задержка на месте
		commandArray[29].duration = 1000;
		commandArray[29].velL = 0.0;
		commandArray[29].velR = 0.0;

		commandArray[30].mode = 3; // Прямо 113 метр
		commandArray[30].len = -0.113;

		commandArray[31].mode = 1; // Задержка на месте
		commandArray[31].duration = 1000;
		commandArray[31].velL = 0.0;
		commandArray[31].velR = 0.0;
		//------------------------------------------------------------------------
		//-----------------------
		commandArray[32].mode = 9;
	}
	if (verCommand_ == 5)
	{
		commandArray[0].mode = 2;
		commandArray[0].angle = -30;
		commandArray[0].velAngle = 0.03;

		commandArray[1].mode = 1;
		commandArray[1].duration = 5000;

		commandArray[2].mode = 2;
		commandArray[2].angle = 0;
		commandArray[2].velAngle = 0.03;

		commandArray[3].mode = 1;
		commandArray[3].duration = 5000;

		commandArray[4].mode = 2;
		commandArray[4].angle = 30;
		commandArray[4].velAngle = 0.03;

		commandArray[5].mode = 1;
		commandArray[5].duration = 5000;

		commandArray[6].mode = 2;
		commandArray[6].angle = 0;
		commandArray[6].velAngle = 0.03;

		commandArray[7].mode = 1;
		commandArray[7].duration = 5000;

		// commandArray[8].mode = 2;
		// commandArray[8].angle = 90;
		// commandArray[8].velAngle = 0.02;

		// commandArray[9].mode = 1;
		// commandArray[9].duration = 10000;

		// commandArray[10].mode = 2;
		// commandArray[10].angle = 0;
		// commandArray[10].velAngle = 0.02;

		// commandArray[11].mode = 1;
		// commandArray[11].duration = 10000;

		commandArray[8].mode = 9;

		for (int i = 0; i < 13; i++)
		{
			ROS_INFO("i=%i angle= %f", i, commandArray[i].angle);
		}
	}
	if (verCommand_ == 6)
	{
		commandArray[0].mode = 2; // Управление по углу
		commandArray[0].angle = -90;
		commandArray[0].velAngle = 0.02;

		commandArray[1].mode = 1; // Управление по времени
		commandArray[1].duration = 10000;
		commandArray[1].velL = 0.00;
		commandArray[1].velR = 0.00;

		commandArray[2].mode = 2;
		commandArray[2].angle = 0;
		commandArray[2].velAngle = 0.02;

		commandArray[3].mode = 1;
		commandArray[3].duration = 10000;
		commandArray[3].velL = 0.0;
		commandArray[3].velR = 0.0;

		commandArray[4].mode = 2;
		commandArray[4].angle = 90;
		commandArray[4].velAngle = 0.02;

		commandArray[5].mode = 1;
		commandArray[5].duration = 10000;
		commandArray[5].velL = 0.0;
		commandArray[5].velR = 0.0;

		commandArray[6].mode = 2;
		commandArray[6].angle = 0;
		commandArray[6].velAngle = 0.02;

		commandArray[7].mode = 1;
		commandArray[7].duration = 50000;
		commandArray[7].velL = 0.0;
		commandArray[7].velR = 0.0;

		commandArray[8].mode = 9;
	}
}

void readParam() // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
{
	ros::NodeHandle nh_private("~");
	// Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
	if (!nh_private.getParam("verComand", verComand))
		verComand = 999;

	ROS_INFO("--- Start node with parametrs:");
	ROS_INFO("verComand = %i", verComand);
	ROS_INFO("---");
}

// Выводим справочно время работы цикла
void timeCycle(ros::Time timeStart_, ros::Time timeNow_)
{
	ros::Time timeEnd = ros::Time::now();				// Захватываем конечный момент времени
	ros::Duration durationEnd = timeEnd - timeNow_;		// Находим разницу между началом и концом
	ros::Duration durationStart = timeEnd - timeStart_; // Находим разницу между началом и концом
	double dtEnd = durationEnd.toSec() * 1000;			// Получаем количество милисекунд
	double dtStart = durationStart.toSec();				// Получаем количество секунд
	if (dtEnd > 5)										// Если цикл занял бользе 5 милисекунд значит что не уложились в 200 Нz
		ROS_INFO("    !!! cycle = %8.3f msec", dtEnd);	// Время цикла в милисекундах
	else
		ROS_INFO_THROTTLE(1, "    dtStart = %7.0f sec | last cycle = %8.3f msec", dtStart, dtEnd); // Время цикла в милисекундах
}

/**
 * @brief Вычисляет максимально допустимую угловую скорость для поворота на месте, используя ГРАДУСЫ.
 * @param angular_error_deg Оставшаяся ошибка по углу (градусы). Должна быть положительной.
 * @param max_angular_acceleration_degs2 Максимальное угловое замедление (град/с^2). Должно быть положительным.
 * @return Максимально допустимая угловая скорость (град/с).
 */
float calculate_max_safe_angular_speed_degrees(    float angular_error_deg,     float max_angular_acceleration_degs2) //Вычисляет максимально допустимую угловую скорость для поворота на месте, используя ГРАДУСЫ.
{
    float D_phi = std::fabs(angular_error_deg); // Используем абсолютное значение угловой ошибки.
    if (D_phi <= std::numeric_limits<float>::epsilon() || max_angular_acceleration_degs2 <= 0.0) // Защита от нулевой ошибки или ускорения 
        return 0.0;
    return std::sqrt(2.0 * max_angular_acceleration_degs2 * D_phi); // Формула: omega_max = sqrt(2 * alpha_max * D_phi)    // Все величины должны быть в согласованных единицах (градусы, град/с, град/с^2)
}

/**
 * @brief Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с).  * * Эта скорость должна быть подана на одно колесо со знаком '+', а на другое со знаком '-'.
 * @param angular_speed_degs Угловая скорость робота (град/с).
 * @param wheel_base_m Расстояние между колесами (база) (метры).
 * @return Требуемая линейная скорость колеса (м/с).
 */
float convert_angular_speed_to_linear_wheel_speed(float angular_speed_degs, float wheel_base_m) //Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с). 
{
    float omega_rads = angular_speed_degs * (M_PI / 180.0); // 1. Конвертация угловой скорости из градусов в секунду (град/с) в радианы в секунду (рад/с)
    float linear_wheel_speed = omega_rads * (wheel_base_m / 2.0);// 2. Расчет линейной скорости колеса     // Формула: v = omega_rads * R, где R (радиус поворота) = wheel_base_m / 2
    return linear_wheel_speed;
}


// Тут отрабатываем алгоритм отслеживания угла при повороте
void workAngle(float angle_, u_int64_t &time_, float velAngle_)
{

	static float minAngleMistake = 0.02;  // Минимальная ошибка по углу в Градусах
	static float angleMistake = 0;		 // Текущая ошибка по углу в градусах
	const float max_angular_acceleration_degs2 = 30; // Угловое ускорение/замедление в градусах в секунду
	static float max_deceleration = 0.2;   // Линейное Ускорение/замедление метры в секунду
	static float speedCurrent = 0;

	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;
	float accel = max_deceleration * dt; // Ускорение/замедление

	float angleFact = msg_Pose.th.odom;			// Угол который отслеживаем
	angleMistake = angle_ - RAD2DEG(angleFact); // Смотрим какой угол.// Смотрим куда нам надо Считаем ошибку по углу и включаем колеса в нужную сторону с учетом ошибки по углу и максимально заданой скорости на колесах
	ROS_INFO_THROTTLE(0.1, "    angle_ = %6.3f angleFact = %6.3f angleMistake = %6.3f", angle_, RAD2DEG(angleFact), angleMistake);

	if (flagAngleFirst)
	{
		accel = 0; // Первый запуск
		flagAngleFirst = false;
		ROS_INFO("    Angle Start angleMistake = %f metr", angleMistake);
	}

	if (abs(angleMistake) <= minAngleMistake) // Когда ошибка по углу будет меньше заданной считаем что приехали и включаем время что-бы выйти из данного этапа алгоритма
	{
		speedCurrent = 0;
		controlSpeed.control.speedL = 0;
		controlSpeed.control.speedR = 0;
		flagAngle = false;
		flagAngleFirst = true;
		time_ = millis();
		ROS_INFO("    Angle Final angleMistake = %f gradus", angleMistake);
	}
	else
	{
		// static float angleKoef = 0.01;		 // P коефициент пид регулятора
		// float speedCurrent = abs(angleMistake * angleKoef);
		// ROS_INFO_THROTTLE(0.1, "    speedCurrent koef = %f", speedCurrent);

		float V_max_ang = calculate_max_safe_angular_speed_degrees(angleMistake, max_angular_acceleration_degs2); // Считаем максимальную скорость с которой успеем остановиться
		float V_max_lin = convert_angular_speed_to_linear_wheel_speed( V_max_ang, DISTANCE_WHEELS); //Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с). 
		ROS_INFO_THROTTLE(0.1, "    workAngle V_max_ang = %f  speedCurrent V_max_lin = %f", V_max_ang, V_max_lin);

		speedCurrent = speedCurrent + accel; // Ускорение.Увеличиваем скорость

		if (V_max_lin <= speedCurrent) // Если наша скорость больше чем допустимо то снижаем до допустимой  ЭТО ТОРМОЖЕНИЕ
			speedCurrent = V_max_lin;
		else // ЭТО УСКОРЕНИЕ
		{
			if (speedCurrent > velAngle_)		   // Максимальная скорость
			{
				speedCurrent = velAngle_;		   // Если стала больше то ровняем
				// ROS_INFO("   MAX speedCurrent = %f velLen_ = %f ", speedCurrent, velLen_);
			}
		}
		if (speedCurrent < 0.005) // Минимальная скорость
			speedCurrent = 0.005;

		if (angleMistake > 0) // Если угол больше чем надо и положительный то вращается в одну сторону
		{
			controlSpeed.control.speedL = -speedCurrent; // Скороть должна увеличивать до заданой или максимальной с учетом алогритма в data_node  а уменьшать будет по коефициету по ошибке по углу.
			controlSpeed.control.speedR = speedCurrent;
		}
		else
		{
			controlSpeed.control.speedL = speedCurrent;
			controlSpeed.control.speedR = -speedCurrent;
		} 
		ROS_INFO_THROTTLE(0.1, "    speedCurrent real L = %f R = %f ", controlSpeed.control.speedL, controlSpeed.control.speedR);   
	}
}

/** 
 * @brief Вычисляет максимально допустимую скорость для остановки.
 * Рассчитывает максимальную скорость, которую должно иметь тело, чтобы успеть остановиться на расстоянии 'distance_to_stop' при условии немедленного применения максимального замедления 'max_deceleration'.
 * @param distance_to_stop Расстояние до точки остановки (метры).
 * @param max_deceleration Максимально возможное ускорение/замедление (м/с^2).  Должно быть положительным.
 * @return Максимально допустимая скорость (м/с). Возвращает 0.0, если расстояние < 0.
 */
float calculate_max_safe_speed(float distance_to_stop, float max_deceleration)
{

	if (distance_to_stop <= 0.0) // Защита от отрицательного расстояния или нулевого ускорения
		return 0.0;

	if (max_deceleration <= 0.0) // Если ускорение равно 0, то безопасная скорость также 0,  так как мы не сможем остановиться, если уже движемся.
		return 0.0;

	// Формула: v = sqrt(2 * a * S)
	// Где:
	// v - максимально допустимая скорость (м/с)
	// a - максимальное замедление (м/с^2)
	// S - расстояние до остановки (метры)

	return std::sqrt(2.0 * max_deceleration * distance_to_stop);
}

// Тут отрабатываем алгоритм отслеживания длины вектора при движении прямо
void workVector(float len_, SPoint vectorStart_, u_int64_t &time_, float velLen_)
{
	static float minVectorMistake = 0.001; // Минимальная ошибка по вектору в метрах 1 cм
	static float vectorMistake = 0;		   // Текущая ошибка по длине в местрах
	static float max_deceleration = 0.2;   // Ускорение/замедление метры в секунду
	static SPoint vectorEnd;
	static float speedCurrent; // Текущая скорость

	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;
	float accel = max_deceleration * dt; // Ускорение

	vectorEnd.x = msg_Pose.x.odom;
	vectorEnd.y = msg_Pose.y.odom;
	float vectorFact = vectorLen(vectorStart_, vectorEnd); // Находим длину вектора который отслеживаем
	vectorMistake = abs(len_) - vectorFact;				   // Смотрим какое растояние еще надо проехать  Считаем ошибку по длине и включаем колеса в нужную сторону с учетом ошибки максимально заданой скорости на колесах
	ROS_INFO_THROTTLE(0.1, "    len_ = %7.3f vectorFact = %7.3f vectorMistake = %7.3f", abs(len_), vectorFact, vectorMistake);

	if (flagVectorFirst)
	{
		accel = 0; // Первый запуск
		flagVectorFirst = false;
		ROS_INFO("    Vector Start vectorMistake = %f metr", vectorMistake);
	}
	if (abs(vectorMistake) <= minVectorMistake) // Когда ошибка по длине будет меньше заданной считаем что приехали и включаем время что-бы выйти из данного этапа алгоритма
	{
		speedCurrent = 0; // Все скорости обнуляем
		controlSpeed.control.speedL = 0;
		controlSpeed.control.speedR = 0;
		flagVector = false;
		flagVectorFirst = true;
		time_ = millis();
		ROS_INFO("    Vector Final vectorMistake = %f metr", vectorMistake);
	}
	else
	{
		float V_max = calculate_max_safe_speed(vectorMistake, max_deceleration); // Считаем максимальную скорость с которой успеем остановиться
		// ROS_INFO_THROTTLE(0.1, "    workVector V_max = %f", V_max);

		if (V_max <= speedCurrent) // Если наша скорость больше чем допустимо то снижаем до допустимой  ЭТО ТОРМОЖЕНИЕ
			speedCurrent = V_max;
		else // ЭТО УСКОРЕНИЕ
		{
			speedCurrent = speedCurrent + accel; // Ускорение.Увеличиваем скорость
			if (speedCurrent > velLen_)		   // Максимальная скорость
			{
				// ROS_INFO("   MAX speedCurrent = %f velLen_ = %f ", speedCurrent, velLen_);
				speedCurrent = velLen_;		   // Если стала больше то ровняем
			}
		}

		// static float vectorKoef = 3.0;		   // P коефициент пид регулятора
		// float speedCurrent = abs(vectorMistake * vectorKoef); // Это простейший вариант с ПИД регулировнаием по Р
		// ROS_INFO_THROTTLE(0.1, "    speedCurrent vectorKoef = %f", speedCurrent);

		if (speedCurrent < 0.005) // Минимальная скорость
			speedCurrent = 0.005;

		if (len_ > 0) // Если длина положительная то вращается в одну сторону или в другую
		{
			controlSpeed.control.speedL = speedCurrent; // Скороть должна увеличивать до заданой или максимальной с учетом алогритма в data_node  а уменьшать будет по коефициету по ошибке
			controlSpeed.control.speedR = speedCurrent;
		}
		else
		{
			controlSpeed.control.speedL = -speedCurrent;
			controlSpeed.control.speedR = -speedCurrent;
		}
		ROS_INFO_THROTTLE(0.1, "    workVector V_max = %f | fact speedL = %f speedR = %f | dt = %f  accel = %f", V_max, controlSpeed.control.speedL, controlSpeed.control.speedR, dt, accel);
	}
}
// Корректировка скорости движения в зависимости от датчиков растояния перед
// pb_msgs::SControlDriver speedCorrect(pb_msgs::SDriver2Data Driver2Data_msg_, pb_msgs::SControlDriver Data2Driver_)
// {
//     float min = minDistance(Driver2Data_msg_.laserL.distance, Driver2Data_msg_.laserR.distance, Driver2Data_msg_.uzi1.distance); // Находим минимальную дистанцию из 3 датчиков
//     if (min < 0.5)                                                                                                               // Если меньше полметра
//     {
//         long minDist = (long)(min * 1000); // Превращаем в целое и увеличиваем умножая на 1000 для точности
//         if (minDist < 100)
//             minDist = 100;
//         float proc = map(minDist, 100, 500, 0, 100);
//         proc = proc / 100; // Превращаем в проценты
//         // Data2Driver_.control.speed = proc * Data2Driver_.control.speed;
//         // ROS_INFO("Correct speed. Min distance = %f, New speed = %f", min, Data2Driver_.control.speed);
//     }
//     // printf("sp= %f \n", Data2Driver_.control.speed);
//     return Data2Driver_;
// }

// //Функция формирования команды для нижнего уровня на основе всех полученных данных, датчиков и анализа ситуации
// void collectCommand()
// {
//     Command_msg.id++; //Увеличиваем номер команды
//     ROS_INFO_THROTTLE (3,"%s Command_msg.id= %i",NN, Command_msg.id);
//     Command_msg.command_body = Control_msg.startStop; // Команда движение или стоим
//     ROS_INFO_THROTTLE (3,"%s Command_msg.command_body= %i", NN, Command_msg.command_body);

//     if (Control_msg.startStop == 0)  // Если команда  стоп то сбрасываем фактическую скорость
//     {
//         g_my_position.speed.fact = 0;
//     }

//     float radius = map(Control_msg.radius, -100, 100, -MAX_RADIUS * 1000, MAX_RADIUS * 1000) / 1000.0; //Преобразуем диапазон из Remote XY от -100 до 100 в допустимый диапазон, только челые числа функция использует
//     if (radius > 0 )
//     {
//        radius = MAX_RADIUS - radius + 0.02; // Прибавляем чуть-чуть чтобы радиус не получался 0 на краях
//     }
//     if (radius < 0 )
//     {
//        radius = -MAX_RADIUS - radius - 0.02; // Отнимаем чуть-чуть чтобы радиус не получался 0 на краях
//     }
//     Command_msg.radius = g_my_position.radius = radius; // Присваиваем радиус
//     ROS_INFO_THROTTLE (3,"%s Command_msg.radius= %f",NN, Command_msg.radius);

//     Command_msg.motor_video_angle = Control_msg.camera;  // Положение ккамеры от 0 до 100 градусов
//     Command_msg.program_led = Control_msg.led_program;   // Программа для светодиодов

//     //-----------------------------------------------------------
//     g_my_position.speed.target = SPEED_STANDART; // Всегда начинаем с цели в виде типовой скорости и уже потом ее меняем в зависимости от обстоятельств

//     //ROS_INFO("g_my_position.speed.target 1 = %.2f", g_my_position.speed.target);
//     //Получаем цель по скорости на основаниии манипулятора, это типа наше текущее желание по скорости
//     //g_my_position.speed.target = map(Control_msg.speed, 0, 100, 0, SPEED_MAX * 1000) / 1000.0; //Преобразуем диапазон из Remote XY от 0 до 100 в допустимый диапазон, только челые числа функция использует
//     //ROS_INFO("g_my_position.speed.target 2 = %.1f", g_my_position.speed.target);
//     // В фильтрах меняем цель по скорости в зависимости от обстоятельств окружающего мира, препятствия, обрывы
//     // filterUzi();
//     // ROS_INFO("g_my_position.speed.target 3 = %.2f", g_my_position.speed.target);
//     // filterRadius(); // При маленьком радиусе не крутиться с большой скоростью
//     // ROS_INFO("g_my_position.speed.target 4 = %.2f", g_my_position.speed.target);
//     // filterLaser(); // Ограничения что-бы не упасть с обрыва
//     // ROS_INFO("g_my_position.speed.target 5 = %.1f", g_my_position.speed.target);

//     //float speed = newSpeed();  // Устанавливаем новую фактическую скорость после того как разобрались с целью по скорости
//     float speed = 0.2;           // Устанавливаем новую фактическую скорость после того как разобрались с целью по скорости
//     if (radius < 0.05 && radius > 0)
//     {
//         speed = 0.1;
//     }
//     if (radius > -0.05 && radius < 0)
//     {
//         speed = 0.1;
//     }
//     Command_msg.speed = speed;
//     ROS_INFO_THROTTLE(3,"%s Command_msg.speed= %f",NN, Command_msg.speed);

//     //ROS_INFO("newSpeed = %.3f", speed);
//     //INFO ROS_INFO("-------------- ");
//     //-----------------------------------------------------------
// }

#endif
