#ifndef CODE_H
#define CODE_H

#include <cmath>
#include <cstdio> // Для функции printf
#include <iostream>
#include <algorithm>
#include <iomanip>

// #include "pillar.h"
//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************

// Константы
const double PI = 3.14159265358979323846;
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;

void readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

void initCommandArray(int verCommand_); // Заполнение маасива команд

void workAngle(float angle_, u_int64_t &time_, float velAngle_);											   // Тут отрабатываем алгоритм отслеживания угла при повороте
void workVector(float len_, SPoint point_A_, SPoint point_B_, u_int64_t &time_, float velLen_);				   // Тут отрабатываем алгоритм отслеживания длины вектора при движении прямо
float calculate_max_safe_speed(float distance_to_stop, float max_deceleration);								   // Вычисляет максимально допустимую скорость для остановки.
float calculate_max_safe_angular_speed_degrees(float angular_error_deg, float max_angular_acceleration_degs2); // Вычисляет максимально допустимую угловую скорость для поворота на месте, используя ГРАДУСЫ.
float convert_angular_speed_to_linear_wheel_speed(float angular_speed_degs, float wheel_base_m);			   // Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с).
void callback_Joy(sensor_msgs::Joy msg);																	   // Функция обраьтного вызова по подпичке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
SPoint calculate_new_coordinates(SPoint point_A_, float angle_rad, float distance);							   // Расчет новых координат по старому положению и длинне вектора

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

// Функция обраьтного вызова по подписке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
void callback_Joy(sensor_msgs::Joy msg)
{
	flag_msgJoy = true;
	msg_joy = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
}

void callback_Speed(pb_msgs::SSetSpeed msg)
{
	msg_Speed = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgSpeed = true;
}
void callback_Pose(pb_msgs::Struct_PoseRotation msg)
{
	msg_PoseRotation = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
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

		commandArray[18].mode = 9;
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
	ros::Time timeEnd = ros::Time::now();					// Захватываем конечный момент времени
	ros::Duration durationEnd = timeEnd - timeNow_;			// Находим разницу между началом и концом
	ros::Duration durationStart = timeEnd - timeStart_;		// Находим разницу между началом и концом
	double dtEnd = durationEnd.toSec() * 1000;				// Получаем количество милисекунд
	double dtStart = durationStart.toSec();					// Получаем количество секунд
	if (dtEnd > 5)											// Если цикл занял бользе 5 милисекунд значит что не уложились в 200 Нz
		logi.log_r("    !!! cycle = %8.3f msec \n", dtEnd); // Время цикла в милисекундах
															// else
															// 	logi.log("    dtStart = %7.0f sec | last cycle = %8.3f msec \n", dtStart, dtEnd); // Время цикла в милисекундах
}

/**
 * @brief Вычисляет максимально допустимую угловую скорость для поворота на месте, используя ГРАДУСЫ.
 * @param angular_error_deg Оставшаяся ошибка по углу (градусы). Должна быть положительной.
 * @param max_angular_acceleration_degs2 Максимальное угловое замедление (град/с^2). Должно быть положительным.
 * @return Максимально допустимая угловая скорость (град/с).
 */
float calculate_max_safe_angular_speed_degrees(float angular_error_deg, float max_angular_acceleration_degs2) // Вычисляет максимально допустимую угловую скорость для поворота на месте, используя ГРАДУСЫ.
{
	float D_phi = std::fabs(angular_error_deg);													 // Используем абсолютное значение угловой ошибки.
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
float convert_angular_speed_to_linear_wheel_speed(float angular_speed_degs, float wheel_base_m) // Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с).
{
	float omega_rads = angular_speed_degs * (M_PI / 180.0);		  // 1. Конвертация угловой скорости из градусов в секунду (град/с) в радианы в секунду (рад/с)
	float linear_wheel_speed = omega_rads * (wheel_base_m / 2.0); // 2. Расчет линейной скорости колеса     // Формула: v = omega_rads * R, где R (радиус поворота) = wheel_base_m / 2
	return linear_wheel_speed;
}

// Тут отрабатываем алгоритм отслеживания угла при повороте
void workAngle(float angle_, u_int64_t &time_, float velAngle_)
{

	static float minAngleMistake = 0.02;			 // Минимальная ошибка по углу в Градусах
	static float angleMistake = 0;					 // Текущая ошибка по углу в градусах
	const float max_angular_acceleration_degs2 = 30; // Угловое ускорение/замедление в градусах в секунду
	static float max_deceleration = 0.2;			 // Линейное Ускорение/замедление метры в секунду
	static float speedCurrent = 0;

	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;
	float accel = max_deceleration * dt; // Ускорение/замедление

	float angleFact = msg_PoseRotation.th.odom;				// Угол который отслеживаем
	angleMistake = angle_ - RAD2DEG(angleFact);		// Смотрим какой угол.// Смотрим куда нам надо Считаем ошибку по углу и включаем колеса в нужную сторону с учетом ошибки по углу и максимально заданой скорости на колесах
	angleMistake = normalizeAngle180(angleMistake); // Нормализуем +-180

	if (flagAngleFirst)
	{
		accel = 0; // Первый запуск
		flagAngleFirst = false;
		logi.log_b("    Angle Start angleMistake = %f gradus \n", angleMistake);
	}

	if (abs(angleMistake) <= minAngleMistake) // Когда ошибка по углу будет меньше заданной считаем что приехали и включаем время что-бы выйти из данного этапа алгоритма
	{
		speedCurrent = 0;
		controlSpeed.control.speedL = 0;
		controlSpeed.control.speedR = 0;
		flagAngle = false;
		flagAngleFirst = true;
		time_ = millis();
		logi.log_w("    Angle Final angleMistake = %f gradus \n", angleMistake);
	}
	else
	{
		// static float angleKoef = 0.01;		 // P коефициент пид регулятора
		// float speedCurrent = abs(angleMistake * angleKoef);
		// ROS_INFO_THROTTLE(0.1, "    speedCurrent koef = %f", speedCurrent);

		float V_max_ang = calculate_max_safe_angular_speed_degrees(angleMistake, max_angular_acceleration_degs2); // Считаем максимальную скорость с которой успеем остановиться
		float V_max_lin = convert_angular_speed_to_linear_wheel_speed(V_max_ang, DISTANCE_WHEELS);				  // Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с).

		speedCurrent = speedCurrent + accel; // Ускорение.Увеличиваем скорость

		if (V_max_lin <= speedCurrent) // Если наша скорость больше чем допустимо то снижаем до допустимой  ЭТО ТОРМОЖЕНИЕ
			speedCurrent = V_max_lin;
		else // ЭТО УСКОРЕНИЕ
		{
			if (speedCurrent > velAngle_) // Максимальная скорость
			{
				speedCurrent = velAngle_; // Если стала больше то ровняем
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
		logi.log("    workAngle angle = %6.3f angleFact = %6.3f angleMistake = %6.3f | V_max_ang = %f  speedCurrent V_max_lin = %f | speedCurrent real L = %f R = %f \n",
				 angle_, RAD2DEG(angleFact), angleMistake, V_max_ang, V_max_lin, controlSpeed.control.speedL, controlSpeed.control.speedR);
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

/**
 * @brief Рассчитывает новые координаты на основе вектора движения и выводит их через printf.
 * * Предполагается, что angle_rad отсчитывается от положительной оси X (в радианах).
 * * @param current_x Текущая координата X (double)
 * @param current_y Текущая координата Y (double)
 * @param angle_rad Угол направления в радианах
 * @param distance Пройденное расстояние
 */
SPoint calculate_new_coordinates(SPoint point_A_, float angle_rad, float signed_distance)
{
	// Расчет приращения по осям
	float dx = signed_distance * std::cos(angle_rad);
	float dy = signed_distance * std::sin(angle_rad);

	SPoint point_B; // Новые координаты
	point_B.x = point_A_.x + dx;
	point_B.y = point_A_.y + dy;
	// Вывод результатов с использованием printf
	// Используем "%.4f" для вывода чисел с плавающей точкой с точностью до 4 знаков после запятой
	logi.log_g("Initial Position point_A (X, Y): (%.4f, %.4f)\n", point_A_.x, point_A_.y);
	logi.log("Movement Vector (signed_distance, Angle rad): (%.4f, %.4f)\n", signed_distance, angle_rad);
	logi.log("Delta Position (dX, dY): (%.4f, %.4f)\n", dx, dy);
	logi.log_g("New Position point_B (X', Y'): (%.4f, %.4f)\n", point_B.x, point_B.y);
	return point_B;
}

// Тут отрабатываем алгоритм отслеживания длины вектора при движении прямо
void workVector(float len_, SPoint point_A_, SPoint point_B_, u_int64_t &time_, float velLen_)
{
	static float minVectorMistake = 0.001; // Минимальная ошибка по вектору в метрах 1 мм
	static float vectorMistake = 0;		   // Текущая ошибка по длине в местрах
	static float max_deceleration = 0.2;   // Ускорение/замедление метры в секунду
	static SPoint point_C_;
	static float speedCurrent; // Текущая скорость

	static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
	unsigned long time_now = micros();			 // Время в которое делаем расчет
	double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;
	float accel = max_deceleration * dt; // Ускорение

	point_C_.x = msg_PoseRotation.x.odom; // Текущие координаты робота
	point_C_.y = msg_PoseRotation.y.odom;

	// float vectorFact = vectorLen(point_A_, point_C_); // Находим длину вектора который отслеживаем. Насколько уехали от точки старта
	// vectorMistake = abs(len_) - vectorFact;				   // Смотрим какое растояние еще надо проехать  Считаем ошибку по длине и включаем колеса в нужную сторону с учетом ошибки максимально заданой скорости на колесах

	// F теперь используется для задания скорости И направления.
	float signed_velLen = velLen_;		  // Скорость со знаком (F).
	float abs_velLen = std::abs(velLen_); // Абсолютная скорость для расчета .

	vectorMistake = vectorLen(point_C_, point_B_); // Находим длину вектора который отслеживаем. Сколько осталось до конечной точки

	// ROS_INFO_THROTTLE(0.1, "    vectorMistake = %7.3f", vectorMistake);

	if (flagVectorFirst)
	{
		accel = 0; // Первый запуск
		flagVectorFirst = false;
		logi.log_w("    Vector Start vectorMistake = %f metr (%+6.3f, %+6.3f -> %+6.3f, %+6.3f \n", vectorMistake, point_C_.x, point_C_.y, point_B_.x, point_B_.y);
	}
	if (abs(vectorMistake) <= minVectorMistake) // Когда ошибка по длине будет меньше заданной считаем что приехали и включаем время что-бы выйти из данного этапа алгоритма
	{
		speedCurrent = 0; // Все скорости обнуляем
		controlSpeed.control.speedL = 0;
		controlSpeed.control.speedR = 0;
		flagVector = false;
		flagVectorFirst = true;
		time_ = millis();
		logi.log_w("    Vector Final vectorMistake = %f metr (%+6.3f, %+6.3f -> %+6.3f, %+6.3f) \n", vectorMistake, point_C_.x, point_C_.y, point_B_.x, point_B_.y);
	}
	else
	{
		float V_max = calculate_max_safe_speed(vectorMistake, max_deceleration); // Считаем максимальную скорость с которой успеем остановиться

		if (V_max <= speedCurrent) // Если наша скорость больше чем допустимо то снижаем до допустимой  ЭТО ТОРМОЖЕНИЕ
			speedCurrent = V_max;
		else // ЭТО УСКОРЕНИЕ
		{
			speedCurrent = speedCurrent + accel; // Ускорение.Увеличиваем скорость
			if (speedCurrent > abs_velLen)		 // Максимальная скорость
			{
				speedCurrent = abs_velLen; // Если стала больше то ровняем
			}
		}

		// static float vectorKoef = 3.0;		   // P коефициент пид регулятора
		// float speedCurrent = abs(vectorMistake * vectorKoef); // Это простейший вариант с ПИД регулировнаием по Р
		//------------
		// ЭТО УПРАВЛЕНИЕ ПО ТРАЕКТОРИИ. СТРАЕМСЯ ЕХАТЬ НА ТОЧКУ D, лежащуу на отрезке АВ
		float L = 0.1;
		double steering = 0; // Длинна в метрах
		// point_D = findNearestSPointD(point_A, point_B, point_C, L);				 // Находим точку D на прямой между точками А и В и на расстоянии L от точки робота С
		// steering = calculateSteering(point_C, msg_Pose.th.odom, point_D, dt); //  Расчет управляющего сигнала
		steering = 0; // пока обнулим
					  //------------

		if ((speedCurrent - steering) < 0.005) // Минимальная скорость
			speedCurrent = 0.005 + steering;   // Если получается что меньше то берет так чтобы одно колесо было минимум а другое нет

		if (signed_velLen > 0) // Если скорость положительная то вращается в одну сторону или в другую
		{
			controlSpeed.control.speedL = speedCurrent + steering; // Скороть должна увеличивать до заданой или максимальной с учетом алогритма в data_node  а уменьшать будет по коефициету по ошибке
			controlSpeed.control.speedR = speedCurrent - steering;
		}
		else
		{
			controlSpeed.control.speedL = -speedCurrent + steering;
			controlSpeed.control.speedR = -speedCurrent - steering;
		}
		logi.log("    workVector mistake = %7.3f (%+6.3f, %+6.3f -> %+6.3f, %+6.3f) | V_max = %f | fact speedL = %f speedR = %f | dt = %f  accel = %f \n",
				 vectorMistake, point_C_.x, point_C_.y, point_B_.x, point_B_.y, V_max, controlSpeed.control.speedL, controlSpeed.control.speedR, dt, accel);
	}
}

// // Структура для представления точки
// struct Point
// {
//     double x; // Координата X
//     double y; // Координата Y
// };

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

//     //ROS_INFO("newSpeed = %+8.3f", speed);
//     //INFO ROS_INFO("-------------- ");
//     //-----------------------------------------------------------
// }

#endif
