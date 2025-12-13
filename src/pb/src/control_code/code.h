#ifndef CODE_H
#define CODE_H

#include <cmath>
#include <cstdio> // Для функции printf
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cstddef> // Для size_t

#include "l1Controller.h" //Класс для контроллера управления

// #include "pillar.h"
//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************

// Константы
const double PI = 3.14159265358979323846;
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;

void readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

void initCommandArray(int verCommand_); // Заполнение маасива команд

void workAngle(float angle_, float theory_delta, u_int64_t &time_, float velAngle_);						   // Тут отрабатываем алгоритм отслеживания угла при повороте
void workVector(float len_, SPoint point_A_, SPoint point_B_, u_int64_t &time_, float velLen_);				   // Тут отрабатываем алгоритм отслеживания длины вектора при движении прямо
float calculate_max_safe_speed(float distance_to_stop, float max_deceleration);								   // Вычисляет максимально допустимую скорость для остановки.
float calculate_max_safe_angular_speed_degrees(float angular_error_deg, float max_angular_acceleration_degs2); // Вычисляет максимально допустимую угловую скорость для поворота на месте, используя ГРАДУСЫ.
float convert_angular_speed_to_linear_wheel_speed(float angular_speed_degs, float wheel_base_m);			   // Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с).
void callback_Joy(sensor_msgs::Joy msg);																	   // Функция обраьтного вызова по подпичке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
SPoint calculate_new_coordinates(SPoint point_A_, float angle_rad, float distance);							   // Расчет новых координат по старому положению и длинне вектора

// pb_msgs::SControlDriver speedCorrect(pb_msgs::SDriver2Data Driver2Data_msg_, pb_msgs::SControlDriver Data2Driver_); // Корректировка скорости движения в зависимости от датчиков растояния перед
// void collectCommand(); // //Функция формирования команды для нижнего уровня на основе всех полученных данных, датчиков и анализа ситуации

// **********************************************************************************
// void callback_Driver(pb_msgs::Struct_Driver2Data msg)
// {
// 	msg_Driver2Data = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
// 	flag_msgDriver = true;
// }
#include <cstddef> // Для size_t
#include <limits>  // Для numeric_limits

class RisingTrendChecker
{
public:
	// Конструктор: задаём требуемое количество последовательных ростов (N)
	RisingTrendChecker(size_t N)
	{
		if (N == 0)
		{
			N_required_ = 1; // Защита от N=0, минимум 1 рост
		}
		else
		{
			N_required_ = N; // Устанавливаем требуемую глубину роста
		}

		reset(); // Инициализируем состояние
	}

	// Функция для сброса состояния
	void
	reset()
	{
		// Начинаем с очень большого отрицательного числа.
		// Это гарантирует, что первое реальное значение (ошибка > 0)
		// всегда будет больше предыдущего.
		previous_value_ = std::numeric_limits<double>::lowest(); // Инициализируем минимально возможным значением double

		consecutive_rises_ = 0; // Сбрасываем счетчик последовательных ростов
	}

	// Основная функция: Принимает текущее значение ошибки.
	// Возвращает true, если значение возрастало N_required_ раз подряд.
	bool
	check_for_rising_trend(double current_value)
	{
		// 1. Проверяем условие роста: текущее значение строго больше предыдущего
		if (current_value > previous_value_)
		{
			// Условие выполнено: был рост
			consecutive_rises_++; // Увеличиваем счетчик последовательных ростов
		}
		else
		{
			// Условие не выполнено: произошло снижение или плато
			consecutive_rises_ = 0; // Сбрасываем счетчик, тренд прерван
		}

		// 2. Запоминаем текущее значение для следующего шага
		previous_value_ = current_value; // Обновляем предыдущее значение

		// 3. Проверяем условие остановки: достигнут ли требуемый порог ростов?
		if (consecutive_rises_ >= N_required_)
		{
			return true; // Возвращаем true, тренд подтвержден
		}

		return false; // Возвращаем false, условие не достигнуто
	}

private:
	size_t N_required_;		   // Требуемое количество последовательных ростов
	size_t consecutive_rises_; // Счетчик последовательных ростов
	double previous_value_;	   // Последнее переданное значение
};

const size_t N_GROWTH = 5;			  // Требуем 3 последовательных роста
RisingTrendChecker checker(N_GROWTH); // Создаем объект

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
// Возвращаем точку С в зависимости от выбранного режима управления
// SPose getPose_C(int controlMode_)
// {
// 	SPose pose;
// 	switch (controlMode_)
// 	{
// 		case 0:
// 		{
// 			pose.x = msg_PoseRotation.x.odom;
// 			pose.y = msg_PoseRotation.y.odom;
// 			pose.th = msg_PoseRotation.th.odom;
// 			break; // <--- ОБЯЗАТЕЛЬНО
// 		}
// 		case 1:
// 		{
// 			pose.x = msg_PoseRotation.x.model;
// 			pose.y = msg_PoseRotation.y.model;
// 			pose.th = msg_PoseRotation.th.model;
// 			break; // <--- ОБЯЗАТЕЛЬНО
// 		}
// 		case 2:
// 		{
// 			pose.x = msg_PoseRotation.x.est;
// 			pose.y = msg_PoseRotation.y.est;
// 			pose.th = msg_PoseRotation.th.est;
// 			break; // <--- ОБЯЗАТЕЛЬНО
// 		}
// 		default:
// 		{
// 			pose.x = msg_PoseRotation.x.odom;
// 			pose.y = msg_PoseRotation.y.odom;
// 			pose.th = msg_PoseRotation.th.odom;
// 			break; // <--- ОБЯЗАТЕЛЬНО
// 		}
// 	}
// 	return pose;
// }

// Возвращаем точку С. 
// controlMode_ - глобальная настройка (0=odom, 2=est)
// use_smooth_logic - флаг из текущей команды (ехать плавно по модели)
SPose getPose_C(int controlMode_, bool use_smooth_logic)
{
    // 1. ПРИОРИТЕТ: Если команда требует плавности (Model + Offset)
    if (use_smooth_logic)
    {
        SPose p;
        // Берем модель и добавляем замороженный оффсет Мы переходим на управление по Модели.
        // Чтобы робот не дернулся, Модель должна "подхватить" текущую позицию Оценки. Считаем разницу: Offset = Est - Model
        p.x = msg_PoseRotation.x.model + g_transition_offset.x;
        p.y = msg_PoseRotation.y.model + g_transition_offset.y;
        
        // Угол складываем и нормализуем  Для угла важно использовать нормализацию разности!
        // Функция normalizeAngle180 должна быть доступна (или аналог) Если нет функции, используй ручной расчет разницы радианов
        double th_sum = msg_PoseRotation.th.model + g_transition_offset.th;
        while (th_sum > M_PI) th_sum -= 2*M_PI; 
        while (th_sum <= -M_PI) th_sum += 2*M_PI;
        p.th = th_sum;
        
        return p; 
    }

    // 2. ОБЫЧНЫЙ РЕЖИМ (Если плавность не нужна)
    SPose pose;
    switch (controlMode_)
    {
        case 0: // Odom
            pose.x = msg_PoseRotation.x.odom;
            pose.y = msg_PoseRotation.y.odom;
            pose.th = msg_PoseRotation.th.odom;
            break; 
        
        case 1: // Pure Model (без оффсета, обычно не используется для управления)
            pose.x = msg_PoseRotation.x.model;
            pose.y = msg_PoseRotation.y.model;
            pose.th = msg_PoseRotation.th.model;
            break; 
        
        case 2: // Est (По умолчанию)
            pose.x = msg_PoseRotation.x.est;
            pose.y = msg_PoseRotation.y.est;
            pose.th = msg_PoseRotation.th.est;
            break; 
            
        default:
            pose.x = msg_PoseRotation.x.est; // Безопасный дефолт
            pose.y = msg_PoseRotation.y.est;
            pose.th = msg_PoseRotation.th.est;
            break;
    }
    return pose;
}


	void readParam() // Считывание переменных параметров из лаунч или yaml файла при запуске. Там офсеты и режимы работы
	{
		ros::NodeHandle nh_global;										   // <--- Используется для доступа к /pb_config/ // Создаем ГЛОБАЛЬНЫЙ обработчик, который ищет параметры, начиная с корня (/).
		nh_global.param<int>("/pb_config/control_mode", g_controlMode, 0); // # Режим работы управления. Если 0 то по колесам одометрии, 1 управление по скомплеменированному с измерением main
		logi.log_b("+++ Start node with parametrs: controlMode = %i\n", g_controlMode);
		// ros::Duration(10).sleep(); // Подождем пока
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

	/**
	 * @brief Вычисляет максимально допустимую скорость для остановки.
	 * Рассчитывает максимальную скорость, которую должно иметь тело, чтобы успеть остановиться на расстоянии 'distance_to_stop' при условии немедленного применения максимального замедления 'max_deceleration'.
	 * @param distance_to_stop Расстояние до точки остановки (метры).
	 * @param max_deceleration Максимально возможное ускорение/замедление (м/с^2).  Должно быть положительным.
	 * @return Максимально допустимая скорость (м/с). Возвращает 0.0, если расстояние < 0.
	 *  * Рассчитывает торможение так, чтобы скорость упала в 0 за 'margin' метров ДО цели.
	 */
	float calculate_max_safe_speed(float distance_to_stop, float max_deceleration)
	{
		if (distance_to_stop <= 0.0 || max_deceleration <= 0.0) // Защита от отрицательного расстояния или нулевого ускорения
        	return 0.0;

		// ЗАПАС (Margin). Робот будет думать, что стена стоит на 1 см ближе, и оттормозится перед ней.
		float margin = 0.01; // 10 мм
		float dist_for_calc = distance_to_stop - margin; // Эффективная дистанция для расчета торможения

		if (dist_for_calc <= 0.0)      // Если мы уже вошли в зону запаса (ближе 10 мм)
			return 0.0; // Говорим "Стоп". Но в workVector сработает ограничитель минимальной скорости, и робот будет ползти.

    	// Обычная формула, но для укороченной дистанции: v = sqrt(2 * a * S)
		// Где:
		// v - максимально допустимая скорость (м/с) 
		// a - максимальное замедление (м/с^2)
		// S - расстояние до остановки (метры)
    	return std::sqrt(2.0 * max_deceleration * dist_for_calc);

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
		logi.log_g("+++ Initial Position point_A (X, Y): (%.4f, %.4f)\n", point_A_.x, point_A_.y);
		logi.log("    Movement Vector (signed_distance, Angle rad): '%.4f, %.4f'\n", signed_distance, angle_rad);
		logi.log("    Delta Position (dX, dY): /%.4f, %.4f/\n", dx, dy);
		logi.log_g("    New Position point_B (X', Y'): <%.4f, %.4f>\n", point_B.x, point_B.y);
		return point_B;
	}


	// Тут отрабатываем алгоритм отслеживания угла при повороте
	void workAngle(float angle_, float theory_delta, u_int64_t &time_, float velAngle_)
	{

		static float minAngleMistake = 0.02;			 // Минимальная ошибка по углу в Градусах
		static float angleMistake = 0;					 // Текущая ошибка по углу в градусах
		const float max_angular_acceleration_degs2 = 15; // Угловое ускорение/замедление в градусах в секунду
		// static float max_deceleration = 0.1;			 // Линейное Ускорение/замедление метры в секунду
		static float speedCurrent = 0;

		static unsigned long time = micros(); // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
		unsigned long time_now = micros();	  // Время в которое делаем расчет
		
		// === ЛОГИКА ===
		float angleFact = g_poseC.th; // Угол который отслеживаем
		// logi.log("    angleFact main alfa= %+8.3f\n", angleFact);
		angleMistake = angle_ - RAD2DEG(angleFact);		// Смотрим какой угол.// Смотрим куда нам надо Считаем ошибку по углу и включаем колеса в нужную сторону с учетом ошибки по углу и максимально заданой скорости на колесах
		angleMistake = normalizeAngle180(angleMistake); // Нормализуем +-180

		if (flagAngleFirst) // Первый запуск (Инициализация)
		{
			flagAngleFirst = false;
			speedCurrent = 0;
			time = time_now; // "Забываем" старое время
			logi.log_r("    Angle Start fact= %+8.3f target= %+8.3f angle_angleMistake = %+8.3f gradus \n", RAD2DEG(angleFact), angle_, angleMistake);
		}

		double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
		time = time_now;

		if (dt > 0.2)
			dt = 0.1; // Ограничим 100мс  Если программа подвисла, не даем физике "улететь"

		if ((abs(angleMistake) <= minAngleMistake)) // Проверка достижения цели Когда ошибка по углу будет меньше заданной считаем что приехали и включаем время что-бы выйти из данного этапа алгоритма
		{
			speedCurrent = 0;
			controlSpeed.control.speedL = 0;
			controlSpeed.control.speedR = 0;
			flagAngle = false;
			flagAngleFirst = true;
			time_ = millis();
	        stats.end_move(theory_delta, true);  // Передаем theory_angle_delta в статистику как "длину теории" для поворота
			logi.log_w("    Angle Final angleMistake = %f gradus \n", angleMistake);
		}
		else
		{

			float V_max_ang_braking = calculate_max_safe_angular_speed_degrees(angleMistake, max_angular_acceleration_degs2); // Считаем максимальную скорость с которой успеем остановиться Функция должна вернуть макс. скорость в град/сек
			float V_max_lin_braking = convert_angular_speed_to_linear_wheel_speed(V_max_ang_braking, DISTANCE_WHEELS);		  // Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с).

			float linear_accel_per_sec = convert_angular_speed_to_linear_wheel_speed(max_angular_acceleration_degs2, DISTANCE_WHEELS); // Считаем ШАГ УСКОРЕНИЯ (Ramp Up) Синхронизируем разгон с торможением!

			float accel_step = linear_accel_per_sec * dt; // Ускорение/замедление
			speedCurrent = speedCurrent + accel_step;	  // Ускорение.Увеличиваем скорость Применяем разгон

			if (V_max_lin_braking <= speedCurrent) // Если наша скорость больше чем допустимо то снижаем до допустимой  ЭТО ТОРМОЖЕНИЕ
				speedCurrent = V_max_lin_braking;
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

			point_C.x = g_poseC.x; //
			point_C.y = g_poseC.y;

			if (adv_log)
			{
			logi.log("    point C=' %+8.3f %+8.3f ' real= ' %+8.3f ' target= %+8.3f  mistake= %+8.3f | V_max_ang_braking= %+8.3f  V_max_lin_braking= %+8.3f | real L= %+8.3f R= %+8.3f \n",
					 point_C.x, point_C.y, RAD2DEG(angleFact), angle_, angleMistake, V_max_ang_braking, V_max_lin_braking, controlSpeed.control.speedL, controlSpeed.control.speedR);
			}
		}
	}

	// Тут отрабатываем алгоритм отслеживания длины вектора при движении прямо
	void workVector(float len_, SPoint point_A_, SPoint point_B_, u_int64_t &time_, float velLen_)
	{
		static float minVectorMistake = 0.001; // Минимальная ошибка по вектору в метрах 1 мм
		static float vectorMistake = 0;		   // Текущая ошибка по длине в местрах
		static float max_deceleration = 0.1;   // Ускорение/замедление метры в секунду
		static SPoint point_C_;
		static float speedCurrent; // Текущая скорость

		static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
		unsigned long time_now = micros();			 // Время в которое делаем расчет


		point_C_.x = g_poseC.x; // Текущие координаты робота
		point_C_.y = g_poseC.y; // Текущие координаты робота
		// logi.log("    point C main    x = %+8.3f y = %+8.3f \n", point_C.x, point_C.y);

		// F теперь используется для задания скорости И направления.
		float signed_velLen = velLen_;		  // Скорость со знаком (F).
		float abs_velLen = std::abs(velLen_); // Абсолютная скорость для расчета .

		vectorMistake = vectorLen(point_C_, point_B_); // Находим длину вектора который отслеживаем. Сколько осталось до конечной точки. Новая ошибка.

		if (flagVectorFirst)
		{
			flagVectorFirst = false;
    		time = time_now; // И сброс таймера (как в workAngle)!
    		speedCurrent = 0; // Если мы начинаем с места - сброс. Для начала считаем, что всегда стартуем с 0.
			// accel = 0; // Первый запуск
			logi.log_r("    Vector Start vectorMistake = %f metr (%+9.5f, %+9.5f -> %+6.3f, %+6.3f) \n", vectorMistake, point_C_.x, point_C_.y, point_B_.x, point_B_.y);
			 
			 // --- ДОБАВКА: РАСЧЕТ ТОРМОЗНОГО ПУТИ ---
            // Считаем, сколько метров нужно роботу, чтобы остановиться с максимальной скорости (velLen) Формула: S = V^2 / (2 * a)
            float v_target = abs(velLen_);
            float braking_dist = (v_target * v_target) / (2.0 * max_deceleration);
            
            logi.log_b("    [DYNAMICS CHECK] Target V=%.3f m/s. Required Braking Dist=%.3f m.\n", v_target, braking_dist);
            if (braking_dist > vectorMistake) 
                logi.log_r("    WARNING: Path too short for full speed! Will not reach %.3f m/s.\n", v_target);
            
		}

		double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
		time = time_now;
		float accel = max_deceleration * dt; // Ускорение

		bool flagTrendMistake = checker.check_for_rising_trend(abs(vectorMistake)); // Добавляем и получаем логическое true усли есть 3 значения подряд растет ошибка
		// logi.log("    flagTrendMistake= %d \n", flagTrendMistake);

		double speedL = 0; // OUT скорости колес ЛЕВОГО
		double speedR = 0; // OUT скорости колес ПРАВОГО

		if ((abs(vectorMistake) <= minVectorMistake) || flagTrendMistake) // Когда ошибка по длине будет меньше заданной или начнет увеличиваться считаем что приехали и включаем время что-бы выйти из данного этапа алгоритма
		{
			speedCurrent = 0; // Все скорости обнуляем
        	controlSpeed.control.speedL = 0.0;// --- ИСПРАВЛЕНИЕ: Явно останавливаем моторы в структуре ---
        	controlSpeed.control.speedR = 0.0;
			flagVector = false;
			flagVectorFirst = true;
			time_ = millis();
			if (flagTrendMistake)
				logi.log_r("+++ STOP on flagTrendMistake +++ \n");
			logi.log_w("    Vector Final vectorMistake = %+6.3f metr (%+6.3f, %+6.3f -> %+6.3f, %+6.3f) \n", vectorMistake, point_C_.x, point_C_.y, point_B_.x, point_B_.y);
        	
        	stats.end_move(len_, false); // Передаем len_ (теорию) и false (это не поворот, а движение)// Передаем только теорию и тип
		}
		else
		{
			// ============================================== ЭТО УПРАВЛЕНИЕ ОБЩЕЙ СКОРОСТЬ  =================================================
			speedCurrent = abs(speedCurrent);										 // Считаем всегда по модулю, а потом в зависимости от направления меняем знак на - если нужно
			float V_max = calculate_max_safe_speed(vectorMistake, max_deceleration); // Считаем максимальную скорость с которой успеем остановиться
			if (V_max <= speedCurrent)												 // Если наша скорость больше чем допустимо то снижаем до допустимой
				speedCurrent = V_max;												 // ЭТО ТОРМОЖЕНИЕ Тут speedCurrent станет 0 в зоне 10 мм
			else																	 // ЭТО УСКОРЕНИЕ
			{
				speedCurrent = speedCurrent + accel; // Ускорение.Увеличиваем скорость
				if (speedCurrent > abs_velLen)		 // Максимальная скорость
				{
					speedCurrent = abs_velLen; // Если стала больше то ровняем
				}
			}
			if ((speedCurrent) < 0.005) // Минимальная скорость Это и есть режим "ползти", когда V_max уже обнулилась
				speedCurrent = 0.005;	// Ползем

			if (signed_velLen < 0) // Если скорость положительная то вращается в одну сторону или в другую
			{
				speedCurrent = -speedCurrent;
				// logi.log_b("=== speedCurrent = -speedCurrent %+8.3f \n", speedCurrent);
				
			}

			// ============================================== ЭТО УПРАВЛЕНИЕ ПО ТРАЕКТОРИИ. СТРАЕМСЯ ЕХАТЬ НА ТОЧКУ D, лежащуу на отрезке АВ =================================================
			double omega = 0;			 // OUT: Угловая скорость (рад/с).
			double angle_error_rad = 0;	 // OUT: Угловая ошибка рад. По ней считается все управление.
			double target_angle_rad = 0; // Выходные данные (для отладки)
			double heading_used_rad = 0; // Выходные данные (для отладки)
			SPoint point_D;
			SPoint point_D2;

			// if (g_controlMode) // Тут управление колесами. Берем общую скорость как основу и по расчету притормаживаем одно и ускоряем другое колесо.
			// {
			double L_lookahead = 0.25;											  // Дистанция упреждения L Не больше половины ширины колес
			static L1GuidanceController controller(L_lookahead, DISTANCE_WHEELS); // Создание контроллера:
			controller.setMaxOmega(0.20);
			controller.setMaxSteeringRatio(1.0);

			point_D = controller.findNearestPointD_Exact(point_A_, point_B_, point_C_);																								// Находим точку D на прямой между точками А и В
			point_D2 = controller.findNearestPointD_2var(point_A_, point_B_, point_C_);																								// Находим точку D на прямой между точками А и В
			controller.calculateControlCommands(point_C_, g_poseC.th, point_D, point_B_, speedCurrent, omega, speedL, speedR, angle_error_rad, target_angle_rad, heading_used_rad); //  Расчет управляющего сигнала
			// }
			// else // Тут без управления. Просто скорость на колеса со знаком
			// {

			// 	if (signed_velLen > 0) // Если скорость положительная то вращается в одну сторону или в другую
			// 	{
			// 		speedL = speedCurrent; // Скороть должна увеличивать до заданой или максимальной с учетом алогритма в data_node  а уменьшать будет по коефициету по ошибке
			// 		speedR = speedCurrent;
			// 	}
			// 	else
			// 	{
			// 		speedL = -speedCurrent;
			// 		speedR = -speedCurrent;
			// 	}
			// }
			// A= (%+8.3f %+8.3f ) B= (%+8.3f %+8.3f ) point_A_.x, point_A_.y, 							point_B_.x, point_B_.y,
			// D2= ( %+8.3f %+8.3f )point_D2.x, point_D2.y,
			if (adv_log)
			{
			logi.log("    pose C=' %+8.3f %+8.3f %+8.3f 'D= %+8.3f %+8.3f Mistake= %+8.5f |Vel= %+8.3f |speed L= %+8.3f R= %+8.3f '|%= %+6.1f ' omega= %+8.3f |heading= %+8.3f target_angle= %+8.3f angle_error= %+8.3f \n",
					 point_C_.x, point_C_.y, RAD2DEG(g_poseC.th),
					 point_D.x, point_D.y,
					 vectorMistake, speedCurrent,
					 speedL, speedR, (speedL / speedR) * 100, omega,
					 RAD2DEG(heading_used_rad), RAD2DEG(target_angle_rad), RAD2DEG(angle_error_rad));
			}
			controlSpeed.control.speedL = speedL;
			controlSpeed.control.speedR = speedR;

			// logi.log("    workVector mistake = %8.5f (%+9.5f, %+9.5f -> %+6.3f, %+6.3f) | V_max = %f | fact speedL = %f speedR = %f | dt = %f  accel = %f \n",
			// 		 vectorMistake, point_C_.x, point_C_.y, point_B_.x, point_B_.y, V_max, controlSpeed.control.speedL, controlSpeed.control.speedR, dt, accel);
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
