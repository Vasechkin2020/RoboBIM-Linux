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

double normalizeAngle(double angle_rad);												// Функции нормализации угла и PIDController остаются без изменений
SPoint findNearestSPointD(const SPoint &A, const SPoint &B, const SPoint &C, double L); // Находит целевую точку D, которая лежит на окружности C(L) И на отрезке [A, B].
double clampT(double t);																// Вспомогательная функция: Ограничивает параметр t диапазоном [0, 1]
double distanceSq(const SPoint &p1, const SPoint &p2);									// Вспомогательная функция для расчета квадрата расстояния

// Класс ПИД-регулятора
class PIDController
{						   // Начало определения класса ПИД-регулятора
private:				   // Приватные члены класса (доступны только внутри класса)
	double Kp;			   // Коэффициент пропорциональной составляющей
	double Ki;			   // Коэффициент интегральной составляющей
	double Kd;			   // Коэффициент дифференциальной составляющей
	double integral;	   // Накопленная ошибка (для интегральной составляющей)
	double previous_error; // Ошибка на предыдущем шаге (для дифференциальной составляющей)

public:															  // Публичные члены класса (доступны снаружи)
	PIDController(double p, double i, double d)					  // Конструктор класса
		: Kp(p), Ki(i), Kd(d), integral(0.0), previous_error(0.0) // Инициализация коэффициентов и начальных значений
	{
	} // Тело конструктора пустое, так как инициализация выполнена

	double update(double error_rad, double dt)				   // Метод обновления регулятора
	{														   // Начало метода update
		double proportional = Kp * error_rad;				   // Вычисление пропорциональной составляющей (P)
		integral += error_rad * dt;							   // Накопление ошибки (суммирование ошибки * время)
		double integral_term = Ki * integral;				   // Вычисление интегральной составляющей (I)
		double derivative = (error_rad - previous_error) / dt; // Вычисление скорости изменения ошибки
		double derivative_term = Kd * derivative;			   // Вычисление дифференциальной составляющей (D)
		previous_error = error_rad;							   // Сохранение текущей ошибки для следующего шага
		return proportional + integral_term + derivative_term; // Возврат управляющего сигнала
	} // Конец метода update
}; // Конец определения класса

double Kp = 1.0; // Коэффициенты ПИД-регулятора
double Ki = 0.0;
double Kd = 0.05;

PIDController steering_pid(Kp, Ki, Kd);																	   // 2. Создаем ПИД-контроллер
																										   // Упреждающее объявление класса
double calculateSteering(const SPoint &point_C_, double robotAngleDeg, const SPoint &point_D_, double dt); // Главная функция управления
double shortestDistanceToSegment(const SPoint &A, const SPoint &B, const SPoint &C);					   // Вычисляет кратчайшее расстояние от точки C до отрезка AB.

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
		float V_max_lin = convert_angular_speed_to_linear_wheel_speed(V_max_ang, DISTANCE_WHEELS);				  // Преобразует угловую скорость робота (град/с) в линейную скорость колес (м/с).
		ROS_INFO_THROTTLE(0.1, "    workAngle V_max_ang = %f  speedCurrent V_max_lin = %f", V_max_ang, V_max_lin);

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

/**
 * @brief Рассчитывает новые координаты на основе вектора движения и выводит их через printf.
 * * Предполагается, что angle_rad отсчитывается от положительной оси X (в радианах).
 * * @param current_x Текущая координата X (double)
 * @param current_y Текущая координата Y (double)
 * @param angle_rad Угол направления в радианах
 * @param distance Пройденное расстояние
 */
SPoint calculate_new_coordinates(SPoint point_A_, float angle_rad, float distance)
{
	// Расчет приращения по осям
	float dx = distance * std::cos(angle_rad);
	float dy = distance * std::sin(angle_rad);

	SPoint point_B; // Новые координаты
	point_B.x = point_A_.x + dx;
	point_B.y = point_A_.y + dy;
	// Вывод результатов с использованием printf
	// Используем "%.4f" для вывода чисел с плавающей точкой с точностью до 4 знаков после запятой
	printf("--- Calculation Result ---\n");
	printf("Initial Position point_A (X, Y): (%.4f, %.4f)\n", point_A_.x, point_A_.y);
	printf("Movement Vector (Distance, Angle rad): (%.4f, %.4f)\n", distance, angle_rad);
	printf("Delta Position (dX, dY): (%.4f, %.4f)\n", dx, dy);
	printf("New Position point_B (X', Y'): (%.4f, %.4f)\n", point_B.x, point_B.y);
	printf("--------------------------\n");
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

	point_C_.x = msg_Pose.x.odom; // Текущие координаты робота
	point_C_.y = msg_Pose.y.odom;

	// float vectorFact = vectorLen(point_A_, point_C_); // Находим длину вектора который отслеживаем. Насколько уехали от точки старта
	// vectorMistake = abs(len_) - vectorFact;				   // Смотрим какое растояние еще надо проехать  Считаем ошибку по длине и включаем колеса в нужную сторону с учетом ошибки максимально заданой скорости на колесах

	vectorMistake = vectorLen(point_C_, point_B_); // Находим длину вектора который отслеживаем. Сколько осталось до конечной точки

	// ROS_INFO_THROTTLE(0.1, "    vectorMistake = %7.3f", vectorMistake);

	if (flagVectorFirst)
	{
		accel = 0; // Первый запуск
		flagVectorFirst = false;
		ROS_INFO("    Vector Start vectorMistake = %f metr (%+6.3f, %+6.3f -> %+6.3f, %+6.3f)", vectorMistake, point_C_.x, point_C_.y, point_B_.x, point_B_.y);
	}
	if (abs(vectorMistake) <= minVectorMistake) // Когда ошибка по длине будет меньше заданной считаем что приехали и включаем время что-бы выйти из данного этапа алгоритма
	{
		speedCurrent = 0; // Все скорости обнуляем
		controlSpeed.control.speedL = 0;
		controlSpeed.control.speedR = 0;
		flagVector = false;
		flagVectorFirst = true;
		time_ = millis();
		ROS_INFO("    Vector Final vectorMistake = %f metr (%+6.3f, %+6.3f -> %+6.3f, %+6.3f)", vectorMistake, point_C_.x, point_C_.y, point_B_.x, point_B_.y);
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
			if (speedCurrent > velLen_)			 // Максимальная скорость
			{
				// ROS_INFO("   MAX speedCurrent = %f velLen_ = %f ", speedCurrent, velLen_);
				speedCurrent = velLen_; // Если стала больше то ровняем
			}
		}

		// static float vectorKoef = 3.0;		   // P коефициент пид регулятора
		// float speedCurrent = abs(vectorMistake * vectorKoef); // Это простейший вариант с ПИД регулировнаием по Р
		// ROS_INFO_THROTTLE(0.1, "    speedCurrent vectorKoef = %f", speedCurrent);

		// ЭТО УПРАВЛЕНИЕ ПО ТРАЕКТОРИИ. СТРАЕМСЯ ЕХАТЬ НА ТОЧКУ D, лежащуу на отрезке АВ
		float L = 0.1;
		double steering = 0; // Длинна в метрах
		// point_D = findNearestSPointD(point_A, point_B, point_C, L);				 // Находим точку D на прямой между точками А и В и на расстоянии L от точки робота С
		// steering = calculateSteering(point_C, msg_Pose.th.odom, point_D, dt); //  Расчет управляющего сигнала

		steering = 0;						   // пока обнулим
		if ((speedCurrent - steering) < 0.005) // Минимальная скорость
			speedCurrent = 0.005 + steering;   // Если получается что меньше то берет так чтобы одно колесо было минимум а другое нет

		if (len_ > 0) // Если длина положительная то вращается в одну сторону или в другую
		{
			controlSpeed.control.speedL = speedCurrent + steering; // Скороть должна увеличивать до заданой или максимальной с учетом алогритма в data_node  а уменьшать будет по коефициету по ошибке
			controlSpeed.control.speedR = speedCurrent - steering;
		}
		else
		{
			controlSpeed.control.speedL = -speedCurrent + steering;
			controlSpeed.control.speedR = -speedCurrent - steering;
		}
		ROS_INFO_THROTTLE(0.1, "    workVector vectorMistake = %7.3f (%+6.3f, %+6.3f -> %+6.3f, %+6.3f) | V_max = %f | fact speedL = %f speedR = %f | dt = %f  accel = %f",
						  vectorMistake, point_C_.x, point_C_.y, point_B_.x, point_B_.y, V_max, controlSpeed.control.speedL, controlSpeed.control.speedR, dt, accel);
	}
}

// // Структура для представления точки
// struct Point
// {
//     double x; // Координата X
//     double y; // Координата Y
// };

// Вспомогательная функция для расчета квадрата расстояния
double distanceSq(const SPoint &p1, const SPoint &p2)
{
	return std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2); // (dx^2 + dy^2)
}

// Вспомогательная функция: Ограничивает параметр t диапазоном [0, 1]
double clampT(double t)
{
	return std::max(0.0, std::min(t, 1.0)); // Возвращает t, ограниченный отрезком [0, 1]
}

/**
 * @brief Находит целевую точку D, которая лежит на окружности C(L) И на отрезке [A, B].
 * Если пересечений нет, возвращает ближайшую к C точку на отрезке [A, B].
 */
SPoint findNearestSPointD(const SPoint &A, const SPoint &B, const SPoint &C, double L)
{
	// 1. ПРОВЕРКА B: Если B внутри или на границе окружности L

	const double LSq = L * L;	 // Квадрат радиуса (L^2)
	if (distanceSq(C, B) <= LSq) // Если d(C, B) <= L
	{
		return B; // Искомая точка D - это B (по условию: ближайшая к B)
	}

	// 2. ПОИСК ПЕРЕСЕЧЕНИЙ с БЕСКОНЕЧНОЙ прямой AB

	const double dx = B.x - A.x;		   // Разность координат X (вектор AB)
	const double dy = B.y - A.y;		   // Разность координат Y (вектор AB)
	const double abSq = dx * dx + dy * dy; // |AB|^2

	if (abSq == 0.0) // Защита от A == B
	{
		return A; // Возвращаем A или B
	}

	const double Ax = A.x - C.x; // Компонента (xA - xC)
	const double Ay = A.y - C.y; // Компонента (yA - yC)

	// Коэффициенты для квадратного уравнения a*t^2 + b*t + c = 0
	const double a = abSq;						// a = |AB|^2
	const double b = 2.0 * (dx * Ax + dy * Ay); // b = 2 * (AB * AC) (скалярное произведение)
	const double c = Ax * Ax + Ay * Ay - LSq;	// c = |AC|^2 - L^2

	const double D = b * b - 4.0 * a * c; // Дискриминант D = b^2 - 4ac

	// Переменная для хранения наилучшего t (ближайшего к B, т.е. максимального)
	double bestT = -1.0; // Значение < 0 означает, что пересечения на отрезке нет

	if (D >= 0.0) // Если есть действительные пересечения с прямой
	{
		const double sqrtD = std::sqrt(D);			// Корень из дискриминанта
		const double t1 = (-b + sqrtD) / (2.0 * a); // Параметр t для первого решения
		const double t2 = (-b - sqrtD) / (2.0 * a); // Параметр t для второго решения

		// 3. ФИЛЬТРАЦИЯ: Оставляем только t, лежащие на отрезке [0, 1]

		if (t1 >= 0.0 && t1 <= 1.0) // t1 находится на отрезке [A, B]
		{
			bestT = std::max(bestT, t1); // Берем, так как он ближе к B, если t2 < t1
		}

		if (t2 >= 0.0 && t2 <= 1.0) // t2 находится на отрезке [A, B]
		{
			bestT = std::max(bestT, t2); // Берем, если t2 > t1 (или t1 был недопустим)
		}
	}

	// 4. ВЫБОР РЕШЕНИЯ

	if (bestT >= 0.0) // Если найдено хотя бы одно действительное пересечение на отрезке
	{
		// Возвращаем точку D, которая лежит на окружности и на отрезке
		return {
			A.x + bestT * dx, // Px = xA + t*dx
			A.y + bestT * dy  // Py = yA + t*dy
		};
	}

	// 5. НЕТ ПЕРЕСЕЧЕНИЙ на отрезке: Возвращаем ближайшую точку на отрезке (проекцию)

	// Находим параметр t_proj для проекции C на бесконечную прямую AB
	// t_proj = [ (xC - xA) * dx + (yC - yA) * dy ] / |AB|^2
	// Числитель: (xC - xA) * dx + (yC - yA) * dy = -(dx * Ax + dy * Ay) = -b/2
	const double t_proj = -(dx * Ax + dy * Ay) / abSq;

	// Ограничиваем t_proj отрезком [0, 1] (если проекция вне отрезка, берем A или B)
	const double t_clamped = clampT(t_proj);

	return // Возвращаем ближайшую точку на отрезке [A, B] к центру C
		{
			A.x + t_clamped * dx, // Px = xA + t_clamped*dx
			A.y + t_clamped * dy  // Py = yA + t_clamped*dy
		};
}

// Функции нормализации угла и PIDController остаются без изменений
double normalizeAngle(double angle_rad)
{
	angle_rad = std::fmod(angle_rad, 2.0 * PI);
	if (angle_rad <= -PI)
		angle_rad += 2.0 * PI;
	if (angle_rad > PI)
		angle_rad -= 2.0 * PI;
	return angle_rad;
}

/**
 * @brief Главная функция управления
 */
double calculateSteering(const SPoint &point_C_, double robotAngleDeg, const SPoint &point_D_, double dt)
{
	double dx = point_D_.x - point_C_.x; // 1. Расчет целевого угла (Target Angle)
	double dy = point_D_.y - point_C_.y;
	double target_angle_rad = std::atan2(dy, dx);

	double robot_angle_rad = robotAngleDeg * DEG_TO_RAD; // 2. Расчет ошибки по углу (Angle Error)
	double angle_error_rad = target_angle_rad - robot_angle_rad;
	angle_error_rad = normalizeAngle(angle_error_rad);

	double steering_speed = steering_pid.update(angle_error_rad, dt); // 3. Применение ПИД-регулятора

	printf("\n=================================================\n");
	printf("Точка D: (%.4lf, %.4lf) из C: (%.4lf, %.4lf)\n", point_D_.x, point_D_.y, point_C_.x, point_C_.y);
	printf(" -> Целевой угол: %.4lf\u00b0 (%.4lf рад)\n", target_angle_rad * RAD_TO_DEG, target_angle_rad);
	printf(" -> Угол робота:  %.4lf\u00b0 (%.4lf рад)\n", robotAngleDeg, robot_angle_rad);
	printf(" -> ОШИБКА:      %.4lf\u00b0 (%.4lf рад)\n", angle_error_rad * RAD_TO_DEG, angle_error_rad);
	printf(" -> УПРАВЛЕНИЕ:  %.4lf рад/с (Угловая скорость)\n", steering_speed);

	return steering_speed;
}

/**
 * @brief Вычисляет кратчайшее расстояние от точки C до отрезка AB.
 * * @param A Начальная точка отрезка.
 * @param B Конечная точка отрезка.
 * @param C Точка, от которой ищется расстояние (текущая позиция робота).
 * @return double Кратчайшее расстояние от C до отрезка AB.
 */
double shortestDistanceToSegment(const SPoint &A, const SPoint &B, const SPoint &C)
{
	// Шаг 1: Вычисляем векторы и их длины

	// Вектор AB (направление траектории)
	const double dx_AB = B.x - A.x; // Компонента X вектора AB
	const double dy_AB = B.y - A.y; // Компонента Y вектора AB

	// Вектор AC
	const double dx_AC = C.x - A.x; // Компонента X вектора AC
	const double dy_AC = C.y - A.y; // Компонента Y вектора AC

	// Квадрат длины отрезка AB (|AB|^2)
	const double magSq_AB = dx_AB * dx_AB + dy_AB * dy_AB; // |AB|^2

	// Если A и B совпадают, расстояние - это просто d(A, C)
	if (magSq_AB == 0.0)
	{
		return std::sqrt(dx_AC * dx_AC + dy_AC * dy_AC); // Возвращаем d(A, C)
	}

	// Шаг 2: Находим параметр t проекции P точки C на прямую AB

	// t = (Вектор AC · Вектор AB) / |AB|^2
	// Скалярное произведение (AC · AB)
	const double dot_product = dx_AC * dx_AB + dy_AC * dy_AB; // AC · AB

	// Параметр t (на бесконечной прямой)
	const double t = dot_product / magSq_AB; // t для проекции

	// Шаг 3: Проверка принадлежности t отрезку [0, 1]

	double Px, Py; // Координаты ближайшей точки P на отрезке [A, B]

	if (t < 0.0) // Если проекция P лежит "до" точки A
	{
		// Ближайшая точка на отрезке - это сама A
		Px = A.x; // Координата X точки A
		Py = A.y; // Координата Y точки A
	}
	else if (t > 1.0) // Если проекция P лежит "за" точкой B
	{
		// Ближайшая точка на отрезке - это сама B
		Px = B.x; // Координата X точки B
		Py = B.y; // Координата Y точки B
	}
	else // Проекция P лежит на отрезке [A, B]
	{
		// Ближайшая точка P - это сама проекция (A + t * AB)
		Px = A.x + t * dx_AB; // Px = Ax + t * (Bx - Ax)
		Py = A.y + t * dy_AB; // Py = Ay + t * (By - Ay)
	}

	// Шаг 4: Вычисляем расстояние от C до ближайшей точки P

	// Вектор CP
	const double dx_CP = C.x - Px; // Компонента X вектора CP
	const double dy_CP = C.y - Py; // Компонента Y вектора CP

	// Расстояние |CP| (кратчайшее расстояние)
	return std::sqrt(dx_CP * dx_CP + dy_CP * dy_CP); // Возвращаем |CP|
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
