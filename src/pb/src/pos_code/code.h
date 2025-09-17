#ifndef CODE_H
#define CODE_H

#include "kalman.h"
#include "config.h"
// #include "pillar.h"
//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
void callback_Lidar(sensor_msgs::LaserScan::ConstPtr msg); //
void callback_Modul(pb_msgs::Struct_Modul2Data msg);
void callback_Speed(pb_msgs::SSetSpeed msg);

void readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
void calcMode0(); // Расчет одометрии и применения ее для всех режимов

void calcMode123(); // Комплеиентация Mode123

inline double normalize_angle(double a); // Нормализация угла в диапазон [-π, π]

double calculateAngleDifference(double prev_angle, double current_angle); // Функция для вычисления разницы между углами

long map(long x, long in_min, long in_max, long out_min, long out_max); // Переводит значение из одного диапазона в другой, взял из Ардуино

void startPosition(geometry_msgs::Pose2D &startPose2d_); // Разбираем топик со стартовой позицией робота

void testFunction(); // Тест математических ипрочих функций

void angleMPU();  // Расчет угла положения на сонове данных сдатчика MPU
void calcEuler(); // Расчет угла Эллера

SPose convertRotation2Base(SPose pose_, std::string stroka_); // Конвертация координат из Rotattion в Lidar систему
SPose convertBase2Rotation(SPose pose_, std::string stroka_); // Конвертация координат из Lidar в Rotattion систему

float minDistance(float laserL_, float laserR_, float uzi1_); // Находим минимальную дистанцию из 3 датчиков

extern CKalman kalman11;
extern CKalman kalman12;
extern CKalman kalman13;
void initKalman(); // Задаем коэфициенты для Калмана

// pb_msgs::SControlDriver speedCorrect(pb_msgs::SDriver2Data Driver2Data_msg_, pb_msgs::SControlDriver Data2Driver_); // Корректировка скорости движения в зависимости от датчиков растояния перед
// void collectCommand(); // //Функция формирования команды для нижнего уровня на основе всех полученных данных, датчиков и анализа ситуации

// **********************************************************************************
SPose calcNewPose(SPose odom_, STwistDt data_, std::string stroka_, float koef_);	  // На вход подаются старая одометрия и новые угловая угловая скорость. Возвращается новая позиция по данным угловым скоростям
STwistDt calcTwistFromWheel(pb_msgs::SSetSpeed msg_Speed_);							  // Обсчитываем линейные и угловую скорость по данным скоростей от энкодера с колес
STwistDt calcTwistFromMpu(STwistDt mpu_, pb_msgs::Struct_Modul2Data msg_Modul2Data_); // Обсчитываем линейные и угловую скорость датчику IMU
STwistDt calcTwistFused(STwistDt wheelTwist_, STwistDt mpuTwist_);					  // Функция комплементации угловых скоростей полученных с колес и с датчика MPU и угла поворота
float autoOffsetX(float data_);														  // Функция считаем скользящее среднее из 128 элементов как офсет значений при стоянии на месте
float autoOffsetY(float data_);														  // Функция считаем скользящее среднее из 128 элементов как офсет значений при стоянии на месте
float autoOffsetZ(float data_);														  // Функция считаем скользящее среднее из 128 элементов как офсет значений при стоянии на месте
float filtrComplem(float koef_, float oldData_, float newData_);					  // функция фильтрации, берем старое значение с некоторым весом
// void calculateOdometryFromMpu(SMpu mpu_);					   // Обработка пришедших данных.Обсчитываем одометрию по энкодеру

// функция фильтрации, берем старое значение с некоторым весом
float filtrComplem(float koef_, float oldData_, float newData_)
{
	return (1 - koef_) * oldData_ + (koef_ * newData_);
}

void callback_Lidar(pb_msgs::Struct_PoseLidar msg)
{
	msg_lidar = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgLidar = true;
}
void callback_Modul(pb_msgs::Struct_Modul2Data msg)
{
	msg_Modul2Data = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgModul = true;
}
void callback_Speed(pb_msgs::SSetSpeed msg)
{
	msg_Speed = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgSpeed = true;
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

// Расчет угла theta
void calcEuler()
{
	// ROS_INFO_THROTTLE(RATE_OUTPUT,"+++ calcEuler");
	g_angleEuler.roll = msg_Modul2Data.bno.angleEuler.roll;
	g_angleEuler.pitch = msg_Modul2Data.bno.angleEuler.pitch;
	static float prev_yaw = msg_Modul2Data.bno.angleEuler.yaw;								   // При первом запуске этой функции инициализируем тем значением что придет от Modul
	g_angleEuler.yaw -= calculateAngleDifference(prev_yaw, msg_Modul2Data.bno.angleEuler.yaw); // Считаем угол куда смотрим // Расчет угла куда смотрим но пришедшим данным
	prev_yaw = msg_Modul2Data.bno.angleEuler.yaw;
	ROS_INFO_THROTTLE(RATE_OUTPUT, "    msg_Modul2Data.bno.angleEuler.yaw = %.3f g_angleEuler.yaw = %.3f (gradus) %.3f rad", msg_Modul2Data.bno.angleEuler.yaw, g_angleEuler.yaw, DEG2RAD(g_angleEuler.yaw));
}

// Конвертация координат из Rotattion в Lidar систему
SPose convertRotation2Base(SPose pose_, std::string stroka_)
{
	// ROS_INFO_THROTTLE(RATE_OUTPUT,"+++ convertRotation2Base %s", stroka_.c_str());
	SPose ret;
	ret.x = pose_.x - (transformLidar2Rotation.x * cos(pose_.th));
	ret.y = pose_.y - (transformLidar2Rotation.x * sin(pose_.th));
	ret.th = RAD2DEG(pose_.th); // в g_poseBase угол в градусах
	ROS_INFO_THROTTLE(RATE_OUTPUT, "    convertRotation2Base %s x = %.3f y = %.3f theta = %.3f (gradus) %.3f rad", stroka_.c_str(), ret.x, ret.y, ret.th, pose_.th);
	return ret;
}
// Конвертация координат из Lidar в Rotattion систему
SPose convertBase2Rotation(SPose pose_, std::string stroka_)
{
	// ROS_INFO_THROTTLE(RATE_OUTPUT,"+++ convertBase2Rotation %s", stroka_.c_str());
	SPose ret;
	// g_poseRotation.theta = DEG2RAD(45);							  // Присваиваем глобальному углу начальное значение
	ret.x = pose_.x + (transformLidar2Rotation.x * cos(DEG2RAD(pose_.th)));
	ret.y = pose_.y + (transformLidar2Rotation.x * sin(DEG2RAD(pose_.th)));
	ret.th = DEG2RAD(pose_.th);
	ROS_INFO_THROTTLE(RATE_OUTPUT, "    convertBase2Rotation %s x= %.3f y= %.3f th = %.3f (gradus) %.3f rad", stroka_.c_str(), ret.x, ret.y, RAD2DEG(ret.th), ret.th);
	return ret;
}

// Разбираем топик со стартовой позицией робота
void startPosition(geometry_msgs::Pose2D &startPose2d_)
{
	ROS_INFO("+++ startPosition");

	transformLidar2Rotation.x = 0.095; // Данные для трасформации из Lidar в Rotation 95 мм
	transformLidar2Rotation.y = 0;
	transformLidar2Rotation.th = 0;

	g_angleEuler.yaw = startPose2d_.theta; // Присваиваем yaw углу начальное значение
	// g_poseRotation.theta = DEG2RAD(startPose2d_.theta); // Присваиваем глобальному углу начальное значение

	g_poseBase.fused.x = startPose2d_.x; // Устанавливаем координаты для mode10 что-бы по нему начало все считаться
	g_poseBase.fused.y = startPose2d_.y;
	g_poseBase.fused.th = startPose2d_.theta;
	ROS_INFO("    startPose2d x= %.3f y= %.3f theta= %.3f ", startPose2d_.x, startPose2d_.y, startPose2d_.theta);
	g_poseBase.lidar = g_poseBase.fused;
	g_poseBase.laser = g_poseBase.fused;

	g_poseRotation.fused = convertBase2Rotation(g_poseBase.fused, "fused"); // Конвентируем координаты заданные для точки в системе Base в систему Rotation
	ROS_INFO("    start g_poseRotation.fused x= %.3f y= %.3f theta= %.3f ", g_poseRotation.fused.x, g_poseRotation.fused.y, g_poseRotation.fused.th);

	g_poseRotation.odom = g_poseRotation.fused; // Первоначальная установка позиции
	g_poseRotation.mpu = g_poseRotation.fused; // Первоначальная установка позиции

	ROS_INFO("--- startPosition");
}

// Переводит значение из одного диапазона в другой, взял из Ардуино
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
//        radius = MAX_RADIUS - radius + 0.01; // Прибавляем чуть-чуть чтобы радиус не получался 0 на краях
//     }
//     if (radius < 0 )
//     {
//        radius = -MAX_RADIUS - radius - 0.01; // Отнимаем чуть-чуть чтобы радиус не получался 0 на краях
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

// Расчет новой позиции на основе старой позиции и текущих линейных по х и у и угловой скорости yaw
SPose calcNewPose(SPose odom_, STwistDt data_, std::string stroka_, float koef_) // На вход подаются старая одометрия и новые угловая угловая скорость. Возвращается новая позиция по данным угловым скоростям
{
	// ROS_INFO_THROTTLE(RATE_OUTPUT,"+++ calcNewPose %s", stroka_.c_str());
	// ROS_INFO("IN calcNewPose pose.x= % .3f y= % .3f th= % .3f ", odom_.pose.x, odom_.pose.y, RAD2DEG(odom_.pose.th));
	if (data_.dt < 0.003) // Если пришли данные с нулевой дельтой то сразу выходим и ничего не считаем
	{
		ROS_INFO("    calcNewPose dt< 0.003 !!!!  dt = %f", data_.dt);
		return odom_; // Возвращаем что и было
	}

	SPoint pointLoc;
	pointLoc.x = data_.vx * data_.dt * koef_; // Находим проекции скорости на оси за интервал времени это координаты нашей точки в локальной системе координат
	pointLoc.y = data_.vy * data_.dt * koef_;
	// printf(" Local system pointLoc.x= % .3f y= % .3f dt= % .3f th= % .3f | \n", pointLoc.x, pointLoc.y, data_.dt, RAD2DEG(odom_.pose.th));

	// printf("DO pose.x= %.3f pose.y= %.3f pose.th= %.3f / ", odom_.pose.x, odom_.pose.y, RAD2DEG(odom_.pose.th));
	// Находим смещние по осям матрица координаты точки из локальной системы координат в глобальной
	double delta_x = pointLoc.x * cos(odom_.th) + pointLoc.y * sin(odom_.th);
	double delta_y = -pointLoc.x * sin(odom_.th) + pointLoc.y * cos(odom_.th);
	// printf("Global system delta.x= % .3f y= % .3f | \n", delta_x, delta_y);
	//  Меняем координаты и угол на основе вычислений
	//  odom_.pose.x += delta_x; // Вычисляем координаты
	//  odom_.pose.y += delta_y; // Вычисляем координаты

	SPose pose;
	pose.x = odom_.x;
	pose.y = odom_.y;
	pose.th = odom_.th;
	SPoint pointGlob = pointLocal2GlobalRosRAD(pointLoc, pose);
	// ROS_INFO("    New cordinates Rotation %s x= % .3f y= % .3f", stroka_.c_str(), pointGlob.x, pointGlob.y);

	odom_.x = pointGlob.x; // Вычисляем координаты
	odom_.y = pointGlob.y; // Вычисляем координаты

	// printf("twist.x= %.4f y= %.4f th= %.4f gradus ", bno055.twist.vx, bno055.twist.vy, bno055.twist.vth);
	// odom_.twist = data_.twist; // Ничего не меняем в угловой скорости
	// Меняем координаты и угол на основе вычислений
	// ROS_INFO("    IN odom_.th = %.3f data_.dt= %.3f  * data_.twist.vth= %.3f ", odom_.th, data_.dt, data_.twist.vth);

	odom_.th += data_.vth * data_.dt * koef_; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот С коефициентом это ПРЕДСКАЗАНИЕ УГЛА
	// if (odom_.th > (2 * M_PI))
	// 	(odom_.th -= (2 * M_PI));
	// if (odom_.th < 0)
	// 	(odom_.th += (2 * M_PI));
	ROS_INFO_THROTTLE(RATE_OUTPUT, "    calcNewPose Rotation %s pose.x= % .3f y= % .3f | th= % .3f gradus th= % .4f rad", stroka_.c_str(), odom_.x, odom_.y, RAD2DEG(odom_.th), odom_.th);

	// ROS_INFO("--- calcNewPose");
	return odom_;
}
// Обработка пришедших данных.Обсчитываем одометрию по энкодеру
SPose calcNewOdom2(SPose odom_, STwistDt data_, std::string stroka_) // На вход подаются старая одометрия и новые угловая угловая скорость. Возвращается новая позиция по данным угловым скоростям
{
	// ROS_INFO_THROTTLE(RATE_OUTPUT,"+++ calcNewPose %s", stroka_.c_str());
	// ROS_INFO("IN calcNewPose pose.x= % .3f y= % .3f th= % .3f ", odom_.pose.x, odom_.pose.y, RAD2DEG(odom_.pose.th));
	if (data_.dt < 0.003) // Если пришли данные с нулевой дельтой то сразу выходим и ничего не считаем
	{
		ROS_INFO("    calcNewPose dt< 0.003 !!!!  dt = %f", data_.dt);
		return odom_; // Возвращаем что и было
	}

	SPoint pointLoc;
	pointLoc.x = data_.vx * data_.dt; // Находим проекции скорости на оси за интервал времени это координаты нашей точки в локальной системе координат
	pointLoc.y = data_.vy * data_.dt;

	double delta_x = pointLoc.x * cos(odom_.th) + pointLoc.y * sin(odom_.th); // Находим смещение по осям матрица координаты точки из локальной системы координат в глобальной
	double delta_y = -pointLoc.x * sin(odom_.th) + pointLoc.y * cos(odom_.th);

	SPose pose;
	pose.x = odom_.x;
	pose.y = odom_.y;
	pose.th = odom_.th;
	SPoint pointGlob = pointLocal2GlobalRosRAD(pointLoc, pose);
	// ROS_INFO("    New cordinates Rotation %s x= % .3f y= % .3f", stroka_.c_str(), pointGlob.x, pointGlob.y);

	odom_.x = pointGlob.x; // Вычисляем координаты
	odom_.y = pointGlob.y; // Вычисляем координаты
	// ROS_INFO("    IN odom_.th = %.3f data_.dt= %.3f  * data_.twist.vth= %.3f ", odom_.th, data_.dt, data_.twist.vth);

	float k = 0.1;
	odom_.th = (odom_.th + data_.vth * data_.dt) * (1 - k) + DEG2RAD(g_angleEuler.yaw) * k; // Комплементарный фильтр. Берем старое значение и увеличиваем на новый , берем с коефициентом большим и стабилизируем по точному значению
	ROS_INFO_THROTTLE(RATE_OUTPUT, "    calcNewOdom2 Rotation %s pose.x= % .3f y= % .3f | th= % .3f gradus th= % .4f rad", stroka_.c_str(), odom_.x, odom_.y, RAD2DEG(odom_.th), odom_.th);

	return odom_;
}

// Обсчитываем линейные и угловую скорость по данным скоростей от энкодера с колес
STwistDt calcTwistFromWheel(pb_msgs::SSetSpeed msg_Speed_)
{
	STwistDt ret;
	double radius = 0;
	double theta = 0;
	double lenArc = 0;
	static SPose pose;
	STwistDt twist;

	// ROS_INFO_THROTTLE(RATE_OUTPUT,"+++ calcTwistFromWheel");

	static ros::Time start_time = ros::Time::now(); // Захватываем начальный момент времени
	ros::Time end_time = ros::Time::now();			// Захватываем конечный момент времени
	ros::Duration duration = end_time - start_time; // Находим разницу между началом и концом
	double dt = duration.toSec();					// Получаем количество секунд и преобразуем в миллисекунды
	start_time = end_time;

	static unsigned long time = micros();				// Время предыдущего расчета// Функция из WiringPi.
	unsigned long time_now = micros();					// Время в которое делаем расчет
	double dt_micros = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	time = time_now;
	// ROS_INFO("    micros dt_micros = %f sec millis = %f", dt_micros, millis());

	if (dt < 0.002) // При первом запуске просто выходим из функции
	{
		ROS_INFO("    First start. alcTwistFromWheel dt< 0.002 dt= %f | dt_micros = %f sec", dt, dt_micros);
		return ret;
	}
	// double speedL = PERIMETR * Driver2Data.motor.rpsEncodL; // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
	// double speedR = PERIMETR * Driver2Data.motor.rpsEncodR; // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
	// double speedL = PERIMETR * msg_Speed_.speedL; // Тут скорость заданная в оборотах в секунду преврщаем в метри в секунду
	// double speedR = PERIMETR * msg_Speed_.speedR; // Тут скорость заданная в оборотах в секунду преврщаем в метри в секунду
	double speedL = msg_Speed_.speedL; // Скорость в метрах в секуду
	double speedR = msg_Speed_.speedR; // Скорость в метрах в секуду

	double sumSpeed = speedL + speedR;
	double deltaSpeed = speedL - speedR;
	double speed = (speedR + speedL) / 2.0; // Находим скорость всего обьекта.
	// ROS_INFO("    dt = %.3f sec speedL = %.4f speedR = %.4f speed robot (speedR + speedL) / 2.0 = %.4f", dt, speedL, speedR, speed);
	//*******************************************************************************************************************************************************
	double w = -deltaSpeed / DISTANCE_WHEELS; // Находим уголовую скорость движения по радиусу. Плюс по часовой минус против часовой

	// ROS_INFO("speedL= %.4f speedR= %.4f speed= %.4f w = %.4f ///  ", speedL, speedR, speed, RAD2DEG(w));

	// if (speedL == 0 && speedR == 0) // Стоим на месте. Скорости равны нулю
	// {
	// 	radius = 0;
	// 	speed = 0;
	// 	theta = 0;
	// 	// ROS_INFO("    0 STOIM NA MESTE radius = %.4f theta gradus = %.4f ", radius, RAD2DEG(theta));
	// }
	// else
	// {
	// 	if (abs(sumSpeed) < 0.01 && speedL != 0 && speedR != 0) // Если сумма скоростей очень маленькая или ноль значит крутимся на месте или стоим на месте и тогда совсем иной расчет чем если движемся// Крутимся на месте
	// 	{
	// 		radius = 0.5 * DISTANCE_WHEELS; // Радиус в таком случае это половина между колесами
	// 		if (speedL > speedR)			// Значит крутимся по часовой и знак угловой скорсти минус
	// 		{
	// 			lenArc = -speedL; // меняем скорость со среднй на скорость одного колеса, наружнего // Находим путь какой проехали. Это длинна дуги.
	// 		}
	// 		else
	// 		{
	// 			lenArc = speedR; //  Значит крутимся против часовой и знак минус будет у угловой скорости // Находим путь какой проехали. Это длинна дуги.
	// 		}
	// 		speed = 0; // Обнуляем скорость чтобы дальше позиция не сдвигалась, мы же на месте.
	// 		// theta = lenArc / radius; // Отношение улинны дуги окружности к радиусу дает угол в радианах. Нахождение центрального угла по дуге и радиусу.
	// 		theta = w;
	// 		// ROS_INFO("    1 KRUTIMSA NA MESTE radius = %.4f theta gradus = %.4f lenArc = %.4f speedL = %.4f speedR = %.4f ", radius, RAD2DEG(theta), lenArc, speedL, speedR);
	// 	}
	// 	else // Тут нормальный расчет что мы движемся или по прямой или по радиусу
	// 	{
	// 		lenArc = speed;				// Находим путь какой проехали за время в течении которого энкодер собирал данные. Это длинна дуги.
	// 		if (abs(deltaSpeed) < 0.01) // Если раздница скоростей незначительна то считаем что едем прямо вперед или назад
	// 		{
	// 			radius = 0; // Едем прямо или назад и все по нулям
	// 			theta = 0;	// Если едем прямо то угол поворота отклонения от оси равен 0
	// 					   // ROS_INFO("    2 EDEM PRIAMO radius = %.4f theta gradus = %.4f ", radius, RAD2DEG(theta));
	// 		}
	// 		else // Едем по радиусу и надо все считать
	// 		{
	// 			radius = (0.5 * DISTANCE_WHEELS) * (sumSpeed / deltaSpeed); // Находим радиус движения
	// 			// theta = lenArc / radius;									// Отношение улинны дуги окружности к радиусу дает угол в радианах. Нахождение центрального угла по дуге и радиусу.
	// 			theta = w;
	// 			// ROS_INFO("    3 EDEM RADIUS radius = %.4f theta gradus = %.4f ", radius, RAD2DEG(theta));
	// 		}
	// 	}
	// }

	theta = w;
	// printf (" theta = %.3f \n", theta);
	//  Находим линейные скорости из моего вектора скорости. Я задаю общую скорость движения (длинна вектора), ее надо разложить на проекции по осям x y. Это будут линейные скорости
	//  speed = 0.26;
	twist.vx = speed * cos(theta * dt); // Проекция моей скорости на ось X получаем линейную скорость по оси за секунуду
	twist.vy = speed * sin(theta * dt); // Проекция моей скорости на ось Y получаем линейную скорость по оси за секунуду
	twist.vth = theta;					// Угловая скорость в радианах.
	twist.dt = dt;

	ROS_INFO_THROTTLE(RATE_OUTPUT, "    Twist Wheel dt = %.3f vx= %.3f vy= %.3f vth= %.3f w= %.3f gradus/sec  %.3f rad/sec", dt, twist.vx, twist.vy, RAD2DEG(twist.vth), RAD2DEG(w), w);
	// if (w==0)
	// ROS_INFO("NULL");

	// printf("vy= % .4f", twist.vy);
	// printf("speed= %.4f twist.vth = %.4f / sin(twist.vth )= %.4f cos(twist.vth ) = %.4f / ", speed, RAD2DEG(twist.vth), sin(twist.vth ), cos(twist.vth ));
	// printf("speed= %.4f twist.vth = %.8f / ", speed, RAD2DEG(twist.vth));
	// ROS_INFO("SPEED= %.3f Linear speed twist.vx = %.3f twist.vy = %.3f Angular speed twist.vth = %.3f for sec.", speed, twist.vx, twist.vy, RAD2DEG(twist.vth));
	// //==============================================================================================================================
	ret = twist;

	// }
	// ROS_INFO("--- calcTwistFromWheel");
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

		odomMode0.pose = pose;
		odomMode0.twist = twist;
	}
}
*/

// Нормализация угла в диапазон [-π, π]
inline double normalize_angle(double a)
{
	while (a > M_PI)
		a -= 2.0 * M_PI;
	while (a <= -M_PI)
		a += 2.0 * M_PI;
	return a;
}

// Обсчитываем линейные и угловую скорость датчику IMU
STwistDt calcTwistFromMpu(STwistDt mpu_, pb_msgs::Struct_Modul2Data msg_Modul2Data_)
{
	static STwistDt ret;
	// ROS_INFO_THROTTLE(RATE_OUTPUT,"+++ calcTwistFromMpu");
	static ros::Time start_time = ros::Time::now(); // Захватываем начальный момент времени
	ros::Time end_time = ros::Time::now();			// Захватываем конечный момент времени
	ros::Duration duration = end_time - start_time; // Находим разницу между началом и концом
	double dt = duration.toSec();					// Получаем количество секунд и преобразуем в миллисекунды
	start_time = end_time;
	// ROS_INFO("    dt = %f sec", dt);
	ret.dt = dt; // Сохраняем новое dt

	if (dt < 0.003) // При первом запуске просто выходим из функции
	{
		ROS_INFO("    calcTwistFromMpu dt< 0.003 !!!! dt = %f", dt);
		ret.vth = DEG2RAD(msg_Modul2Data_.icm.angleEuler.yaw); //
		return ret;
	}

	ret.vth = normalize_angle(DEG2RAD(msg_Modul2Data_.icm.angleEuler.yaw) - mpu_.vth) / dt; // Вычисляем разницу углов в радианах/ делим на интервал получаем угловую скорость в радинах/секунду

	ret.vx = mpu_.vx + msg_Modul2Data_.bno.linear.x * dt; // Линейное ускорение по оси метры за секунуду умножаем на интервал, получаем ускорение за интервал и суммируем в скорость линейную по оси
	ret.vy = mpu_.vy + msg_Modul2Data_.bno.linear.y * dt; // Линейное ускорение по оси метры за секунуду умножаем на интервал, получаем ускорение за интервал и суммируем в скорость линейную по оси

	ROS_INFO_THROTTLE(RATE_OUTPUT, "    Twist MPU   dt = %.3f | vx= %.3f vy= %.3f | vth= %.3f gradus/sec %.4f rad/sec |", dt, ret.vx, ret.vy, RAD2DEG(ret.vth), ret.vth);
	// ROS_INFO("--- calcTwistFromMpu");
	return ret;

	/*
	SMpu mpu_;
	mpu_.linear.x = msg_Modul2Data_.bno.linear.x; // Копируем в локальную перемнную нужные параметры
	mpu_.linear.y = msg_Modul2Data_.bno.linear.y;
	mpu_.linear.z = msg_Modul2Data_.bno.linear.z;
	mpu_.angleEuler.z = msg_Modul2Data_.bno.angleEuler.yaw;

	float koef_ = 0.2;
	static double predAngleZ = 0;

	// static unsigned long time = micros();		 // Время предыдущего расчета// Функция из WiringPi.// Замеряем интервалы по времени между запросами данных
	// unsigned long time_now = micros();			 // Время в которое делаем расчет
	// double dt = ((time_now - time) / 1000000.0); // Интервал расчета переводим сразу в секунды Находим интревал между текущим и предыдущим расчетом в секундах
	// time = time_now;

	static float offsetX = 0;
	static float offsetY = 0;
	static float offsetZ = 0;
	static float complX = 0; // Значение после комплементарного фильтра
	static float complY = 0; // Значение после комплементарного фильтра
	static float complZ = 0; // Значение после комплементарного фильтра

	// ROS_INFO("    Mpu x = % .3f y = % .3f z = % .3f | dt = % .3f ", mpu_.linear.x, mpu_.linear.y, mpu_.linear.z, dt);

	if (Data2Driver.control.speedL == 0 && Data2Driver.control.speedR == 0) // Если стоим на месте, то считаем офсет. Как только тронемся, его и будем применять до следующей остановки
	{
		offsetX = autoOffsetX(mpu_.linear.x);
		offsetY = autoOffsetY(mpu_.linear.y);
		offsetZ = autoOffsetZ(mpu_.linear.z);
	}
	// printf(" |offset % .4f % .4f | ", offsetX, offsetY);
	// ROS_INFO("    Offset x= % .4f y= % .4f z= % .4f ", offsetX, offsetY, offsetZ);

	mpu_.linear.x = mpu_.linear.x - offsetX;
	mpu_.linear.y = mpu_.linear.y - offsetY;
	mpu_.linear.z = mpu_.linear.z - offsetZ;

	// ROS_INFO("    Average  x = % .3f y = % .3f z = % .3f ", mpu_.linear.x, mpu_.linear.y, mpu_.linear.z);
	// printf(" |Average % .3f | ", mpu_.linear.y);

	complX = filtrComplem(koef_, complX, mpu_.linear.x);
	complY = filtrComplem(koef_, complY, mpu_.linear.y);
	complZ = filtrComplem(koef_, complZ, mpu_.linear.z);
	// ROS_INFO("    Compl x= % .3f y= % .3f  z= % .3f", complX, complY, complZ);

	//Примечание. Сигнал линейного ускорения обычно не может быть интегрирован для восстановления скорости или дважды интегрирован для восстановления положения.
	// Ошибка обычно становится больше сигнала менее чем за 1 секунду, если для компенсации этой ошибки интегрирования не используются другие источники датчиков.
	ret.vx += complX * dt; // Линейное ускорение по оси метры за секунуду умножаем на интервал, получаем ускорение за интервал и суммируем в скорость линейную по оси
	ret.vy += complY * dt; // Линейное ускорение по оси метры за секунуду

	// double delta_th = -(mpu_.angleEuler.z - predAngleZ); // считаем величину изменения угла, тут она в градусах
	// // ROS_INFO("    predAngleZ = % .3f mpu_.angleEuler.z= % .3f delta_th= % .3f", predAngleZ, mpu_.angleEuler.z, delta_th);
	// predAngleZ = mpu_.angleEuler.z;
	// if (delta_th > 180)
	// 	(delta_th = delta_th - 360); // Если
	// if (delta_th < -180)
	// 	(delta_th = delta_th + 360);  // Если
	// ret.vth = DEG2RAD(delta_th) / dt; // превращаем в радианы в секунды Угловая скорость вращения

	// float kk = 0.05;
	// g_linAngVel.filtr_mpu.vth = g_linAngVel.filtr_mpu.vth * (1 - kk) + ret.vth * kk; // Фильтр

	// if (msg_Speed.speedL == 0 && msg_Speed.speedR == 0) // Если обе скорости равны нулю то обнуляем расчеты по mpu. Так как стоим на мете и никаких линейных скоростей быть не может. Стои на месте.
	// {
	// 	ret.vx = ret.vy = ret.vth = 0;
	// 	// ROS_INFO("    msg_Speed.speedL = %.3f speedR = %.3f", msg_Speed.speedL, msg_Speed.speedR);
	// }
	*/
	// printf(" ||| LinearSpeed vx= % .3f vy=  % .3f vth= % .6f | ", ret.twist.vx, ret.twist.vy, ret.twist.vth);
	// printf(" |Vel= % .3f % .3f % .3f\n", ret.twist.vx, ret.twist.vy, ret.twist.vth);
}
// Функция комплементации угловых скоростей полученных с колес и с датчика MPU и угла поворота
STwistDt calcTwistFused(STwistDt wheelTwist_, STwistDt mpuTwist_)
{
	// ROS_INFO_THROTTLE(RATE_OUTPUT,"+++ calcTwistUnited");
	STwistDt ret;
	float dt = wheelTwist_.dt * 0.5 + mpuTwist_.dt * 0.5;
	if (dt < 0.003) // При первом запуске просто выходим из функции
	{
		ROS_INFO("    calcTwistUnited dt< 0.003 !!!!  dt = %f", dt);
		return ret;
	}
	float koef = 0.5; // Коефициант по умолчанию.Пополам.

	ret.vx = wheelTwist_.vx * (1 - koef) + mpuTwist_.vx * koef;
	ret.vy = wheelTwist_.vy * (1 - koef) + mpuTwist_.vy * koef;

	// float koefTh = 0.5; // Коефициант по умолчанию.Пополам.
	// ret.vth = g_linAngVel.filtr_mpu.vth * (1 - koefTh) + g_linAngVel.wheel.vth * koefTh;
	ret.vth = wheelTwist_.vth * (1 - koef) + mpuTwist_.vth * koef;

	// fused = alpha * (fused_prev + gyro * dt) + (1 - alpha) * odom_yaw; //Простой комплементарный фильтр (рекомендую как старт)

	/*

	// пример вызова на каждом шаге (100 Гц)
	double yaw = update(imu_delta, odom_delta, yaw_abs, have_abs);



	#include <math.h>
	#include <stdio.h>

	double yaw_est = 0.0;  // текущее состояние фильтра (оценка угла)

	// нормализация угла в диапазон [-pi, pi]
	double normalize_angle(double a)
	{
		return atan2(sin(a), cos(a));
	}

	// обновление фильтра
	double update(double imu_yaw_delta, double odom_yaw_delta,double yaw_abs, int use_abs)
	{
		// 1. прогноз по IMU
		double yaw_imu = yaw_est + imu_yaw_delta;

		// 2. прогноз по одометрии
		double yaw_odom = yaw_est + odom_yaw_delta;

		// 3. инновация между IMU и одометрией
		double innovation = normalize_angle(yaw_odom - yaw_imu);

		// 4. адаптивный вес (IMU vs ODOM)
		double base_alpha = 0.98;                     // базовый вес IMU
		double max_innovation = 10.0 * M_PI / 180.0;  // предел разумного расхождения (10°)

		double scale = 1.0 - fabs(innovation) / max_innovation;
		if (scale < 0.0) scale = 0.0;
		if (scale > 1.0) scale = 1.0;

		double alpha = base_alpha + (1.0 - base_alpha) * scale;
		if (alpha < 0.0) alpha = 0.0;
		if (alpha > 1.0) alpha = 1.0;

		// 5. объединение IMU и одометрии
		yaw_est = alpha * yaw_imu + (1.0 - alpha) * yaw_odom;

		// 6. если пришло абсолютное измерение
		if (use_abs)
		{
			double abs_innovation = normalize_angle(yaw_abs - yaw_est);

			// вес абсолютного датчика (0.01..0.1)
			double gamma = 0.05;
			yaw_est = normalize_angle(yaw_est + gamma * abs_innovation);
		}

		// 7. нормализация
		yaw_est = normalize_angle(yaw_est);

		return yaw_est;
	}

	*/
	/*
		// Нормализация угла в диапазон [-π, π]
		inline double normalize_angle(double a) {
			while (a > M_PI)  a -= 2.0 * M_PI;
			while (a <= -M_PI) a += 2.0 * M_PI;
			return a;
		}

		// Разница углов (a - b) с учётом wrap-around, результат в [-π, π]
		inline double angle_diff(double a, double b) {
			return atan2(sin(a - b), cos(a - b));
		}

		// Среднее значение двух углов
		inline double angle_average(double a, double b) {
			double x = cos(a) + cos(b);
			double y = sin(a) + sin(b);
			return atan2(y, x);
		}
	*/

	ret.dt = dt;

	ROS_INFO_THROTTLE(RATE_OUTPUT, "    fused Wheel | %.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f || %.3f %.3f %.3f %.3f  ",
					  wheelTwist_.vx, wheelTwist_.vy, wheelTwist_.vth, wheelTwist_.dt,
					  mpuTwist_.vx, mpuTwist_.vy, mpuTwist_.vth, mpuTwist_.dt,
					  ret.vx, ret.vy, ret.vth, ret.dt);

	return ret;
}

float autoOffsetX(float data_) // УМНЫЙ РАСЧЕТ УБИРАЮЩИЙ ПЛАВАНИЕ УСКОРЕНИЯ С ДАТЧИКА BNO055
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
float autoOffsetZ(float data_)
{
	static uint16_t i = 0;
	static uint16_t k = 0;
	static float sum = 0;
	if (k < 128)
		k++;
	else
		sum = sum - linearOffsetZ[i]; // Убираем из среднего прежнее значение
	linearOffsetZ[i] = data_;		  // Меняем значение в массиве
	sum = sum + linearOffsetZ[i];	  // Добавляем в среднее новое значение
	i++;
	if (i >= 128)
		i = 0;
	// printf(" sumX= % .3f ", sum);
	return sum / k;
}
// Расчет угла положения на сонове данных сдатчика MPU
void angleMPU()
{
	static float predAngleZ = 0;
	static bool first = true;
	printf("flag_msgDriver in... ");
	if (first) // Делаем в первый приход данных
	{
		predAngleZ = msg_Modul2Data.bno.angleEuler.yaw; // Запоминаяем угол поворота Для следующего обсчета
		first = false;
		printf("First start -> ");
	}
	else // Всегда кроме первого
	{
		g_angleMPU += (msg_Modul2Data.bno.angleEuler.yaw - predAngleZ); // Меняем угол поворота увеличивая на разницу. Разобраться с 360 и переходом через 0
		predAngleZ = msg_Modul2Data.bno.angleEuler.yaw;					// Запоминаяем угол поворота Для следующего обсчета
																		// dataNode.parsingDriver(msg_Driver2Data);
	}
	printf("g_angleMPU = % .3f \n", g_angleMPU);
}
// Расчет одометрии и применения ее для всех режимов
void calcMode0()
{
	ROS_INFO("+++ calcMode0");
	// printf("1 RAD2DEG(odomMode0.pose.th) = % .3f \n", RAD2DEG(odomMode0.pose.th));
	g_poseRotation.odom = calcNewPose(g_poseRotation.odom, g_linAngVel.odom, "odom", 1); // На основе линейных скоростей считаем новую позицию и угол по колесам

	// g_poseBase.mode0.x = odomMode0.pose.x;
	// g_poseBase.mode0.y = odomMode0.pose.y;
	// g_poseBase.mode0.th = RAD2DEG(odomMode0.pose.th);

	// ROS_WARN_THROTTLE(THROTTLE_PERIOD_3, "    MODE0 pose.x= %.3f y= %.3f theta= %.3f ", g_poseBase.mode0.x, g_poseBase.mode0.y, g_poseBase.mode0.th);
	ROS_INFO("--- calcMode0");
}

// Комплементация Mode123 это усреднение данных по Mode0 Mode2 Mode3
void calcMode123()
{
	// ROS_INFO("+++ calcMode123");
	// SPose pose;
	// pose.x = (g_poseBase.mode1.x + g_poseBase.mode2.x + g_poseBase.mode3.x) / 3.0;
	// pose.y = (g_poseBase.mode1.y + g_poseBase.mode2.y + g_poseBase.mode3.y) / 3.0;
	// pose.th = (g_poseBase.mode1.th + g_poseBase.mode2.th + g_poseBase.mode3.th) / 3.0;
	// if (isnan(pose.x) || isnan(pose.y) || isnan(pose.th))
	// {
	// 	ROS_ERROR("calcMode123 ERROR.");
	// 	exit(123);
	// }
	// else
	// {
	// 	g_poseBase.mode123 = pose;
	// 	ROS_WARN_THROTTLE(THROTTLE_PERIOD_3, "    MODE123 pose.x= % .3f y= % .3f theta= %.3f", g_poseBase.mode123.x, g_poseBase.mode123.y, g_poseBase.mode123.th);
	// }
	// ROS_INFO("--- calcMode123.");
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
/*
void startColibrovka(CTopic &topic)
{
	static pb_msgs::Struct_Data2Modul dataControlModul;
	static u_int64_t time1Colibrovka = millis() + 5000;
	static u_int64_t time2Colibrovka;
	static u_int64_t time3Colibrovka;
	static u_int64_t time4Colibrovka;
	static u_int64_t time5Colibrovka;

	static bool flag1Colibrovka = true;
	static bool flag2Colibrovka = false;
	static bool flag3Colibrovka = false;
	static bool flag4Colibrovka = false;
	static bool flag5Colibrovka = false;

	struct SColibrData
	{
		float angleL = 0;
		float angleR = 0;
		float distMin = 100;  // Дистанция что измерили
		float angleMin = 0;	  // Угол при котором измеряли
		float distMin2 = 100; // Дистанция что измерили
		float angleMin2 = 0;  // Угол при котором измеряли
	};

	static SColibrData colibrData[4]; // Массив со всеми переменными для расчета

	if (time1Colibrovka < millis() && flag1Colibrovka)
	{
		//  Считаем угол на столбы и считаем на какой угол от него надо отклониться чтобы гарантированно отсканировать столб
		laser.calcAnglePillarForLaser(pillar.pillar, g_poseBase.mode1); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
		// Включаем назерные датчики
		dataControlModul.controlLaser.mode = 1;
		dataControlModul.controlMotor.mode = 1;
		// Поворачиваем на этот угол
		dataControlModul.controlMotor.angle[0] = g_angleLaser[0];
		dataControlModul.controlMotor.numPillar[0] = g_numPillar[0];
		dataControlModul.controlMotor.angle[1] = g_angleLaser[1];
		dataControlModul.controlMotor.numPillar[1] = g_numPillar[1];
		dataControlModul.controlMotor.angle[2] = g_angleLaser[2];
		dataControlModul.controlMotor.numPillar[2] = g_numPillar[2];
		dataControlModul.controlMotor.angle[3] = g_angleLaser[3];
		dataControlModul.controlMotor.numPillar[3] = g_numPillar[3];
		topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
		flag1Colibrovka = false;
		time2Colibrovka = millis() + 1000; // Начинаем 2 этап через 3 секунды
		flag2Colibrovka = true;
		printf("1 ETAP \n");
	}
	if (time2Colibrovka < millis() && flag2Colibrovka)
	{
		printf("2 ETAP \n");
		//==========================================================

		for (int i = 0; i < 4; i++) // Перебираем моторы(лазеры)
		{
			SPoint a, b;
			a.x = msg_startPose2d.x;
			a.y = msg_startPose2d.y;
			b.x = pillar.pillar[g_numPillar[i]].x_true;
			b.y = pillar.pillar[g_numPillar[i]].y_true;
			float len = vectorLen(a, b);
			float angle = atan(PILLAR_RADIUS / len);
			float angleL = g_angleLaser[i] - RAD2DEG(angle) - 0.1;
			float angleR = g_angleLaser[i] + RAD2DEG(angle) + 0.1;
			printf("len = % .3f angle= % .3f angleL= % .3f angleR= % .3f \n", len, RAD2DEG(angle), angleL, angleR);
			colibrData[i].angleL = angleL;
			colibrData[i].angleR = angleR;
			dataControlModul.controlMotor.angle[i] = angleL;
		}
		topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
		printf("2 ETAP END\n");
		flag2Colibrovka = false;
		time3Colibrovka = millis() + 2500; // Начинаем 2 этап через 3 секунды
		flag3Colibrovka = true;
	}
	//==========================================================
	if (time3Colibrovka < millis() && flag3Colibrovka)
	{
		printf("3 ETAP START \n");
		if (flag_msgModul) // Флаг что пришло сообщение от ноды Data по Modul. Тут пишем какую-то обработку данных если нужно.
		{
			flag_msgModul = false;
			// Поворачиваемся на 0,1125 градус каждые 0,33 секунды и запоминаем показания в массив
			for (int i = 0; i < 4; i++) // Перебираем моторы(лазеры)
			{
				printf("distance = % .3f angle = % .3f numPillar = %i \n", msg_Modul2Data.laser[i].distance, msg_Modul2Data.laser[i].angle, msg_Modul2Data.laser[i].numPillar);
				if (msg_Modul2Data.laser[i].numPillar != -1) // Если пришли корректные значения от нужного столба. ПЕрвый раз почему-то приходят с -1
				{
					// Останавливаемся когда дойдем до другого угла
					if (dataControlModul.controlMotor.angle[i] < colibrData[i].angleR)
					{
						// dataControlModul.controlMotor.angle[i] += 0.1125; // 360 ГРАДУСОВ /400 шагов мотор /8 шагов драйвер
						// Могу измерять 3 Hz Могу поворачиваться на 0,11 значит за 1 секунду повернусь на 0,33 значит угол равен 0,33/Частоту работы RATE
						// dataControlModul.controlMotor.angle[i] += 0.0033; // 360 ГРАДУСОВ /400 шагов мотор /8 шагов драйвер
						float stepp = (RATE_LASER * STEP_LASER_MOTOR) / RATE;
						dataControlModul.controlMotor.angle[i] += (RATE_LASER * STEP_LASER_MOTOR) / RATE; // 360 ГРАДУСОВ /400 шагов мотор /16 шагов драйвер
						//  Смотри какое новое расстоние пришло, сравниваем со старым, ищем минимум и запоминаем угол с минимальным расстоянием
						if (colibrData[i].distMin > msg_Modul2Data.laser[i].distance)
						{
							colibrData[i].distMin = msg_Modul2Data.laser[i].distance; // Если новое измерение меньше то меняем раастояние
							colibrData[i].angleMin = msg_Modul2Data.laser[i].angle;
						}
						printf("distMin = %.3f angleMin = %.3f stepp= %f \n", colibrData[i].distMin, colibrData[i].angleMin, stepp);
					}
				}
			}
			topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
			printf("3 ETAP END \n");
			if (dataControlModul.controlMotor.angle[0] >= colibrData[0].angleR && // ЕСли все углы отработали
				dataControlModul.controlMotor.angle[1] >= colibrData[1].angleR &&
				dataControlModul.controlMotor.angle[2] >= colibrData[2].angleR &&
				dataControlModul.controlMotor.angle[3] >= colibrData[3].angleR)
			{
				flag3Colibrovka = false;
				time4Colibrovka = millis() + 1000; // Начинаем 4 этап через 1 секунды
				flag4Colibrovka = true;
			}
		}
	}
	//==========================================================
	if (time4Colibrovka < millis() && flag4Colibrovka)
	{
		printf("4 ETAP START \n");
		if (flag_msgModul) // Флаг что пришло сообщение от ноды Data по Modul. Тут пишем какую-то обработку данных если нужно.
		{
			flag_msgModul = false;
			// Поворачиваемся на 0,1125 градус каждые 0,33 секунды и запоминаем показания в массив
			for (int i = 0; i < 4; i++) // Перебираем моторы(лазеры)
			{
				printf("distance = % .3f angle = % .3f numPillar = %i \n", msg_Modul2Data.laser[i].distance, msg_Modul2Data.laser[i].angle, msg_Modul2Data.laser[i].numPillar);
				if (msg_Modul2Data.laser[i].numPillar != -1) // Если пришли корректные значения от нужного столба. ПЕрвый раз почему-то приходят с -1
				{
					// Останавливаемся когда дойдем до другого угла
					if (dataControlModul.controlMotor.angle[i] > colibrData[i].angleL)
					{
						// dataControlModul.controlMotor.angle[i] += 0.1125; // 360 ГРАДУСОВ /400 шагов мотор /8 шагов драйвер
						// Могу измерять 3 Hz Могу поворачиваться на 0,11 значит за 1 секунду повернусь на 0,33 значит угол равен 0,33/Частоту работы RATE
						// dataControlModul.controlMotor.angle[i] += 0.0033; // 360 ГРАДУСОВ /400 шагов мотор /8 шагов драйвер
						float stepp = (RATE_LASER * STEP_LASER_MOTOR) / RATE;
						dataControlModul.controlMotor.angle[i] -= (RATE_LASER * STEP_LASER_MOTOR) / RATE; // 360 ГРАДУСОВ /400 шагов мотор /16 шагов драйвер
						//  Смотри какое новое расстоние пришло, сравниваем со старым, ищем минимум и запоминаем угол с минимальным расстоянием
						if (colibrData[i].distMin2 > msg_Modul2Data.laser[i].distance)
						{
							colibrData[i].distMin2 = msg_Modul2Data.laser[i].distance; // Если новое измерение меньше то меняем раастояние
							colibrData[i].angleMin2 = msg_Modul2Data.laser[i].angle;
						}
						printf("distMin2 = %.3f angleMin2 = %.3f stepp= %f \n", colibrData[i].distMin2, colibrData[i].angleMin2, stepp);
					}
				}
			}
			topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
			printf("4 ETAP END \n");
			if (dataControlModul.controlMotor.angle[0] <= colibrData[0].angleL && // ЕСли все углы отработали
				dataControlModul.controlMotor.angle[1] <= colibrData[1].angleL &&
				dataControlModul.controlMotor.angle[2] <= colibrData[2].angleL &&
				dataControlModul.controlMotor.angle[3] <= colibrData[3].angleL)
			{
				flag4Colibrovka = false;
				time5Colibrovka = millis() + 1000; // Начинаем 4 этап через 1 секунды
				flag5Colibrovka = true;
			}
		}
	}
	//==========================================================
	if (time5Colibrovka < millis() && flag5Colibrovka)
	{
		// Устанавливаем лазер в полученный угол
		for (int i = 0; i < 4; i++) // Перебираем моторы(лазеры)
		{
			dataControlModul.controlMotor.angle[i] = (colibrData[i].angleMin + colibrData[i].angleMin2) / 2.0;
			printf(" numPillar = %i Teoria angle= %.3f |", g_numPillar[i], g_angleLaser[i]);
			printf("angle1= %.3f angle2= %.3f  | distMin1 = %.3f distMin2 = %.3f \n", colibrData[i].angleMin, colibrData[i].angleMin2, colibrData[i].distMin, colibrData[i].distMin2);
			printf("i= %i angle= %.3f distMin = %.3f \n", i, dataControlModul.controlMotor.angle[i], (colibrData[i].distMin + colibrData[i].distMin2) / 2.0);
		}
		topic.publicationControlModul(dataControlModul); // Формируем и Публикуем команды для управления Modul
		flag5Colibrovka = false;
		//==========================================================
		// Устанавливаем и корректируем погрешность для расчета угла по лидару и корректируем угол начальной позиции
		float correctAngle = 0;
		for (int i = 0; i < 4; i++)
		{
			correctAngle += (g_angleLaser[i] - colibrData[i].angleMin); // Находим на сколько полученный угол не совпадает с расчетным по начальной позиции
		}
		correctAngle = correctAngle / 4.0; // Находим среднюю ошибку
		printf(" correctAngle= %.3f\n\n", correctAngle);
		exit(1);
	}
	/*
					geometry_msgs::Pose2D startPose2ver;
					startPose2ver.theta = correctAngle;
					startPosition(startPose2ver); // Определяем начальное положение
					offsetAngle = correctAngle; // Меняем поправочный коэффициент
					//==========================================================
					// меняем флаг что-бы сюда больше не попадать
					//modeColibrovka = false;
					*/
/*
}
*/
// Задаем коэфициенты для Калмана
void initKalman()
{

	kalman11.setParametrX(1, 1);
	kalman11.setParametrY(1, 1);

	kalman12.setParametrX(1, 1);
	kalman12.setParametrY(1, 1);

	kalman13.setParametrX(1, 1);
	kalman13.setParametrY(1, 1);
}

// Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
void readParam()
{
	ROS_INFO("+++ readParam");
	ros::NodeHandle nh_private("~");
	// Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим

	//<!-- Указываем стартовую позицию робота. В какое место поставили и куда направили-->
	if (!nh_private.getParam("x", msg_startPose2d.x))
		msg_startPose2d.x = 0.11;
	if (!nh_private.getParam("y", msg_startPose2d.y))
		msg_startPose2d.y = 0.11;
	if (!nh_private.getParam("theta", msg_startPose2d.theta))
		msg_startPose2d.theta = 0.11;

	//<!-- Указываем места расположения столбов на локальной карте -->
	if (!nh_private.getParam("x0", msg_pillar.pillar[0].x))
		msg_pillar.pillar[0].x = 0.11;
	if (!nh_private.getParam("y0", msg_pillar.pillar[0].y))
		msg_pillar.pillar[0].y = 0.11;

	if (!nh_private.getParam("x1", msg_pillar.pillar[1].x))
		msg_pillar.pillar[1].x = 1.11;
	if (!nh_private.getParam("y1", msg_pillar.pillar[1].y))
		msg_pillar.pillar[1].y = 1.11;

	if (!nh_private.getParam("x2", msg_pillar.pillar[2].x))
		msg_pillar.pillar[2].x = 2.11;
	if (!nh_private.getParam("y2", msg_pillar.pillar[2].y))
		msg_pillar.pillar[2].y = 2.11;

	if (!nh_private.getParam("x3", msg_pillar.pillar[3].x))
		msg_pillar.pillar[3].x = 3.11;
	if (!nh_private.getParam("y3", msg_pillar.pillar[3].y))
		msg_pillar.pillar[3].y = 3.11;

	ROS_INFO("startPose x = %.3f y = %.3f theta = %.3f", msg_startPose2d.x, msg_startPose2d.y, msg_startPose2d.theta);
	ROS_INFO("start PillarPose");
	ROS_INFO("x0= %.3f y0 = %.3f", msg_pillar.pillar[0].x, msg_pillar.pillar[0].y);
	ROS_INFO("x1= %.3f y1 = %.3f", msg_pillar.pillar[1].x, msg_pillar.pillar[1].y);
	ROS_INFO("x2= %.3f y2 = %.3f", msg_pillar.pillar[2].x, msg_pillar.pillar[2].y);
	ROS_INFO("x3= %.3f y3 = %.3f", msg_pillar.pillar[3].x, msg_pillar.pillar[3].y);
	ROS_INFO("--- readParam");
}

// Функция для вычисления разницы между углами
double calculateAngleDifference(double prev_angle, double current_angle)
{
	// ROS_INFO("+++ calculateAngleDifference");
	double diff = fmod((current_angle - prev_angle), 360.0); // Вычисляем первоначальную разницу
	if (diff > 180.0)										 // Корректируем разницу, чтобы она находилась в диапазоне [-180, 180]
	{
		diff -= 360.0;
	}
	else if (diff <= -180.0)
	{
		diff += 360.0;
	}
	ROS_INFO_THROTTLE(RATE_OUTPUT, "    calculateAngleDifference = %.3f gradus   %.3f rad", diff, DEG2RAD(diff));
	return diff;
}

// Расчет частоты изменения данных с лазеров
void rateLaserData()
{
	ROS_INFO_THROTTLE(RATE_OUTPUT, "+++ raterateLaserData");
	static uint32_t rateLaserData = 0;	   // Частота с какой меняются данные по лазерам
	static uint32_t timeRateLaserData = 0; // время для расчета
	static float prevSum = 0;			   // Предыдущее значение дистанции с лазера 0
	// Считаем сумму всех значений и смотри если она поменялась, значит какие-то данные изменились
	if (prevSum != (msg_Modul2Data.laser[0].distance + msg_Modul2Data.laser[1].distance + msg_Modul2Data.laser[2].distance + msg_Modul2Data.laser[3].distance))
	{
		rateLaserData++;
		prevSum = (msg_Modul2Data.laser[0].distance + msg_Modul2Data.laser[1].distance + msg_Modul2Data.laser[2].distance + msg_Modul2Data.laser[3].distance); // Запоминаем на следущее сравнение
		ROS_INFO("    prevSum = %f", prevSum);
	}

	if (timeRateLaserData + 1000 <= millis()) // Показываю в логе и обнуляю
	{
		ROS_INFO("    rateLaserData = %lu", rateLaserData);
		rateLaserData = 0;
		timeRateLaserData = millis();
	}
	// ROS_INFO("--- raterateLaserData");
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
// Расчет времени когда остановились. ЕСли движемся то выдаем текущее время. Если стоим то время когда остановились
ros::Time timeStopping(pb_msgs::SSetSpeed msgSpeed_)
{
	static ros::Time timeRet = ros::Time::now();
	static float speedPrevL = msgSpeed_.speedL;
	static float speedPrevR = msgSpeed_.speedR;
	// ROS_INFO("    SumSpeed  %f", speed); //

	if (msgSpeed_.speedL != 0 || msgSpeed_.speedR != 0) // Если текущая скорость не ноль по люблму колесу то возвращаем текущее время
	{
		timeRet = ros::Time::now();
	}
	else if ((msgSpeed_.speedL == 0 && msgSpeed_.speedR == 0) && (speedPrevL != 0 && speedPrevR != 0)) // Если текущая сумма скоростей равна нулю а предыдущая нет то меняем время на текущее и в дальнейшем выдвем его пока не тронемся.
	{
		timeRet = ros::Time::now();
		ROS_INFO("    timeStopping = %8.3f sec", timeRet.toSec()); // Время
	}
	speedPrevL = msgSpeed_.speedL;
	speedPrevR = msgSpeed_.speedR;
	return timeRet;
}

#endif
