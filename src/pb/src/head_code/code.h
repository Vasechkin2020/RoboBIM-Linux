#ifndef CODE_H
#define CODE_H

// #include "pillar.h"
//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
void callback_Lidar(sensor_msgs::LaserScan::ConstPtr msg); //
void callback_Pillar(pb_msgs::topicPillar msg);			   //
void callback_StartPose2D(pb_msgs::point msg);			   //

void callback_Driver(pb_msgs::Struct_Driver2Data msg); //
void callback_Modul(pb_msgs::Struct_Modul2Data msg);

void calculationOdometry(); // Расчет одометрии и применения ее для всех режимов

void poseComplementMode10(); // Комплеиентация Odom10

long map(long x, long in_min, long in_max, long out_min, long out_max); // Переводит значение из одного диапазона в другой, взял из Ардуино

void startPosition(geometry_msgs::Pose2D &startPose2d_); // Разбираем топик со стартовой позицией робота

void testFunction(); // Тест математических ипрочих функций

void angleMPU(); // Расчет угла положения на сонове данных сдатчика MPU

float minDistance(float laserL_, float laserR_, float uzi1_); // Находим минимальную дистанцию из 3 датчиков
// pb_msgs::SControlDriver speedCorrect(pb_msgs::SDriver2Data Driver2Data_msg_, pb_msgs::SControlDriver Data2Driver_); // Корректировка скорости движения в зависимости от датчиков растояния перед
// void collectCommand(); // //Функция формирования команды для нижнего уровня на основе всех полученных данных, датчиков и анализа ситуации

// **********************************************************************************
void calcNewOdom(SOdom &odom_, STwistDt data_);						// На вход подаются старая одометрия и новые угловая угловая скорость. Возвращается новая позиция по данным угловым скоростям // Обработка пришедших данных.Обсчитываем одометрию по энкодеру
STwistDt calcTwistFromWheel(pb_msgs::SSetSpeed control_);			// Обработка пришедших данных.Обсчитываем одометрию по энкодеру
STwistDt calcTwistFromMpu(SMpu mpu_, float koef_);					// Обработка пришедших данных.Обсчитываем угловые скорости по энкодеру
STwistDt calcTwistUnited(STwistDt wheelTwist_, STwistDt mpuTwist_); // Функция комплементации угловых скоростей полученных с колес и с датчика MPU и угла поворота
float autoOffsetX(float data_);										// Функция считаем скользящее среднее из 128 элементов как офсет значений при стоянии на месте
float autoOffsetY(float data_);										// Функция считаем скользящее среднее из 128 элементов как офсет значений при стоянии на месте
float filtrComplem(float koef_, float oldData_, float newData_);	// функция фильтрации, берем старое значение с некоторым весом
// void calculateOdometryFromMpu(SMpu mpu_);					   // Обработка пришедших данных.Обсчитываем одометрию по энкодеру

void initArray();

// функция фильтрации, берем старое значение с некоторым весом
float filtrComplem(float koef_, float oldData_, float newData_)
{
	return (1 - koef_) * oldData_ + (koef_ * newData_);
}

void callback_Lidar(sensor_msgs::LaserScan::ConstPtr msg)
{
	msg_lidar = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgLidar = true;
}
void callback_Pillar(pb_msgs::topicPillar msg)
{
	msg_pillar = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgPillar = true;
}
void callback_StartPose2D(geometry_msgs::Pose2D msg)
{
	msg_startPose2d = msg; // Пишем в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgStartPose = true;
}
void callback_Driver(pb_msgs::Struct_Driver2Data msg)
{
	msg_Driver2Data = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
	flag_msgDriver = true;
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
// Разбираем топик со стартовой позицией робота
void startPosition(geometry_msgs::Pose2D &startPose2d_)
{
	ROS_INFO("------------------------- startPosition -------------------------------------");
	g_poseLidar.mode0.x = startPose2d_.x; // Пока считаем что передаем положение центра лидара и поэтому ему присваиваем значение, потом надо будет добавлять смещение до центра поворота между колесами
	g_poseLidar.mode0.y = startPose2d_.y;
	g_poseLidar.mode0.th = startPose2d_.theta;

	// g_poseLidar.mode1.x = startPose2d_.x; // Пока считаем что передаем положение центра лидара и поэтому ему присваиваем значение, потом надо будет добавлять смещение до центра поворота между колесами
	// g_poseLidar.mode1.y = startPose2d_.y;
	// g_poseLidar.mode1.th = startPose2d_.theta;

	// g_poseLidar.mode2.x = startPose2d_.x; // Пока считаем что передаем положение центра лидара и поэтому ему присваиваем значение, потом надо будет добавлять смещение до центра поворота между колесами
	// g_poseLidar.mode2.y = startPose2d_.y;
	// g_poseLidar.mode2.th = startPose2d_.theta;

	// g_poseLidar.mode3.x = startPose2d_.x; // Пока считаем что передаем положение центра лидара и поэтому ему присваиваем значение, потом надо будет добавлять смещение до центра поворота между колесами
	// g_poseLidar.mode3.y = startPose2d_.y;
	// g_poseLidar.mode3.th = startPose2d_.theta;

	// g_poseLidar.mode4.x = startPose2d_.x; // Пока считаем что передаем положение центра лидара и поэтому ему присваиваем значение, потом надо будет добавлять смещение до центра поворота между колесами
	// g_poseLidar.mode4.y = startPose2d_.y;
	// g_poseLidar.mode4.th = startPose2d_.theta;

	// g_poseLidar.mode10.x = startPose2d_.x; // Пока считаем что передаем положение центра лидара и поэтому ему присваиваем значение, потом надо будет добавлять смещение до центра поворота между колесами
	// g_poseLidar.mode10.y = startPose2d_.y;
	// g_poseLidar.mode10.th = startPose2d_.theta;

	g_poseLidar.mode1 = g_poseLidar.mode0;
	g_poseLidar.mode2 = g_poseLidar.mode0;
	g_poseLidar.mode3 = g_poseLidar.mode0;
	g_poseLidar.mode4 = g_poseLidar.mode0;
	g_poseLidar.mode10 = g_poseLidar.mode0;

	g_angleMPU = startPose2d_.theta;

	odomMode0.pose.x = startPose2d_.x;
	odomMode0.pose.y = startPose2d_.y;
	odomMode0.pose.th = DEG2RAD(startPose2d_.theta); // В одометрии угол в радианах
	odomMode10.pose.x = startPose2d_.x;
	odomMode10.pose.y = startPose2d_.y;
	odomMode10.pose.th = DEG2RAD(startPose2d_.theta); // В одометрии угол в радианах

	printf("START RAD2DEG(odomMode0.pose.th) = % .3f \n", RAD2DEG(odomMode0.pose.th));

	ROS_INFO("startPosition lidarPose x= %.3f y= %.3f th= %.3f ", g_poseLidar.mode1.x, g_poseLidar.mode1.y, g_poseLidar.mode1.th);
	ROS_INFO("-------------------------            ------------------------------------- \n");
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
// Тест математических ипрочих функций
void testFunction()
{

	// SPoint test1;
	// SPose test2;
	// test1.x = 1376.27;
	// test1.y = 1079.32;

	// test2.x= 500;
	// test2.y= 900;
	// test2.th= 15;

	// SPoint ggg = pointGlobal2Local(test1,test2);

	// test1.x=800;
	// test1.y=400;
	// ggg = pointLocal2Global(test1,test2);

	// test1.x=2699.55;
	// test1.y=428.29;

	// float rrr = angleThetaFromPoint(test1);
}

// Обработка пришедших данных.Обсчитываем одометрию по энкодеру
void calcNewOdom(SOdom &odom_, STwistDt data_) // На вход подаются старая одометрия и новые угловая угловая скорость. Возвращается новая позиция по данным угловым скоростям
{
	// ROS_INFO("IN calcNewOdom pose.x= % .3f y= % .3f th= % .3f ", odom_.pose.x, odom_.pose.y, RAD2DEG(odom_.pose.th));
	if (data_.dt < 0.005) // Если пришли данные с нулевой дельтой то сразу выходим и ничего не считаем
	{
		printf("calcNewOdom dt< 0.005 !!!! \n");
		return;
	}
	odom_.twist = data_.twist; // Ничего не меняем в угловой скорости
	SPoint pointLoc;
	pointLoc.x = data_.twist.vx * data_.dt; // Находим проекции скорсти на оси за интревал времени это коокрдинаты нашей точки в локальной системе координат
	pointLoc.y = data_.twist.vy * data_.dt;
	// printf(" Local system pointLoc.x= % .3f y= % .3f dt= % .3f th= % .3f | \n", pointLoc.x, pointLoc.y, data_.dt, RAD2DEG(odom_.pose.th));

	// printf("DO pose.x= %.3f pose.y= %.3f pose.th= %.3f / ", odom_.pose.x, odom_.pose.y, RAD2DEG(odom_.pose.th));
	// Находим смещние по осям матрица координаты точки из локальной системы координат в глобальной
	double delta_x = pointLoc.x * cos(odom_.pose.th) + pointLoc.y * sin(odom_.pose.th);
	double delta_y = -pointLoc.x * sin(odom_.pose.th) + pointLoc.y * cos(odom_.pose.th);
	// printf("Global system delta.x= % .3f y= % .3f | \n", delta_x, delta_y);
	//  Меняем координаты и угол на основе вычислений
	//  odom_.pose.x += delta_x; // Вычисляем координаты
	//  odom_.pose.y += delta_y; // Вычисляем координаты

	SPoint pointGlob = pointLocal2GlobalRosRAD(pointLoc, odom_.pose);
	// printf("Global system x= % .3f y= % .3f | \n", pointGlob.x, pointGlob.y);

	odom_.pose.x = pointGlob.x; // Вычисляем координаты
	odom_.pose.y = pointGlob.y; // Вычисляем координаты

	// printf("twist.x= %.4f y= %.4f th= %.4f gradus ", bno055.twist.vx, bno055.twist.vy, bno055.twist.vth);
	// Меняем координаты и угол на основе вычислений
	odom_.pose.th += data_.twist.vth * data_.dt; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот
	if (odom_.pose.th > (2 * M_PI))
		(odom_.pose.th -= (2 * M_PI));
	if (odom_.pose.th < 0)
		(odom_.pose.th += (2 * M_PI));

	ROS_WARN("OUT calcNewOdom pose.x= % .3f y= % .3f th= % .3f ", odom_.pose.x, odom_.pose.y, RAD2DEG(odom_.pose.th));
}

// Обработка пришедших данных.Обсчитываем одометрию по энкодеру
STwistDt calcTwistFromWheel(pb_msgs::SSetSpeed control_)
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
	printf("calc dt= %f | ", dt);
	time = time_now;
	if (dt < 0.005) // При первом запуске просто выходим из функции
	{
		printf("First start. alcTwistFromWheel dt< 0.005 !!!! \n");
		return ret;
	}
	// double speedL = PERIMETR * Driver2Data.motor.rpsEncodL; // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
	// double speedR = PERIMETR * Driver2Data.motor.rpsEncodR; // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
	// double speedL = PERIMETR * control_.speedL; // Тут скорость заданная в оборотах в секунду преврщаем в метри в секунду
	// double speedR = PERIMETR * control_.speedR; // Тут скорость заданная в оборотах в секунду преврщаем в метри в секунду
	double speedL = control_.speedL; // Скорость в метрах в секуду
	double speedR = control_.speedR; // Скорость в метрах в секуду

	double sumSpeed = speedL + speedR;
	double deltaSpeed = speedL - speedR;
	double speed = (speedR + speedL) / 2.0; // Находим скорость всего обьекта.
	printf("speed car= %.6f | ", speed);
	//*******************************************************************************************************************************************************
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

	// printf (" theta = %.3f \n", theta);
	//  Находим линейные скорости из моего вектора скорости. Я задаю общую скорость движения (длинна вектора), ее надо разложить на проекции по осям x y. Это будут линейные скорости
	//  speed = 0.26;
	twist.vx = speed * cos(theta * dt); // Проекция моей скорости на ось X получаем линейную скорость по оси за секунуду
	twist.vy = speed * sin(theta * dt); // Проекция моей скорости на ось Y получаем линейную скорость по оси за секунуду
	twist.vth = theta;					// Угловая скорость в радианах.

	printf(" Wheel % .3f % .3f \n", twist.vx, twist.vy);
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

		odomMode0.pose = pose;
		odomMode0.twist = twist;
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
// Расчет угла положения на сонове данных сдатчика MPU
void angleMPU()
{
	static float predAngleZ = 0;
	static bool first = true;
	printf("flag_msgDriver in... ");
	if (first) // Делаем в первый приход данных
	{
		predAngleZ = msg_Driver2Data.mpu.angleEuler.yaw; // Запоминаяем угол поворота Для следующего обсчета
		first = false;
		printf("First start -> ");
	}
	else // Всегда кроме первого
	{
		g_angleMPU += (msg_Driver2Data.mpu.angleEuler.yaw - predAngleZ); // Меняем угол поворота увеличивая на разницу. Разобраться с 360 и переходом через 0
		predAngleZ = msg_Driver2Data.mpu.angleEuler.yaw;				 // Запоминаяем угол поворота Для следующего обсчета
																		 // dataNode.parsingDriver(msg_Driver2Data);
	}
	printf("g_angleMPU = % .3f \n", g_angleMPU);
}
// Расчет одометрии и применения ее для всех режимов
void calculationOdometry()
{
	// printf("1 RAD2DEG(odomMode0.pose.th) = % .3f \n", RAD2DEG(odomMode0.pose.th));
	wheelTwistDt = calcTwistFromWheel(msg_Speed); // Обработка пришедших данных. По ним считаем линейные скорости по осям и угловую по углу. Запоминаем dt
	calcNewOdom(odomMode0, wheelTwistDt);		  // На основе линейных скоростей считаем новую позицию и угол по колесам
	g_poseLidar.mode0.x = odomMode0.pose.x;
	g_poseLidar.mode0.y = odomMode0.pose.y;
	g_poseLidar.mode0.th = RAD2DEG(odomMode0.pose.th);
	//---------------
	calcNewOdom(odomMode10, wheelTwistDt); // На основе линейных скоростей считаем новую позицию и угол для скомплементированной одометрии 100 Герц считаем и потом 10 Герц правим

	// printf("2 RAD2DEG(odomMode0.pose.th) = % .3f \n", RAD2DEG(odomMode0.pose.th));

	// mpuTwistDt = calcTwistFromMpu(Driver2Data.bno055, 0.2); // Расчет и оформление в структуру ускорений по осям (линейных скоростей) и  разделить получение угловых скоростей и расчет сновой точки на основе этих скоростей
	// calcNewOdom(odomMpu, mpuTwistDt);                       // Обработка пришедших данных.Обсчитываем одометрию по датчику MPU BNO055
	// topic.publishOdomMpu();

	// // тут написать функцию комплементации данных угловых скоростей с разными условиями когда и в каком соотношении скомплементировать скорсти с двух источников
	// unitedTwistDt = calcTwistUnited(wheelTwistDt, mpuTwistDt);
	// calcNewOdom(odomUnited, unitedTwistDt); // // На основе линейных скоростей считаем новую позицию и угол
	// topic.publishOdomUnited();              // Публикация одометрии по моторам с корректировкой с верхнего уровня
	//-------------------------
}
// Комплеиентация Odom10
void poseComplementMode10()
{
	// ROS_WARN("1odomMode10.pose.x = % .3f odomMode10.pose.y = % .3f odomMode10.pose.th = %.3f", odomMode10.pose.x, odomMode10.pose.y, RAD2DEG(odomMode10.pose.th));
	odomMode10.pose.x = 0.8 * odomMode10.pose.x + 0.2 * g_poseLidar.mode1.x;
	odomMode10.pose.y = 0.8 * odomMode10.pose.y + 0.2 * g_poseLidar.mode1.y;
	// Комплементация угла
	float mediumTheta = (g_poseLidar.mode1.th + g_poseLidar.mode2.th) / 2.0;
	// printf("mediumTheta DEG= %.3f mediumTheta RAD= %.3f \n", mediumTheta, DEG2RAD(mediumTheta));
	odomMode10.pose.th = 0.6 * g_angleMPU + 0.3 * odomMode10.pose.th + 0.1 * DEG2RAD(mediumTheta);
	printf("g_angleMPU = % .3f odomMode10.pose.th = % .3f mediumTheta = % .3f \n", g_angleMPU, odomMode10.pose.th, DEG2RAD(mediumTheta));
	ROS_WARN("odomMode10.pose.x = % .3f odomMode10.pose.y = % .3f odomMode10.pose.th = %.3f", odomMode10.pose.x, odomMode10.pose.y, RAD2DEG(odomMode10.pose.th));
	if (isnan(odomMode10.pose.x) || isnan(odomMode10.pose.y) || isnan(odomMode10.pose.th))
	{
		exit(0);
	}
	// Копируем в переменную для применения там где использую g_poseLidar
	g_poseLidar.mode10.x = odomMode10.pose.x;
	g_poseLidar.mode10.y = odomMode10.pose.y;
	g_poseLidar.mode10.th = RAD2DEG(odomMode10.pose.th);
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

void startColibrovka()
{

}

#endif
