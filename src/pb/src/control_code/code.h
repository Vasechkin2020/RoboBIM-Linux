#ifndef CODE_H
#define CODE_H

// #include "pillar.h"
//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
// void callback_Driver(pb_msgs::Struct_Driver2Data msg); //

void readParam(); // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы

void initCommandArray(int verCommand_); // Заполнение маасива команд

// pb_msgs::SControlDriver speedCorrect(pb_msgs::SDriver2Data Driver2Data_msg_, pb_msgs::SControlDriver Data2Driver_); // Корректировка скорости движения в зависимости от датчиков растояния перед
// void collectCommand(); // //Функция формирования команды для нижнего уровня на основе всех полученных данных, датчиков и анализа ситуации

// **********************************************************************************
float filtrComplem(float koef_, float oldData_, float newData_); // функция фильтрации, берем старое значение с некоторым весом

// функция фильтрации, берем старое значение с некоторым весом
float filtrComplem(float koef_, float oldData_, float newData_)
{
	return (1 - koef_) * oldData_ + (koef_ * newData_);
}
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
		commandArray[0].mode = 1;
		commandArray[0].duration = 20000;
		commandArray[0].velL = 0.05;
		commandArray[0].velR = 0.05;

		commandArray[1].mode = 1;
		commandArray[1].duration = 22000;
		commandArray[1].velL = 0.1;
		commandArray[1].velR = 0.1;

		commandArray[2].mode = 1;
		commandArray[2].duration = 10000;
		commandArray[2].velL = 0;
		commandArray[2].velR = 0;

		commandArray[3].mode = 1;
		commandArray[3].duration = 20000;
		commandArray[3].velL = -0.05;
		commandArray[3].velR = -0.05;

		commandArray[4].mode = 1;
		commandArray[4].duration = 22000;
		commandArray[4].velL = -0.1;
		commandArray[4].velR = -0.1;

		commandArray[5].mode = 1;
		commandArray[5].duration = 50000;
		commandArray[5].velL = 0;
		commandArray[5].velR = 0;

		commandArray[6].mode = 9;
	}
	if (verCommand_ == 2)
	{
		commandArray[0].mode = 1;
		commandArray[0].duration = 100000;
		commandArray[0].velL = 0.1;
		commandArray[0].velR = 0.1;

		commandArray[1].mode = 1;
		commandArray[1].duration = 5000;
		commandArray[1].velL = 0.0;
		commandArray[1].velR = 0.0;

		commandArray[2].mode = 1;
		commandArray[2].duration = 100000;
		commandArray[2].velL = -0.1;
		commandArray[2].velR = -0.1;

		commandArray[3].mode = 1;
		commandArray[3].duration = 5000;
		commandArray[3].velL = 0.0;
		commandArray[3].velR = 0.0;

		commandArray[4].mode = 9;
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
		commandArray[0].mode = 1;
		commandArray[0].duration = 2000;
		commandArray[0].velL = 0.02;
		commandArray[0].velR = -0.02;

		commandArray[1].mode = 1;
		commandArray[1].duration = 2000;
		commandArray[1].velL = 0.0;
		commandArray[1].velR = 0.0;

		commandArray[2].mode = 1;
		commandArray[2].duration = 2000;
		commandArray[2].velL = 0.02;
		commandArray[2].velR = -0.02;

		commandArray[3].mode = 1;
		commandArray[3].duration = 2000;
		commandArray[3].velL = 0.0;
		commandArray[3].velR = 0.0;

		commandArray[4].mode = 1;
		commandArray[4].duration = 2000;
		commandArray[4].velL = 0.02;
		commandArray[4].velR = -0.02;

		commandArray[5].mode = 1;
		commandArray[5].duration = 2000;
		commandArray[5].velL = 0.0;
		commandArray[5].velR = 0.0;

		commandArray[6].mode = 1;
		commandArray[6].duration = 2000;
		commandArray[6].velL = 0.02;
		commandArray[6].velR = -0.02;

		commandArray[7].mode = 1;
		commandArray[7].duration = 2000;
		commandArray[7].velL = 0.0;
		commandArray[7].velR = 0.0;

		commandArray[8].mode = 1;
		commandArray[8].duration = 2000;
		commandArray[8].velL = 0.02;
		commandArray[8].velR = -0.02;

		commandArray[9].mode = 1;
		commandArray[9].duration = 2000;
		commandArray[9].velL = 0.0;
		commandArray[9].velR = 0.0;

//****************************************
		commandArray[10].mode = 1;
		commandArray[10].duration = 2000;
		commandArray[10].velL = -0.02;
		commandArray[10].velR = 0.02;

		commandArray[11].mode = 1;
		commandArray[11].duration = 2000;
		commandArray[11].velL = 0.0;
		commandArray[11].velR = 0.0;

		commandArray[12].mode = 1;
		commandArray[12].duration = 2000;
		commandArray[12].velL = -0.02;
		commandArray[12].velR = 0.02;

		commandArray[13].mode = 1;
		commandArray[13].duration = 2000;
		commandArray[13].velL = 0.0;
		commandArray[13].velR = 0.0;

		commandArray[14].mode = 1;
		commandArray[14].duration = 2000;
		commandArray[14].velL = -0.02;
		commandArray[14].velR = 0.02;

		commandArray[15].mode = 1;
		commandArray[15].duration = 2000;
		commandArray[15].velL = 0.0;
		commandArray[15].velR = 0.0;

		commandArray[16].mode = 1;
		commandArray[16].duration = 2000;
		commandArray[16].velL = -0.02;
		commandArray[16].velR = 0.02;

		commandArray[17].mode = 1;
		commandArray[17].duration = 2000;
		commandArray[17].velL = 0.0;
		commandArray[17].velR = 0.0;

		commandArray[18].mode = 1;
		commandArray[18].duration = 2000;
		commandArray[18].velL = -0.02;
		commandArray[18].velR = 0.02;

		commandArray[19].mode = 1;
		commandArray[19].duration = 2000;
		commandArray[19].velL = 0.0;
		commandArray[19].velR = 0.0;

		commandArray[20].mode = 1;
		commandArray[20].duration = 2000;
		commandArray[20].velL = -0.02;
		commandArray[20].velR = 0.02;

		commandArray[21].mode = 1;
		commandArray[21].duration = 2000;
		commandArray[21].velL = 0.0;
		commandArray[21].velR = 0.0;

		//-----------------------
		commandArray[22].mode = 9;
	}
		if (verCommand_ == 5)
	{
		commandArray[0].mode = 2;
		commandArray[0].angle = -40;

		commandArray[1].mode = 1;
		commandArray[1].duration = 5000;

		commandArray[2].mode = 2;
		commandArray[2].angle = 0;

		commandArray[3].mode = 1;
		commandArray[3].duration = 5000;

		commandArray[4].mode = 2;
		commandArray[4].angle = 40;

		commandArray[5].mode = 1;
		commandArray[5].duration = 5000;
		
		commandArray[6].mode = 2;
		commandArray[6].angle = 0;

		commandArray[7].mode = 1;
		commandArray[7].duration = 5000;

		commandArray[8].mode = 9;

		for (int i = 0; i < 8; i++)
		{
			ROS_INFO("i=%i angle= %f",i,commandArray[i].angle);
		}
		
	}
		if (verCommand_ == 6)
	{
		commandArray[0].mode = 2;
		commandArray[0].angle = -40;

		commandArray[1].mode = 1;
		commandArray[1].duration = 5000;

		commandArray[2].mode = 2;
		commandArray[2].angle = 40;

		commandArray[3].mode = 1;
		commandArray[3].duration = 5000;

		commandArray[4].mode = 2;
		commandArray[4].angle = 0;

		commandArray[5].mode = 1;
		commandArray[5].duration = 5000;
		
		commandArray[6].mode = 9;
	}
}

void readParam() // Считывание переменных параметров из лаунч файла при запуске. Там офсеты и режимы работы
{
	ros::NodeHandle nh_private("~");
    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
	if (!nh_private.getParam("verComand", verComand))
		verComand = 999;

    ROS_INFO("--- Start node with parametrs:");
    ROS_INFO("verComand = %i",verComand);
    ROS_INFO("---");
}


void timeCycle(ros::Time timeStart_, ros::Time timeNow_) // Выводим справочно время работы цикла
{
	    ros::Time timeEnd = ros::Time::now(); // Захватываем конечный момент времени
        ros::Duration durationEnd = timeEnd - timeNow_; // Находим разницу между началом и концом
        ros::Duration durationStart = timeEnd - timeStart_; // Находим разницу между началом и концом
        double dtEnd = durationEnd.toSec()*1000;            // Получаем количество милисекунд
        double dtStart = durationStart.toSec();            // Получаем количество секунд
		if (dtEnd>5) // Если цикл занял бользе 5 милисекунд значит что не уложились в 200 Нz
        	ROS_INFO("    !!! cycle = %8.3f msec", dtEnd); // Время цикла в милисекундах
		else
        	ROS_INFO_THROTTLE(1,"    dtStart = %7.0f sec | last cycle = %8.3f msec", dtStart, dtEnd); // Время цикла в милисекундах
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
