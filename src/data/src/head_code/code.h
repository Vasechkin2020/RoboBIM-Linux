#ifndef CODE_H
#define CODE_H

// Функция обраьтного вызова по подпичке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
void callback_Joy(sensor_msgs::Joy msg)
{
    // joy.data = msg;  // Присваиваем в публичную перменную класса данные полученные по spinOnce()
    joy2Head = joy.processing(msg); // Записываем данные в класс
}

void callback_Pillar(sensor_msgs::LaserScan::ConstPtr msg)
{
    pillar.scanCallback(msg); // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
}

void callback_Driver(data::Struct_Driver2Data msg)
{
    Driver2Data = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
}
//  Разбор и установка параметров которые задали в launch файле при запуске
void setParam(ros::NodeHandle nh_private_)
{
    
    // Установка начальных координат у игла направления для машинки
    nh_private_.param<double>("start_x", position.x, 1.11);
    nh_private_.param<double>("start_y", position.y, 1.11);
    nh_private_.param<double>("start_th", position.th, 1.11);

    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
    if (!nh_private_.getParam("/x0", pillar.pillarOut[0].x))
        pillar.pillarOut[0].x = 1.11;
    if (!nh_private_.getParam("/y0", pillar.pillarOut[0].y))
        pillar.pillarOut[0].y = 1.11;

    if (!nh_private_.getParam("/x1", pillar.pillarOut[1].x))
        pillar.pillarOut[0].x = 1.11;
    if (!nh_private_.getParam("/y1", pillar.pillarOut[1].y))
        pillar.pillarOut[0].y = 1.11;

    if (!nh_private_.getParam("/x2", pillar.pillarOut[2].x))
        pillar.pillarOut[0].x = 1.11;
    if (!nh_private_.getParam("/y2", pillar.pillarOut[2].y))
        pillar.pillarOut[0].y = 1.11;

    if (!nh_private_.getParam("/x3", pillar.pillarOut[3].x))
        pillar.pillarOut[0].x = 1.11;
    if (!nh_private_.getParam("/y3", pillar.pillarOut[3].y))
        pillar.pillarOut[0].y = 1.11;

    if (!nh_private_.getParam("/x4", pillar.pillarOut[4].x))
        pillar.pillarOut[0].x = 1.11;
    if (!nh_private_.getParam("/y4", pillar.pillarOut[4].y))
        pillar.pillarOut[0].y = 1.11;

    // ROS_INFO("pos.x = %f pos.y = %f pos.th = %f ",pos.pos.x,pos.pos.y,pos.pos.th);
    // ROS_INFO("pillarOut[0].x = %f pillarOut[0].y = %f ",pillar.pillarOut[0].x,pillar.pillarOut[0].y);
}
// Находим минимальную дистанцию из 3 датчиков
float minDistance(float lazer1_, float lazer2_, float uzi1_)
{
    float min = lazer1_;
    if (lazer2_ < min)
    {
        min = lazer2_;
    }
    if (uzi1_ < min)
    {
        min = uzi1_;
    }
    return min;
}

// Переводит значение из одного диапазона в другой, взял из Ардуино
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Корректировка скорости движения в зависимости от датчиков растояния перед
data::Struct_Data2Driver speedCorrect(data::Struct_Data2Driver Data2Driver_)
{
    float min = minDistance(Driver2Data.lazer1.distance, Driver2Data.lazer2.distance, Driver2Data.uzi1.distance); // Находим минимальную дистанцию из 3 датчиков
    if (min < 0.5)                                                                                                // Если меньше полметра
    {
        long minDist = (long)(min * 1000); // Превращаем в целое и увеличиваем умножая на 1000 для точности
        if (minDist < 100)
            minDist = 100;
        float proc = map(minDist, 100, 500, 0, 100);
        proc = proc / 100; // Превращаем в проценты
        Data2Driver_.control.speed = proc * Data2Driver_.control.speed;
        // ROS_INFO("Correct speed. Min distance = %f, New speed = %f", min, Data2Driver_.control.speed);
    }
    // printf("sp= %f \n", Data2Driver_.control.speed);
    return Data2Driver_;
}

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

#endif
