#ifndef CODE_H
#define CODE_H

// Обратный вызов при опросе топика Body
void message_callback_Body(const my_msgs::Body &msg)
{
    Body_msg = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
    //ROS_INFO("Message: %f", msg.bme280.pressure);
}

// Обратный вызов при опросе топика Control
void message_callback_Control(const my_msgs::Control &msg)
{
    Control_msg = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
    //ROS_INFO("Message: %f", msg.bme280.pressure);
}
// Переводит значение из одного диапазона в другой, взял из Ардуино
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Функция вычисляющая новую скорость в зависимости от условий
float newSpeed()
{
    float delta_speed = g_my_position.speed.target - g_my_position.speed.fact; // Находим разницу скоростей
    if (delta_speed > 0)                                                       // Если заданна скорость больше чем есть, тогда ускоряемся
    {
        if (delta_speed > step_accel_up)
        {
            g_my_position.speed.fact += step_accel_up; // Увеличиваем скорость на величину ускорения
        }
        else
        {
            g_my_position.speed.fact += delta_speed; // Увеличиваем скорость на оставшуюся разницу
        }
    }
    if (delta_speed < 0) // Если заданна меньше меньше чем есть, тогда тормозим
    {
        if (delta_speed < -step_accel_down)
        {
            g_my_position.speed.fact -= step_accel_down; // Уменьшаем скорость на величину ускорения
        }
        else
        {
            g_my_position.speed.fact += delta_speed; // Уменьшаем скорость на оставшуюся разницу
        }
    }
    return g_my_position.speed.fact;
}
// //Функция фильтрующая целевую скорость в зависимости от датчика растояния лазерного
// void filterLaser()
// {
//     //Если растояние будет меньше границы диапазона или больше границы диапазона
//     if (Body_msg.distance_lazer > (DISTANCE_LAZER + DIAPAZON) || // Это обрыв
//         Body_msg.distance_lazer < (DISTANCE_LAZER - DIAPAZON))   // Это преграда
//     {
//         // В таком случае движение зависит от радиуса, разворачиваться не мешаем. а ехать вперед или по большому радиусу запрещаем
//         if (g_my_position.radius == 0 ||
//             abs(g_my_position.radius) > 0.2)
//         {
//             g_my_position.speed.target = 0;
//         }
//     }
//     ROS_INFO("Body_msg.distance_lazer= %.2f", Body_msg.distance_lazer);
//     ROS_INFO("g_my_position.speed.target= %.2f", g_my_position.speed.target);
// }
// //Функция фильтрующая целевую скорость в зависимости от датчика растояния ультразвукового
// void filterUzi()
// {
//     // Увеличиваем на 1000 так как команда MAP только в целый числах
//     long standart = (long)(SPEED_STANDART * 1000.0);
//     long speed_max = (long)(SPEED_MAX * 1000.0);
//     long dist = (long)(Body_msg.distance_uzi * 1000.0);

//     if (Body_msg.distance_uzi < 0.2) //Если расстояние меньше 0.2 метра то прямо и с большим радиусом ехать не разрешаем, можно только крутиться и разварачиваться на месте
//     {
//         if (g_my_position.radius == 0 ||
//             abs(g_my_position.radius) > 0.2)
//         {
//             g_my_position.speed.target = 0; // До 20 см
//         }
//     }
//     else
//     {
//         if (Body_msg.distance_uzi < 1) // Если от 0.2 метра до 1 метров
//         {
//             g_my_position.speed.target = map(dist, 200, 1000, 0, speed_max) / 1000.0; // От стандартной до максимальной
//         }
//         else
//         {
//             g_my_position.speed.target = SPEED_MAX; // Максимальная скорость
//         }
//     }
//     ROS_INFO("Body_msg.distance_uzi= %.2f", Body_msg.distance_uzi);
//     ROS_INFO("g_my_position.speed.target= %.2f", g_my_position.speed.target);
// }
// //Функция фильтрующая целевую скорость в зависимости от радиуса движения, что-бы быстро не крутился
// void filterRadius()
// {
//     ROS_INFO("g_my_position.radius= %.2f", g_my_position.radius);
//     if (abs(g_my_position.radius) < 1.5 * DISTANCE_WHEELS && g_my_position.radius != 0 )
//     {
//         // if (g_my_position.speed.target > SPEED_ROTATION) // Если скорость по цели превышает то ограничиваем, а ели нет то можно вращаться с меньшей скоростью
//         // {
//             g_my_position.speed.target = SPEED_ROTATION; // Устанавливаем скорость вращения
//         // }
//     }
// }
//Функция формирования команды для нижнего уровня на основе всех полученных данных, датчиков и анализа ситуации
void collectCommand()
{
    Command_msg.id++; //Увеличиваем номер команды
    ROS_INFO_THROTTLE (3,"%s Command_msg.id= %i",NN, Command_msg.id);
    Command_msg.command_body = Control_msg.startStop; // Команда движение или стоим
    ROS_INFO_THROTTLE (3,"%s Command_msg.command_body= %i", NN, Command_msg.command_body);

    if (Control_msg.startStop == 0)  // Если команда  стоп то сбрасываем фактическую скорость
    {
        g_my_position.speed.fact = 0;
    }
    
    float radius = map(Control_msg.radius, -100, 100, -MAX_RADIUS * 1000, MAX_RADIUS * 1000) / 1000.0; //Преобразуем диапазон из Remote XY от -100 до 100 в допустимый диапазон, только челые числа функция использует
    if (radius > 0 ) 
    {
       radius = MAX_RADIUS - radius + 0.01; // Прибавляем чуть-чуть чтобы радиус не получался 0 на краях
    }
    if (radius < 0 ) 
    {
       radius = -MAX_RADIUS - radius - 0.01; // Отнимаем чуть-чуть чтобы радиус не получался 0 на краях
    }
    Command_msg.radius = g_my_position.radius = radius; // Присваиваем радиус
    ROS_INFO_THROTTLE (3,"%s Command_msg.radius= %f",NN, Command_msg.radius);

    Command_msg.motor_video_angle = Control_msg.camera;  // Положение ккамеры от 0 до 100 градусов
    Command_msg.program_led = Control_msg.led_program;   // Программа для светодиодов

    //-----------------------------------------------------------
    g_my_position.speed.target = SPEED_STANDART; // Всегда начинаем с цели в виде типовой скорости и уже потом ее меняем в зависимости от обстоятельств

    //ROS_INFO("g_my_position.speed.target 1 = %.2f", g_my_position.speed.target);
    //Получаем цель по скорости на основаниии манипулятора, это типа наше текущее желание по скорости
    //g_my_position.speed.target = map(Control_msg.speed, 0, 100, 0, SPEED_MAX * 1000) / 1000.0; //Преобразуем диапазон из Remote XY от 0 до 100 в допустимый диапазон, только челые числа функция использует
    //ROS_INFO("g_my_position.speed.target 2 = %.1f", g_my_position.speed.target);
    // В фильтрах меняем цель по скорости в зависимости от обстоятельств окружающего мира, препятствия, обрывы
    // filterUzi();
    // ROS_INFO("g_my_position.speed.target 3 = %.2f", g_my_position.speed.target);
    // filterRadius(); // При маленьком радиусе не крутиться с большой скоростью
    // ROS_INFO("g_my_position.speed.target 4 = %.2f", g_my_position.speed.target);
    // filterLaser(); // Ограничения что-бы не упасть с обрыва
    // ROS_INFO("g_my_position.speed.target 5 = %.1f", g_my_position.speed.target);

    //float speed = newSpeed(); // Устанавливаем новую фактическую скорость после того как разобрались с целью по скорости
    float speed = 0.2; // Устанавливаем новую фактическую скорость после того как разобрались с целью по скорости
    if (radius < 0.05 && radius > 0)
    {
        speed = 0.1;
    }
    if (radius > -0.05 && radius < 0)
    {
        speed = 0.1;
    }
    Command_msg.speed = speed;
    ROS_INFO_THROTTLE(3,"%s Command_msg.speed= %f",NN, Command_msg.speed);
    
    //ROS_INFO("newSpeed = %.3f", speed);
    //INFO ROS_INFO("-------------- ");
    //-----------------------------------------------------------
}

#endif

/*
начать считать фактическое положение по одометрии и по ускорению от IMU в ноде Body 

проверить настройки лазерного датчика, а то больно сильно скачут значения

*/