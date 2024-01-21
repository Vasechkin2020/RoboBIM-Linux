#ifndef CODE_H
#define CODE_H

// #include "pillar.h"
//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************

void callback_Joy(sensor_msgs::Joy msg);                   // Функция обраьтного вызова по подпичке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
void callback_Lidar(sensor_msgs::LaserScan::ConstPtr msg); //
void callback_Driver(data::Struct_Driver2Data msg);        //
void callback_Pillar(data::topicPillar msg);               //
void callback_Car(data::point msg);                        //

void formationPillar();                                                 // Формируем перемнную с собщением для публикации
long map(long x, long in_min, long in_max, long out_min, long out_max); // Переводит значение из одного диапазона в другой, взял из Ардуино

void startPosition(geometry_msgs::Pose2D &car_); // Разбираем топик со стартовой позицией робота

float minDistance(float lazer1_, float lazer2_, float uzi1_);                 // Находим минимальную дистанцию из 3 датчиков
data::Struct_Data2Driver speedCorrect(data::Struct_Data2Driver Data2Driver_); // Корректировка скорости движения в зависимости от датчиков растояния перед
// void collectCommand(); // //Функция формирования команды для нижнего уровня на основе всех полученных данных, датчиков и анализа ситуации

// **********************************************************************************

// Функция обраьтного вызова по подписке на топик джойстика nh.subscribe("joy", 16, callback_Joy);
void callback_Joy(sensor_msgs::Joy msg)
{
    flag_msgJoy = true;
    msg_joy = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
}

void callback_Lidar(sensor_msgs::LaserScan::ConstPtr msg)
{
    flag_msgLidar = true;
    msg_lidar = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
}
void callback_Pillar(data::topicPillar msg)
{
    flag_msgPillar = true;
    msg_pillar = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
}
void callback_Car(geometry_msgs::Pose2D msg)
{
    // ROS_WARN("callback_Car");
    flag_msgCar = true;
    msg_car = msg; // Пишем в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
}

void callback_Driver(data::Struct_Driver2Data msg)
{
    msg_Driver2Data = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
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
// Разбираем топик со стартовой позицией робота
void startPosition(geometry_msgs::Pose2D &car_)
{
    ROS_INFO("------------------------- startPosition -------------------------------------");
    poseLidar.mode1.x = car_.x; // Пока считаем что передаем положение центра лидара и поэтому ему присваиваем значение, потом надо будет добавлять смещение до центра поворота между колесами
    poseLidar.mode1.y = car_.y;
    poseLidar.mode1.theta = car_.theta;
    ROS_INFO("startPosition lidarPose x= %.3f y= %.3f theta= %.3f ", poseLidar.mode1.x, poseLidar.mode1.y, poseLidar.mode1.theta);
    ROS_INFO("-------------------------            -------------------------------------");
}

// Переводит значение из одного диапазона в другой, взял из Ардуино
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Корректировка скорости движения в зависимости от датчиков растояния перед
data::Struct_Data2Driver speedCorrect(data::Struct_Data2Driver Data2Driver_)
{
    float min = minDistance(msg_Driver2Data.lazer1.distance, msg_Driver2Data.lazer2.distance, msg_Driver2Data.uzi1.distance); // Находим минимальную дистанцию из 3 датчиков
    if (min < 0.5)                                                                                                            // Если меньше полметра
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

void formationPillar() // Формируем перемнную с собщением для публикации
{
    //ROS_INFO("!!! %i",pillar.countPillar);
    //pillar_out_msg.data[0].azimuth = 0;
    for (int i = 0; i < pillar.countPillar; i++)
    {
        pillar_out_msg.data[i].status = pillar.pillar[i].status;
        pillar_out_msg.data[i].azimuth = pillar.pillar[i].azimuth;
        pillar_out_msg.data[i].hypotenuse = pillar.pillar[i].hypotenuse;
        pillar_out_msg.data[i].distance = pillar.pillar[i].distance;
        pillar_out_msg.data[i].x_true = pillar.pillar[i].x_true;
        pillar_out_msg.data[i].y_true = pillar.pillar[i].y_true;
        pillar_out_msg.data[i].y_lidar = pillar.pillar[i].y_lidar;
        pillar_out_msg.data[i].x_lidar = pillar.pillar[i].x_lidar;
        //ROS_INFO("Status= %i azimuth= %.3f",pillar_out_msg.data[i].status,pillar_out_msg.data[i].azimuth);
    }
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
