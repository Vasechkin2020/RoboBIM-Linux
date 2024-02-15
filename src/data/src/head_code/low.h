#ifndef DATA_H
#define DATA_H

class CLow
{
private:
    // Структура сосдержит всю информацию по мотору на основании данных энкодера
    struct SEncoder
    {
        float rpsSet = 0;   // Текущая скорость вращения ( обороты в секунду)
        float rpsEncod = 0; // Текущая скорость вращения ( обороты в секунду)
    };

    struct SMpu // Структура с данными с mpu bno055
    {
        SPose pose;
        STwist twist;
        SEuler angleEuler;
    };

    // Структура состояния датчика расстония
    struct SSensor
    {
        int32_t status; // статус состояния
        float distance; // расстояние до препятствия
    };

    // Структура принимаемых данных от контроллера Driver в Data
    struct SDriver2Data
    {
        uint32_t id = 0; // id команды
        SEncoder motorLeft;
        SEncoder motorRight;
        double dtEncoder; // Время за которое данные с энкодера
        SMpu bno055;      // Данные с датчика BNO055
        SSensor laserL;
        SSensor laserR;
        SSensor uzi;
    };

public:
    CLow(/* args */);
    ~CLow();
    void parsingDriver(data::SDriver2Data data_); // Метод разбирает пришедшее сообщение из топика и копирует данные в переменную для дальнейшего использования
    void calculateOdometryFromEncoder();          // Расчет линейных и угловой скорости из энкодера
    void calculateOdometryFromMpu();              // Расчет линейных и угловой скорости из MPU BNO055
    SDriver2Data _driver;                         // Тут все переменные что получили от драйвера
    SOdom _encoderOdom;                           // Одометрия по Энкодеру
    SOdom _mpuOdom;                               // Одометрия по MPU BNO055
};

// Формирование одометрии по данным MPU BNO055
void CLow::calculateOdometryFromMpu()
{
    _mpuOdom.pose = _driver.bno055.pose;
    _mpuOdom.twist = _driver.bno055.twist;
}
// Расчет линейных и угловой скорости из энкодера
// ОЧЕНЬ ХОРОШИЕ ЛЕКЦИИ И МНОГИ ФОРМУЛЫ ОТТУДА https://www.youtube.com/watch?v=xJYhNCifPj8&list=PL2PmRem6srUk-Jflnt3-RuzuICb7TN0FR&index=3
void CLow::calculateOdometryFromEncoder()
{
    float radius = 0;
    float theta = 0;
    float lenArc = 0;
    static SPose pose;
    static STwist twist;

    float speedL = PERIMETR * _driver.motorLeft.rpsEncod;  // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
    float speedR = PERIMETR * _driver.motorRight.rpsEncod; // По формуле периметр колеса на обороты это и есть пройденный путь за секунду Это и есть скорость за секунду
    float sumSpeed = speedL + speedR;
    float deltaSpeed = speedL - speedR;
    float speed = (speedR + speedL) / 2.0;  // Находим скорость всего обьекта.
    float w = deltaSpeed / DISTANCE_WHEELS; // Находим уголовую скорость движения по радиусу. Плюс по часовой минус против часовой
    printf("speedL= %.4f speedR= %.4f speed= %.4f w = %.4f ///  ", speedL, speedR, speed, RAD2DEG(w));

    if (abs(sumSpeed) < 0.01) // Если сумма скоростей очень маленькая или ноль значит крутимся на месте или стоим на месте и тогда совсем иной расчет чем если движемся
    {
        if (speedL == 0 && speedR == 0) // Стоим на месте. Скорости равны нулю
        {
            radius = 0;
            theta = 0;
            printf("0 STOIM NA MESTE radius = %.4f theta gradus = %.4f \n", radius, RAD2DEG(theta));
        }
        else // Крутимся на месте
        {
            radius = 0.5 * DISTANCE_WHEELS; // Радиус в таком случае это половина между колесами
            if (speedL > speedR)            // Значит крутимся по часовой и знак угловой скорсти плюс
            {
                speed = speedL; // меняем скорость со среднй на скорость одного колеса, наружнего
            }
            else
            {
                speed = speedR; //  Значит крутимся против часовой и знак минус будет у угловой скорости
            }
            lenArc = speed; // Находим путь какой проехали. Это длинна дуги.
            theta = lenArc / radius;            // Отношение улинны дуги окружночти к радиусу дает угол в радианах. Нахождение центрального угла по дуге и радиусу.
            printf("1 KRUTIMSA NA MESTE radius = %.4f theta gradus = %.4f \n", radius, RAD2DEG(theta));
        }
    }
    else // Тут нормальный расчет что мы движемся или по прямой или по радиусу
    {
        lenArc = speed; // Находим путь какой проехали за время в течении которого энкодер собирал данные. Это длинна дуги.
        if (abs(deltaSpeed) < 0.01)         // Если раздница скоростей незначительна то считаем что едем прямо вперед или назад
        {
            radius = 0; // Едем прямо или назад и все по нулям
            theta = 0;  // Если едем прямо то угол поворота отклонения от оси равен 0
            printf("2 EDEM PRIAMO radius = %.4f theta gradus = %.4f \n", radius, RAD2DEG(theta));
        }
        else // Едем по радиусу и надо все считать
        {
            radius = (0.5 * DISTANCE_WHEELS) * (sumSpeed / deltaSpeed); // Находим радиус движения
            theta = lenArc / radius;                                    // Отношение улинны дуги окружночти к радиусу дает угол в радианах. Нахождение центрального угла по дуге и радиусу.
            printf("3 EDEM RADIUS radius = %.4f theta gradus = %.4f \n", radius, RAD2DEG(theta));
        }
    }
    // Находим линейные скорости из моего вектора скорости. Я задаю общую скорость движения (длинна вектора), ее надо разложить на проекции по осям x y. Это будут линейные скорости
    twist.vx = speed * sin(theta); // Проекция моей скорости на ось Y получаем линейную скорость по оси
    twist.vy = speed * cos(theta); // Проекция моей скорости на ось X получаем линейную скорость по оси
    twist.vth = theta;             // Угловая скорость

    // Находим смещние по осям за это время с этой линейной и угловой скоростями
    double delta_x = (twist.vx * cos(pose.th) - twist.vy * sin(pose.th)) * _driver.dtEncoder;
    double delta_y = (twist.vx * sin(pose.th) + twist.vy * cos(pose.th)) * _driver.dtEncoder;
    double delta_th = theta * _driver.dtEncoder; // тут вроде в радианах выходит

    // Меняем координаты и угол на основе вычислений
    pose.x += delta_x;   // Вычисляем координаты
    pose.y += delta_y;   // Вычисляем координаты
    pose.th += delta_th; // Прибавляем к текущему углу и получаем новый угол куда смотрит наш робот
    (pose.th > 360) ? (pose.th -= 360) : (pose.th = pose.th);
    (pose.th < 0) ? (pose.th += 360) : (pose.th = pose.th);

    _encoderOdom.pose = pose;
    _encoderOdom.twist = twist;

    //**************************************************************** СТАРЫЙ ПРИМЕР РАСЧЕТА С НИЖНЕГО УРОВНЯ ******************************************
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
}

CLow::CLow(/* args */)
{
}

CLow::~CLow()
{
}
// Метод разбирает пришедшее сообщение из топика и копирует данные в переменную для дальнейшего использования
void CLow::parsingDriver(data::SDriver2Data data_)
{
    _driver.id = data_.id;
    _driver.motorLeft.rpsSet = data_.motorLeft.rpsSet;
    _driver.motorLeft.rpsEncod = data_.motorLeft.rpsEncod;
    _driver.motorRight.rpsSet = data_.motorRight.rpsSet;
    _driver.motorRight.rpsEncod = data_.motorRight.rpsEncod;
    _driver.dtEncoder = data_.dtEncoder;
    _driver.bno055.angleEuler.roll = data_.bno055.angleEuler.roll;
    _driver.bno055.angleEuler.pitch = data_.bno055.angleEuler.pitch;
    _driver.bno055.angleEuler.yaw = data_.bno055.angleEuler.yaw;
    _driver.bno055.pose.x = data_.bno055.pose.x;
    _driver.bno055.pose.y = data_.bno055.pose.y;
    _driver.bno055.pose.th = data_.bno055.pose.th;
    _driver.bno055.twist.vx = data_.bno055.twist.vx;
    _driver.bno055.twist.vy = data_.bno055.twist.vy;
    _driver.bno055.twist.vth = data_.bno055.twist.vth;
    _driver.laserL.status = data_.laserL.status;
    _driver.laserL.distance = data_.laserL.distance;
    _driver.laserR.status = data_.laserR.status;
    _driver.laserR.distance = data_.laserR.distance;
    _driver.uzi.status = data_.uzi.status;
    _driver.uzi.distance = data_.uzi.distance;
}

// float checkMinRadius(float radius_) // Проверка радиуса на реалистичность в соответствии с реальными возможностями робота
// {
//     // printf("checkMinRadius radius_ IN= %f", radius_);
//     //  Проверка радиуса на возможность робота. Радиус не может быть меньше расстояния между колесами
//     float min_r = DISTANCE_WHEELS;
//     if (radius_ > 0 && radius_ < min_r) // радиус не может быть меньше минимума за этим нужно следить на верхнем уровне
//     {
//         radius_ = min_r; // Присваиваем минимально возможный
//     }
//     if (radius_ < 0 && radius_ > -min_r) // радиус не может быть меньше минимума за этим нужно следить на верхнем уровне
//     {
//         radius_ = -min_r; // Присваиваем минимально возможный
//     }
//     // printf(" real= %f \n", radius_);
//     return radius_;
// }

// // Проверка на максимальную скорость учитывая радиус который задан и максимальную скорость моторов с какой они могут вращаться. Проверка по внешнему колесу, как сомому быстрому
// float checkMaxSpeedRadius(float radius_, float speed_)
// {
//     // printf("getRealSpeed speed_ IN= %f", speed_);
//     if (radius_ != 0) // Вычисляем если радиус не ноль (движение прямо)
//     {
//         float radius_centre = abs(radius_) - (DISTANCE_WHEELS / 2.0); // Вычисляем радиус по центру машинки, так как просто радиус это внешний радиус по внешнему колесу.
//         float speed_v = (abs(radius_) * speed_) / radius_centre;      // Вычисляем скорость на внешнем колесе учитывая что задана скорость по центру и учитывая радиус
//         if (speed_v > MAX_SPEED)                                      // Понижаем скорость если скорость на внешнем колесе превысила максимум
//             speed_v = MAX_SPEED;
//         if (speed_v < -MAX_SPEED)
//             speed_v = -MAX_SPEED;
//         speed_ = (speed_v * radius_centre) / abs(radius_); // Обратное вычисление скорости по центру
//     }
//     // printf(" real= %f \n", speed_);
//     return speed_;
// }
// Установка скорости моторов в зависимости от радиуса и направления
// void setSpeedRadius(float speed_, float radius_)
// {
//     car.radiusSet = checkMinRadius(radius_); // Проверка на реалистичность Запоминаяем радиус для расчета одометрии теперь у нас такой радиус отнего все и считаем Это центр робота
//     // Радиус делаем главным. Далее считаем скорость по колесам и если скорость превышает возможную то подгоняем ее по максимуму и едем с возможноц скоростью, но по заданному радиусу
//     car.speedSet = checkMaxSpeedRadius(car.radiusSet, speed_); // Проверка на MAXSPEED Запоминаяем скорость для расчета одометрии теперь у нас такая скорость от нее все и считаем

//     float k2 = 0;
//     if (car.radiusSet != 0) // Вычисляем коефициент перобразования иначе он остается равен 0
//     {
//         // k2 = (DISTANCE_WHEELS / 2.0) / abs(g_radius); // Находим долю какую составляем растояние половинки между колесами тела и радиусом поворота.
//         k2 = (DISTANCE_WHEELS / 2) / (abs(car.radiusSet) - (DISTANCE_WHEELS / 2)); // Находим долю какую составляем растояние половинки между колесами тела и радиусом поворота.
//         // printf("k2= %f \n", k2);
//     }
//     float internal_speed = car.speedSet * (1 - k2);
//     float external_speed = car.speedSet * (1 + k2);
//     // printf(" internal_speed= %f  center_speed= %f external_speed= %f \n", internal_speed, speed_, external_speed);

//     // Определяем направление движения
//     if (car.radiusSet == 0) // Едем по прямой
//     {
//         setSpeed_L(car.speedSet);
//         setSpeed_R(car.speedSet);
//     }
//     if (car.radiusSet < 0) //  едем против часовой стрелки, поворачиваем налево
//     {
//         setSpeed_L(internal_speed); // Левое едет медленней
//         setSpeed_R(external_speed); // Правое едет быстрее
//     }
//     if (car.radiusSet > 0) // По часовой стрелке направо
//     {
//         setSpeed_L(external_speed); // Левое едет быстрее
//         setSpeed_R(internal_speed); // Правое едет медленно
//     }
// }

// Расчет коеффициента соотношения ускорений для ускорения в повороте Пример расчета для 1,2 ускорения и скорорстей 9 и 1
// void getKoefAccel()
// {
//     float kL = 0;
//     float kR = 0;
//     acceler_L = 1;                                   //
//     acceler_R = 1;                                   //
//     float d_gived_L = abs(rps_L_gived - rps_L_fact); // Находим разницу скоростей между тем что есть и тем что надо
//     float d_gived_R = abs(rps_R_gived - rps_R_fact);
//     if (d_gived_L != 0 || d_gived_R != 0) // Если хоть одна разница скоростей не равна нулю
//     {
//         float k = (d_gived_L + d_gived_R) / 2; // находим среднюю скорость  9+1 =10/2=5
//         kL = d_gived_L / k;                    // Ускорение на левом моторе 9/5=1.8
//         kR = d_gived_R / k;                    // Ускорение на левом моторе 1/5=0.2

//         if (kL >= kR)
//         {
//             acceler_R = kR / kL; //
//         }
//         else
//         {
//             acceler_L = kL / kR; //
//         }
//         // if (acceler_R < 0 || acceler_R > 1)
//         // {
//         //     Serial.print("============= > ");
//         //     Serial.print(acceler_R);
//         // }
//         // if (acceler_L < 0 || acceler_L > 1)
//         // {
//         //     Serial.print("============= > ");
//         //     Serial.print(acceler_L);
//         // }
//     }
//     // Serial.print(" rps_L_fact= ");
//     // Serial.print(rps_L_fact);
//     // Serial.print(" d_gived_L= ");
//     // Serial.print(d_gived_L);

//     // Serial.print(" !  rps_R_fact= ");
//     // Serial.print(rps_R_fact);
//     // Serial.print(" d_gived_R= ");
//     // Serial.print(d_gived_R);

//     // Serial.print(" !  kL= ");
//     // Serial.print(kL);

//     // Serial.print(" kR= ");
//     // Serial.print(kR);

//     // Serial.print(" !  acceler_L= ");
//     // Serial.print(acceler_L);
//     // Serial.print(" acceler_R= ");
//     // Serial.println(acceler_R);
// }
// Функция ускорения на правом моторе. Смотрит с какими оборотами надо и какие уже установлены и прибавляет или убаваляет обороты
// void setAcceleration_R()
// {
//     if (flag_motor_R_EN) // Если мотор работает то и эта функция отрабатывает
//     {
//         float rps_new = rps_R_fact;                       // По умолчанию оставляем какие есть.
//         float accel = (ACCELERATION * acceler_R) / 100.0; // Ускорение

//         if (rps_R_fact < rps_R_gived) // Если меньше чем надо то прибавим оборотов
//         {
//             rps_new = rps_R_fact + (accel);
//         }
//         if (rps_R_fact > rps_R_gived) // Если больше чем надо то убавим оборотов
//         {
//             rps_new = rps_R_fact - (accel);
//             if (rps_new < 0.001) // Если совсем уменьшили меньше минимума
//             {
//                 rps_new = 0;
//             }
//         }
//         setTimeing_Step_R(rps_new);

//         Serial.print(" rps_new R= ");
//         Serial.print(rps_new);
//     }
// }

// Функция ускорения на правом моторе. Смотрит с какими оборотами надо и какие уже установлены и прибавляет или убаваляет обороты
// void setAcceleration_L()
// {
//     if (flag_motor_L_EN) // Если мотор работает то и эта функция отрабатывет
//     {
//         float rps_new = rps_L_fact;                       // По умолчанию оставляем какие есть.
//         float accel = (ACCELERATION * acceler_L) / 100.0; // Ускорение
//         if (rps_L_fact < rps_L_gived)                     // Если меньше чем надо то прибавим оборотов
//         {
//             rps_new = rps_L_fact + (accel);
//         }
//         if (rps_L_fact > rps_L_gived) // Если больше чем надо то убавим оборотов
//         {
//             rps_new = rps_L_fact - (accel);
//             if (rps_new <= 0.01) // Если совсем уменьшили меньше минимума
//             {
//                 rps_new = 0;
//             }
//         }

//         Serial.print(" rps_L_gived= ");
//         Serial.print(rps_L_gived);
//         Serial.print(" rps_L_fact= ");
//         Serial.print(rps_L_fact);
//         Serial.print(" accel= ");
//         Serial.print(accel);
//         Serial.print(" rps_new L= ");
//         Serial.println(rps_new);

//         setTimeing_Step_L(rps_new);
//     }
// }

// void setOdomToTf(ros::NodeHandle nh_, tf::TransformBroadcaster odom_broadcaster_, ros::Time current_time_); // Функция записи в нужные места данных одометрии в tf и в odom
// void transOdom();											  // Перенес то что было на главном файле

// #include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>
// nav_msgs::Odometry odom;
// ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
// tf::TransformBroadcaster odom_broadcaster;
// setOdomToTf(nh, odom_broadcaster, current_time); // Функция которая одометрию пищет куда нужно, передаем этой функции все переменные котороые создали в гласной функции main
// transOdom();

// // Функция записи в нужные места данных одометрии в tf и в odom
// void setOdomToTf(ros::NodeHandle nh_, tf::TransformBroadcaster odom_broadcaster_, ros::Time current_time_)
// {
// 	// Пример кода взят отсюда
// 	//  http://library.isr.ist.utl.pt/docs/roswiki/navigation(2f)Tutorials(2f)RobotSetup(2f)Odom.html

// 	// current_time_ = ros::Time::now(); // Получаем текущее время в ROS

// 	// //since all odometry is 6DOF we'll need a quaternion created from yaw
// 	// // получаем из моего направления куда смотрит робот кватернион
// 	// geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Body.odom_enc.th);
// 	// //first, we'll publish the transform over tf
// 	// geometry_msgs::TransformStamped odom_trans;
// 	// odom_trans.header.stamp = current_time_;
// 	// odom_trans.header.frame_id = "odom";
// 	// odom_trans.child_frame_id = "base_link";

// 	// odom_trans.transform.translation.x = Body.odom_enc.x;
// 	// odom_trans.transform.translation.y = Body.odom_enc.x;
// 	// odom_trans.transform.translation.z = 0.0;
// 	// odom_trans.transform.rotation = odom_quat;

// 	// //send the transform
// 	// odom_broadcaster_.sendTransform(odom_trans);

// 	// //next, we'll publish the odometry message over ROS

// 	// odom.header.stamp = current_time_;
// 	// odom.header.frame_id = "odom";

// 	// //set the position
// 	// odom.pose.pose.position.x = Body.odom_enc.x;
// 	// odom.pose.pose.position.y = Body.odom_enc.y;
// 	// odom.pose.pose.position.z = 0.0;
// 	// odom.pose.pose.orientation = odom_quat;

// 	// //set the velocity
// 	// odom.child_frame_id = "base_link";
// 	// odom.twist.twist.linear.x = Body.odom_enc.vel_x;
// 	// odom.twist.twist.linear.y = Body.odom_enc.vel_y;
// 	// odom.twist.twist.angular.z = Body.odom_enc.vel_th;
// }

// void transOdom() // Перенес то что было на главном файле
// {
// 	//     current_time = ros::Time::now(); // Получаем текущее время в ROS

// 	//     //since all odometry is 6DOF we'll need a quaternion created from yaw
// 	//     // получаем из моего направления куда смотрит робот кватернион
// 	//     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Body.odom_enc.th);
// 	//     //first, we'll publish the transform over tf
// 	//     geometry_msgs::TransformStamped odom_trans;
// 	//     odom_trans.header.stamp = current_time;
// 	//     odom_trans.header.frame_id = "odom";
// 	//     odom_trans.child_frame_id = "base_link";

// 	//     odom_trans.transform.translation.x = Body.odom_enc.x;
// 	//     odom_trans.transform.translation.y = Body.odom_enc.y;
// 	//     odom_trans.transform.translation.z = 0.0;
// 	//     odom_trans.transform.rotation = odom_quat;

// 	//     //send the transform
// 	//     odom_broadcaster.sendTransform(odom_trans);

// 	//     //next, we'll publish the odometry message over ROS

// 	//     odom.header.stamp = current_time;
// 	//     odom.header.frame_id = "odom";

// 	//     //set the position
// 	//     odom.pose.pose.position.x = Body.odom_enc.x;
// 	//     odom.pose.pose.position.y = Body.odom_enc.y;
// 	//     odom.pose.pose.position.z = 0.0;
// 	//     odom.pose.pose.orientation = odom_quat;

// 	//     //set the velocity
// 	//     odom.child_frame_id = "base_link";
// 	//     odom.twist.twist.linear.x = Body.odom_enc.vel_x;
// 	//     odom.twist.twist.linear.y = Body.odom_enc.vel_y;
// 	//     odom.twist.twist.angular.z = Body.odom_enc.vel_th;

// 	//     odom_pub.publish(odom); //publish the message
// }
#endif