#ifndef JOY_H
#define JOY_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

// #include <data/Struct_Joy.h>
// #include <data/Struct_Data2Driver.h>

class CJoy
{
private:
    float speed_max = 1;       // Максимальная скорость
    float radius_min = 0.5;    // Минисмальный радиус разворота
    float radius_max = 3;      // Максимальный радиус поворота
    float radius_delta = 0.25; // Шаг увеличения или уменьшения радиуса
    float offset = 0;          // Смещение для управление радиусом
    float getRadius(float & offset); // Возвращает какой должен быть радиус при указанном смещении

public:
    CJoy(float speed_max_, float radius_min_);
    ~CJoy();
    data::Struct_Joy joy2Head;                   // Структура в которую пишем обработанные данные от джойстика и выдаем наружу из класса
    void parsingJoy(const sensor_msgs::Joy &data_);                                                                              // Функция обработки и пробразований данных в мою понятную структуру с кнопками
    data::SControlDriver transform(data::Struct_Joy &joy2Head, data::Struct_Joy &joy2Head_prev, data::SControlDriver &Data2Driver_); // Преобразование кнопок джойстика в реальные команды
};

CJoy::CJoy(float speed_max_, float radius_min_)
{
    speed_max = speed_max_; // Меняем значения с умолчания на те что при инициализации
    radius_min = radius_min_;
}

CJoy::~CJoy()
{
}
// Функция обработки и пробразований данных в мою понятную структуру с кнопками
void CJoy::parsingJoy(const sensor_msgs::Joy &data_) 
{
                                                 //--------------------------- BUTTON -----------------------------
    joy2Head.button_cross = data_.buttons[0];    // 0 или 1, по умолчанию 0
    joy2Head.button_circle = data_.buttons[1];   // 0 или 1, по умолчанию 0
    joy2Head.button_triangle = data_.buttons[2]; // 0 или 1, по умолчанию 0
    joy2Head.button_square = data_.buttons[3];   // 0 или 1, по умолчанию 0

    joy2Head.button_left_right = data_.axes[6]; // !!!!!!!!! 1 или -1 по умолчанию 0
    joy2Head.button_up_down = data_.axes[7];    // !!!!!!!!! 1 или -1 по умолчанию 0

    joy2Head.button_share = data_.buttons[8];  // 0 или 1, по умолчанию 0
    joy2Head.button_option = data_.buttons[9]; // 0 или 1, по умолчанию 0
    joy2Head.button_ps4 = data_.buttons[10];   // 0 или 1, по умолчанию 0

    joy2Head.button_L1 = data_.buttons[4]; // 0 или 1, по умолчанию 0
    joy2Head.button_R1 = data_.buttons[5]; // 0 или 1, по умолчанию 0

    joy2Head.button_L2 = data_.buttons[6]; // 0 или 1, по умолчанию 0
    joy2Head.button_R2 = data_.buttons[7]; // 0 или 1, по умолчанию 0
                                           //---------------------------- AXES-----------------------------
    joy2Head.axes_L_Hor = data_.axes[0]; // От 1 до -1 float, по умолчанию 0
    joy2Head.axes_L_Ver = data_.axes[1]; // От 1 до -1 float, по умолчанию 0

    joy2Head.axes_R_Hor = data_.axes[3]; // От 1 до -1 float, по умолчанию 0
    joy2Head.axes_R_Ver = data_.axes[4]; // От 1 до -1 float, по умолчанию 0

    joy2Head.axes_L2 = data_.axes[2]; // От 1 до -1 float с нулем по середине, по умолчанию 1 !!!!!!!!!
    joy2Head.axes_R2 = data_.axes[5]; // От 1 до -1 float с нулем по середине, по умолчанию 1 !!!!!!!!!!!!!!
}

// Преобразование кнопок джойстика в реальные команды
data::SControlDriver CJoy::transform(data::Struct_Joy &joy2Head_, data::Struct_Joy &joy2Head_prev_, data::SControlDriver &Data2Driver_prev_) 
{
    static data::SControlDriver Data2Driver; // Тут формируем команды делаем static что-бы командв сохранялись

    // printf(" joy2Head_.button_option %f ! ", joy2Head_.button_option);
    // printf(" joy2Head_prev_.button_option %f ", joy2Head_prev_.button_option);
    // printf(" Data2Driver_prev_.led.num_program %i ", Data2Driver_prev_.led.num_program);
/*
    if (joy2Head_.button_ps4 != joy2Head_prev_.button_ps4 && joy2Head_.button_ps4 == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        if (Data2Driver_prev_.control.startStop == 1) // Если предыдущий статус команды 1 то меняем на 0
        {
            Data2Driver.control.startStop = 0;
        }
        else // иначе меняем на 1
        {
            Data2Driver.control.startStop = 1;
        }
    }

    if (joy2Head_.button_L1 != joy2Head_prev_.button_L1 && joy2Head_.button_L1 == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        if (Data2Driver_prev_.control.command1 == 1) // Если предыдущий статус команды 1 то меняем на 0
        {
            Data2Driver.control.command1 = 0;
        }
        else // иначе меняем на 1
        {
            Data2Driver.control.command1 = 1;
        }
    }
    if (joy2Head_.button_R1 != joy2Head_prev_.button_R1 && joy2Head_.button_R1 == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        if (Data2Driver_prev_.control.command2 == 1) // Если предыдущий статус команды 1 то меняем на 0
        {
            Data2Driver.control.command2 = 0;
        }
        else // иначе меняем на 1
        {
            Data2Driver.control.command2 = 1;
        }
    }

    if (joy2Head_.button_option != joy2Head_prev_.button_option && joy2Head_.button_option == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        Data2Driver.led.num_program++; // Увеличиваем номер команды на 1
    }
    if (joy2Head_.button_share != joy2Head_prev_.button_share && joy2Head_.button_share == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        Data2Driver.led.num_program = 0; // Выключаем свет, номер команды делаем 0
    }

    // Скорость --------------------------
    if (joy2Head_.button_cross != joy2Head_prev_.button_cross && joy2Head_.button_cross == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        Data2Driver.control.speed = 0; // Скорость ноль
    }
    if (joy2Head_.button_circle != joy2Head_prev_.button_circle && joy2Head_.button_circle == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        Data2Driver.control.speed += 0.1;           // Скорость увеличиваем на 0.1
        if (Data2Driver.control.speed >= speed_max) // Максимальная скорость ограничена
        {
            Data2Driver.control.speed = speed_max;
        }
    }
    if (joy2Head_.button_square != joy2Head_prev_.button_square && joy2Head_.button_square == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        Data2Driver.control.speed -= 0.1;  // Скорость уменьшаем на 0.1
        if (Data2Driver.control.speed < 0) // МИнимальная скорость ограничена 0
        {
            Data2Driver.control.speed = 0;
        }
    }
    if (joy2Head_.button_triangle != joy2Head_prev_.button_triangle && joy2Head_.button_triangle == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        Data2Driver.control.speed = speed_max; // Скорость устанавливаем в максимум
    }
    // Радиус. ----------------------------------------------------------------------------------
    // Всем управляем через смещение

    if (joy2Head_.button_up_down != joy2Head_prev_.button_up_down && joy2Head_.button_up_down == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        offset = 0;
        Data2Driver.control.radius = getRadius(offset);
    }

    if (joy2Head_.button_up_down != joy2Head_prev_.button_up_down && joy2Head_.button_up_down == -1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        if (offset >= 0) // Если в момент разворота ехали прямо или поворачивали напрво, то разворачиваемся через правую сторону
        {
            offset = radius_max - radius_min;
        }
        else // Разворот через левую сторону
        {
            offset = -(radius_max - radius_min);
        }
        Data2Driver.control.radius = getRadius(offset);
    }

    if (joy2Head_.button_left_right != joy2Head_prev_.button_left_right && joy2Head_.button_left_right == -1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        offset = offset + radius_delta;
        Data2Driver.control.radius = getRadius(offset);
    }
    if (joy2Head_.button_left_right != joy2Head_prev_.button_left_right && joy2Head_.button_left_right == 1) // Если изменилась кнопка и стала единицей сейчас тогда делаем команду
    {
        offset = offset - radius_delta;
        Data2Driver.control.radius = getRadius(offset);
    }

    // printf(" control.startStop %i ", Data2Driver.control.startStop);
    // printf(" led.num_program %i \n", Data2Driver.led.num_program);

    joy2Head_prev_ = joy2Head_;      // Запоминаем состояние для следующего раза
    Data2Driver_prev_ = Data2Driver; // Запоминаем статус команд для следующего раза
*/
    return Data2Driver;
}
//Функция возвращает радиус по заданному смещению
float CJoy::getRadius(float &offset_)
{
    float radius = 0;
    // Ограничиваем размер смещения
    if (abs(offset_) > radius_max - radius_min) // Если смещение по модулю выходит за границы допустимого, то ограничиваем
    {
        if (offset_ > 0)
        {
            offset_ = radius_max - radius_min;
        }
        if (offset_ < 0)
        {
            offset_ = -(radius_max - radius_min);
        }
    }
    // В зависимости от смещения считаем радиус
    if (offset_ == 0)
    {
        radius = 0;
    }

    if (offset_ > 0) // Если не равен нулю, то есть не ехали прямо
    {
        radius = radius_max - offset_;
    }
    if (offset_ < 0) // Если не равен нулю, то есть не ехали прямо
    {
        radius = -radius_max - offset_;
    }
    return radius;
}

#endif

// ВАРИАНТ КАК НАДО ОФОРМЛЯТЬ ВЫЗОВ ФУНКЦИИ ВНУТРИ КЛАССА!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ros::NodeHandle nh_joy;
// ros::Publisher publisher_Joy2Head;
// ros::Subscriber subscriber_Joy = nh_joy.subscribe("joy", 16, &Joy::callback_Joy,this); // Это мы подписываемся на то что публикует нода джойстика

// publisher_Joy2Head = nh_joy.advertise<sensor_msgs::Joy>("Joy2Head", 16); // Это мы публикуем структуру которую получили с драйвера

// void callback_Joy(sensor_msgs::Joy msg)
// {
//     joy = msg;
// }
// // Обратный вызов при опросе топика Head2Data
// void callback_Joy(const sensor_msgs::Joy &msg)
// {
//     // msg_ControlDriver = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
//     //  ROS_INFO("message_callback_Command.");
// }