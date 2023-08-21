#ifndef JOY_H
#define JOY_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class Joy
{
private:
    // ros::NodeHandle nh_joy;
    // ros::Publisher publisher_Joy2Head;

    /* data */
public:
    sensor_msgs::Joy data;
    Joy(/* args */);
    ~Joy();
    void processing() // Расчеты или обработка на основе данных в перемнной data
    {
        if (data.buttons[2] == 1)
        {
        }

    }

    void init()
    {
        // subscriber_Joy = nh_joy.subscribe("joy", 16, Joy::callback_Joy); // Это мы подписываемся на то что публикует нода джойстика

        // publisher_Joy2Head = nh_joy.advertise<sensor_msgs::Joy>("Joy2Head", 16); // Это мы публикуем структуру которую получили с драйвера
    }
    // void callback_Joy(sensor_msgs::Joy msg)
    // {
    //     joy = msg;
    // }
};
Joy::Joy(/* args */)
{
}

Joy::~Joy()
{
}

#endif
