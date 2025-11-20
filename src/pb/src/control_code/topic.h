#ifndef TOPIC_H
#define TOPIC_H

/*
 Класс для функций для формирования топиков в нужном виде и формате и всех публикаций
*/
#include "config.h"

class CTopic
{
public:
    CTopic(/* args */);
    ~CTopic();
    //**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
    void publicationControlDriver(); // Публикация данных для управления Driver
    void publicationControlPrint();// Публикация данных для управления Print

private:
    ros::NodeHandle _nh;
    ros::Time ros_time; // Время ROS

    ros::Publisher pub_ControlDriver = _nh.advertise<pb_msgs::Struct_Data2Driver>("pb/Control/ControlDriver", 16); // Это мы публикуем структуру которую отправляем к исполнению на драйвер
    ros::Publisher pub_ControlPrint = _nh.advertise<pb_msgs::Struct_Data2Print>("pb/Control/ControlPrint", 1); // Это мы публикуем структуру которую отправляем к исполнению на драйвер
};

CTopic::CTopic(/* args */)
{
}

CTopic::~CTopic()
{
}

// Публикация данных для управления Driver
void CTopic::publicationControlDriver()
{
    pb_msgs::Struct_Data2Driver data;
    static unsigned long led_time = 0;
    static int color = 0;

    if ((millis() - led_time) > 250)
    {
        color = 1 - color;
        led_time = millis();
    }
    for (int i = 0; i < 24; i++)
    {
        data.led.led[i] = color;
    }
    
    data.control = controlSpeed.control;
    pub_ControlDriver.publish(data);
}

// Публикация данных для управления Print
void CTopic::publicationControlPrint()
{
    pb_msgs::Struct_Data2Print data;
    data.id = controlPrint.id;
    data.controlPrint.mode = controlPrint.controlPrint.mode;
    data.controlPrint.status = controlPrint.controlPrint.status;
    data.controlPrint.intensity = 0;
    data.controlPrint.speed = 0;
    data.controlPrint.position = 0;
    data.controlPrint.velocity =  0;
    data.controlPrint.torque =  controlPrint.controlPrint.torque;
    pub_ControlPrint.publish(data);
    // printf("    publicationControlPrint id= %i \n",data.id);
}

#endif