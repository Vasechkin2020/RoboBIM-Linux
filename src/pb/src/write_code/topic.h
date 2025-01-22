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
    void publicationWrite2Data(); // Публикация данных для управления Driver

private:
    ros::NodeHandle _nh;
    ros::Time ros_time; // Время ROS

    ros::Publisher pub_ControlDriver = _nh.advertise<pb_msgs::Struct_Data2Print>("pbWrite/Write2Print", 16); // Это мы публикуем структуру которую отправляем к исполнению на драйвер
};

CTopic::CTopic(/* args */)
{
}

CTopic::~CTopic()
{
}

// Публикация данных для управления Driver
void CTopic::publicationWrite2Data()
{
    pb_msgs::Struct_Data2Print data;
    data.id++;
    data.controlPrint.mode = Write2Data.controlPrint.mode;
    data.controlPrint.status = Write2Data.controlPrint.status;
    data.controlPrint.intensity = Write2Data.controlPrint.intensity;
    data.controlPrint.speed = Write2Data.controlPrint.speed;
    data.controlPrint.position = Write2Data.controlPrint.position;
    data.controlPrint.velocity =  Write2Data.controlPrint.velocity;
    data.controlPrint.torque =  Write2Data.controlPrint.torque;
    pub_ControlDriver.publish(data);
}

#endif