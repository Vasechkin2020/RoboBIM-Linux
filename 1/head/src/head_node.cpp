
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <my_msgs/Body.h>
#include <my_msgs/Control.h>
#include <my_msgs/Command.h>



#include "config.h"
#include "code.h"

int main(int argc, char **argv)
{
    ROS_INFO("%s ----------------------------------------------------",NN);
    ROS_INFO("%s START Head Module HighLevel ROS Raspberry Pi 4B !!! ",NN);
    ROS_INFO("%s ------------------ROS_INFO----------------------------------",NN);
    ROS_WARN("%s ------------------ROS_WARN----------------------------------",NN);
    ROS_ERROR("%s -----------------ROS_ERROR-----------------------------------",NN);

    ros::init(argc, argv, "head_node");
    ros::NodeHandle nh;
    ros::Subscriber body_sub = nh.subscribe("body_topic", 16, message_callback_Body);          //Это мы подписываемся
    ros::Subscriber control_sub = nh.subscribe("control_topic", 16, message_callback_Control); // Это мы подписываемся

    ros::Publisher com_pub = nh.advertise<my_msgs::Command>("command_topic", 16); //Это мы публикуем структуру
    ros::Rate r(RATE);                                                            // Частота в Герцах - задержка
    Command_msg.id = 0;                                                           //Начальное значение номера команд

    while (ros::ok())
    {
        ros::spinOnce();              // Опрашиваем ядро ROS и по этой команде срабатывают обратные вызовы где мы получаем данные на которы подписаны
        collectCommand();             // Формируем команду на основе полученных данных
        com_pub.publish(Command_msg); //Публикация данных команды
        ros::spinOnce();              // Опрашиваем ядро ROS и по этой команде наши данные публикуются
        r.sleep();                    // Интеллектуальная задержка на указанную частоту
    }

    return 0;
}