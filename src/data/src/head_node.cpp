
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

#include <data/Struct_Joy.h>

// #include <my_msgs/Body.h>
// #include <my_msgs/Control.h>
// #include <my_msgs/Command.h>

#include "head_code/c_joy.h"
Joy joy; // Обьявляем экземпляр класса

#include "head_code/config.h"
#include "head_code/code.h"

data::Struct_Joy msg_Joy2Head;

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_INFO("%s START Head Module HighLevel ROS Raspberry Pi 4B !!! ver 1.00 ", NN);
    ROS_INFO("%s ------------------ROS_INFO-----------------------------------", NN);
    ROS_WARN("%s ------------------ROS_WARN-----------------------------------", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------", NN);

    ros::init(argc, argv, "head_node");
    ros::NodeHandle nh;
    ros::Subscriber subscriber_Joy = nh.subscribe("joy", 16, callback_Joy); // Это мы подписываемся на то что публикует нода джойстика
    ros::Publisher publisher_Joy2Head = nh.advertise<sensor_msgs::Joy>("Joy2Head", 16); // Это мы публикуем структуру которую получили с драйвера

    // ros::Subscriber body_sub = nh.subscribe("body_topic", 16, message_callback_Body);          //Это мы подписываемся
    // ros::Subscriber control_sub = nh.subscribe("control_topic", 16, message_callback_Control); // Это мы подписываемся

    // ros::Publisher com_pub = nh.advertise<my_msgs::Command>("command_topic", 16); //Это мы публикуем структуру
    ros::Rate r(RATE); // Частота в Герцах - задержка
    // Command_msg.id = 0;                                                           //Начальное значение номера команд

    while (ros::ok())
    {
        ros::spinOnce();  // Опрашиваем ядро ROS и по этой команде срабатывают обратные вызовы где мы получаем данные на которы подписаны
        joy.processing(); // Расчеты или обработка на основе данных в перемнной data, которые там обновились посде callback
        publisher_Joy2Head.publish(msg_Joy2Head); //Публикация полученных данных


        // collectCommand();             // Формируем команду на основе полученных данных
        // com_pub.publish(Command_msg); //Публикация данных команды
        // ros::spinOnce();              // Опрашиваем ядро ROS и по этой команде наши данные публикуются
        r.sleep(); // Интеллектуальная задержка на указанную частоту
    }

    return 0;
}