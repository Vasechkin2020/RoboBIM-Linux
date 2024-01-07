
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include "sensor_msgs/LaserScan.h"



#include <data/Struct_Joy.h>
#include <data/Struct_Data2Driver.h>
#include <data/Struct_Driver2Data.h>

// #include <my_msgs/Body.h>
// #include <my_msgs/Control.h>
// #include <my_msgs/Command.h>

#include "head_code/config.h"
pos_struct position;            // Обьявляем переменную для позиции машинки

#include "head_code/c_joy.h"
Joy joy(MAX_SPEED, 0.5); // Обьявляем экземпляр класса в нем вся обработка джойстика
#include "head_code/position.h"

#include "head_code/pillar.h"
Pillar pillar;           // Обьявляем экземпляр класса в нем вся обработка и обсчет столбов


data::Struct_Joy joy2Head;                 // Структура в которую пишем обработанные данные от джойстика
data::Struct_Joy joy2Head_prev;            // Структура в которую пишем обработанные данные от джойстика предыдущее состоние
data::Struct_Data2Driver Data2Driver;      // Структура с командами которую публикуем и которую потом Driver исполняет
data::Struct_Data2Driver Data2Driver_prev; // Структура с командами которую публикуем и которую потом Driver исполняет предыдущее состоние
data::Struct_Driver2Data Driver2Data;      // Сообщение которое считываем из топика

#include "head_code/code.h"

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_INFO("%s START Head Module HighLevel ROS Raspberry Pi 4B !!! ver 1.06 ", NN);
    ROS_INFO("%s ------------------ROS_INFO-----------------------------------", NN);
    ROS_WARN("%s ------------------ROS_WARN-----------------------------------", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------", NN);

    ros::init(argc, argv, "head_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    setParam(nh_private); // Разбор и установка параметров которые задали в launch файле при запуске

    ros::Subscriber subscriber_Joy = nh.subscribe("joy", 16, callback_Joy); // Это мы подписываемся на то что публикует нода джойстика
    ros::Subscriber subscriber_Pillar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, callback_Pillar);
    ros::Subscriber subscriber_Driver = nh.subscribe<data::Struct_Driver2Data>("Driver2Data", 1000, callback_Driver);

    ros::Publisher publisher_Joy2Head = nh.advertise<data::Struct_Joy>("Joy2Head", 16);             // Это мы публикуем структуру которую сформировали по данным с джойстика
    ros::Publisher publisher_Head2Driver = nh.advertise<data::Struct_Data2Driver>("Head2Data", 16); // Это мы публикуем структуру которую отправляем к исполнению на драйвер

    // ros::Subscriber body_sub = nh.subscribe("body_topic", 16, message_callback_Body);          //Это мы подписываемся
    // ros::Subscriber control_sub = nh.subscribe("control_topic", 16, message_callback_Control); // Это мы подписываемся
    // ros::Publisher com_pub = nh.advertise<my_msgs::Command>("command_topic", 16); //Это мы публикуем структуру

    ros::Rate r(RATE); // Частота в Герцах - задержка
    // Command_msg.id = 0;                                                           //Начальное значение номера команд

    while (ros::ok())
    {
        ros::spinOnce();                      // Опрашиваем ядро ROS и по этой команде срабатывают обратные вызовы где мы получаем данные на которы подписаны
        publisher_Joy2Head.publish(joy2Head); // Публикация полученных данных
        // ROS_INFO("%s 1 joy2Head.axes_L2 = %f, joy2Head.axes_R2 = %f", NN, joy2Head.axes_L2, joy2Head.axes_R2);

        Data2Driver = joy.transform(joy2Head, joy2Head_prev, Data2Driver_prev); // Преобразование кнопок джойстика в реальные команды
        Data2Driver = speedCorrect(Data2Driver);                                // Корректировка скорости движения в зависимости от растояния до преграды
        publisher_Head2Driver.publish(Data2Driver);                             // Публикация сформированных данных структуры управления для исполнения драйвером

        // pillar.pillar[2].azimuth
        // collectCommand();             // Формируем команду на основе полученных данных
        // com_pub.publish(Command_msg); //Публикация данных команды

        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши данные публикуются
        r.sleep();       // Интеллектуальная задержка на указанную частоту
    }

    return 0;
}