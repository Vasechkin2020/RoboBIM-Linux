
#include <ros/ros.h>
// #include <std_msgs/String.h>
//#include <geometry_msgs/Point32.h>
//geometry_msgs::Point32 pillar[4];      //

#include <data/topicPillar.h>
data::topicPillar topicPillar; // Структура в которую пишем обработанны данные о местонахождении столбов

#include "pillar_code/config.h"
#include "pillar_code/code.h"

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_INFO("%s START Pillar Node  ROS Raspberry Pi 4B !!! ver 1.0 ", NN);
    ROS_INFO("%s ------------------ROS_INFO-----------------------------------", NN);
    ROS_WARN("%s ------------------ROS_WARN-----------------------------------", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------", NN);

    ros::init(argc, argv, "pillar_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Rate r(RATE); // Частота в Герцах - задержка

    ros::Publisher pub_topicPillar = nh.advertise<data::topicPillar>("topicPillar", 16); // Это мы обьявляем структуру для публикации которую сформировали по данным с джойстика

    //setParam(nh_private); // Разбор и установка параметров которые задали в launch файле при запуске
    
    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
    if (!nh_private.getParam("x0", topicPillar.pillar[0].x))
        topicPillar.pillar[0].x = 0.11;
    if (!nh_private.getParam("y0", topicPillar.pillar[0].y))
        topicPillar.pillar[0].y = 0.11;

    if (!nh_private.getParam("x1", topicPillar.pillar[1].x))
        topicPillar.pillar[1].x = 1.11;
    if (!nh_private.getParam("y1", topicPillar.pillar[1].y))
        topicPillar.pillar[1].y = 1.11;

    if (!nh_private.getParam("x2", topicPillar.pillar[2].x))
        topicPillar.pillar[2].x = 2.11;
    if (!nh_private.getParam("y2", topicPillar.pillar[2].y))
        topicPillar.pillar[2].y = 2.11;

    if (!nh_private.getParam("x3", topicPillar.pillar[3].x))
        topicPillar.pillar[3].x = 3.11;
    if (!nh_private.getParam("y3", topicPillar.pillar[3].y))
        topicPillar.pillar[3].y = 3.11;

    for (int i = 0; i < 4; i++)
    {
        printf(" x= %f y= %f \n", topicPillar.pillar[i].x, topicPillar.pillar[i].y);
    }

    while (ros::ok())
    {
        pub_topicPillar.publish(topicPillar); // Публикация полученных данных
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши данные публикуются
        r.sleep();       // Интеллектуальная задержка на указанную частоту
    }

    return 0;
}