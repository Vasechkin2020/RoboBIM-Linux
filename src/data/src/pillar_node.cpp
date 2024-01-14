#define NN "\x1b[32;40m Pillar_node"

#include <ros/ros.h>
#include <data/topicPillar.h>
data::topicPillar pillar_msg; // Структура в которую пишем обработанны данные о местонахождении столбов

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_WARN("%s Pillar Node PrintBIM 2024 ROS 1.0 Raspberry Pi 4B !!! ver 1.0 ", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------\n", NN);

    ros::init(argc, argv, "pillar_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher pub_topicPillar = nh.advertise<data::topicPillar>("pbPillar", 16); // Это мы обьявляем структуру для публикации которую сформировали по данным с джойстика
    ros::Duration(1).sleep();
    
    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
    if (!nh_private.getParam("x0", pillar_msg.pillar[0].x))
        pillar_msg.pillar[0].x = 0.11;
    if (!nh_private.getParam("y0", pillar_msg.pillar[0].y))
        pillar_msg.pillar[0].y = 0.11;

    if (!nh_private.getParam("x1", pillar_msg.pillar[1].x))
        pillar_msg.pillar[1].x = 1.11;
    if (!nh_private.getParam("y1", pillar_msg.pillar[1].y))
        pillar_msg.pillar[1].y = 1.11;

    if (!nh_private.getParam("x2", pillar_msg.pillar[2].x))
        pillar_msg.pillar[2].x = 2.11;
    if (!nh_private.getParam("y2", pillar_msg.pillar[2].y))
        pillar_msg.pillar[2].y = 2.11;

    if (!nh_private.getParam("x3", pillar_msg.pillar[3].x))
        pillar_msg.pillar[3].x = 3.11;
    if (!nh_private.getParam("y3", pillar_msg.pillar[3].y))
        pillar_msg.pillar[3].y = 3.11;

    ROS_INFO("Start pub_topicPillar.publish");
    for (int i = 0; i < pillar_msg.pillar.static_size; i++)
    {
        ROS_INFO(" x= %f y= %f ", pillar_msg.pillar[i].x, pillar_msg.pillar[i].y);
    }

    if (ros::ok())
    {
        pub_topicPillar.publish(pillar_msg); // Публикация полученных данных
        ros::Duration(1).sleep();
        ROS_INFO("End pub_topicPillar.publish \n");
    }

    return 0;
}