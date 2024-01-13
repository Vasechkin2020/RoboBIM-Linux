
#define NN "\x1b[32;40m Сar_node"

#include <ros/ros.h>
#include <data/point.h>
data::point car_msg; // Структура сообщения в которую пишем обработанны данные о местонахождении машины в  начальный момент времени

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_INFO("%s START Pillar Node  ROS Raspberry Pi 4B !!! ver 1.0 ", NN);
    ROS_INFO("%s ------------------ROS_INFO-----------------------------------", NN);
    ROS_WARN("%s ------------------ROS_WARN-----------------------------------", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------", NN);

    ros::init(argc, argv, "car_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher pub_topicCar = nh.advertise<data::point>("pbCar", 16); // Это мы обьявляем структуру для публикации которую сформировали по данным с джойстика
    ros::Duration(1).sleep();
    
    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
    if (!nh_private.getParam("x0", car_msg.x))
        car_msg.x = 0.11;
    if (!nh_private.getParam("y0", car_msg.y))
        car_msg.y = 0.11;

        printf(" x= %f y= %f \n", car_msg.x, car_msg.y);

    if (ros::ok())
    {
        ROS_INFO("Start pub_topicCar.publish");
        pub_topicCar.publish(car_msg); // Публикация полученных данных
        ros::Duration(1).sleep();
        ROS_INFO("End pub_topicCar.publish");
    }

    return 0;
}