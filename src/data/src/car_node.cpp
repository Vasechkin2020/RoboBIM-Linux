
#define NN "\x1b[32;40m Car_node"

#include <ros/ros.h>
//#include <data/point.h>
#include <geometry_msgs/Pose2D.h>
//data::point car_msg; // Структура сообщения в которую пишем обработанны данные о местонахождении машины в  начальный момент времени
geometry_msgs::Pose2D car_msg; // Структура сообщения в которую пишем обработанны данные о местонахождении машины в  начальный момент времени

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_WARN("%s   Car Node PrintBIM 2024 ROS 1.0 Raspberry Pi 4B !!! ver 1.1 ", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------\n", NN);

    ros::init(argc, argv, "car_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher pub_topicCar = nh.advertise<geometry_msgs::Pose2D>("pbCar", 16); // Это мы обьявляем структуру для публикации которую сформировали по данным с джойстика
    ros::Duration(1).sleep();
    
    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
    if (!nh_private.getParam("x0", car_msg.x))
        car_msg.x = 0.11;
    if (!nh_private.getParam("y0", car_msg.y))
        car_msg.y = 0.11;
    if (!nh_private.getParam("theta", car_msg.theta))
        car_msg.theta = 0.11;

        ROS_INFO("Start pub_topicCar.publish");
        ROS_INFO(" x= %.3f y= %.3f th= %.3f", car_msg.x, car_msg.y,car_msg.theta);

    if (ros::ok())
    {
        pub_topicCar.publish(car_msg); // Публикация полученных данных
        ros::Duration(1).sleep();
        ROS_INFO("End pub_topicCar.publish \n");
    }

    return 0;
}