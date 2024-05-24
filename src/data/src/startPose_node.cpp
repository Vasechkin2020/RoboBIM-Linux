
#define NN "\x1b[32;40m StartPose_node"

#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
geometry_msgs::Pose2D startPose2D_msg;             // Структура сообщения в которую пишем обработанны данные о местонахождении машины в  начальный момент времени

int main(int argc, char **argv)
{
    ROS_INFO("%s -------------------------------------------------------------", NN);
    ROS_WARN("%s StartPose Node PrintBIM 2024 ROS 1.0 Raspberry Pi 4B !!! ver 1.1 ", NN);
    ROS_ERROR("%s ------------------ROS_ERROR----------------------------------\n", NN);

    ros::init(argc, argv, "startPose");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher pub_StartPose2D = nh.advertise<geometry_msgs::Pose2D>("pbStart/Pose2D", 16);             // Это мы обьявляем структуру для публикации которую сформировали по данным с джойстика
    ros::Duration(1).sleep();

    // Имя можно с палкой или без, смотря как в лаунч файле параметры обявлены. связано с видимостью глобальной или локальной. относительным поиском переменной как сказал Максим
    if (!nh_private.getParam("x", startPose2D_msg.x))
        startPose2D_msg.x = 0.11;
    if (!nh_private.getParam("y", startPose2D_msg.y))
        startPose2D_msg.y = 0.11;
    if (!nh_private.getParam("theta", startPose2D_msg.theta))
        startPose2D_msg.theta = 0.11;

    ROS_INFO("Start pub_StartPose2D.publish");
    ROS_INFO(" x= %.3f y= %.3f th= %.3f", startPose2D_msg.x, startPose2D_msg.y, startPose2D_msg.theta);

    if (ros::ok())
    {
        pub_StartPose2D.publish(startPose2D_msg); // Публикация полученных данных
        ros::Duration(1).sleep();
        ROS_INFO("End pub_StartPose.publish \n");
    }
    return 0;
}