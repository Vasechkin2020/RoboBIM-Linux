#ifndef MYCLASS_H
#define MYCLASS_H

#define RATE2 10
#include <ros/ros.h>

class MyClass
{
private:
    /* data */
public:
    ros::NodeHandle nh;
    ros::Publisher str_pub_body ;
    ros::Publisher str_pub_control;
    ros::Subscriber command_sub;
    ros::Rate r; 

    void setup(int argc, char **argv);
    void loop();

    MyClass(
        
        /* args */);
    ~MyClass();
};

MyClass::MyClass():r(RATE2)
{
}
MyClass::~MyClass()
{
}


void MyClass::setup(int argc_, char **argv_)
{
    ros::init(argc_, argv_, "data_node");

    str_pub_body = nh.advertise<my_msgs::Body>("body_topic", 16);              //Это мы публикуем структуру
    str_pub_control = nh.advertise<my_msgs::Control>("control_topic", 16);     //Это мы публикуем структуру
    command_sub = nh.subscribe("command_topic", 16, message_callback_Command); // Это мы подписываемся на то что публигует Главная нода для Body

}
void MyClass::loop()
{
    ros::Rate r(RATE2);  
    while (ros::ok())
    {

        ros::spinOnce(); // Обновление в данных в ядре ROS
        r.sleep();       // Интеллектуальная задержка на указанную частоту
    }
}




#endif