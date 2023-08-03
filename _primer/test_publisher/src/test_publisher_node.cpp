#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"test_publisher_node");
    ros::NodeHandle nh;
    ros::Publisher str_pub = nh.advertise<std_msgs::String> ("test_publish",16);
    ros::Rate r(10);
    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "Hello Vasiliy 111";
        str_pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}