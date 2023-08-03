#include <ros/ros.h>
#include <std_msgs/String.h>

void message_callback(const std_msgs::String &msg)
{
    ROS_INFO("Message: %s", msg.data.c_str());
}
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"test_subscriber_node");
    ros::NodeHandle nh;
    ros::Subscriber str_sub = nh.subscribe("test_publish",16, message_callback);
    ros::spin();
    return 0;
}