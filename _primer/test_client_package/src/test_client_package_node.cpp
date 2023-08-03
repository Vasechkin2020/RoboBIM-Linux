#include <ros/ros.h>
#include "test_service/example_service.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_client_package_node");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<test_service::example_service>("triangle_square");
    test_service::example_service srv;
    srv.request.a = 3.5;
    srv.request.h = 5.2;
    if (client.call(srv))
    {
        ROS_INFO("S: %f",(float) srv.response.s);
    }
    else
    {
        ROS_ERROR ("Failed to call service triangle_square ");
        return 1;
    }
    return 0;
}