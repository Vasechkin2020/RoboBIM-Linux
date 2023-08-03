#include "ros/ros.h"
#include "test_service/example_service.h"

bool calculate (test_service::example_service::Request &req,
              test_service::example_service::Response &res)
{
    res.s = 0.5 * req.a * req.h;
    ROS_INFO ("request: a=%f, y=%f", (float) req.a, (float) req.h);
    ROS_INFO ("sending back response: [%f]",(float) res.s);
    return true;
}

int main (int argc, char** argv)
{
    ros::init(argc,argv,"test_service_package_node");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("triangle_square",calculate);
    ROS_INFO("Ready to calculate.");
    ros::spin();
    return 0;
    
}