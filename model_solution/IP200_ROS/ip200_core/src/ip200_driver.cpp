#include <ros/ros.h>
#include <ros/console.h>
#include <ip200_core/ip200_driver.hpp>

using namespace aidl;

int main (int argc, char **argv) {
    ros::init(argc, argv, "ip200_driver");
    ros::NodeHandle nh;
    ip200BodyNode ip200 = ip200BodyNode(&nh);

	ros::AsyncSpinner spinner(0);
    spinner.start();
	
    ROS_INFO("ip200 Driver Node is started");
	ros::waitForShutdown();
}