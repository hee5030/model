#ifndef CAN_HANDLER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <can_msgs/Frame.h>
#include <std_srvs/Trigger.h>

#include <cstdint>
#include <string>
#include <array>
#include <iostream>

class CanPacketHandler
{
public:
    CanPacketHandler(ros::NodeHandle *nh);
    ~CanPacketHandler();

    void make_frame(const uint id, const boost::array<uint8_t, 8UL>& data);
    void write_frame();
    can_msgs::Frame read_frame();

    void callback_read_frame(const can_msgs::Frame::ConstPtr &msg);
    bool callback_reset_frame(std_srvs::Trigger::Request &req, 
                              std_srvs::Trigger::Response &res);

private:
    ros::Publisher frame_pub_;
    ros::Subscriber read_sub_;
    ros::ServiceServer reset_service_;

    can_msgs::Frame write_frame_;
    can_msgs::Frame read_frame_;
};

#endif // CAN_HANDLER_H_