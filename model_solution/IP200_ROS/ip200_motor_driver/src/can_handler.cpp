// motor_driver.cpp
#include <ip200_motor_driver/can_handler.h>

CanPacketHandler::CanPacketHandler(ros::NodeHandle *nh)
{
    frame_pub_ = nh->advertise<can_msgs::Frame>("/sent_messages", 10);
    // read_sub_  = nh->subscribe("/received_messages", 10, &CanPacketHandler::callback_read_frame, this);
    reset_service_ = nh->advertiseService("/reset_packet", &CanPacketHandler::callback_reset_frame, this);
}

CanPacketHandler::~CanPacketHandler()
{}

void CanPacketHandler::make_frame(const uint id, const boost::array<uint8_t, 8UL>& data)
{
    write_frame_.header.stamp = ros::Time::now();
    write_frame_.header.frame_id = "";

    write_frame_.id = id;
    write_frame_.is_rtr = false;
    write_frame_.is_extended = false;
    write_frame_.is_error = false;
    write_frame_.dlc = 8;
    write_frame_.data = data;
}

void CanPacketHandler::write_frame()
{
    frame_pub_.publish(write_frame_);
    ros::Duration(0.001).sleep();
}

can_msgs::Frame CanPacketHandler::read_frame()
{
    return read_frame_;
}

void CanPacketHandler::callback_read_frame(const can_msgs::Frame::ConstPtr &msg){
    
    read_frame_ = *msg;
}

bool CanPacketHandler::callback_reset_frame(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ROS_INFO("Resetting packet");

    boost::array<uint8_t, 8UL> reset_packet = {0x00, 0, 0, 0, 0, 0, 0, 0};

    make_frame(0x000, reset_packet);
    res.success = true;
    res.message = "Reset packet";
}

/***
int main(int argc, char **argv){
    ros::init(argc, argv, "can_packet_handler");
    ros::NodeHandle nh;
    CanPacketHandler handler = CanPacketHandler(&nh);
    ros::Rate loop_rate(1000);

    handler.make_frame(0x060, {0x00, 0, 0, 0, 0, 0, 0, 0});

    while(ros::ok()){
        handler.write_frame();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
***/