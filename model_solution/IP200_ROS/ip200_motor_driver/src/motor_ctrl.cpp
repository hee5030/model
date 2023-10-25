#include <cstdint>
#include <memory>
#include <ros/ros.h>
#include <ros/console.h>
#include <can_msgs/Frame.h>

// little endian
#define CAN_32BYTE1(n)       ((uint16_t)(((uint64_t)(n)) & 0xff))
#define CAN_32BYTE2(n)       ((uint16_t)((((uint64_t)(n)) >> 8) & 0xff))
#define CAN_32BYTE3(n)       ((uint16_t)((((uint64_t)(n)) >> 16) & 0xff))
#define CAN_32BYTE4(n)       ((uint16_t)((((uint64_t)(n)) >> 24) & 0xff))

#define MODE_BYTE 1

#define VEL_BYTE1 2
#define VEL_BYTE2 3
#define VEL_BYTE3 4
#define VEL_BYTE4 5

#define ACC_BYTE1 0
#define ACC_BYTE2 1
#define ACC_BYTE3 2
#define ACC_BYTE4 3

#define DACC_BYTE1 0
#define DACC_BYTE2 1
#define DACC_BYTE3 2
#define DACC_BYTE4 3

#define DXL_LOBYTE(w) ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w) ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

#define ALL_NODE 0x00
#define ID0      0x001
#define ID1      0x003
#define ID2      0x004
#define NMT      0x000
#define SDO      0x600
#define PDO1     0x200
#define PDO2     0x300
#define PDO3     0x400 // VEL_RX_PDO
#define PDO4     0x500 // ACC_RX_PDO

#define READY2SWITCH_ON 0x06
#define SWITCH_ON       0x07
#define OPERATION_EN    0x0F

boost::array<uint8_t, 8UL>  RESET_ALL_NODE = {0x81, 0, 0, 0, 0, 0, 0, 0};
boost::array<uint8_t, 8UL>  START_ALL_NODE = {0x01, 0, 0, 0, 0, 0, 0, 0};
boost::array<uint8_t, 8UL>  ACC_RX_PDO     = {0x81, 0, 0, 0, 0, 0, 0, 0};  // 32 ACC Profile, 32 DACC Profile
boost::array<uint8_t, 8UL>  VEL_RX_PDO     = {0x81, 0, 0, 0, 0, 0, 0, 0};  // 16 Contol word, 32 target vel, 8 op mod
//////// test CAN


// boost::array<uint8_t, 8UL>  CAN1_     = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
// boost::array<uint8_t, 8UL>  CAN2_     = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
// boost::array<uint8_t, 8UL>  CAN4_     = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
// boost::array<uint8_t, 8UL>  CAN5_     = {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
// boost::array<uint8_t, 8UL>  CAN6_     = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
// boost::array<uint8_t, 8UL>  CAN7_     = {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
// boost::array<uint8_t, 8UL>  CAN8_     = {0x23, 0x83, 0x60, 0x00, 0x90, 0xD0, 0x03, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
// boost::array<uint8_t, 8UL>  CAN9_     = {0x23, 0xFF, 0x60, 0x00, 0x90, 0xD0, 0x03, 0x00};  // 16 Contol word, 32 target vel, 8 op mod

// model solution ZLAC8015D
// boost::array<uint8_t, 2UL>  CAN1_     = {0x01, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN2_     = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN3_     = {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN4_     = {0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN5_     = {0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN6_     = {0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN7_     = {0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN8_     = {0x23, 0x81, 0x60, 0x01, 0x3C, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN9_     = {0x23, 0x81, 0x60, 0x02, 0x3C, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN10_     = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN11_     = {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN12_     = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod

// boost::array<uint8_t, 8UL>  CAN7_     = {0x23, 0x83, 0x60, 0x00, 0x90, 0xD0, 0x03, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
// boost::array<uint8_t, 8UL>  CAN13_     = {0x23, 0xFF, 0x60, 0x03, 0x90, 0xD0, 0x03, 0x00};  // 16 Contol word, 32 target vel, 8 op mod
boost::array<uint8_t, 8UL>  CAN13_     = {0x23, 0xFF, 0x60, 0x03, 0x00, 0x64, 0x00, 0x00};  // 16 Contol word, 32 target vel, 8 op mod

ros::Publisher pub;
can_msgs::Frame frame;

bool is_init = false;

can_msgs::Frame frame_gen(const uint cod_id, const boost::array<uint8_t, 8UL> &data){
    can_msgs::Frame can_frame;

    can_frame.header.stamp = ros::Time::now();
    can_frame.header.frame_id = "";

    can_frame.id = cod_id;
    can_frame.is_rtr = false;
    can_frame.is_extended = false;
    can_frame.is_error = false;
    can_frame.dlc = 8;
    can_frame.data = data;

    return can_frame;
}

void target_acc(can_msgs::Frame &frame, const uint32_t target_acc){
    frame.data[ACC_BYTE1] = CAN_32BYTE1(target_acc);
    frame.data[ACC_BYTE2] = CAN_32BYTE2(target_acc);
    frame.data[ACC_BYTE3] = CAN_32BYTE3(target_acc);
    frame.data[ACC_BYTE4] = CAN_32BYTE4(target_acc);

    frame.data[DACC_BYTE1] = CAN_32BYTE1(target_acc);
    frame.data[DACC_BYTE2] = CAN_32BYTE2(target_acc);
    frame.data[DACC_BYTE3] = CAN_32BYTE3(target_acc);
    frame.data[DACC_BYTE4] = CAN_32BYTE4(target_acc);
}
void target_rpm(can_msgs::Frame &frame, const uint32_t target_rpm){

    frame.data[VEL_BYTE1] = CAN_32BYTE1(target_rpm);
    frame.data[VEL_BYTE2] = CAN_32BYTE2(target_rpm);
    frame.data[VEL_BYTE3] = CAN_32BYTE3(target_rpm);
    frame.data[VEL_BYTE4] = CAN_32BYTE4(target_rpm);
}

void set_controlword(can_msgs::Frame &frame, const uint8_t controlword){
    can_msgs::Frame controlword_frame = frame_gen(PDO3 + ID1, ACC_RX_PDO);
    controlword_frame.data[MODE_BYTE] = controlword;
}

void pub_canframe() {
    can_msgs::Frame test;
    ros::Duration(1).sleep();
    // test = frame_gen(0x000, CAN1_); pub.publish(test); ros::Duration(0.1).sleep();
    // test = frame_gen(0x000, CAN2_); pub.publish(test); ros::Duration(0.1).sleep();  
    test = frame_gen(0x601, CAN2_); pub.publish(test); ros::Duration(0.1).sleep();  
    test = frame_gen(0x601, CAN3_); pub.publish(test); ros::Duration(0.1).sleep();
    test = frame_gen(0x601, CAN4_); pub.publish(test); ros::Duration(0.1).sleep();
    test = frame_gen(0x601, CAN5_); pub.publish(test); ros::Duration(0.1).sleep();
    test = frame_gen(0x601, CAN6_); pub.publish(test); ros::Duration(0.1).sleep();
    test = frame_gen(0x601, CAN7_); pub.publish(test); ros::Duration(0.1).sleep();
    test = frame_gen(0x601, CAN8_); pub.publish(test); ros::Duration(0.1).sleep();
    test = frame_gen(0x601, CAN9_); pub.publish(test); ros::Duration(0.1).sleep();
    test = frame_gen(0x601, CAN10_); pub.publish(test); ros::Duration(0.1).sleep();
    test = frame_gen(0x601, CAN11_); pub.publish(test); ros::Duration(0.1).sleep();
    test = frame_gen(0x601, CAN12_); pub.publish(test); ros::Duration(0.1).sleep();
}

void pub_(){
    can_msgs::Frame test;

    test = frame_gen(0x601, CAN13_); pub.publish(test); ros::Duration(0.1).sleep();
    // test = frame_gen(0x604, CAN9_); pub.publish(test); ros::Duration(0.1).sleep();
}
s
int main (int argc, char **argv) {
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;
    pub = nh.advertise<can_msgs::Frame>("/sent_messages", 10);
    ros::Rate loop_rate(1000);
    bool test = false;
    while (ros::ok()) {
        if(test == false) {
            pub_canframe();
            test = true;
        }
        pub_();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}