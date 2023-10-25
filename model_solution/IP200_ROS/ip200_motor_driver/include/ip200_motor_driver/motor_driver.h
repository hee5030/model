// MOTOR_DRIVER_H_

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <can_msgs/Frame.h>
#include <std_srvs/SetBool.h>

#include <ip200_msgs/RobotMotor.h>
#include <ip200_motor_driver/can_handler.h>

#include <cstdint>
#include <string>
#include <memory>

// #define LEFT_ID  0x03 // parameter id 
// #define RIGHT_ID 0x04 
#define LEFT_ID  0x01 // parameter id 
#define RIGHT_ID 0x01

#define PULSE_PER_ROUND 1000 
#define CANOPEN_SYNC_FREQUENCY 1  // in Hz
#define CANOPEN_RPM_FREQUNECY 100 // in Hz      // 100인데 수정함
#define READ_RPM_FREQUENCY 100 // in Hz

// Packet Control Table Macro for little endian
#define CAN_32MAKEWORD(a, b, c, d) ((uint32_t)(((uint8_t)(a)) | ((uint32_t)((uint8_t)(b))) << 8 | ((uint32_t)((uint8_t)(c))) << 16 | ((uint32_t)((uint8_t)(d))) << 24))
// #define CAN_32BYTE1(n) ((uint16_t)(((uint64_t)(n)) & 0xff))
// #define CAN_32BYTE2(n) ((uint16_t)((((uint64_t)(n)) >> 8) & 0xff))
// #define CAN_32BYTE3(n) ((uint16_t)((((uint64_t)(n)) >> 16) & 0xff))
// #define CAN_32BYTE4(n) ((uint16_t)((((uint64_t)(n)) >> 24) & 0xff))
#define CAN_32BYTE1(n) ((uint16_t)(((uint16_t)(n)) & 0xff))
#define CAN_32BYTE2(n) ((uint16_t)((((uint16_t)(n)) >> 8) & 0xff))
#define CAN_32BYTE3(n) ((uint16_t)((((uint64_t)(n)) >> 16) & 0xff))
#define CAN_32BYTE4(n) ((uint16_t)((((uint64_t)(n)) >> 24) & 0xff))

#define VEL_PPR_BYTE1 4
#define VEL_PPR_BYTE2 5
#define VEL_PPR_BYTE3 6
#define VEL_PPR_BYTE4 7

#define CMD_PPR_BYTE1 2
#define CMD_PPR_BYTE2 3
#define CMD_PPR_BYTE3 4
#define CMD_PPR_BYTE4 5

#define ACC_BYTE1 0
#define ACC_BYTE2 1
#define ACC_BYTE3 2
#define ACC_BYTE4 3

#define DACC_BYTE1 4
#define DACC_BYTE2 5
#define DACC_BYTE3 6
#define DACC_BYTE4 7

#define PPR 10000  // pulse per round
#define ACC_DACC_VALUE 750000 // pulse per sec^2  

// (ACC_DACC_VALUE / PPR) == round / sec^2
//  (ACC_DACC_VALUE / PPR) / GEAR_RATIO == wheel_round / sec^2

const std::string aidl = "ip200";

const int NMT  = 0x000;
const int SYNC = 0x080;

const int RXPDO1 = 0x200; // controlword 16bit, target pos 32bit, mode of op 8bit 
const int RXPDO2 = 0x300; // profile vel 32bit, profile acc 32bit
const int RXPDO3 = 0x400; // control word 16bit, target vel 32bit, mode of op 8bit
const int RXPDO4 = 0x500; // profile acc 32bit, profile dacc 32bit

const int TXPDO1 = 0x180; // status word 16bit
const int TXPDO2 = 0x280; // status word 16bit, position val 32bit, mod of op 8bit
const int TXPDO3 = 0x380; // position val 32bit, velocity val 32bit
const int TXPDO4 = 0x480; // position acc 32bit, digital input val 32bit 

const int SDO = 0x600;

const boost::array<uint8_t, 8UL> RESET_ALLNODE_FRAME = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const boost::array<uint8_t, 8UL> START_ALL_NODE_FRAME = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const boost::array<uint8_t, 8UL> SYNC_ALL_NODE_FRAME = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const boost::array<uint8_t, 8UL> ACC_DACC_FRAME = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};        // 16 Contol word, 32 target vel, 8 op mod
const boost::array<uint8_t, 8UL> PROFILE_VEL_ACC_FRAME = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 16 Contol word, 32 target vel, 8 op mod

// const boost::array<uint8_t, 8UL> CONTROLWORD_READY2SW_FRAME = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 16 Contol word, 32 target vel, 8 op mod
// const boost::array<uint8_t, 8UL> CONTROLWORD_SW_ON_FRAME = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};    // 16 Contol word, 32 target vel, 8 op mod
// const boost::array<uint8_t, 8UL> CONTROLWORD_OP_EN_FRAME = {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00};    // 16 Contol word, 32 target vel, 8 op mod
// const boost::array<uint8_t, 8UL> RPM_CMD_FRAME = {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00};         // 16 Contol word, 32 target vel, 8 op mod
// const boost::array<uint8_t, 8UL> CONTROLWORD_OP_EN_FRAME = {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};    // 16 Contol word, 32 target vel, 8 op mod
const boost::array<uint8_t, 8UL> RPM_CMD_FRAME = {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00};         // 16 Contol word, 32 target vel, 8 op mod

// 내가 만든 것
const boost::array<uint8_t, 8UL> CONTROLWORD_READY2SW_FRAME = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00}; // 16 Contol word, 32 target vel, 8 op mod
const boost::array<uint8_t, 8UL> CONTROLWORD_SW_ON_FRAME = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00}; 
const boost::array<uint8_t, 8UL> CONTROLWORD_OP_EN_FRAME = {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00};

const boost::array<uint8_t, 8UL> SET_PDO_FRAME = {0x2F, 0x02, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00}; 
const boost::array<uint8_t, 8UL> START_PDO_FRAME = {0x2F, 0x02, 0x16, 0x00, 0x03, 0x00, 0x00, 0x00}; 
const boost::array<uint8_t, 8UL> FIRST_PDO_FRAME = {0x23, 0x02, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60}; 
const boost::array<uint8_t, 8UL> SECOND_PDO_FRAME = {0x23, 0x02, 0x16, 0x02, 0x20, 0x03, 0xFF, 0x60}; 
const boost::array<uint8_t, 8UL> THIRD_PDO_FRAME = {0x23, 0x02, 0x16, 0x03, 0x08, 0x00, 0x60, 0x60};

const boost::array<uint8_t, 8UL> PROFILE_LEFT_ACC_FRAME = {0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
const boost::array<uint8_t, 8UL> PROFILE_RIGHT_ACC_FRAME = {0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};
const boost::array<uint8_t, 8UL> PROFILE_LEFT_DACC_FRAME = {0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
const boost::array<uint8_t, 8UL> PROFILE_RIGHT_DACC_FRAME = {0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};

class MotorDriver
{
public:
    MotorDriver(ros::NodeHandle *nh);
    ~MotorDriver();

    void writePPR(const int32_t left_ppr, const int32_t right_ppr);
    // void writePPR(const int16_t left_ppr, const int16_t right_ppr);
    // void writePPR(const float left_rpm, const float right_rpm);

	void cmd_rpm_callback(const ip200_msgs::RobotMotor::ConstPtr &msg);
    void read_PPR_callback(const can_msgs::Frame::ConstPtr &msg); // 

    void rpm_pub(const ros::TimerEvent &e); 

    void CAN_sync_pub(const ros::TimerEvent& e); // CANOPEN_SYNC_FREQUENCY 1Hz
    void CAN_cmd_rpm_pub(const ros::TimerEvent &e);

    bool init(const std::string aidlrobot);
    bool endup();

private:

    enum ControlWord 
    {
        RESET_ALL_NODE_,
        START_ALL_NODE_,
        SET_PDO,
        FIRST_PDO,
        SECOND_PDO,
        THIRD_PDO,
        START_PDO,
        PROFILE_LEFT_ACC,
        PROFILE_RIGHT_ACC,
        PROFILE_LEFT_DACC,
        PROFILE_RIGHT_DACC,
        READY_2_SWICH,
        SWITCH_ON,
        OP_ENABLE,
        OP_DISENABLE
    };

    //CanPacketHandler *CAN_handler_;
    std::shared_ptr<CanPacketHandler> CAN_handler_;
    std::shared_ptr<CanPacketHandler> CAN_sync_;

    uint8_t left_motor_id_;
    uint8_t right_motor_id_;

    uint32_t pulse_per_round_;
    uint32_t max_rpm_;

    ros::Publisher rpm_pub_;
    ros::Subscriber rpm_velocity_sub_;
    ros::Subscriber ppr_packet_sub_;

    ros::Timer can_sync_pub_timer_;
    ros::Timer can_cmd_rpm_pub_timer_;
    ros::Timer rpm_pub_timer_;
    ros::Timer readRPM_timer_;

    int left_PPR_;
    int right_PPR_;

    float left_RPM_;
    float right_RPM_;
    
    bool node_up_;
    bool torque_;

    void set_profile_acc_(const uint32_t acc_value);
    void set_node_mode_(const ControlWord mode);
};

#endif // _MOTOR_DRIVER_H_