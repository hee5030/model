// motor_driver.cpp
#include <ip200_motor_driver/motor_driver.h>

// constructor
MotorDriver::MotorDriver(ros::NodeHandle *nh)
: left_motor_id_(LEFT_ID),
  right_motor_id_(RIGHT_ID),
  pulse_per_round_(0),
  left_PPR_(0),
  right_PPR_(0),
  left_RPM_(0),
  right_RPM_(0),
  node_up_(false)
{
    CAN_handler_ = std::shared_ptr<CanPacketHandler>(new CanPacketHandler(nh));
    CAN_sync_    = std::shared_ptr<CanPacketHandler>(new CanPacketHandler(nh)); // 

    torque_ = false;
    max_rpm_ = 0;
    rpm_pub_ = nh->advertise<ip200_msgs::RobotMotor>("/cur_rpm", 100);

    can_sync_pub_timer_ = nh -> createTimer(
        ros::Duration(1.0 / CANOPEN_SYNC_FREQUENCY),
        &MotorDriver::CAN_sync_pub, this);

    can_cmd_rpm_pub_timer_ = nh -> createTimer(
        ros::Duration(1.0 / CANOPEN_RPM_FREQUNECY), 
        &MotorDriver::CAN_cmd_rpm_pub, this);

    rpm_pub_timer_ = nh -> createTimer(
        ros::Duration(1.0 / CANOPEN_RPM_FREQUNECY),
        &MotorDriver::rpm_pub, this);

    rpm_velocity_sub_ = nh -> subscribe(
        "cmd_rpm", 50, &MotorDriver::cmd_rpm_callback, this);

    ppr_packet_sub_ = nh -> subscribe(
        "received_messages", 100, &MotorDriver::read_PPR_callback, this);

    node_up_ = true;
}

// destructor
MotorDriver::~MotorDriver(){
    CAN_handler_.reset();
    CAN_sync_.reset();
}

// setting CANOpen Node mode
void MotorDriver::set_node_mode_(const ControlWord mode){
    int left_motor_PDO = RXPDO3 + left_motor_id_;
    int right_motor_PDO = RXPDO3 + right_motor_id_;
    int sdo = SDO + left_motor_id_;
    switch (mode)
    {
    case RESET_ALL_NODE_:
        CAN_handler_->make_frame(NMT, RESET_ALLNODE_FRAME);
        CAN_handler_->write_frame();
        break;
    case START_ALL_NODE_:
        CAN_handler_->make_frame(NMT, START_ALL_NODE_FRAME);
        CAN_handler_->write_frame();
        break;

    case SET_PDO:
        CAN_handler_->make_frame(sdo, SET_PDO_FRAME);
        CAN_handler_->write_frame();
        break;
    case FIRST_PDO:
        CAN_handler_->make_frame(sdo, FIRST_PDO_FRAME);
        CAN_handler_->write_frame();
        break;
    case SECOND_PDO:
        CAN_handler_->make_frame(sdo, SECOND_PDO_FRAME);
        CAN_handler_->write_frame();
        break;
    case THIRD_PDO:
        CAN_handler_->make_frame(sdo, THIRD_PDO_FRAME);
        CAN_handler_->write_frame();
        break;
    case START_PDO:
        CAN_handler_->make_frame(sdo, START_PDO_FRAME);
        CAN_handler_->write_frame();
        break;

    case PROFILE_LEFT_ACC:
        CAN_handler_->make_frame(sdo, PROFILE_LEFT_ACC_FRAME);
        CAN_handler_->write_frame();
        break;
    case PROFILE_RIGHT_ACC:
        CAN_handler_->make_frame(sdo, PROFILE_RIGHT_ACC_FRAME);
        CAN_handler_->write_frame();
        break;
    case PROFILE_LEFT_DACC:
        CAN_handler_->make_frame(sdo, PROFILE_LEFT_DACC_FRAME);
        CAN_handler_->write_frame();
        break;
    case PROFILE_RIGHT_DACC:
        CAN_handler_->make_frame(sdo, PROFILE_RIGHT_DACC_FRAME);
        CAN_handler_->write_frame();
        break;

    case READY_2_SWICH:
        CAN_handler_->make_frame(left_motor_PDO, CONTROLWORD_READY2SW_FRAME);
        CAN_handler_->write_frame();
        // CAN_handler_->make_frame(right_motor_PDO, CONTROLWORD_READY2SW_FRAME);
        // CAN_handler_->write_frame();
        break;
    case SWITCH_ON:
        CAN_handler_->make_frame(left_motor_PDO, CONTROLWORD_SW_ON_FRAME);
        CAN_handler_->write_frame();
        // CAN_handler_->make_frame(right_motor_PDO, CONTROLWORD_SW_ON_FRAME);
        // CAN_handler_->write_frame();
        break;
    case OP_ENABLE:
        CAN_handler_->make_frame(left_motor_PDO, CONTROLWORD_OP_EN_FRAME);
        CAN_handler_->write_frame();
        // CAN_handler_->make_frame(right_motor_PDO, CONTROLWORD_OP_EN_FRAME);
        // CAN_handler_->write_frame();
        break;
    case OP_DISENABLE:
        CAN_handler_->make_frame(left_motor_PDO, CONTROLWORD_SW_ON_FRAME);
        CAN_handler_->write_frame();
        // CAN_handler_->make_frame(right_motor_PDO, CONTROLWORD_SW_ON_FRAME);
        // CAN_handler_->write_frame();
        break;
    
    default:
        break;
    }
}

// setting Acc profile value [Pulse/sec^2]
void MotorDriver::set_profile_acc_(const uint32_t acc_value){
    boost::array<uint8_t, 8UL> profile_acc = ACC_DACC_FRAME;

    int left_motor_PDO  = RXPDO4 + left_motor_id_;
    int right_motor_PDO = RXPDO4 + right_motor_id_;
 
    profile_acc[ACC_BYTE1]  = CAN_32BYTE1(acc_value);
    profile_acc[ACC_BYTE2]  = CAN_32BYTE2(acc_value);
    profile_acc[ACC_BYTE3]  = CAN_32BYTE3(acc_value);
    profile_acc[ACC_BYTE4]  = CAN_32BYTE4(acc_value);
    profile_acc[DACC_BYTE1] = CAN_32BYTE1(acc_value);
    profile_acc[DACC_BYTE2] = CAN_32BYTE2(acc_value);
    profile_acc[DACC_BYTE3] = CAN_32BYTE3(acc_value);
    profile_acc[DACC_BYTE4] = CAN_32BYTE4(acc_value);

    CAN_handler_->make_frame(left_motor_PDO, profile_acc);
    CAN_handler_->write_frame();
    CAN_handler_->make_frame(right_motor_PDO, profile_acc);
    CAN_handler_->write_frame();
}

// write left and right RPR command to ROS CAN bridge node
// void MotorDriver::writePPR(const int32_t left_ppr, const int32_t right_ppr){
//     boost::array<uint8_t, 8UL> left_rpm_cmd  = RPM_CMD_FRAME;
//     boost::array<uint8_t, 8UL> right_rpm_cmd = RPM_CMD_FRAME;

//     int left_motor_PDO  = RXPDO3 + left_motor_id_;
//     int right_motor_PDO = RXPDO3 + right_motor_id_;

//     left_rpm_cmd[CMD_PPR_BYTE1] = CAN_32BYTE1(left_ppr);
//     left_rpm_cmd[CMD_PPR_BYTE2] = CAN_32BYTE2(left_ppr);
//     left_rpm_cmd[CMD_PPR_BYTE3] = CAN_32BYTE3(left_ppr);
//     left_rpm_cmd[CMD_PPR_BYTE4] = CAN_32BYTE4(left_ppr);

//     right_rpm_cmd[CMD_PPR_BYTE1] = CAN_32BYTE1(right_ppr);
//     right_rpm_cmd[CMD_PPR_BYTE2] = CAN_32BYTE2(right_ppr);
//     right_rpm_cmd[CMD_PPR_BYTE3] = CAN_32BYTE3(right_ppr);
//     right_rpm_cmd[CMD_PPR_BYTE4] = CAN_32BYTE4(right_ppr);

//     CAN_handler_->make_frame(left_motor_PDO, left_rpm_cmd);
//     CAN_handler_->write_frame();
//     CAN_handler_->make_frame(right_motor_PDO, right_rpm_cmd);
//     CAN_handler_->write_frame();
// }

// 테스트용
//void MotorDriver::writePPR(const int16_t left_ppr, const int16_t right_ppr){
void MotorDriver::writePPR(const int32_t left_ppr, const int32_t right_ppr){
    boost::array<uint8_t, 8UL> left_rpm_cmd  = RPM_CMD_FRAME;
    boost::array<uint8_t, 8UL> right_rpm_cmd = RPM_CMD_FRAME;

    int left_motor_PDO  = RXPDO3 + left_motor_id_;
    int right_motor_PDO = RXPDO3 + right_motor_id_;

    // int16_t max_speed = 350;
    // int16_t min_speed = -350;

    int32_t left_rpm_tmp = (left_ppr * 60) / PPR * 0.014298324;
    int32_t right_rpm_tmp = (right_ppr * 60) / PPR * 0.014298324;

    // if (left_rpm_tmp > max_speed){
    //     left_rpm_tmp = max_speed;}
    // else if (left_rpm_tmp < min_speed){
    //     left_rpm_tmp = min_speed;}

    // if (right_rpm_tmp > max_speed){
    //     right_rpm_tmp = max_speed;}
    // else if (right_rpm_tmp < min_speed){
    //     right_rpm_tmp = min_speed;}

    uint32_t left_rpm_de = -left_rpm_tmp;
    uint32_t right_rpm_de = -right_rpm_tmp;
    // uint16_t left_rpm_de = - (left_ppr * 60) / PPR;
    // uint16_t right_rpm_de = - (right_ppr * 60) / PPR;     // 속도 check용

    left_rpm_cmd[CMD_PPR_BYTE1] = CAN_32BYTE1(left_rpm_de);
    left_rpm_cmd[CMD_PPR_BYTE2] = CAN_32BYTE2(left_rpm_de);
    // left_rpm_cmd[CMD_PPR_BYTE3] = CAN_32BYTE3(left_ppr);
    // left_rpm_cmd[CMD_PPR_BYTE4] = CAN_32BYTE4(left_ppr);

    // right_rpm_cmd[CMD_PPR_BYTE1] = CAN_32BYTE1(right_ppr);
    // right_rpm_cmd[CMD_PPR_BYTE2] = CAN_32BYTE2(right_ppr);
    left_rpm_cmd[CMD_PPR_BYTE3] = CAN_32BYTE1(right_rpm_de);
    left_rpm_cmd[CMD_PPR_BYTE4] = CAN_32BYTE2(right_rpm_de);

    CAN_handler_->make_frame(left_motor_PDO, left_rpm_cmd);
    // CAN_handler_->write_frame();
    // CAN_handler_->make_frame(right_motor_PDO, right_rpm_cmd);
    CAN_handler_->write_frame();
}


// call back fucntion for cmd_rpm
void MotorDriver::cmd_rpm_callback(const ip200_msgs::RobotMotor::ConstPtr &msg){
    int cmd_left_rpm = msg->left; 
    int cmd_right_rpm = msg->right;

    left_PPR_  = (cmd_left_rpm * PPR) / 60;
    right_PPR_ = (cmd_right_rpm * PPR) / 60;
}

// read Packet from Motor Driver CAN Node and extract RPM value
void MotorDriver::read_PPR_callback(const can_msgs::Frame::ConstPtr &msg)
{
    if(node_up_)
    {   
        can_msgs::Frame CAN_msg = *msg;   
        int left_pps; 
        int right_pps; 
        
        if (CAN_msg.id == TXPDO3 + left_motor_id_)
        {  
            left_pps = CAN_32MAKEWORD(CAN_msg.data[VEL_PPR_BYTE1], 
                                          CAN_msg.data[VEL_PPR_BYTE2], 
                                          CAN_msg.data[VEL_PPR_BYTE3], 
                                          CAN_msg.data[VEL_PPR_BYTE4]);
            left_RPM_ = float(left_pps) / PPR * 60;
        }
        else if (CAN_msg.id == TXPDO3 + right_motor_id_)
        {
            right_pps = CAN_32MAKEWORD(CAN_msg.data[VEL_PPR_BYTE1], 
                                           CAN_msg.data[VEL_PPR_BYTE2], 
                                           CAN_msg.data[VEL_PPR_BYTE3], 
                                           CAN_msg.data[VEL_PPR_BYTE4]);
            right_RPM_ = float(right_pps) / PPR * 60;
        }
    }
}

// pulish real rpm value 
void MotorDriver::rpm_pub(const ros::TimerEvent &e){
    if(node_up_){
        ip200_msgs::RobotMotor rpm_msg;
        // rpm_msg.left = left_RPM_;
        // rpm_msg.right = right_RPM_;
        rpm_msg.left = float(left_PPR_) * 60 / PPR;     // for publishing /cur_rpm topic
        rpm_msg.right = float(right_PPR_) * 60 / PPR;
        rpm_pub_.publish(rpm_msg);
        //ros::Duration(0.001).sleep();
    }
}

// publish SYNC CAN Packet command to ROS CAN bridge node
void MotorDriver::CAN_sync_pub(const ros::TimerEvent &e){
    if(node_up_){
        CAN_sync_->make_frame(SYNC, SYNC_ALL_NODE_FRAME);
        CAN_sync_->write_frame();
    }
}

// publish RPM Packet commands to ROS CAN bridge node
void MotorDriver::CAN_cmd_rpm_pub(const ros::TimerEvent &e){
    if(node_up_){
        writePPR(left_PPR_, right_PPR_);
        // writePPR(left_RPM_, right_RPM_);
    }
}

// initialize ROS Node
bool MotorDriver::init(std::string aidlrobot)
{
    ROS_INFO("Initializing motor driver");
    ros::Duration(0.5).sleep();
    // set_node_mode_(RESET_ALL_NODE_);  // NMT reset all node
    // ros::Duration(0.2).sleep();
    // set_node_mode_(START_ALL_NODE_);  // NMT start all node

    ros::Duration(0.2).sleep();
    set_node_mode_(SET_PDO);
    ros::Duration(0.2).sleep();
    set_node_mode_(FIRST_PDO);
    ros::Duration(0.2).sleep();
    set_node_mode_(SECOND_PDO);
    ros::Duration(0.2).sleep();
    set_node_mode_(THIRD_PDO);
    ros::Duration(0.2).sleep();
    set_node_mode_(START_PDO);

    ros::Duration(0.2).sleep();
    set_node_mode_(PROFILE_LEFT_ACC);
    ros::Duration(0.2).sleep();
    set_node_mode_(PROFILE_RIGHT_ACC);
    ros::Duration(0.2).sleep();
    set_node_mode_(PROFILE_LEFT_DACC);
    ros::Duration(0.2).sleep();
    set_node_mode_(PROFILE_RIGHT_DACC);

    ros::Duration(0.5).sleep();
    set_node_mode_(READY_2_SWICH);    // set control word
    ros::Duration(0.5).sleep();
    set_node_mode_(SWITCH_ON);        // SWITCH ON
    ros::Duration(0.5).sleep();
    set_node_mode_(OP_ENABLE);        // Operation ENABLE
    ros::Duration(0.5).sleep();
    // set_profile_acc_(ACC_DACC_VALUE); // acc & dacc set
    // ros::Duration(0.5).sleep();

    node_up_ = true;
    return true;
}

// end up ROS Node
bool MotorDriver::endup(){
    ROS_INFO("Endup motor driver");
    set_node_mode_(OP_DISENABLE);
    ros::Duration(0.2).sleep();
    set_node_mode_(READY_2_SWICH);
    ros::Duration(0.2).sleep();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle nh;

    MotorDriver motor_driver = MotorDriver(&nh);

	ros::AsyncSpinner spinner(0);
    motor_driver.init(aidl);
    spinner.start();

    ROS_INFO("ip200 Driver Node is started");
	ros::waitForShutdown();

    return 0;
}