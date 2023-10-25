// _ip200_DRIVER_HPP_
#ifndef _ip200_DRIVER_HPP_
#define _ip200_DRIVER_HPP_

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <std_srvs/Trigger.h>
#include <ip200_msgs/RobotMotor.h>

#define PI 3.141592
#define WHELL_DIAMETER 0.130
// #define WHELL_WITH 0.4 // ip050
#define WHELL_WITH 0.45 // model_solution
#define GEAR_RATIO 70

namespace aidl
{

	class ip200BodyNode
	{
	private:
		enum
		{
			COMPUTE_ODOMETRY_FREQUENCY = 100
		};
		enum
		{
			PUBLISH_CURRENT_ODOMETRY_FREQUENCY = 100
		};
		enum
		{
			PUBLISH_CMD_RPM_FREQUENCY = 100
		};

		double x_, y_, th_, vx_, vy_, vth_;
		double cur_x_, cur_y_, cur_th_,
			cur_vx_, cur_vy_, cur_vth_;
		double delta_x_, delta_y_, delta_th_, delta_t_;
		double motor_left_rpm_, motor_right_rpm_;
		double publish_current_odometry_frequency_;

		double wheel_diameter_;
		double wheel_width_;

		ros::NodeHandle *nh_private_;

		ros::Publisher odometry_publisher_;
		ros::Publisher cmd_rpm_publisher_;
		ros::Subscriber cmd_velocity_Subscriber_;
		ros::Subscriber cur_rpm_Subscriber_;

		ros::ServiceServer reset_odometry_service_;

		ros::Timer compute_odometry_timer_, publish_odometry_timer_, publish_cmd_rpm_timer_;
		ros::Time current_time_, last_time_;

		bool broadcast_tf_;
		tf::TransformBroadcaster odom_broadcaster_;
		geometry_msgs::TransformStamped odom_trans_;
		nav_msgs::Odometry odom_;

	public:
		// constructor"
		ip200BodyNode(ros::NodeHandle *nh) : x_(0), y_(0), th_(0), delta_x_(0), delta_y_(0), delta_th_(0), vx_(0), vy_(0), vth_(0),
											   cur_x_(0), cur_y_(0), cur_th_(0), cur_vx_(0), cur_vy_(0), cur_vth_(0)
		{
			nh_private_ = nh;

			motor_left_rpm_ = 0;
			motor_right_rpm_ = 0;

			if (!ros::param::get("~broadcast_tf", broadcast_tf_))
				broadcast_tf_ = true;
			if (!ros::param::get("~wheel_diameter", wheel_diameter_))
				wheel_diameter_ = WHELL_DIAMETER;
			if (!ros::param::get("~wheel_width", wheel_width_))
				wheel_width_ = WHELL_WITH;
			if (!ros::param::get("~publish_current_odometry_frequency", publish_current_odometry_frequency_))
				publish_current_odometry_frequency_ = PUBLISH_CURRENT_ODOMETRY_FREQUENCY;

			odometry_publisher_ = nh_private_->advertise<nav_msgs::Odometry>("odom", 50);
			cmd_rpm_publisher_ = nh_private_->advertise<ip200_msgs::RobotMotor>("cmd_rpm", 50);

			cmd_velocity_Subscriber_ = nh_private_->subscribe(
				"cmd_vel", 50, &ip200BodyNode::cmd_vel_callback, this);
			cur_rpm_Subscriber_ = nh_private_->subscribe(
				"cur_rpm", 50, &ip200BodyNode::rpm_vel_callback, this);

			reset_odometry_service_ = nh_private_->advertiseService(
				"reset_odometry", &ip200BodyNode::reset_odometry_callback, this);
			compute_odometry_timer_ = nh_private_->createTimer(
				ros::Duration(1.0 / COMPUTE_ODOMETRY_FREQUENCY),
				&ip200BodyNode::odom_compute, this);
			publish_odometry_timer_ = nh_private_->createTimer(
				ros::Duration(1.0 / publish_current_odometry_frequency_),
				&ip200BodyNode::odom_publish, this);
			publish_cmd_rpm_timer_ = nh_private_->createTimer(
				ros::Duration(1.0 / PUBLISH_CMD_RPM_FREQUENCY),
				&ip200BodyNode::cmd_rpm_velocity_publish, this);
		}

		// destructo
		~ip200BodyNode() {}

		// callback of cmd_vel msg
		void cmd_vel_callback(const geometry_msgs::Twist &msg)
		{

			double twist_x = msg.linear.x;
			double twist_th = msg.angular.z;

			vx_ = twist_x;
			vth_ = twist_th;
		}

		// rpm callback. convert rpm to twist velocity
		void rpm_vel_callback(const ip200_msgs::RobotMotor &msg)
		{
			// velocity = (rpm / 60) * PI * diameter

			double rpm_left = msg.left;
			double rpm_right = msg.right;

			double cur_left = 0;
			double cur_right = 0;

			cur_left = (rpm_left / 60) * PI * wheel_diameter_ / GEAR_RATIO;
			cur_right = -1 * ((rpm_right / 60) * PI * wheel_diameter_ / GEAR_RATIO);

			cur_vx_ = (cur_left + cur_right) / 2;
			cur_vy_ = 0;
			cur_vth_ = (cur_right - cur_left) / wheel_width_;
		}

		// service
		bool reset_odometry_callback(std_srvs::Trigger::Request &req,
									 std_srvs::Trigger::Response &res)
		{
			nav_msgs::Odometry odom;
			geometry_msgs::TransformStamped odom_trans;
			x_ = 0.0;
			y_ = 0.0;
			th_ = 0.0;

			odom_ = odom;
			odom_trans_ = odom_trans;
			res.success = true;
			res.message = "Reset Odometry Success";
			return false;
		}

		// publish rpm velcity to motor driver node
		void cmd_rpm_velocity_publish(const ros::TimerEvent &e)
		{
			// [m/s]
			ip200_msgs::RobotMotor motor_msg;

			double motor_right_vel = vx_ + vth_ * wheel_width_ / 2.0f;
			double motor_left_vel = vx_ - vth_ * wheel_width_ / 2.0f;

			// [RPM]
			// rpm = (velocity / (PI * diameter)) * 60
			motor_left_rpm_ = motor_left_vel * 60.0f / (wheel_diameter_ * PI) * GEAR_RATIO;
			motor_right_rpm_ = -1 * (motor_right_vel * 60.0f / (wheel_diameter_ * PI) * GEAR_RATIO);

			motor_msg.left = motor_left_rpm_;
			motor_msg.right = motor_right_rpm_;

			cmd_rpm_publisher_.publish(motor_msg);
		}

		void odom_publish(const ros::TimerEvent &e)
		{
			if (broadcast_tf_)
				odom_broadcaster_.sendTransform(odom_trans_);
			odometry_publisher_.publish(odom_);
		}

		void odom_compute(const ros::TimerEvent &e)
		{
			nav_msgs::Odometry odom;
			geometry_msgs::TransformStamped odom_trans;
			geometry_msgs::Quaternion odom_quat;

			current_time_ = ros::Time::now();

			delta_t_ = (current_time_ - last_time_).toSec();
			// delta_x_ = (0.55)*(cur_vx_ * cos(th_) - cur_vy_ * sin(th_)) * delta_t_;
			delta_x_ = (1)*(cur_vx_ * cos(th_) - cur_vy_ * sin(th_)) * delta_t_; // ip050
			// delta_x_ = (0.9376)*(cur_vx_ * cos(th_) - cur_vy_ * sin(th_)) * delta_t_; // model_solution
			delta_y_ = (cur_vx_ * sin(th_) + cur_vy_ * cos(th_)) * delta_t_;
			delta_th_ = (0.9694)*cur_vth_ * delta_t_;                                    // model_solution
			// delta_th_ = (1.1312)*cur_vth_ * delta_t_; // ip050
			// delta_th_ = (0.635)*cur_vth_ * delta_t_;


			x_ += delta_x_;
			y_ += delta_y_;
			th_ += delta_th_;

			odom_quat = tf::createQuaternionMsgFromYaw(th_);

			odom_trans.header.stamp = current_time_;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_footprint";

			odom_trans.transform.translation.x = x_;
			odom_trans.transform.translation.y = y_;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			odom.header.stamp = current_time_;
			odom.header.frame_id = "odom";
			odom.pose.pose.position.x = x_;
			odom.pose.pose.position.y = y_;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;
			odom.pose.covariance = (boost::array<double, 36UL>)
			{(1e-3), (0), (0), (0), (0), (0), 
			 (0), (1e-3), (0), (0), (0), (0), 
			 (0), (0), (1e-3), (0), (0), (0), 
			 (0), (0), (0), (1e-3), (0), (0), 
			 (0), (0), (0), (0), (1e-3), (0), 
			 (0), (0), (0), (0), (0), (3e2)};
			odom.child_frame_id = "base_footprint";
			odom.twist.twist.linear.x = cur_vx_;
			odom.twist.twist.linear.y = cur_vy_;
			odom.twist.twist.angular.z = cur_vth_;
			odom.twist.covariance = (boost::array<double, 36UL>)
			{(1e-3), (0), (0), (0), (0), (0), 
			(0), (1e-3), (0), (0), (0), (0), 
			(0), (0), (1e-3), (0), (0), (0), 
			(0), (0), (0), (1e-3), (0), (0), 
			(0), (0), (0), (0), (1e-3), (0), 
			(0), (0), (0), (0), (0), (3e2)};

			last_time_ = current_time_;

			odom_ = odom;
			odom_trans_ = odom_trans;
		}
	};

} // namespace

#endif // _ip200_DRIVER_HPP_
