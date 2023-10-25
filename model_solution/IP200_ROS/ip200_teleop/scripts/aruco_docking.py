#!/usr/bin/env python3

import rospy
import math
import actionlib
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from adl200_msgs.msg import ArucoMarker

class DockingController:
    def __init__(self):

        self.waypoints = [
            {'position': {'x': 0.0, 'y': 0.0}, 'orientation': {'z': 0.0, 'w': 0.0}}
        ]
        self.get_location()

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        self.target_x = 320  # target x position (image center)
        self.target_y = 480  # target y position (image bottom)
        self.aruco_center_x = 0
        self.aruco_center_y = 0
        self.id = int(23)
        self.last_id = int(0)

        # tune Kp, Ki, Kd gains
        self.kp = 0.001  # proportional gain
        self.ki = 0.00  # integral gain
        self.kd = 0.00  # derivative gain

        self.error_x = 0
        self.previous_error_x = 0
        self.integral_x = 0

        self.error_y = 0
        self.previous_error_y = 0
        self.integral_y = 0

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.aruco_sub = rospy.Subscriber('/aruco', ArucoMarker, self.update_aruco_info)
        self.rotate_velocity = rospy.get_param('~rotate_velocity', 0.03)
        self.rotate_error_threshold = rospy.get_param('~rotate_error_threshold', 5)
        self.linear_velocity = rospy.get_param('~linear_velocity', 0.1)
        self.camera_height = rospy.get_param('~camera_height', 0.3)
        self.rate = rospy.Rate(20)
        # self.control_loop()

    def get_location(self):
        for i in range(1):
            if i==0:
                self.waypoints[i]['position']['x'] = rospy.get_param('~params_docking/goal_position/x')
                self.waypoints[i]['position']['y'] = rospy.get_param('~params_docking/goal_position/y')
                self.waypoints[i]['orientation']['z'] = rospy.get_param('~params_docking/goal_orientation/z')
                self.waypoints[i]['orientation']['w'] = rospy.get_param('~params_docking/goal_orientation/w')

    def move_base_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint['position']['x']
        goal.target_pose.pose.position.y = waypoint['position']['y']
        goal.target_pose.pose.orientation.z = waypoint['orientation']['z']
        goal.target_pose.pose.orientation.w = waypoint['orientation']['w']
        return goal
    
    def move_to_docking_station(self, point):
        try:
            self.clear_costmaps_service()
            rospy.loginfo("Successfully clear costmap.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            

        goal = self.move_base_goal(self.waypoints[0])
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"목적지에 도착: {self.waypoints[0]}")
            try:
                self.clear_costmaps_service()
                rospy.loginfo("Successfully clear costmap.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        else:
            rospy.logerr(f"이동 실패: {self.waypoints[0]}")

    def make_cmd_vel(self, output_y, output_x):
        msg = Twist()
        # msg.linear.x = output_y  # forward / backward
        msg.angular.z = output_x  # left / right
        return msg
    
    def odom_callback(self, odom_msg):
        self.current_pos = odom_msg.pose.pose.position

    def update_aruco_info(self, aruco_msg):
        self.id = aruco_msg.id
        self.aruco_center_x = aruco_msg.x
        self.aruco_center_y = aruco_msg.y
        self.aruco_depth = aruco_msg.depth

        self.error_x = self.target_x - aruco_msg.x
        self.integral_x += self.error_x

        self.error_y = self.target_y - aruco_msg.y
        self.integral_y += self.error_y

    def rotate_to_docking(self):
        while not rospy.is_shutdown():
            vel_msg = Twist()
            if self.aruco_center_x > self.target_x:
                vel_msg.angular.z = -self.rotate_velocity
            else:
                vel_msg.angular.z = self.rotate_velocity

            error_x = abs(self.target_x - self.aruco_center_x)
            rospy.loginfo(f'error_x : {error_x}')

            if self.rotate_error_threshold < error_x:
                self.cmd_pub.publish(vel_msg)
            else:
                vel_msg.angular.z = 0.0
                self.cmd_pub.publish(vel_msg)
                rospy.loginfo('Rotation Finished')
                break
        rospy.sleep(1.0)

    def go_straight_to_docking(self):
        vel = self.linear_velocity
        remain_distance = math.sqrt(self.aruco_depth**2 - self.camera_height**2)

        # 전진
        vel_msg = Twist()
        vel_msg.linear.x = vel

        while self.current_pos.x < remain_distance - (vel/4) and not rospy.is_shutdown():
            self.cmd_pub.publish(vel_msg)
            self.rate.sleep()
            rospy.loginfo(f"Move forward: {self.current_pos.x}")

        vel_msg.linear.x = 0.0
        self.cmd_pub.publish(vel_msg)
        rospy.sleep(1.0)

    def control_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # PID for x
            derivative_x = self.error_x - self.previous_error_x
            output_x = self.kp*self.error_x + self.ki*self.integral_x + self.kd*derivative_x
            self.previous_error_x = self.error_x

            # PID for y
            derivative_y = self.error_y - self.previous_error_y
            output_y = self.kp*self.error_y + self.ki*self.integral_y + self.kd*derivative_y
            self.previous_error_y = self.error_y

            # 앞으로 가다가 없어지면 aruco 마커 없어지면 멈춤
            current_id = self.id
            if self.last_id==23 and current_id==0:
                output_y, output_x = 0.0, 0.0
            self.last_id = current_id

            cmd_msg = self.make_cmd_vel(output_x=output_x)      
            self.cmd_pub.publish(cmd_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('docking_controller')
    dc = DockingController()
    dc.move_to_docking_station()
    dc.rotate_to_docking()
    dc.go_straight_to_docking()
    rospy.spin()
