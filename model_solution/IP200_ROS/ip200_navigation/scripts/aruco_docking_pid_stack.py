#!/usr/bin/env python3

import rospy
import math
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ip200_msgs.msg import ArucoMarker

class DockingController:
    def __init__(self):

        self.waypoints = [
            {'position': {'x': 0.0, 'y': 0.0}, 'orientation': {'z': 0.0, 'w': 0.0}}
        ]
        self.get_location()

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        # self.target_x = 400  # target x position (image center)
        self.target_x = 400  # target x position (image center)
        self.target_y = 470  # target y position (image bottom)
        self.aruco_center_x = 0
        self.aruco_center_y = 0
        self.last_aruco_center_y = 0
        self.id = 0
        self.last_id = int(0)
        self.last_last_id = int(0)

        # tune Kp, Ki, Kd gains
        self.kp_linear = 0.0004  # proportional gain
        self.kp_ang = 0.001
        self.ki = 0.0  # integral gain
        self.kd = 0.0  # derivative gain

        self.error_x = 0
        self.previous_error_x = 0
        self.integral_x = 0

        self.error_y = 0
        self.previous_error_y = 0
        self.integral_y = 0

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.aruco_sub = rospy.Subscriber('/aruco', ArucoMarker, self.update_aruco_info)

        self.rotate_velocity = rospy.get_param('~rotate_velocity', 0.2)
        self.rotate_error_threshold = rospy.get_param('~rotate_error_threshold', 5)
        self.linear_velocity = rospy.get_param('~linear_velocity', 0.05)  # 0.1m/s
        self.camera_height = rospy.get_param('~camera_height', 0.1)  # 0.1m
        self.rate = rospy.Rate(10)

        rospy.loginfo('Start!')

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
    
    def make_cmd_vel(self, lin_x, ang_z):
        msg = Twist()
        msg.linear.x = lin_x  # forward / backward (output_y)
        msg.angular.z = ang_z  # left / right (output_x)
        return msg
    
    def odom_callback(self, odom_msg):
        self.current_pos = odom_msg.pose.pose.position

    def clear_costmap(self):
        try:
            self.clear_costmaps_service()
            rospy.loginfo("Successfully clear costmap.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def move_to_docking_station(self):
        self.clear_costmap()
            
        goal = self.move_base_goal(self.waypoints[0])
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"목적지에 도착: {self.waypoints[0]}")
            self.clear_costmap()
        else:
            rospy.logerr(f"이동 실패: {self.waypoints[0]}")

    def rotate_to_find_aruco(self, rotate_velocity):
        vel_msg = self.make_cmd_vel(lin_x = 0.0, ang_z = rotate_velocity)
        
        self.cmd_pub.publish(vel_msg)
        rospy.loginfo(f'Rotate_velocity : {rotate_velocity}')
        # self.rate.sleep()
        
    def go_straight_to_docking(self, distance=0.2, threshold=0.05):
        vel = self.linear_velocity
        # remain_distance = math.sqrt((float(self.aruco_depth)*0.001)**2 - self.camera_height**2)
        print(math.sqrt((float(self.aruco_depth)*0.001)**2 - self.camera_height**2))

        remain_distance = distance  # 0.5m
        rospy.loginfo(f'remain distance: {remain_distance}m')

        initial_position = (self.current_pos.x, self.current_pos.y)
        target_distance = remain_distance

        # 전진
        vel_msg = self.make_cmd_vel(vel, 0.0)

        # target_distance = self.current_pos.x + remain_distance - (vel/4)

        # while self.current_pos.x < target_distance and not rospy.is_shutdown():
        while not rospy.is_shutdown():
            current_distance = math.sqrt((self.current_pos.x - initial_position[0])**2 + (self.current_pos.y - initial_position[1])**2)
            if target_distance - current_distance < threshold:  # Using a threshold value to determine if the robot is close enough
                break

            self.cmd_pub.publish(vel_msg)
            self.rate.sleep()
            # rospy.loginfo(f"Move forward: {target_distance - self.current_pos.x}")
            rospy.loginfo(f"Move forward: {current_distance}")

        vel_msg = self.make_cmd_vel(0.0, 0.0)
        self.cmd_pub.publish(vel_msg)
        rospy.sleep(1.0)
        
    def update_aruco_info(self, aruco_msg):
        self.id = aruco_msg.id
        self.aruco_center_x = aruco_msg.x
        self.aruco_center_y = aruco_msg.y
        if aruco_msg.depth != 0:
            self.aruco_depth = aruco_msg.depth

        self.error_x = self.target_x - aruco_msg.x
        self.integral_x += self.error_x

        self.error_y = self.target_y - aruco_msg.y
        self.integral_y += self.error_y

    def id_disappear(self):
        current_id = self.id
        if self.last_id==23 and current_id==0 and (self.last_aruco_center_y > 420) and (self.aruco_center_y==0):
            self.last_aruco_center_y = self.aruco_center_y
            self.last_id = current_id
            return True
        else:
            self.last_aruco_center_y = self.aruco_center_y
            self.last_id = current_id
            return False
        
    def is_aruco(self):
        if self.id == 23:
            print(self.id)
            return True
        else:
            return False
    
    def control_loop(self):
        self.rate.sleep()  # 20 Hz
        control_stack = [] # (output_x, output_y) stack에 쌓을 것임

        # aruco 마커 찾기 위해서 회전
        while not rospy.is_shutdown():
            if self.is_aruco():
                self.rotate_to_find_aruco(0.0)
                rospy.loginfo('Aruco is found')
                rospy.loginfo('loop start')
                break
            else:
                rospy.loginfo('No Aruco marker')
                self.rotate_to_find_aruco(self.rotate_velocity)
        
        # 찾았으면 방향 맞추면서 docking 시작
        while not rospy.is_shutdown():
            # if miss aruco
            if self.error_x == self.target_x:
                self.error_x = 0
            if self.error_y == self.target_y:
                self.error_y = 0

            # PID for x
            output_x = self.kp_ang*self.error_x # + self.ki*self.integral_x + self.kd*derivative_x
            if abs(output_x) < 0.0075:
                output_x = 0.0
            
            # PID for y
            output_y = self.kp_linear*self.error_y  # + self.ki*self.integral_y + self.kd*derivative_y
            
            # 앞으로 가다가 없어지면 aruco 마커 없어지면 멈춤
            if self.id_disappear():
                output_y, output_x = 0.0, 0.0
                cmd_msg = self.make_cmd_vel(lin_x=output_y, ang_z=output_x)      
                self.cmd_pub.publish(cmd_msg)
                break
            else:
                control_stack.append((output_x, output_y))  # 스택에 추가
                cmd_msg = self.make_cmd_vel(lin_x=output_y, ang_z=output_x)
                self.cmd_pub.publish(cmd_msg)
            
            self.rate.sleep()
        
        # stack 안의 값으로 로봇을 역으로 제어
        rospy.loginfo('Go back')
        while control_stack:
            output_x, output_y = control_stack.pop()
            cmd_msg = self.make_cmd_vel(lin_x=-output_y, ang_z=output_x)
            self.cmd_pub.publish(cmd_msg)
            self.rate.sleep()

        # 다시 앞으로 맞추면서 전진
        while not rospy.is_shutdown():
            # if miss aruco
            if self.error_x == self.target_x:
                self.error_x = 0
            if self.error_y == self.target_y:
                self.error_y = 0

            # PID for x
            output_x = self.kp_ang*self.error_x # + self.ki*self.integral_x + self.kd*derivative_x
            if abs(output_x) < 0.0075:
                output_x = 0.0
            
            # PID for y
            output_y = self.kp_linear*self.error_y  # + self.ki*self.integral_y + self.kd*derivative_y
            
            # 앞으로 가다가 없어지면 aruco 마커 없어지면 멈춤
            if self.id_disappear():
                output_y, output_x = 0.0, 0.0
                cmd_msg = self.make_cmd_vel(lin_x=output_y, ang_z=output_x)      
                self.cmd_pub.publish(cmd_msg)
                break
            else:
                cmd_msg = self.make_cmd_vel(lin_x=output_y, ang_z=output_x)
                self.cmd_pub.publish(cmd_msg)
            
            self.rate.sleep()

            
    def main(self):
        self.move_to_docking_station()
        rospy.sleep(1.0)
        self.control_loop()
        rospy.loginfo('Loop finished')
        rospy.sleep(4.0)
        self.go_straight_to_docking(distance=0.2, threshold=0.05)
        rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('docking_controller')
    dc = DockingController()
    dc.main()
    rospy.spin()