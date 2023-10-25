#!/usr/bin/env python3

import rospy
import math
import actionlib
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion
from ip200_msgs.msg import ArucoMarkerVector

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
        self.target_x = 0.0  # target x position (image center)
        self.target_z = 0.3  # target y position (image bottom)

        self.id = 0
        self.last_id = int(0)
        self.last_last_id = int(0)

        self.rvec_x = 0.0
        self.rvec_y = 0.0
        self.rvec_z = 0.0

        self.tvec_x = 0.0
        self.tvec_y = 0.0
        self.tvec_z = 0.0
        
        self.last_yaw = 0.0

        # tune Kp, Ki, Kd gains
        self.kp_linear = 0.4  # proportional gain
        self.kp_ang = 1.3
        self.ki = 0.0  # integral gain
        self.kd = 0.0  # derivative gain

        self.error_x = 0
        self.previous_error_x = 0
        self.integral_x = 0

        self.error_z = 0
        self.previous_error_z = 0
        self.integral_z = 0

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.aruco_sub = rospy.Subscriber('/aruco', ArucoMarkerVector, self.update_aruco_info)

        self.rotate_velocity = rospy.get_param('~rotate_velocity', 0.1)
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

        _, _, yaw = euler_from_quaternion([
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        ])
        self.current_angle = yaw

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
        # print(math.sqrt((float(self.aruco_depth)*0.001)**2 - self.camera_height**2))

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
        rospy.loginfo('Forward Done!')
        rospy.sleep(1.0)
        
    def update_aruco_info(self, aruco_msg):
        self.id = aruco_msg.id
        if aruco_msg.rvec.x != 0.0:
            self.rvec_x = aruco_msg.rvec.x
            self.rvec_y = aruco_msg.rvec.y
            self.rvec_z = aruco_msg.rvec.z

            self.tvec_x = aruco_msg.tvec.x
            self.tvec_y = aruco_msg.tvec.y
            self.tvec_z = aruco_msg.tvec.z

        self.error_x = self.target_x - aruco_msg.tvec.x
        # self.integral_x += self.error_x

        self.error_z = abs(self.target_z - aruco_msg.tvec.z)
        # self.integral_y += self.error_y

    def close_enough_to_station(self):
        if self.tvec_z < 0.32:
            return True
        else:
            return False
        
    def is_aruco(self):
        if self.id == 23:
            print(self.id)
            return True
        else:
            return False
    
    def rotate_clockwise_or_counter(self):
        direction = ""
        rvec_z_list = []
        count = 0

        while count < 30:
            rvec_z_list.append(self.rvec_z)
            count += 1
            rospy.sleep(0.1)

         # 양수와 음수의 빈도 계산
        positive_count = sum(1 for val in rvec_z_list if val > 0)
        negative_count = sum(1 for val in rvec_z_list if val < 0)

        # rvec_z가 (+)일 때 반시계방향 회전, (-)일 때 시계방향 회전
        if positive_count > negative_count:
            direction = "counterclockwise"
        else:
            direction = "clockwise"
        rospy.loginfo(f'direction: {direction}')

        return direction

    def calculate_rotation_degree(self, direction):
        # 반시계 105, 시계 70 기준 
        angle_xz = np.degrees(np.arctan2(self.tvec_z, self.tvec_x))
        rospy.loginfo(f'angle_xz: {angle_xz}')

        if direction == "counterclockwise":
            if angle_xz < 105:
                return 90 - (105 - angle_xz)
            else:
                return 90 + (angle_xz - 105)
        elif direction == "clockwise":
            if angle_xz > 70:
                return 90 - (angle_xz - 70)
            else:
                return 90 + (70 - angle_xz)

    def rotate(self, direction, target_angle_degree):
        vel_msg = Twist()
        rate = rospy.Rate(10)  # 10Hz

        target_angle = np.radians(target_angle_degree)
        initial_yaw = self.current_angle

        if direction == "counterclockwise":
            vel_msg.angular.z = 0.2
        elif direction == "clockwise":
            vel_msg.angular.z = -0.2

        rospy.loginfo(f'current_angle: {self.current_angle}')
        while abs(self.current_angle - initial_yaw) < abs(target_angle) and not rospy.is_shutdown():
            rospy.loginfo(f'remain: {abs(target_angle) - abs(self.current_angle - initial_yaw)}')
            self.cmd_pub.publish(vel_msg)
            rate.sleep()
        
        # 회전 정지
        vel_msg.angular.z = 0
        self.cmd_pub.publish(vel_msg)
        rospy.loginfo("Rotation done!")

    def control_loop(self):
        self.rate.sleep()  # 20 Hz
        control_stack = [] # (output_x, output_y) stack에 쌓을 것임

        # aruco 마커 찾기 위해서 회전 (전진으로 바꿔야 함)
        while not rospy.is_shutdown():
            if self.is_aruco():
                self.rotate_to_find_aruco(0.0)
                rospy.loginfo('Aruco is found')
                rospy.loginfo('loop start')
                break
            else:
                rospy.loginfo('No Aruco marker')
                self.rotate_to_find_aruco(self.rotate_velocity)
        
        # # 시계방향으로 돌아야하는지 반시계방향으로 돌아야 하는지 체크
        # direction = self.rotate_clockwise_or_counter()

        # # 몇 도 돌아야 하는지 계산
        # angle = self.calculate_rotation_degree(direction)
        # tvec_x = self.tvec_x
        # # 회전
        # self.rotate(direction, angle)
        # # 차이나는 만큼 전진
        # self.go_straight_to_docking(distance=tvec_x, threshold=0.0)
        # # 다시 aruco 마커를 향해 반대로 90도 회전
        # if direction == 'clockwise':
        #     self.rotate(direction='counterclockwise', target_angle_degree=90)
        # elif direction == 'counterclockwise':
        #     self.rotate(direction='clockwise', target_angle_degree=90)
        
        

        # 찾았으면 방향 맞추면서 docking 시작
        while not rospy.is_shutdown():
            # if miss aruco
            if self.error_x == self.target_x:
                self.error_x = 0.0
            if self.error_z == self.target_z:
                self.error_z = 0.0
            # PID for tvec_x
            output_x = self.kp_ang*self.error_x # + self.ki*self.integral_x + self.kd*derivative_x
            # if abs(output_x) < 0.005:
            #     output_x = 0.0
            
            # PID for tvec_z
            output_z = self.kp_linear*self.error_z  # + self.ki*self.integral_y + self.kd*derivative_y
            
            # rospy.loginfo(f'error_x: {self.error_x}')
            # rospy.loginfo(f'error_z: {self.error_z}')
            # rospy.loginfo(f'output_x: {output_x}')
            # rospy.loginfo(f'output_z: {output_z}')

            # 앞으로 가다가 없어지면 aruco 마커 없어지면 멈춤
            if self.close_enough_to_station():
                output_z, output_x = 0.0, 0.0
                # cmd_msg = self.make_cmd_vel(lin_x=output_z, ang_z=output_x)
                cmd_msg = self.make_cmd_vel(lin_x=output_z, ang_z=output_x)      
                self.cmd_pub.publish(cmd_msg)
                break
            else:
                # control_stack.append((output_x, output_z))  # 스택에 추가
                # cmd_msg = self.make_cmd_vel(lin_x=output_z, ang_z=output_x)
                cmd_msg = self.make_cmd_vel(lin_x=output_z, ang_z=output_x)
                self.cmd_pub.publish(cmd_msg)
            
            self.rate.sleep()
        
        # # stack 안의 값으로 로봇을 역으로 제어
        # rospy.loginfo('Go back')
        # while control_stack:
        #     output_x, output_y = control_stack.pop()
        #     cmd_msg = self.make_cmd_vel(lin_x=-output_y, ang_z=output_x)
        #     self.cmd_pub.publish(cmd_msg)
        #     self.rate.sleep()

        # # 다시 앞으로 맞추면서 전진
        # while not rospy.is_shutdown():
        #     # if miss aruco
        #     if self.error_x == self.target_x:
        #         self.error_x = 0
        #     if self.error_y == self.target_y:
        #         self.error_y = 0

        #     # PID for x
        #     output_x = self.kp_ang*self.error_x # + self.ki*self.integral_x + self.kd*derivative_x
        #     if abs(output_x) < 0.0075:
        #         output_x = 0.0
            
        #     # PID for y
        #     output_y = self.kp_linear*self.error_y  # + self.ki*self.integral_y + self.kd*derivative_y
            
        #     # 앞으로 가다가 없어지면 aruco 마커 없어지면 멈춤
        #     if self.id_disappear():
        #         output_y, output_x = 0.0, 0.0
        #         cmd_msg = self.make_cmd_vel(lin_x=output_y, ang_z=output_x)      
        #         self.cmd_pub.publish(cmd_msg)
        #         break
        #     else:
        #         cmd_msg = self.make_cmd_vel(lin_x=output_y, ang_z=output_x)
        #         self.cmd_pub.publish(cmd_msg)
            
        #     self.rate.sleep()

    def main(self):
        self.move_to_docking_station()
        rospy.sleep(1.0)
        self.control_loop()
        rospy.loginfo('Loop finished')
        rospy.sleep(4.0)
        self.go_straight_to_docking(distance=self.tvec_z, threshold=0.0)
        rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('docking_controller')
    dc = DockingController()
    dc.main()
    rospy.spin()