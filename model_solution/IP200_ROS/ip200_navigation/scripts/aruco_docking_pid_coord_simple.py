#!/usr/bin/env python3

# move_base goal로 이동해서 aruco 마커 정중앙으로 맞추면서 들어감

import rospy
import math
import tf
import actionlib
import numpy as np

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
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
        self.docking_service = rospy.Service('docking_service', Empty, self.handle_docking_request)
        
        self.target_x = 0.02  # target x position (aruco의 tvec_x: aruco 마커의 중심으로 맞추기 위함)
        self.target_z = 0.29  # target z position (aruco의 tvec_z: aruco 마커 가까이 가게하기 위함)

        self.id = 0
        self.last_id = int(0)

        self.rvec_x = 0.0
        self.rvec_y = 0.0
        self.rvec_z = 0.0

        self.tvec_x = 0.0
        self.tvec_y = 0.0
        self.tvec_z = 0.0
        
        self.last_yaw = 0.0
        self.rotating = False
        self.angle_tolerance = 0.05

        # tune Kp, Ki, Kd gains
        self.kp_linear = 0.35  # proportional gain
        self.kp_ang = 1.5
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
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        self.rotate_velocity = rospy.get_param('~rotate_velocity', 0.1)
        self.rotate_error_threshold = rospy.get_param('~rotate_error_threshold', 5)
        self.linear_velocity = rospy.get_param('~linear_velocity', 0.05)  # 0.05m/s
        self.camera_height = rospy.get_param('~camera_height', 0.1)  # 0.1m (실제 카메라의 height와 다름)
        self.rate = rospy.Rate(10)

        rospy.loginfo('Start!')

    def amcl_pose_callback(self, data):
        # 현재 로봇의 방향 저장
        self.current_orientation = (data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                    data.pose.pose.orientation.z, data.pose.pose.orientation.w)

    def calculate_angle_difference(self):
        # goal 도착 후 조금씩 회전하지 않고 aruco 정면 바라보도록 계산 
        # 원하는 방향 저장
        desired_orientation = (0, 0, self.waypoints[0]['orientation']['z'], self.waypoints[0]['orientation']['w'])

        # 쿼터니언 -> 오일러 변환 후 남은 각도 차이 계산
        angle_difference = tf.transformations.euler_from_quaternion(self.current_orientation)[2] - \
                           tf.transformations.euler_from_quaternion(desired_orientation)[2]
        
        return angle_difference

    def rotate_by_angle_difference(self, angle_difference):
        self.target_angle = self.current_angle + angle_difference
        rospy.loginfo('================')
        rospy.loginfo(f'current_angle: {self.current_angle}')
        rospy.loginfo(f'target_angle: {self.target_angle}')
        cmd = Twist()

        if angle_difference > 0:
            cmd.angular.z = 0.2  # 반시계방향 회전 -> z>0
        else:
            cmd.angular.z = -0.2  # 시계방향 회전 -> z<0

        self.rotating = True

        while self.rotating and not rospy.is_shutdown():
            # 목표 각도와 현재 각도의 차이가 tolerance보다 작으면 멈춤
            if abs(self.current_angle - self.target_angle) < self.angle_tolerance:
                cmd = Twist()
                self.cmd_pub.publish(cmd)
                self.rotating = False
            else:
                self.cmd_pub.publish(cmd)

            self.rate.sleep()

    def handle_docking_request(self, request):
        # 배터리가 부족하면 main함수 실행하는 service callback 함수
        rospy.loginfo('Docking started!')
        self.main()
        rospy.loginfo('Done!')
        return []
    
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
        msg.linear.x = lin_x  # forward / backward (output_z)
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
    
    def go_straight_to_find_aruco(self, linear_velocity):
        count = 0
        while not rospy.is_shutdown():
            if self.is_aruco():
                # 정지
                vel_msg = self.make_cmd_vel(lin_x = 0.0, ang_z = 0.0)
                self.cmd_pub.publish(vel_msg)
                rospy.loginfo('Aruco is found')
                break
            if 0 <= count < 10:
                # 앞으로 전진
                vel_msg = self.make_cmd_vel(lin_x = linear_velocity, ang_z = 0.0)
                self.cmd_pub.publish(vel_msg)
                rospy.loginfo(f'Linear_velocity : {linear_velocity}')
            elif 10 <= count < 20:
                # 뒤로 후진
                vel_msg = self.make_cmd_vel(lin_x = -linear_velocity, ang_z = 0.0)
                self.cmd_pub.publish(vel_msg)
                rospy.loginfo(f'Linear_velocity : {-linear_velocity}')

            count += 1
            if count >= 20:
                count = 0
            self.rate.sleep()
        
    def go_straight_to_docking(self, distance=0.2, threshold=0.05):
        vel = self.linear_velocity

        remain_distance = distance  # 0.5m
        rospy.loginfo(f'remain distance: {remain_distance}m')

        initial_position = (self.current_pos.x, self.current_pos.y)
        target_distance = remain_distance

        # 전진
        vel_msg = self.make_cmd_vel(vel, 0.0)

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
        self.docking_status = "Docked"
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

        # rvec_z가 (+)일 때 반시계방향 회전, (-)일 때 시계방향 회전 (의 반대)
        if positive_count > negative_count:
            direction = "clockwise"
        else:
            direction = "counterclockwise"
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

    def check_rvec_z_to_ok(self, num):
        count = 0
        ok = 0
        while count < num:
            if abs(self.rvec_z) < 2.5:
                ok += 1
                count += 1
                rospy.loginfo(f'ok: {ok}')

        if ok >= num / 2:
            return False
        else:
            return True

    def control_loop(self):
        self.rate.sleep()  # 20 Hz
        control_stack = [] # (output_x, output_z) stack에 쌓을 것임
        small_angular_threshold = 0.05
        stack_threshold = 0.3

        ## aruco 마커 찾기 위해서 회전 (전진으로 바꿔야 함)
        # while not rospy.is_shutdown():
        #     if self.is_aruco():
        #         self.rotate_to_find_aruco(0.0)
        #         rospy.loginfo('Aruco is found')
        #         rospy.loginfo('loop start')
        #         rospy.sleep(1.0)
        #         break
        #     else:
        #         rospy.loginfo('No Aruco marker')
        #         self.rotate_to_find_aruco(self.rotate_velocity)
        self.go_straight_to_find_aruco(linear_velocity=0.05)  # 전진

        while not rospy.is_shutdown(): # rvec_z의 절대값이 3보다 작거나 같은 동안 반복
            rospy.loginfo('Go Front')

            # 찾았으면 방향 맞추면서 docking 시작
            while not rospy.is_shutdown():
                # if miss aruco
                if self.error_x == self.target_x:
                    self.error_x = 0.0
                if self.error_z == self.target_z:
                    self.error_z = 0.1

                # PID for tvec_x
                output_x = self.kp_ang*self.error_x
                
                # PID for tvec_z
                output_z = self.kp_linear*self.error_z

                # 전진 속도 너무 빠르지 않도록 설정
                if output_z > 0.1:
                    output_z = 0.1
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
                    # output_x(회전값) 너무 크면 제한함
                    if abs(output_x) < stack_threshold:
                        control_stack.append((output_x, output_z))
                    else:
                        control_stack.append((stack_threshold, output_z))
                    # control_stack.append((output_x, output_z))  # 스택에 추가
                    cmd_msg = self.make_cmd_vel(lin_x=output_z, ang_z=output_x)
                    self.cmd_pub.publish(cmd_msg)
                self.rate.sleep()

            # aruco 마커의 z축이 몇 degree 이상 틀어지면 반시계 방향/ 시계 방향으로 후진할지 정하고 후진
            # 전진하면서 줬던 속도 명령을 stack에 저장했다가 역으로 coeff (0.9, 1.3)를 곱하면서 후진
            direction = self.rotate_clockwise_or_counter()

            if abs(self.rvec_z) > 3.0:
                rospy.loginfo(f'Degree of Z: {self.rvec_z}')
                # 모든 angular.z 값들이 기준값보다 작은지 체크하고 coeff 조절
                all_small_angular = all(abs(val[0]) < small_angular_threshold for val in control_stack)
                if all_small_angular:
                    ang_z_coeff = 1.7
                else:
                    ang_z_coeff = 1.3
                
                # # aruco 마커의 z축 차이만큼 회전한 다음에 뒤로 후진
                # self.rotate_by_angle_difference(2 * -self.rvec_z)
                # ang_z_coeff = 0.0


                rospy.loginfo('Go back')
                while control_stack:
                    output_x, output_y = control_stack.pop()
                    if direction == "counterclockwise":
                        if output_x < 0.0:
                            output_x = 0.0
                    elif direction == "clockwise":
                        if output_x > 0.0:
                            output_x = 0.0
                    rospy.loginfo(f'ang_z: {output_x}')

                    # cmd_msg = self.make_cmd_vel(lin_x=-0.9*output_y, ang_z=1.3*output_x)
                    cmd_msg = self.make_cmd_vel(lin_x=-0.9*output_y, ang_z=ang_z_coeff*output_x)
                    self.cmd_pub.publish(cmd_msg)
                    self.rate.sleep()
            else:
                rospy.loginfo(f'rvec_z: {self.rvec_z}')
                break
            
            # 다시 aruco 마커 찾음
            if direction == "counterclockwise":
                rot_velocity = self.rotate_velocity
            elif direction == "clockwise":
                rot_velocity = -self.rotate_velocity

            while not rospy.is_shutdown():
                if self.is_aruco():
                    self.rotate_to_find_aruco(0.0)
                    rospy.loginfo('Aruco is found')
                    rospy.loginfo('loop start')
                    break
                else:
                    rospy.loginfo('No Aruco marker')
                    self.rotate_to_find_aruco(rot_velocity)

        rospy.loginfo('Loop finished')

    def main(self):
        self.get_location()
        self.move_to_docking_station()
        angle_difference = self.calculate_angle_difference()
        self.rotate_by_angle_difference(angle_difference)
        rospy.sleep(1.0)
        self.control_loop()
        rospy.sleep(2.0)
        self.go_straight_to_docking(distance=self.tvec_z, threshold=-0.02)
        rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('docking_controller')
    dc = DockingController()
    # dc.main()
    rospy.spin()