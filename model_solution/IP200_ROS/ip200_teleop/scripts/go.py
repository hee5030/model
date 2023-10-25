#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist

class RobotControl():
    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/onoff', UInt16, queue_size=4)
        self.rate = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            # 전진 2m
            move_cmd = Twist()
            move_cmd.linear.x = 0.2  # 0.2m/s
            move_cmd.angular.z = 0.0
            distance = 0.0
            while distance < 2.0:
                self.pub.publish(move_cmd)
                self.rate.sleep()
                distance += 0.2 * self.rate.sleep_dur.to_sec()  # 총 이동거리 계산
            move_cmd.linear.x = 0.0
            self.pub.publish(move_cmd)
            rospy.sleep(1.0)

            # 회전 180도
            move_cmd.angular.z = 0.5  # 0.5rad/s
            angle = 0.0
            while angle < 3.14:
                self.pub.publish(move_cmd)
                self.rate.sleep()
                angle += 0.2 * self.rate.sleep_dur.to_sec()  # 총 회전각도 계산
            move_cmd.angular.z = 0.0
            self.pub.publish(move_cmd)
            rospy.sleep(1.0)

            # lift 토픽 발행
            self.pub2.publish(1)
            rospy.sleep(1.0)
            self.pub2.publish(0)
            rospy.sleep(1.0)

if __name__ == '__main__':
    robot_control = RobotControl()
    robot_control.run()