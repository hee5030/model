#!/usr/bin/env python3

import rospy
import math
import dynamic_reconfigure.client
from std_msgs.msg import Int64


# Set move_base/max_vel_x when object is coming close.
# For ModelSolution Project

class ControlVelocity:
    def __init__(self):
        self.depth_threshold = rospy.get_param('~depth_threshold', 800)  # 750mm = 0.75m
        self.count_threshold = rospy.get_param('~count_threshold', 1)  # detect at least () times
        self.update_max_vel_x = rospy.get_param('~update_max_vel_x', 0.1)  # m/s
        self.update_min_vel_x = rospy.get_param('~update_min_vel_x', 0.1)  # m/s
        self.default_max_vel_x = rospy.get_param('move_base/DWAPlannerROS/max_vel_x')
        self.default_min_vel_x = rospy.get_param('move_base/DWAPlannerROS/min_vel_x')
        rospy.loginfo(f'default_max_vel_x: {self.default_max_vel_x}')
        rospy.loginfo(f'update_max_vel_x: {self.update_max_vel_x}')

        self.camera_height = rospy.get_param('~camera_height', 0.2)  # 0.2m
        self.robot_length = rospy.get_param('~robot_length', 0.5)    # 0.5m
        self.count = 0
        self.distance = 0.0

        try:
            self.dyn_reconf_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS', timeout=30)
            rospy.loginfo('Dynamic_reconfigure Start!')
        except:
            rospy.loginfo('Move_base is not opened! Please check it first')

        self.depth_sub = rospy.Subscriber('depth', Int64, self.depth_callback)
        rospy.loginfo('Listening depth of object')
        
    def depth_callback(self, depth):
        if 0 < depth.data < self.depth_threshold:
            self.count += 1

            if self.count == self.count_threshold:
                rospy.loginfo(f'Depth: {depth.data}, Count: {self.count}')
                self.dyn_reconf_client.update_configuration({'max_vel_x': self.update_max_vel_x})
                self.distance = math.sqrt((depth.data/1000)**2 - self.camera_height**2)  # distance -> 0.Xm
                # self.dyn_reconf_client.update_configuration({'min_vel_x': self.update_min_vel_x})
                rospy.loginfo(f'Update max velocity from {self.default_max_vel_x} to {self.update_max_vel_x}')

                now = rospy.get_rostime()
                while (rospy.get_rostime() - now).to_sec() < ((self.distance+(self.robot_length/2)) / self.update_max_vel_x):  # 1m / (0.2m/s) -> 5s
                    pass

                self.dyn_reconf_client.update_configuration({'max_vel_x': self.default_max_vel_x})
                rospy.loginfo(f'Update max velocity from {self.update_max_vel_x} to {self.default_max_vel_x}')
                self.count = 0

if __name__ == "__main__":
    rospy.init_node('control_velocity_node')
    cv = ControlVelocity()
    rospy.spin()
