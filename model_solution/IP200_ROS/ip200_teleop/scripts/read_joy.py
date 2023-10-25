#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import UInt16
# Define a flag to keep track of the state
on = False
def callback(data):
    global on
    # Check the value of the first button
    if data.buttons[5] == 1:
        on = True
        rospy.loginfo("Up")
        pub2.publish(1)
    # Check the value of the second button
    elif data.buttons[7] == 1:
        on = False
        rospy.loginfo("Down")
        pub2.publish(0)
    elif data.buttons[0] == 1:
        on = False
        rospy.loginfo("Stop")
        pub2.publish(2)
def joy_subscriber():
    # Initialize the node
    rospy.init_node('joy_subscriber', anonymous=True)
    # Publish topic
    global pub2
    pub2 = rospy.Publisher('onoff', UInt16, queue_size =4)
    # Subscribe to the joy topic
    rospy.Subscriber("joy", Joy, callback)
    # Spin until the node is shutdown
    rospy.spin()
if __name__ == '__main__':
    joy_subscriber()