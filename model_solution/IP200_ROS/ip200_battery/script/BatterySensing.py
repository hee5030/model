#!/usr/bin/env python3

# 배터리 총량이 29.5v이고 전압분배회로를 거쳐서 1/6.1강하, A0핀으로 전압체크
import rospy
from std_msgs.msg import Int32, Float32
from std_srvs.srv import Empty

class BatteryStatusListener:

    def __init__(self):
        self.prev_A0_msg = None
        self.voltage_msg = None
        self.percent_msg = None
        self.low_battery_flag = True
        self.full_battery_flag = True
        # self.low_battery_pub = rospy.Publisher("Low_battery", Float32, queue_size=10)
        # self.full_battery_pub = rospy.Publisher("Full_battery", Float32, queue_size=10)
        self.service_client = rospy.ServiceProxy('docking_service', Empty)

    # def A0_callback(self, A0_msg):
    #     if self.prev_A0_msg is not None and A0_msg.data > self.prev_A0_msg:
    #         if self.voltage_msg >= 29.3:
    #             rospy.loginfo("voltage: {:.3f}V , percent: {:.3f}%, full charging".format(self.voltage_msg, self.percent_msg))
    #         else:
    #             rospy.loginfo("voltage: {:.3f}V, percent: {:.3f}%, charging".format(self.voltage_msg, self.percent_msg))
    #     else:
    #         rospy.loginfo("voltage: {:.3f}V , percent: {:.3f}%".format(self.voltage_msg, self.percent_msg))

    #     self.prev_A0_msg = A0_msg.data
    def A0_callback(self, A0_msg):
        self.prev_A0_msg = A0_msg.data
        if self.voltage_msg is not None and self.percent_msg is not None:
            if self.prev_A0_msg is not None and A0_msg.data > self.prev_A0_msg:
                if self.voltage_msg >= 29.4:
                    rospy.loginfo("voltage: {:.3f}V , percentage: {:.3f}%, full charging".format(self.voltage_msg, self.percent_msg))
                else:
                    rospy.loginfo("voltage: {:.3f}V, percentage: {:.3f}%, charging".format(self.voltage_msg, self.percent_msg))
            else:
                rospy.loginfo("voltage: {:.3f}V , percentage: {:.3f}%".format(self.voltage_msg, self.percent_msg))
        else:
            rospy.loginfo("voltage_msg or percent_msg is None.")
    
    def voltage_callback(self, voltage_msg):
        self.voltage_msg = voltage_msg.data
        if self.voltage_msg <= 27.7 and self.low_battery_flag == True: #26.5
            try:
                self.service_client()
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: " + str(exc))

            self.low_battery_flag = False
        elif self.voltage_msg > 28.0:
            self.low_battery_flag = True
            
    def percent_callback(self, percent_msg):
        self.percent_msg = percent_msg.data

        # if self.percent_msg >= 99.9 and self.full_battery_flag == True:
        #     self.full_battery_pub.publish(self.percent_msg)
        #     self.full_battery_flag = False
        # elif self.percent_msg < 99.9:
        #     self.full_battery_flag = True
    
    def battery_status_listener(self):
        rospy.Subscriber("A0_status", Int32, self.A0_callback, queue_size=10)
        rospy.Subscriber("Voltage_status", Float32, self.voltage_callback, queue_size=10)
        rospy.Subscriber("BatteryPer_status", Float32, self.percent_callback, queue_size=10)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('battery_status_listener', anonymous=True)

    battery_status = BatteryStatusListener()
    battery_status.battery_status_listener()
