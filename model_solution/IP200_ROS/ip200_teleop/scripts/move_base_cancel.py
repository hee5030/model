#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalID

class CancelAndGo():
    def __init__(self):
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.wait_for_server(self.mb_client)

        self.ob_sub = rospy.Subscriber('/object_detection', Bool, self.ob_sub_callback)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_sub_callback) 
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        self.goal_id = None
        self.goal_pos_x = 0.0
        self.goal_pos_y = 0.0
        self.goal_ori_z = 0.0
        self.goal_ori_w = 0.0
        self.status = None
        self.isObject = False
        # self.record_pose_start = False
        self.pre_pose_x = None
        self.pre_pose_y = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.rate = rospy.Rate(10)

    # object_detection
    def ob_sub_callback(self, ob_msg):
        if ob_msg.data == True:
            if self.status == 1:  # ACTIVE
                self.cancel_pub.publish(self.goal_id)
                self.isObject = True
                # self.record_pose_start = True
                self.pub_cmd()

    def amcl_sub_callback(self, amcl_msg):
        self.pose = PoseWithCovarianceStamped()
        self.pose.pose.pose.position = amcl_msg.pose.pose.position
        self.pose.pose.pose.orientation = amcl_msg.pose.pose.orientation
        # if self.record_pose_start:
        #     self.pre_pose_x = self.pose.pose.pose.position.x
        #     self.pre_pose_y = self.pose.pose.pose.position.y
        #     self.record_pose_start = False

    # move_base action server의 status 체크
    def status_callback(self, status_data):
        self.goal_id = GoalID()
        self.goal_id.stamp = rospy.Time.now()
        
        if len(status_data.status_list) > 0:
            # 0: PENDING, 1: ACTIVE, 2: PREEMPTED, 3: SUCCEEDED, 4: ABORTED, 5: REJECTED ...
            self.status = status_data.status_list[-1].status
            # rospy.loginfo(f"Goal is active, current status: {self.status}")
            self.goal_id.id = status_data.status_list[-1].goal_id.id
        else:
            rospy.loginfo('No current goal')
    
    # goal 지점 저장
    def goal_callback(self, goal_data):
        # if self.status 
        self.goal = MoveBaseGoal()
        self.goal_pos_x = goal_data.goal.target_pose.pose.position.x
        self.goal_pos_y = goal_data.goal.target_pose.pose.position.y
        self.goal_ori_z = goal_data.goal.target_pose.pose.orientation.z
        self.goal_ori_w = goal_data.goal.target_pose.pose.orientation.w

    def wait_for_server(self, client):
        rospy.loginfo('Waiting for action server...')
        client.wait_for_server()
        rospy.loginfo('Connected to action server')

    # object_detect하고 cancel_goal되면 앞으로 일정거리 움직임
    def pub_cmd(self):
        self.pre_pose_x = self.pose.pose.pose.position.x
        self.pre_pose_y = self.pose.pose.pose.position.y
        if self.isObject:
            vel_msg = Twist()
            vel_msg.linear.x = 0.1        # Go Slowly

            while not rospy.is_shutdown():
                if abs(self.pose.pose.pose.position.x - self.pre_pose_x) > 0.7:   # 직선거리로 바꿔야함
                    break
                self.cmd_pub.publish(vel_msg)
                self.rate.sleep()

            vel_msg.linear.x = 0.0
            self.cmd_pub.publish(vel_msg)  # Stop
            self.pre_pose_x = 0.0
            self.isObject = False
            self.resend_goal()

    def resend_goal(self):
        tmp_goal = MoveBaseGoal()
        tmp_goal.target_pose.header.frame_id = "map"
        tmp_goal.target_pose.header.stamp = rospy.Time.now()
        tmp_goal.target_pose.pose.position.x = self.goal_pos_x
        tmp_goal.target_pose.pose.position.y = self.goal_pos_y
        tmp_goal.target_pose.pose.orientation.z = self.goal_ori_z
        tmp_goal.target_pose.pose.orientation.w = self.goal_ori_w
        self.mb_client.send_goal(tmp_goal)

if __name__ == '__main__':
    rospy.init_node('cancel_and_go')
    move_base_controller = CancelAndGo()
    rospy.spin()