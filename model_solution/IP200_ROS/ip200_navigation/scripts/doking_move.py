#!/usr/bin/env python3
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
waypoints = [
        ['one', (1.665886546493766, 0.6177923193032874), (0.0, 0.0, 0.0348937935611379, 0.9993910261608879)],
        ['two', (1.5601809179311832, 0.6177923193032874), (0.0, 0.0, -0.984047240305, 0.177907360295)]
    ]
class Waypoint(State):
        def __init__(self, position, orientation):
            State.__init__(self, outcomes=['success'])
            # Get an action client
            self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.client.wait_for_server()
            # Define the goal
            self.goal = MoveBaseGoal()
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.pose.position.x = position[0]
            self.goal.target_pose.pose.position.y = position[1]
            self.goal.target_pose.pose.position.z = 0.0
            self.goal.target_pose.pose.orientation.x = orientation[0]
            self.goal.target_pose.pose.orientation.y = orientation[1]
            self.goal.target_pose.pose.orientation.z = orientation[2]
            self.goal.target_pose.pose.orientation.w = orientation[3]
        
        def execute(self, userdata):
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            return 'success'
if __name__ == '__main__':
        rospy.init_node('patrol')
        patrol = StateMachine('success')
        with patrol:
            for i,w in enumerate(waypoints):
                StateMachine.add(w[0],
                                 Waypoint(w[1], w[2]),
                                 transitions={'success':waypoints[(i + 1) % \
                                 len(waypoints)][0]})
        patrol.execute()