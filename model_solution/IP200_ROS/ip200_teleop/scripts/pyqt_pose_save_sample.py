#! /usr/bin/env python3

import rospy
import yaml
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QMessageBox

class PoseSaver:
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)
        self.current_pose = PoseWithCovarianceStamped()
        # self.current_file_number = 0
        rospy.loginfo('Start!')

    def update_pose(self, data):
        self.current_pose = data.pose.pose

    def save_current_pose(self, where: str):
        self.current_file_location = where

        # 경로에 맞게 '/home/jusang/Desktop/params/' 부분 변경해주세요. aidl이 아니라 robo이면 /home/aidl을 /home/robo로 바꿔주세요. 
        # filename = os.path.join('/home/jusang/Desktop/params/', 'params' + '_' + str(self.current_file_location) + '.yaml')
        filename = os.path.join('/home/aidl/catkin_ws/src/IP200_ROS/ip200_navigation/config/', 'params' + '_' + str(self.current_file_location) + '.yaml')
        
        data = {
            'goal_position': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                # 'x': 0.0,
                # 'y': 0.0,
            },
            'goal_orientation': {
                'z': self.current_pose.orientation.z,
                'w': self.current_pose.orientation.w
                # 'x': 0.0,
                # 'y': 0.0,
            }
        }
        
        with open(filename, 'w') as f:
            yaml.safe_dump(data, f)
        # rospy.loginfo(f'Saved pose to params{self.current_file_number}')
        rospy.loginfo(f'filename: {filename}')
        self.show_popup('저장이 완료되었습니다.')

    def show_popup(self, message):
        msg = QMessageBox()
        msg.setWindowTitle('완료')
        msg.setText(message)
        msg.exec_()

class MainWindow(QMainWindow):
    def __init__(self, pose_saver):
        super().__init__()

        self.pose_saver = pose_saver

        self.setWindowTitle('Pose Saver')

        self.button1 = QPushButton('A 좌표에 저장')
        self.button1.clicked.connect(lambda: self.pose_saver.save_current_pose('a'))

        self.button2 = QPushButton('B 좌표에 저장')
        self.button2.clicked.connect(lambda: self.pose_saver.save_current_pose('b'))

        self.button3 = QPushButton('Docking 좌표에 저장')
        self.button3.clicked.connect(lambda: self.pose_saver.save_current_pose('docking'))

        layout = QVBoxLayout()
        layout.addWidget(self.button1)
        layout.addWidget(self.button2)
        layout.addWidget(self.button3)

        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)

        self.resize(300, 300)

if __name__ == '__main__':
    rospy.init_node('pose_saver_node')
    pose_saver = PoseSaver()

    app = QApplication([])
    window = MainWindow(pose_saver)
    window.show()

    app.exec_()
