#!/usr/bin/env python3

# aruco 마커의 정보(id, rvec, tvec)를 얻어서 /aruco 토픽(메세지 타입: ArucoMarkerVector)에 publish

import rospy
import cv2
import numpy as np

from cv2 import aruco
from rostopic import get_topic_type
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from ip200_msgs.msg import ArucoMarkerVector

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.id = int(23)  # set aruco_marker ID

        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic", "/camera/color/image_raw/compressed"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"
        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.image_callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.image_callback, queue_size=1
            )
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
        
        self.aruco_pub = rospy.Publisher('/aruco', ArucoMarkerVector, queue_size=5)

        # Parameters
        self.aruco_type = cv2.aruco.DICT_6X6_250
        self.cam_src_num = 4
        self.show_img = rospy.get_param('/show_img', True)

        # self.intrinsic_camera = np.array(((594.0705387, 0., 310.21281441), (0., 591.79870457, 250.43981239), (0., 0., 1.)))
        # self.distortion = np.array([[ 2.67285112e-01, -1.18930743e+00, -9.60580634e-04,  6.35610786e-03, 1.16625781e+00]])

        self.intrinsic_camera = np.array(((616.1589965820312, 0.0, 336.71173095703125), (0.0, 616.2099609375, 232.3656463623047), (0.0, 0.0, 1.0)))
        self.distortion = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_type)
        self.parameters = cv2.aruco.DetectorParameters()

    def make_aruco_msgs(self, id, rvec, tvec) -> ArucoMarkerVector:
        x_ax, y_ax, z_ax = rvec[0][0]
        x_lin, y_lin, z_lin = tvec[0][0]

        aruco_msg = ArucoMarkerVector()
        aruco_msg.id = id

        aruco_msg.rvec.x = x_ax * 57.2958  # rad -> degree
        aruco_msg.rvec.y = y_ax * 57.2958
        aruco_msg.rvec.z = z_ax * 57.2958
        
        aruco_msg.tvec.x = x_lin
        aruco_msg.tvec.y = y_lin
        aruco_msg.tvec.z = z_lin
        return aruco_msg

    def image_callback(self, data):
        # convert the image data to cv2 Image
        if self.compressed_input:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # detect markers
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        # rospy.loginfo('Listening...')
        
        # extract information of aruco_marker

        if ids is not None:
            for i ,id in enumerate(ids):
                if self.id in id:
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.024, self.intrinsic_camera, self.distortion)
                    
                    aruco_msg = self.make_aruco_msgs(self.id, rvec, tvec)

                    # if self.show_img:
                    #     cv2.aruco.drawDetectedMarkers(cv_image, corners)
                    #     cv2.drawFrameAxes(cv_image, self.intrinsic_camera, self.distortion, rvec, tvec, 0.05)
                    #     cv2.imshow('Image window', cv_image)
                    #     cv2.waitKey(1)
        else:
            empty_vector = np.array([[[0.0, 0.0, 0.0]]])
            aruco_msg = self.make_aruco_msgs(0, empty_vector, empty_vector)

        self.aruco_pub.publish(aruco_msg)

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    ad = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
