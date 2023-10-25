#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from cv2 import aruco
from std_msgs.msg import Int64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from adl200_msgs.msg import ArucoMarker

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.id = int(23)  # set aruco_marker ID
        self.image_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_image_callback, queue_size=1)
        
        self.aruco_pub = rospy.Publisher('/aruco', ArucoMarker, queue_size=5)

    def make_aruco_msgs(self, id, x, y, depth) -> ArucoMarker:
        aruco_msg = ArucoMarker()
        aruco_msg.id = id
        aruco_msg.x = x
        aruco_msg.y = y
        aruco_msg.depth = depth
        return aruco_msg

    def depth_image_callback(self, depth_data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, depth_data.encoding)
        except CvBridgeError as e:
            print(e)

    def image_callback(self, data):
        # convert the image data to cv2 Image
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")

        # create aruco dictionary
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        # initialize detector parameters
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)

        # detect markers
        corners, ids, rejectedImgPoints = detector.detectMarkers(cv_image)
        # rospy.loginfo('Listening...')

        # extract the center of aruco_marker
        # & depth of aruco_marker
        if ids is not None:
            for i ,id in enumerate(ids):
                if self.id in id:
                    corner = corners[i]
                    corner = corner[0]
                    center = corner.mean(axis=0)

                    if self.depth_image is not None:
                        x, y = center[0], center[1]
                        depth = self.depth_image[int(y), int(x)]
                        
                        aruco_msg = self.make_aruco_msgs(self.id, int(x), int(y), int(depth))

                        rospy.loginfo(f'x: {x}, y: {y}, depth: {depth}')
        else:
            aruco_msg = self.make_aruco_msgs(0, 0, 0, 0)

        self.aruco_pub.publish(aruco_msg)

        aruco_image = aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv2.imshow('Image window', aruco_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    ad = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
