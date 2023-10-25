#!/usr/bin/env python3

# realsense 카메라로 yolo를 통해 단차를 인식하고 BoundingBox의 중심값의 depth값을 publish
 
import rospy
from sensor_msgs.msg import Image as msg_Image, Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import pyrealsense2 as rs2
from detection_msgs.msg import BoundingBoxes
import numpy as np
from std_msgs.msg import Int64

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.x = 0
        self.y = 0
        self.depth=0.0
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        # self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
        self.bounding_boxes_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.bounding_boxes_callback)
        self.x_center_pub = rospy.Publisher('x_center_value', Int64, queue_size=10)
        self.y_center_pub = rospy.Publisher('y_center_value', Int64, queue_size=10)
        self.depth_pub = rospy.Publisher('depth', Int64, queue_size=10)
        self.depth_threshold = rospy.get_param('~depth_threshold', 750)
        self.intrinsics = None

    def bounding_boxes_callback(self, msg):
        min_depth = float('inf')
        closest_bbox = None

        if not msg.bounding_boxes or self.depth_image is None:
            self.x = 0
            self.y = 0
            
        # Iterate through each bounding box in the message
        # for bounding_box in msg.boundingBoxesResults_.bounding_boxes:
        else:
            for bounding_box in msg.bounding_boxes:
                if bounding_box.Class == "molding":
                    # Extract the x and y coordinates of the center of the bounding box
                    x = bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin) / 2
                    y = bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin) / 2

                    depth = self.depth_image[int(y), int(x)]
                    # rospy.loginfo(f'depth: {depth}(m)')
                    if depth > self.depth_threshold:
                        continue
                        
                    if depth < min_depth:
                        min_depth = depth
                        closest_bbox = bounding_box
                        self.x = x
                        self.y = y

        if closest_bbox is None:
            self.x = 0
            self.y = 0
            min_depth = 0

        self.x_center_pub.publish(int(self.x))
        self.y_center_pub.publish(int(self.y))
        self.depth_pub.publish(int(min_depth))
        rospy.loginfo(f'depth: {self.depth_threshold}(m)')
        rospy.loginfo(f'bounding box Depth at center({self.x}, {self.y}): {min_depth/1000}(m)')

        return self.x, self.y, min_depth

    def imageDepthCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)

    # def imageDepthInfoCallback(self, cameraInfo):
    #     try:
    #         # import pdb; pdb.set_trace()
    #         if self.intrinsics:
    #             return
    #         self.intrinsics = rs2.intrinsics()
    #         self.intrinsics.width = cameraInfo.width
    #         self.intrinsics.height = cameraInfo.height
    #         self.intrinsics.ppx = cameraInfo.K[2]
    #         self.intrinsics.ppy = cameraInfo.K[5]
    #         self.intrinsics.fx = cameraInfo.K[0]
    #         self.intrinsics.fy = cameraInfo.K[4]
    #         if cameraInfo.distortion_model == 'plumb_bob':
    #             self.intrinsics.model = rs2.distortion.brown_conrady
    #         elif cameraInfo.distortion_model == 'equidistant':
    #             self.intrinsics.model = rs2.distortion.kannala_brandt4
    #         self.intrinsics.coeffs = [i for i in cameraInfo.D]
    #     except CvBridgeError as e:
    #         print(e)
    #         return

def main():
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]

    rospy.init_node(node_name)
    main()
    rospy.spin()