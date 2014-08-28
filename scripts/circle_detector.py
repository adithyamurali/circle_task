import IPython
import cv
import cv2
import cv_bridge
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo

DEBUG = True

class CircleDetector:
	def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.left_image = None
        self.right_image = None
        self.info = {'l': None, 'r': None, 'b': None, 'd': None}

if __name__ == "__main__":
    rospy.init_node('circle_detector')
    a = CircleDetector()
    rospy.spin()