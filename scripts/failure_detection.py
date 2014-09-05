import IPython
import cv
import cv2
import cv_bridge
import numpy as np

import roslib
roslib.load_manifest('RavenDebridement')

import rospy
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

import tfx
import RavenDebridement.foam_util as Util

from scipy.optimize import leastsq

DEBUG = True
USE_SAVED_IMAGES = True


class FailureDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.left_image = None
        self.right_image = None
        self.info = {'l': None, 'r': None, 'b': None, 'd': None}

        #========SUBSCRIBERS========#
        # image subscribers
        rospy.Subscriber("/AD/left/image_rect_color", Image,
                         self.left_image_callback, queue_size=1)
        rospy.Subscriber("/AD/right/image_rect_color", Image,
                         self.right_image_callback, queue_size=1)
        # info subscribers
        rospy.Subscriber("/AD/left/camera_info",
                         CameraInfo, self.left_info_callback)
        rospy.Subscriber("/AD/right/camera_info",
                         CameraInfo, self.right_info_callback)


        #========PUBLISHERS=========#
        # self.thresh_image_pub = rospy.Publisher("/circle_cut/thresh_image", Image)
        # self.disparity_image_pub = rospy.Publisher("/circle_cut/disparity_image", Image)
        # self.points_3d_pub = rospy.Publisher('/circle_cut/points_3d', MarkerArray)
        # self.leastsq_plane_pub = rospy.Publisher('/circle_cut/leastsq_plane', PoseStamped)
        # self.projected_points_pub = rospy.Publisher('/circle_cut/projected_points_3d', MarkerArray)
        # self.ellipse_points_pub = rospy.Publisher('/circle_cut/ellipse_points_3d', MarkerArray)

    def left_info_callback(self, msg):
        if self.info['l']:
            return
        self.info['l'] = msg

    def right_info_callback(self, msg):
        if self.info['r']:
            return
        self.info['r'] = msg

    def right_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        if USE_SAVED_IMAGES:
            self.right_image = cv2.imread('circle_images/right.jpg')
        else:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


    def left_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        if USE_SAVED_IMAGES:
            self.left_image = cv2.imread('circle_images/left.jpg')
        else:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        processed_image = self.process_image()


    # steps:
    # 1. get the window in which to look for motion
    # 2. identify the contours to keep track of
    # 4. blur and dilate the image 
    # 3. save the position of the contours
    # 4. play the action
    # 5. check for changes
    # 6. output the kind of failure detected


    def process_image(self):
    	return None

if __name__ == "__main__":
    rospy.init_node('failure_mode_detector')
    a = FailureDetector()
    rospy.spin()