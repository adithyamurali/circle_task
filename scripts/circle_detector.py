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
USE_SAVED_IMAGES = False

class CircleDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.left_image = None
        self.right_image = None
        self.info = {'l': None, 'r': None, 'b': None, 'd': None}

        #========SUBSCRIBERS========#
        # image subscribers
        rospy.Subscriber("/BC/left/image_rect_color", Image,
                         self.left_image_callback, queue_size=1)
        rospy.Subscriber("/BC/right/image_rect_color", Image,
                         self.right_image_callback, queue_size=1)
        # info subscribers
        rospy.Subscriber("/BC/left/camera_info",
                         CameraInfo, self.left_info_callback)
        rospy.Subscriber("/BC/right/camera_info",
                         CameraInfo, self.right_info_callback)


        #========PUBLISHERS=========#
        self.thresh_image_pub = rospy.Publisher("/circle_cut/thresh_image", Image)
        self.disparity_image_pub = rospy.Publisher("/circle_cut/disparity_image", Image)
        self.points_3d_pub = rospy.Publisher('/circle_cut/points_3d', MarkerArray)
        self.leastsq_plane_pub = rospy.Publisher('/circle_cut/leastsq_plane', PoseStamped)
        self.projected_points_pub = rospy.Publisher('/circle_cut/projected_points_3d', MarkerArray)
        self.ellipse_points_pub = rospy.Publisher('/circle_cut/ellipse_points_3d', MarkerArray)

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

    def process_image(self):
        rospy.loginfo("Processing image...")
        if self.left_image is None or self.right_image is None:
            return # because we need both images

        left_points = self.get_circle_edge_points(self.left_image.copy())
        right_points = self.get_circle_edge_points(self.right_image.copy(), seed = left_points[0])

        left_points = left_points[1:]
        right_points = right_points[1:]
        # left_points = right_points

        # get the 3d points from the left and right points
        points_3d = self.get_points_3d(left_points, right_points)

        # fit a plane to the 3d points amd get the normal to the plane
        normal = self.least_squares_plane_normal(points_3d)

        # convert the points to the frame of the normal and project them onto the yz plane (x values all zero)
        projected_points = self.project_points_onto_plane(normal, points_3d)

        # create an array of 2d points and fit an ellipse to them
        points_2d = np.array([[pt.y, pt.z] for pt in projected_points], dtype=np.float32)
        ellipse = cv2.fitEllipse(points_2d)
        ellipse_points = self.get_ellipse_points(ellipse)

        # project the ellipse points back to 3d and transform back to global frame
        ellipse_points_3d = [tfx.point([0, a[0], a[1]]) for a in ellipse_points]
        # for a in ellipse_points_3d:
        #     a.frame = 'left_BC'
        # print ellipse_points_3d
        ellipse_points_3d = [normal.as_transform()*a for a in ellipse_points_3d] # convert back to global frame

        if DEBUG:
            markers = MarkerArray()
            for i in range(len(ellipse_points_3d)):
                point = ellipse_points_3d[i]
                marker = Util.createMarker(tfx.pose(point).msg.PoseStamped(), id=i+200, lifetime=2, r=0, g=255, b=0, scale = 0.004)
                markers.markers.append(marker)

            self.ellipse_points_pub.publish(markers)



    def get_ellipse_points(self, ellipse):
        """ returns the four vertices that comprise the major axes of the ellipse """
        center = ellipse[0]
        width, height = ellipse[1]
        theta = ellipse[2] # in degrees
        v1 = (width/2*np.cos(np.radians(theta)), width/2*np.sin(np.radians(theta)))
        v2 = (height/2*np.cos(np.radians(theta+90)), height/2*np.sin(np.radians(theta+90)))
        left = [center[0]-v1[0], center[1]-v1[1]]
        right = [center[0]+v1[0], center[1]+v1[1]]
        top = [center[0]-v2[0], center[1]-v2[1]]
        bottom = [center[0]+v2[0], center[1]+v2[1]]
        IPython.embed
        return (center,left,right,top,bottom)


    def get_circle_edge_points(self, image, seed=None):
        # threshold the image
        threshold = 40
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, thresholded_image = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY_INV)
        
        # dilate the image
        dilation_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        thresholded_image = cv2.dilate(thresholded_image, dilation_kernel)
        thresholded_image = cv2.dilate(thresholded_image, dilation_kernel)
        thresholded_image = cv2.dilate(thresholded_image, dilation_kernel)

        # if DEBUG:
        #     debug_image = thresholded_image
        
        #     self.image_publisher(debug_image, self.thresh_image_pub)

        contours, contour_hierarchy = cv2.findContours(thresholded_image.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # remove contours that are too small and too big to the be the circle
        filtered = []
        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) > 8000 and cv2.contourArea(contours[i]) < 100000:
                filtered.append(contours[i])
        contours = filtered

        # find the area and centroid of each contour
        areas = [cv2.contourArea(a) for a in contours]
        centroids = []
        for contour in contours:
            moments = cv2.moments(contour)
            cx = moments['m10']/moments['m00']
            cy = moments['m01']/moments['m00']
            centroid = (cx,cy)
            centroids.append(centroid)

        # helper function that computes euclidean distance
        def dist(x,y):
            # IPython.embed()
            return np.sqrt(np.power(x[0]-y[0], 2) + np.power(x[1]-y[1], 2))

        # find the inner and outer contour of the rubberband 
        index_a = None
        index_b = None
        for i in range(len(centroids)):
            for j in range(len(centroids)):
                if i != j: # make sure we aren't comparing the same contour to itself
                    if dist(centroids[i], centroids[j]) < 10: # the centroids of the contours are close to each other
                        if areas[i]/areas[j] < 1.25 and areas[i]/areas[j] > 0.8: # the contours have similar areas
                            index_a = i
                            index_b = j
                            break

            if index_a is not None or index_b is not None:
                break

        # identify which contour is bigger
        if areas[index_a] > areas[index_b]:
            contour = contours[index_a]
        else:
            contour = contours[index_b]

        # get the extreme points
        mid_leftmost = tuple(contour[contour[:,:,0].argmin()][0])
        mid_rightmost = tuple(contour[contour[:,:,0].argmax()][0])
        # topmost = tuple(contour[contour[:,:,1].argmin()][0])
        # bottommost = tuple(contour[contour[:,:,1].argmax()][0])



        def get_corresponding_point(mid_leftmost, delta):
            if seed is not None:
                mid_leftmost = seed
            """ helper function that intersects a line with the contour at a delta away from the leftmost point to 
                generate 2 more correspondence points """
            # get two points above
            line_image = image.copy()
            line_image[:] = 0 # zero it out
            contour_image = line_image.copy()
            cv2.drawContours(contour_image, [contour], 0, 255, 1)
            line_image[mid_leftmost[1]-delta, :] = 255
            intersect_image = cv2.bitwise_and(contour_image, line_image) # intersection between contour and horizontal line
            top_contours, contour_hierarchy = cv2.findContours(intersect_image.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            return [tuple(top_contours[1][0][0]), tuple(top_contours[0][0][0])]


        # generating additional points along the circle
        extreme_points = [mid_leftmost, mid_rightmost]
        for i in range(-80, 81, 15):
            extreme_points = extreme_points + get_corresponding_point(mid_leftmost, i)

        if DEBUG:
            debug_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            
            # draw the rubber band's contour
            cv2.drawContours(debug_image, [contour], 0, (0,255,0), 3)

            # draw its extreme points
            for point in extreme_points:
                cv2.circle(debug_image, point, 5, (255,0,0), thickness=3, lineType=8, shift=0)
        
            self.image_publisher(debug_image, self.thresh_image_pub)


        return extreme_points


    def get_points_3d(self, left_points, right_points):
        """ this method assumes that corresponding points are in the right order
            and returns a list of 3d points """

        # both lists must be of the same lenghth otherwise return None
        if len(left_points) != len(right_points):
            rospy.logerror("The number of left points and the number of right points is not the same")
            return None

        points_3d = []
        for i in range(len(left_points)):
            a = left_points[i]
            b = right_points[i]
            disparity = abs(a[0]-b[0])
            pt = Util.convertStereo(a[0], a[1], disparity, self.info)
            points_3d.append(pt) 
        # IPython.embed()
        # publish the points if in debug mode
        if DEBUG:
            markers = MarkerArray()
            for i in range(len(points_3d)):
                point = points_3d[i]
                marker = Util.createMarker(tfx.pose(point).msg.PoseStamped(), id=i+1, lifetime=2)
                markers.markers.append(marker)

        self.points_3d_pub.publish(markers)
        return points_3d


    def least_squares_plane_normal(self, points_3d):
        """ returns a pose perpendicular to the plance of the points.
            note: the pose is arbitrary set to have the position as the first point 
                  in points_3d"""

        # form lists for x, y, and z values
        x_list = [a.point.x for a in points_3d]
        y_list = [a.point.y for a in points_3d]
        z_list = [a.point.z for a in points_3d]

        # convert to numpy arrays
        x_data = np.array(x_list)
        y_data = np.array(y_list)
        z_data = np.array(z_list)
        A = np.vstack([x_data, y_data, np.ones(len(x_data))]).T
        result = np.linalg.lstsq(A, z_data)[0]
        normal = np.array([result[0], result[1], -1]) # vector normal to fitted plane

        # get a pose by producing two arbitrary vectors perpendicular to normal
        x_axis = normal
        y_axis = np.array([0.5/x_axis[0], 0.5/x_axis[1], 1]) # arbitrary perpendicular vector
        z_axis = np.cross(x_axis, y_axis)

        # construct the pose
        rotMat = np.vstack((x_axis, y_axis, z_axis)).T
        tbRot = tfx.tb_angles(rotMat).matrix
        quat = tfx.tb_angles(tbRot).quaternion
        position = [x_list[0], y_list[0], z_list[0]]
        pose = tfx.pose(position, quat, frame='left_BC')
        
        if DEBUG:
            self.leastsq_plane_pub.publish(pose.msg.PoseStamped())

        return pose


    def project_points_onto_plane(self, normal, points_3d):
        # convert the points to the local frame of normal
        local_points = [normal.as_transform()*a for a in points_3d]
        # project onto yz plane by making x values zero
        projected_points = []
        for i in range(len(local_points)):
            pt = local_points[i].copy()
            pt.x = 0
            projected_points.append(pt)

        # to test it out, convert back to global frame and publish the result    
        if DEBUG:
            markers = MarkerArray()
            for i in range(len(projected_points)):
                point = normal.inverse().as_transform()*projected_points[i]
                marker = Util.createMarker(tfx.pose(point).msg.PoseStamped(), id=i+100, lifetime=2, r=2255, g=0, b=0)
                markers.markers.append(marker)

        self.points_3d_pub.publish(markers)

        return projected_points


    def image_publisher(self, image, image_publisher):
        img_msg = self.bridge.cv_to_imgmsg(cv.fromarray(image))
        image_publisher.publish(img_msg)



if __name__ == "__main__":
    rospy.init_node('circle_detector')
    a = CircleDetector()
    rospy.spin()