#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import time
import smach
import rospy
import sys

from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, Bool

import pickle
import IPython
import tfx

from multiprocessing import Pool

class Follower(object):
    def __init__(self, traj, davinciArmLeft):
        print "Follower Init"
        self.sub = rospy.Subscriber("/circle/command_left", Int16, self.phase_cb)
        self.davinciArmLeft = davinciArmLeft
        self.traj = traj
        print "Set traj"
        self.pub = rospy.Publisher("/circle/command_complete", Bool)
        self.sm_complete = False
        self.sub2 = rospy.Subscriber("/circle/sm_complete", Bool, self.sm_complete_cb)
        print "Follower completed initializing"
        self.pub2 = rospy.Publisher("circle_cut/capture_after_image", Bool)

    def sm_complete_cb(self, msg):
        print "SM Complete:", msg
        self.sm_complete = msg
        if self.sm_complete == True:
            self.davinciArmLeft.stop()

    def phase_cb(self, msg):
        print "L - ", msg.data
        self.pub.publish(True)
        self.davinciArmLeft.executeStampedPoseGripperTrajectory(self.traj[msg.data])
        rospy.sleep(1)
        self.pub.publish(False)

    def pull_away(self):
        currPose = self.davinciArmLeft.getGripperPose()
        newPose = tfx.pose(currPose, copy = True)
        newPose.position.y += 0.012
        newPose.position.x += -0.02
        self.davinciArmLeft.goToGripperPose(newPose, startPose = currPose, speed = 0.01)
        rospy.sleep(1)

    def pull_back(self):
        currPose = self.davinciArmLeft.getGripperPose()
        newPose = tfx.pose(currPose, copy = True)
        newPose.position.y += 0.01
        newPose.position.x += 0.015
        self.davinciArmLeft.goToGripperPose(newPose, startPose = currPose, speed = 0.01)
        rospy.sleep(1)

    def pull_away_less(self):
        currPose = self.davinciArmLeft.getGripperPose()
        newPose = tfx.pose(currPose, copy = True)
        newPose.position.y += 0.006
        self.davinciArmLeft.goToGripperPose(newPose, startPose = currPose, speed = 0.01)
        rospy.sleep(1)

if __name__ == '__main__':
    pass