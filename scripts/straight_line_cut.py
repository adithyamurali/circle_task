#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import numpy as np
import os
import math
import rospy
import tfx
import IPython

# rename so no conflict with raven_2_msgs.msg.Constants
from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm


class StraightCut:

    def __init__(self, arm = raven_constants.Arm.Right):
        rospy.init_node('raven_commander',anonymous=True)
        self.davinciArm = RavenArm(arm)
        self.davinciArm.start()
        rospy.sleep(1)
        self.publisher = rospy.Publisher("/gak/curr_cut_pose", PoseStamped)

    def goToCutPose(self):
        startPose = self.davinciArm.ravenController.currentPose
        print "Start Right Pose:"
        print repr(startPose)

        # Need to fill this in
        cutPoint = tfx.pose()

        self.publisher.publish(cutPoint.msg.PoseStamped())
        rospy.loginfo('Press enter to go to Cut Point')
        raw_input()

        self.davinciArm.setGripperPositionDaVinci(1)
        self.davinciArm.goToGripperPose(cutPoint)

    def cuttingAction(self):
        for i in range(3):
            self.cut()

    def cut(self):
        startPose = self.davinciArm.ravenController.currentPose
        print "Start Right Pose:"
        print repr(startPose)

        cutPose = tfx.pose(startPose, copy = True)
        cutPose.position.x += 0.005
        self.davinciArm.goToGripperPose(cutPose)
        self.davinciArm.setGripperPositionDaVinci(-1.5)
        self.davinciArm.goToGripperPose(cutPose)
        rospy.sleep(1)
        for i in range(8):
            self.davinciArm.setGripperPositionDaVinci(0.10 * i)
            self.davinciArm.goToGripperPose(pose)
        cutPose.position.x += 0.002
        self.davinciArm.goToGripperPose(cutPose)

if __name__ == '__main__':

    a = StraightCut()
    a.goToCutPose()
    a.cuttingAction()