#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import time
import smach
import rospy

from base_classes import *

from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

import pickle


def TestStraightTraj():
    rospy.init_node('TestStraightTraj',anonymous=False)
    davinciArmRight = RavenArm(raven_constants.Arm.Right)
    traj = pickle.load(open('straight2.p', 'rb'))
    startPoseRight = davinciArmRight.ravenController.currentPose
    print "startPoseRight:"
    print repr(startPoseRight)
    rospy.loginfo('Enter to execute Recorded Trajectory')
    raw_input()
    davinciArmRight.executeStampedPoseGripperTrajectory(traj)
    davinciArmRight.stop()

if __name__ == '__main__':
    TestStraightTraj()