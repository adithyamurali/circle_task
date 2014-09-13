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
from std_msgs.msg import Float32

import pickle

from multiprocessing import Pool
import multiprocessing as mp

def f(arm_traj):
    curr_proc = mp.current_process()
    curr_proc.daemon = False
    arm = arm_traj[0]
    print arm
    arm = RavenArm(arm)
    traj = arm_traj[1]
    # rospy.sleep(3)
    arm.executeStampedPoseGripperTrajectory(traj)
    arm.stop()

def TestStraightTraj():
    input_bag_name = sys.argv[1]
    rospy.init_node('TestStraightTraj',anonymous=False)
    # davinciArmRight = RavenArm(raven_constants.Arm.Right)
    # davinciArmLeft = RavenArm(raven_constants.Arm.Left)
    traj = pickle.load(open(input_bag_name, 'rb'))
    # startPoseRight = davinciArmRight.ravenController.currentPose
    # print "startPoseRight:"
    # print repr(startPoseRight)
    # rospy.loginfo('Enter to execute Recorded Trajectory')
    # raw_input()
    p = Pool(2)
    p.map(f, [(raven_constants.Arm.Left, traj[0]), (raven_constants.Arm.Right, traj[1])])
    # davinciArmLeft.executeStampedPoseGripperTrajectory(traj[0])
    # davinciArmRight.executeStampedPoseGripperTrajectory(traj[1])
    # davinciArmRight.stop()
    # davinciArmLeft.stop()

if __name__ == '__main__':
    TestStraightTraj()