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

from multiprocessing import Pool

class Follower(object):
    def __init__(self, traj, davinciArmLeft):
        print "Follower Init"
        self.sub = rospy.Subscriber("/circle/command_left", Int16, self.phase_cb)
        self.davinciArmLeft = davinciArmLeft
        # self.davinciArmLeft = None
        self.traj = traj
        print "Set traj"
        self.pub = rospy.Publisher("/circle/command_complete", Bool)
        self.sm_complete = False
        self.sub2 = rospy.Subscriber("/circle/sm_complete", Bool, self.sm_complete_cb)
        print "Follower completed initializing"

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

if __name__ == '__main__':
    # rospy.init_node('Follower',anonymous=False)
    # input_file_name = sys.argv[1]
    # Follower(input_file_name)
    # rospy.spin()
    # a = get_full_traj()
    # IPython.embed()
    pass