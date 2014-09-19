#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import time
import tfx
import smach
import rospy
import pickle
from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped
from std_msgs.msg import Int16, Bool

import IPython

SLEEP_TIME = 0

class MasterClass(smach.State):
    """ This is a the base test counterlass for the testing states.

        The excute method simply iterates through all the possible transitions
        from the state. """

    def __init__(self):
        self.outcomes = None
        self.homePoseLeft = tfx.pose([-0.04907726751924995, 0.027288735500984575, -0.1211606501908539],
            (0.9835887535507493, -0.09932464655036198, -0.14884734393715604, 0.023070472014753367))
        self.homePoseRight = tfx.pose([0.061241857051286236, 0.014307808069346816, -0.10446866837544996],
            (-0.9689889616996428, 0.1939060552483166, -0.1474787309756946, 0.04136251626876463))
        rospy.sleep(5)

    def execute(self, userdata):        
        smach.loginfo(self.__class__.__name__ + " must be subclassed. It does nothing.")
        time.sleep(SLEEP_TIME)
        self.counter += 1
        if self.outcomes is None:
            self.outcomes = self.get_registered_outcomes()
        return self.outcomes[self.counter%len(self.outcomes)]

# -----------------------   Phase 1  -------------------------------------

class Start(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class DetectCircle(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class Warp(MasterClass):
    def __init__(self, davinciArmRight, traj, left_arm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'], output_keys = ['new_traj_R'])
        self.davinciArmRight = davinciArmRight
        self.traj = traj
        self.left_arm = left_arm
        self.original_left_arm_traj = self.left_arm.traj
        self.sub = rospy.Subscriber("/circle_cut/ellipse_points_3d", MarkerArray, self.circle_position_cb)
        self.pub1_L = rospy.Publisher("/circle_cut/warped_traj_L", PoseStamped)
        self.pub2_L = rospy.Publisher("/circle_cut/original_traj_L", PoseStamped)
        self.pub1_R = rospy.Publisher("/circle_cut/warped_traj_R", PoseStamped)
        self.pub2_R = rospy.Publisher("/circle_cut/original_traj_R", PoseStamped)
        self.circle_0 = tfx.pose([-0.00746613866728, 0.0368420724794, 0.384582169117], (0.0, 0.0, 0.0, 1.0))
        self.circle_1 = None
        # Transformation between circle_0 and circle_1
        self.A = None
        # Transform from camera frame to left arm robot frame
        self.B_L = tfx.lookupTransform('one_remote_center_link', 'left_AD')
        # Transform from camera frame to right arm robot frame
        self.B_R = tfx.lookupTransform('two_remote_center_link', 'left_AD')

    def circle_position_cb(self, data):
        self.circle_1 = tfx.pose(data.markers[0].pose)
        self.A = self.circle_1.as_tf() * self.circle_0.inverse().as_tf()

    def warp_traj_left(self):
        curr_traj = self.left_arm.traj
        new_traj = {}
        for i in range(8):
            if i != 0:
                new_list_of_pairs = []
                j = 0
                for pair in curr_traj[i]:
                    if j % 20 == 0:
                        a = self.B_L.as_tf() * self.A.as_tf() * self.B_L.inverse().as_tf()
                        a.from_frame = None
                        a.to_frame = None
                        # IPython.embed()
                        a.position.z = 0
                        new_pose = a * pair[0]
                        new_pose.frame = "one_remote_center_link"
                        new_pair = (new_pose, pair[1])
                        new_list_of_pairs.append(new_pair)
                    j += 1
                new_traj[i] = new_list_of_pairs
        # assert len(curr_traj) == len(new_traj)
        # assert len(curr_traj[1]) == len(new_traj[1])
        # assert len(curr_traj[2]) == len(new_traj[2])
        # assert len(curr_traj[3]) == len(new_traj[3])
        # assert len(curr_traj[4]) == len(new_traj[4])
        # assert len(curr_traj[5]) == len(new_traj[5])
        # assert len(curr_traj[6]) == len(new_traj[6])
        # assert len(curr_traj[7]) == len(new_traj[7])
        self.left_arm.traj = new_traj

    def warp_traj_right(self):
        curr_traj = self.traj
        new_traj = {}
        for i in range(8):
            if i != 0:
                new_list_of_pairs = []
                j = 0
                for pair in curr_traj[i]:
                    if j % 20 == 0:                    
                        a = self.B_R.as_tf() * self.A.as_tf() * self.B_R.inverse().as_tf()
                        a.from_frame = None
                        a.to_frame = None
                        # IPython.embed()
                        a.position.z = 0
                        new_pose = a * pair[0]
                        new_pose.frame = "two_remote_center_link"
                        new_pair = (new_pose, pair[1])
                        new_list_of_pairs.append(new_pair)
                    j += 1
                new_traj[i] = new_list_of_pairs
        # assert len(curr_traj) == len(new_traj)
        # assert len(curr_traj[1]) == len(new_traj[1])
        # assert len(curr_traj[2]) == len(new_traj[2])
        # assert len(curr_traj[3]) == len(new_traj[3])
        # assert len(curr_traj[4]) == len(new_traj[4])
        # assert len(curr_traj[5]) == len(new_traj[5])
        # assert len(curr_traj[6]) == len(new_traj[6])
        # assert len(curr_traj[7]) == len(new_traj[7])
        return new_traj

    def publish_traj_ros(self, traj, stage = 1):
        stage_poses_original_R = self.traj[stage]
        stage_poses_warped_R = traj[stage]
        stage_poses_original_L = self.original_left_arm_traj[stage]
        stage_poses_warped_L = self.left_arm.traj[stage]
        i = 0
        while (i < len(stage_poses_original_R)):
            a = stage_poses_original_R[i][0]
            a.stamp = None
            self.pub2_R.publish(a.msg.PoseStamped())
            rospy.sleep(0.002)
            i += 1

        i = 0
        while (i < len(stage_poses_warped_R)):
            b = stage_poses_warped_R[i][0]
            b.stamp = None
            self.pub1_R.publish(b.msg.PoseStamped())
            i += 1
            rospy.sleep(0.02)

        i = 0
        while (i < len(stage_poses_original_L)):
            a = stage_poses_original_L[i][0]
            a.stamp = None
            self.pub2_L.publish(a.msg.PoseStamped())
            i += 1
            rospy.sleep(0.002)

        i = 0
        while (i < len(stage_poses_warped_L)):
            b = stage_poses_warped_L[i][0]
            b.stamp = None
            self.pub1_L.publish(b.msg.PoseStamped())
            i += 1
            rospy.sleep(0.02)


    def execute(self, userdata):
        self.warp_traj_left()
        new_traj_R = self.warp_traj_right()
        userdata.new_traj_R = new_traj_R
        self.publish_traj_ros(new_traj_R, 1)
        # IPython.embed()
        # self.publish_traj_ros(new_traj_R, 2)
        # self.publish_traj_ros(new_traj_R, 3)
        # self.publish_traj_ros(new_traj_R, 4)
        # self.publish_traj_ros(new_traj_R, 5)
        # self.publish_traj_ros(new_traj_R, 6)
        # self.publish_traj_ros(new_traj_R, 7)
        return 'success'

class ExecuteRegrasp(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class GraspGauze(MasterClass):
    def __init__(self, davinciArmRight, traj, left_arm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'], input_keys = ['new_traj_R'])
        self.davinciArmRight = davinciArmRight
        self.sub = rospy.Subscriber("/circle/command_complete", Bool, self.phase_cb)
        self.pub = rospy.Publisher("/circle/command_left", Int16)
        self.pub2 = rospy.Publisher("/circle/sm_complete", Bool)
        self.left_arm_executing = False
        self.traj = traj
        self.left_arm = left_arm

    def phase_cb(self, msg):
        print "Left Arm executing? - ", msg
        self.left_arm_executing = msg.data

    def wait(self):
        while self.left_arm_executing:
            rospy.sleep(0.2)
        return

    def execute(self, userdata):
        print "R - 1"
        self.left_arm_executing = True
        self.pub.publish(1)
        self.davinciArmRight.executeStampedPoseGripperTrajectory(userdata.new_traj_R[1])
        rospy.sleep(1)
        self.wait()
        return 'success'

class CheckGrasp(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class WarpGrasp(MasterClass):
    def __init__(self, davinciArmRight, traj, left_arm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.traj = traj
        self.left_arm = left_arm

    def execute(self, userdata):
        return 'success'

class ExecuteNotchCut(MasterClass):
    def __init__(self, davinciArmRight, traj, left_arm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'], input_keys = ['new_traj_R'])
        self.davinciArmRight = davinciArmRight
        self.sub = rospy.Subscriber("/circle/command_complete", Bool, self.phase_cb)
        self.pub = rospy.Publisher("/circle/command_left", Int16)
        self.pub2 = rospy.Publisher("/circle/sm_complete", Bool)
        self.left_arm_executing = False
        self.traj = traj
        self.left_arm = left_arm

    def phase_cb(self, msg):
        print "Left Arm executing? - ", msg
        self.left_arm_executing = msg.data

    def wait(self):
        while self.left_arm_executing:
            rospy.sleep(0.2)
        return

    def execute(self, userdata):
        self.left_arm_executing = True
        self.pub.publish(2)
        self.davinciArmRight.executeStampedPoseGripperTrajectory(userdata.new_traj_R[2])
        rospy.sleep(1)
        self.wait()
        return 'success'

class CheckNotch(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

# -----------------------   Phase 2  -------------------------------------

class ExecutePartialTraj2(MasterClass):
    def __init__(self, davinciArmRight, traj, left_arm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['complete'], input_keys = ['new_traj_R'])
        self.davinciArmRight = davinciArmRight
        self.counter = 0
        self.sub = rospy.Subscriber("/circle/command_complete", Bool, self.phase_cb)
        self.pub = rospy.Publisher("/circle/command_left", Int16)
        self.pub2 = rospy.Publisher("/circle/sm_complete", Bool)
        self.left_arm_executing = False
        self.traj = traj
        self.left_arm = left_arm

    def phase_cb(self, msg):
        print "Left Arm executing? - ", msg
        self.left_arm_executing = msg.data

    def wait(self):
        while self.left_arm_executing:
            rospy.sleep(0.2)
        return

    def execute(self, userdata):
        self.left_arm_executing = True
        self.pub.publish(3)
        self.davinciArmRight.executeStampedPoseGripperTrajectory(userdata.new_traj_R[3])
        rospy.sleep(1)
        self.wait()
        return 'complete'

class CheckPostion2(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'below', 'above'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class WarpAbove2(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class MoveAbove2(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class WarpBelow2(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class MoveBelow2(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class PhaseTransfer(MasterClass):
    def __init__(self, davinciArmRight, traj, left_arm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'], input_keys=['new_traj_R'])
        self.davinciArmRight = davinciArmRight
        self.counter = 0
        self.sub = rospy.Subscriber("/circle/command_complete", Bool, self.phase_cb)
        self.pub = rospy.Publisher("/circle/command_left", Int16)
        self.pub2 = rospy.Publisher("/circle/sm_complete", Bool)
        self.left_arm_executing = False
        self.traj = traj
        self.left_arm = left_arm

    def phase_cb(self, msg):
        print "Left Arm executing? - ", msg
        self.left_arm_executing = msg.data

    def wait(self):
        while self.left_arm_executing:
            rospy.sleep(0.2)
        return

    def execute(self, userdata):
        self.left_arm_executing = True
        self.pub.publish(4)
        self.davinciArmRight.executeStampedPoseGripperTrajectory(userdata.new_traj_R[4])
        rospy.sleep(1)
        self.wait()
        return 'success'


# -----------------------   Phase 3  -------------------------------------

class ExecutePartialTraj3(MasterClass):
    def __init__(self, davinciArmRight, traj, left_arm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['abridged_state_machine'], input_keys=['new_traj_R'])
        self.davinciArmRight = davinciArmRight
        self.counter = 0
        self.sub = rospy.Subscriber("/circle/command_complete", Bool, self.phase_cb)
        self.pub = rospy.Publisher("/circle/command_left", Int16)
        self.pub2 = rospy.Publisher("/circle/sm_complete", Bool)
        self.left_arm_executing = False
        self.traj = traj
        self.left_arm = left_arm

    def phase_cb(self, msg):
        print "Left Arm executing? - ", msg
        self.left_arm_executing = msg.data

    def wait(self):
        while self.left_arm_executing:
            rospy.sleep(0.2)
        return

    def execute(self, userdata):
        self.left_arm_executing = True
        self.pub.publish(5)
        self.davinciArmRight.executeStampedPoseGripperTrajectory(userdata.new_traj_R[5])
        rospy.sleep(1)
        self.wait()

        rospy.sleep(1)
        self.left_arm_executing = True
        self.pub.publish(6)
        self.davinciArmRight.executeStampedPoseGripperTrajectory(userdata.new_traj_R[6])
        rospy.sleep(1)
        self.wait()

        rospy.sleep(1)
        self.left_arm_executing = True
        self.pub.publish(7)
        self.davinciArmRight.executeStampedPoseGripperTrajectory(userdata.new_traj_R[7])
        rospy.sleep(1)
        self.wait()

        self.pub2.publish(True)

        return 'abridged_state_machine'

class CheckPostion3(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'below', 'above'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class WarpAbove3(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class MoveAbove3(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class WarpBelow3(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'

class MoveBelow3(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        return 'success'