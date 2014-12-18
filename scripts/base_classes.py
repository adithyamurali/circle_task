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

        # Code from previous warp technique
        # Transform from camera frame to left arm robot frame
        self.B_L = tfx.lookupTransform('one_remote_center_link', 'left_AD')
        # Transform from camera frame to right arm robot frame
        self.B_R = tfx.lookupTransform('two_remote_center_link', 'left_AD')

    def circle_position_cb(self, data):
        self.circle_1 = tfx.pose(data.markers[0].pose)
        self.A = self.circle_1.as_tf() * self.circle_0.inverse().as_tf()
        self.A.position.z = 0

    def warp_traj_left(self):
        # M = self.B_L.as_tf() * self.A.as_tf() * self.B_L.inverse().as_tf()
        # M.from_frame = None
        # M.to_frame = None
        # print "left arm transform: ", M
        print "A: ", repr(self.A)
        # print "B_L: ", self.B_L
        # M.position.z += 0.001
        curr_traj = self.left_arm.traj
        new_traj = {}
        for i in range(8):
            if i != 0:
                new_list_of_pairs = []
                j = 0
                for pair in curr_traj[i]:
                    if j % 20 == 0:
                        new_pose = pair[0]
                        new_pose.frame = "one_remote_center_link"
                        new_pose = raven_util.convertToFrame(new_pose, "circle_chessboard")
                        new_pose.position.x += self.A.position.x
                        new_pose.position.y += self.A.position.y
                        if i == 2:
                            new_pose.position.z += 0.002
                        # new_pose.position.x += 0.005
                        # new_pose.position.y += -(self.A.position.y)
                        new_pose = raven_util.convertToFrame(new_pose, "one_remote_center_link")
                        new_pair = (new_pose, pair[1])
                        new_list_of_pairs.append(new_pair)

                    j += 1
                new_traj[i] = new_list_of_pairs
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
                        new_pose = pair[0]
                        new_angle = pair[1]
                        new_pose.frame = "two_remote_center_link"
                        new_pose = raven_util.convertToFrame(new_pose, "circle_chessboard")
                        new_pose.position.x += self.A.position.x
                        new_pose.position.y += self.A.position.y
                        if i == 2:
                            new_pose.position.z += -0.003
                            # if j < (len(curr_traj[i]) / 2):
                            #     new_angle = -pair[1]
                        # new_pose.position.y += -(self.A.position.y)
                        new_pose = raven_util.convertToFrame(new_pose, "two_remote_center_link")
                        new_pair = (new_pose, new_angle)
                        new_list_of_pairs.append(new_pair)

                    j += 1
                new_traj[i] = new_list_of_pairs
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
        print "execute warp"
        print "warping right trajectory"
        new_traj_R = self.warp_traj_right()
        print "warping left trajectory"
        self.warp_traj_left()
        # IPython.embed()
        userdata.new_traj_R = new_traj_R
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
        print "ready to execute"
        rospy.sleep(2)
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
        self.sub2 = rospy.Subscriber("/circle_cut/cut_completion", Bool, self.check_final_cut_cb)

        self.left_arm_executing = False
        self.traj = traj
        self.left_arm = left_arm
        self.final_cut = True
        self.pub3 = rospy.Publisher("circle_cut/capture_after_image", Bool)
        self.pub4 = rospy.Publisher("circle_cut/capture_before_image", Bool)

    def check_final_cut_cb(self, msg):
        print "callback", msg
        self.final_cut = False

    def phase_cb(self, msg):
        print "Left Arm executing? - ", msg
        self.left_arm_executing = msg.data

    def wait(self):
        while self.left_arm_executing:
            rospy.sleep(0.2)
        return


    def move_away(self):
        currPose = self.davinciArmRight.getGripperPose()
        newPose = tfx.pose(currPose, copy = True)
        newPose.position.y += -0.01
        newPose.position.z += 0.01
        self.davinciArmRight.goToGripperPose(newPose)

    def execute_final_cut(self):
        for _ in range(8):
            currPose = self.davinciArmRight.getGripperPose()
            self.davinciArmRight.setGripperPositionDaVinci(1)
            self.davinciArmRight.goToGripperPose(currPose)
            rospy.sleep(0.5)
            currPose.position.y += 0.005
            # currPose.position.x += -0.003
            currPose.position.z += 0.001
            self.davinciArmRight.goToGripperPose(currPose, speed = 0.01)
            currPose = self.davinciArmRight.getGripperPose()
            self.davinciArmRight.setGripperPositionDaVinci(-2.5)
            self.davinciArmRight.goToGripperPose(currPose)
            rospy.sleep(0.5)

        currPose = self.davinciArmRight.getGripperPose()
        self.davinciArmRight.setGripperPositionDaVinci(1)
        self.davinciArmRight.goToGripperPose(currPose)
        rospy.sleep(0.5)

    def execute(self, userdata):
        self.left_arm_executing = True
        self.pub.publish(5)
        self.davinciArmRight.executeStampedPoseGripperTrajectory(userdata.new_traj_R[5])
        rospy.sleep(1)
        self.wait()

        count = 0
        for _ in range(10):
            self.pub4.publish(True)
            rospy.sleep(0.1)
        self.left_arm.pull_away()
        rospy.sleep(3)
        for _ in range(10):
            self.pub3.publish(True)
            rospy.sleep(0.1)
        rospy.sleep(1)
        while (self.final_cut == False or count == 0):

            print "Final cut", self.final_cut, "count", count
            if self.final_cut == False:
                self.left_arm.pull_back()
                rospy.sleep(1)
                self.execute_final_cut()
            count += 1
            self.final_cut = True
            # self.left_arm.pull_away_less()
            rospy.sleep(3)
            # self.pub3.publish(True)
            rospy.sleep(1)
            print "Check final Cut ", self.final_cut
            if count == 1:
                break

        # self.move_away()
        # Stop the left arm
        self.pub2.publish(True)
        return 'abridged_state_machine'
