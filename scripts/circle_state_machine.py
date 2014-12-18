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
from follow_right_arm import Follower

class MasterClass:
    def __init__(self, traj):
        print "Master SM Circle"
        rospy.init_node('Master_SM_Circle',anonymous=False)
        self.state_machine = smach.StateMachine(outcomes=['SUCCESS'])
        self.davinciArmRight = RavenArm(raven_constants.Arm.Right)
        self.davinciArmLeft = RavenArm(raven_constants.Arm.Left)
        self.davinciArmLeft.start()
        self.left_arm = Follower(traj[0], self.davinciArmLeft)
        print "Follower started"
        self.setup_state_machine(traj[1], self.left_arm)
    def setup_state_machine(self, traj, left_arm):
        smach.loginfo("Initializing state machine")
        rospy.sleep(1)
        print "Setup SM "
        with self.state_machine:

            # ------ Phase 1 -----

            smach.StateMachine.add('START',
                Start(self.davinciArmRight),
                transitions={'success':'DETECT_CIRCLE'})
            smach.StateMachine.add('DETECT_CIRCLE',
                DetectCircle(self.davinciArmRight),
                transitions={'success':'WARP'})
            smach.StateMachine.add('WARP',
                Warp(self.davinciArmRight, traj, left_arm),
                transitions={'success':'GRASP_GAUZE'}, remapping ={'new_traj_R':'sm_data1'})
            smach.StateMachine.add('GRASP_GAUZE',
                GraspGauze(self.davinciArmRight, traj, left_arm),
                transitions={'success':'CHECK_GRASP'}, remapping ={'new_traj_R':'sm_data1'})
            smach.StateMachine.add('CHECK_GRASP',
                CheckGrasp(self.davinciArmRight),
                transitions={'success':'EXECUTE_NOTCH_CUT'})

            # smach.StateMachine.add('CHECK_GRASP',
            #     CheckGrasp(self.davinciArmRight),
            #     transitions={'success':'EXECUTE_NOTCH_CUT', 'failure': 'WARP_GRASP'},)
            # smach.StateMachine.add('CHECK_GRASP',
            #     CheckGrasp(self.davinciArmRight),
            #     transitions={'success':'EXECUTE_NOTCH_CUT', 'failure': 'WARP_GRASP'},)
            # smach.StateMachine.add('WARP_GRASP',
            #     WarpGrasp(self.davinciArmRight, traj, left_arm),
            #     transitions={'success':'EXECUTE_REGRASP'})
            # smach.StateMachine.add('EXECUTE_REGRASP',
            #     ExecuteRegrasp(self.davinciArmRight),
            #     transitions={'success':'CHECK_GRASP'})



            smach.StateMachine.add('EXECUTE_NOTCH_CUT',
                ExecuteNotchCut(self.davinciArmRight, traj, left_arm),
                transitions={'success':'CHECK_NOTCH'}, remapping ={'new_traj_R':'sm_data1'})

            smach.StateMachine.add('CHECK_NOTCH',
                CheckNotch(self.davinciArmRight),
                transitions={'success':'EXECUTE_PARTIAL_TRAJ_2', 'failure':'EXECUTE_NOTCH_CUT'})

            # ------ Phase 2 -----

            smach.StateMachine.add('EXECUTE_PARTIAL_TRAJ_2',
                ExecutePartialTraj2(self.davinciArmRight, traj, left_arm),
                transitions={'complete':'TRANSFER'}, remapping ={'new_traj_R':'sm_data1'})

            smach.StateMachine.add('TRANSFER',
                PhaseTransfer(self.davinciArmRight, traj, left_arm),
                transitions={'success':'EXECUTE_PARTIAL_TRAJ_3'}, remapping ={'new_traj_R':'sm_data1'})

            # smach.StateMachine.add('EXECUTE_PARTIAL_TRAJ_2',
            #     ExecutePartialTraj2(self.davinciArmRight, traj, left_arm),
            #     transitions={'looping':'CHECK_POSITION_2', 'complete':'EXECUTE_PARTIAL_TRAJ_3'})
            # smach.StateMachine.add('CHECK_POSITION_2',
            #     CheckPostion2(self.davinciArmRight),
            #     transitions={'success':'EXECUTE_PARTIAL_TRAJ_2', 'above': 'WARP_ABOVE_2', 'below': 'WARP_BELOW_2'})
            # smach.StateMachine.add('WARP_ABOVE_2',
            #     WarpAbove2(self.davinciArmRight),
            #     transitions={'success':'MOVE_ABOVE_2'})
            # smach.StateMachine.add('MOVE_ABOVE_2',
            #     MoveAbove2(self.davinciArmRight),
            #     transitions={'success':'CHECK_POSITION_2'})
            # smach.StateMachine.add('WARP_BELOW_2',
            #     WarpBelow2(self.davinciArmRight),
            #     transitions={'success':'MOVE_BELOW_2'})
            # smach.StateMachine.add('MOVE_BELOW_2',
            #     MoveBelow2(self.davinciArmRight),
            #     transitions={'success':'CHECK_POSITION_2'})

            # ------ Phase 3 -----
            smach.StateMachine.add('EXECUTE_PARTIAL_TRAJ_3',
                ExecutePartialTraj3(self.davinciArmRight, traj, left_arm),
                transitions={'abridged_state_machine':'SUCCESS'}, remapping ={'new_traj_R':'sm_data1'})

    def run(self):
        self.davinciArmRight.start()
        rospy.sleep(2)
        print 'master run method'
        rate = rospy.Rate(1)
        try:
            self.state_machine.execute()
        except Exception, e:
            print e

        while not rospy.is_shutdown():
            if not self.state_machine.is_running():
                print "State Machine stopped"
                rospy.sleep(0.5)
                break
            rate.sleep()

        self.davinciArmRight.stop()

def get_full_traj():
    traj_L = {}
    traj_R = {}
    file_prefix = "traj/teleop5_"
    for i in range(8):
        if i != 0:
            pickle_file_name = file_prefix + str(i) + '.p'
            traj_L[i]= pickle.load(open(pickle_file_name, 'rb'))[0]
            traj_R[i]= pickle.load(open(pickle_file_name, 'rb'))[1]
    traj = (traj_L, traj_R)
    return traj

def main():
    # print "Started main"
    # smach.loginfo("Starting main...")
    # # traj = get_full_traj()
    # traj = (1, 1)
    # print "Loaded traj"
    # dummy_left_arm = None
    # right_arm = MasterClass(traj[1], dummy_left_arm)
    # davinciArmLeft = RavenArm(raven_constants.Arm.Left)
    # davinciArmLeft.start()
    # left_arm = Follower(traj[0], davinciArmLeft)
    # print "Follower started"
    # print "Master created"
    # right_arm.run()
    print "Started main"
    smach.loginfo("Starting main...")
    traj = get_full_traj()
    # traj = (1, 1)
    print "Loaded traj"
    master = MasterClass(traj)
    print "Master created"
    master.run()

if __name__ == '__main__':
    main()