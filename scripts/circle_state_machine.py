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

class MasterClass:
    def __init__(self):
        rospy.init_node('Master_SM_Circle',anonymous=False)
        self.state_machine = smach.StateMachine(outcomes=['SUCCESS'])
        # self.davinciArmRight = RavenArm(raven_constants.Arm.Right)
        # self.davinciArmLeft = RavenArm(raven_constants.Arm.Left)
        self.davinciArmRight = None
        self.davinciArmLeft = None
        self.setup_state_machine()

    def setup_state_machine(self):
        smach.loginfo("Initializing state machine")
        rospy.sleep(1)

        with self.state_machine:

            # ------ Phase 1 -----

            smach.StateMachine.add('START',
                Start(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'DETECT_CIRCLE'})
            smach.StateMachine.add('DETECT_CIRCLE',
                DetectCircle(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'WARP'})
            smach.StateMachine.add('WARP',
                Warp(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'GRASP_GAUZE'})
            smach.StateMachine.add('GRASP_GAUZE',
                GraspGauze(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_GRASP'})
            smach.StateMachine.add('CHECK_GRASP',
                CheckGrasp(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'EXECUTE_NOTCH_CUT', 'failure': 'WARP_GRASP'},)
            smach.StateMachine.add('WARP_GRASP',
                WarpGrasp(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'EXECUTE_REGRASP'})
            smach.StateMachine.add('EXECUTE_REGRASP',
                ExecuteRegrasp(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_GRASP'})
            smach.StateMachine.add('EXECUTE_NOTCH_CUT',
                ExecuteNotchCut(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_NOTCH'})
            smach.StateMachine.add('CHECK_NOTCH',
                CheckNotch(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'EXECUTE_PARTIAL_TRAJ_2', 'failure':'EXECUTE_NOTCH_CUT'})

            # ------ Phase 2 -----

            smach.StateMachine.add('EXECUTE_PARTIAL_TRAJ_2',
                ExecutePartialTraj2(self.davinciArmLeft, self.davinciArmRight),
                transitions={'looping':'CHECK_POSITION_2', 'complete':'EXECUTE_PARTIAL_TRAJ_3'})
            smach.StateMachine.add('CHECK_POSITION_2',
                CheckPostion2(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'EXECUTE_PARTIAL_TRAJ_2', 'above': 'WARP_ABOVE_2', 'below': 'WARP_BELOW_2'})
            smach.StateMachine.add('WARP_ABOVE_2',
                WarpAbove2(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'MOVE_ABOVE_2'})
            smach.StateMachine.add('MOVE_ABOVE_2',
                MoveAbove2(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_POSITION_2'})
            smach.StateMachine.add('WARP_BELOW_2',
                WarpBelow2(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'MOVE_BELOW_2'})
            smach.StateMachine.add('MOVE_BELOW_2',
                MoveBelow2(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_POSITION_2'})

            # ------ Phase 3 -----

            smach.StateMachine.add('EXECUTE_PARTIAL_TRAJ_3',
                ExecutePartialTraj2(self.davinciArmLeft, self.davinciArmRight),
                transitions={'looping':'CHECK_POSITION_3', 'complete':'EXECUTE_PARTIAL_TRAJ_TOP'})
            smach.StateMachine.add('CHECK_POSITION_3',
                CheckPostion3(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'EXECUTE_PARTIAL_TRAJ_3', 'above': 'WARP_ABOVE_3', 'below': 'WARP_BELOW_3'})
            smach.StateMachine.add('WARP_ABOVE_3',
                WarpAbove3(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'MOVE_ABOVE_3'})
            smach.StateMachine.add('MOVE_ABOVE_3',
                MoveAbove3(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_POSITION_3'})
            smach.StateMachine.add('WARP_BELOW_3',
                WarpBelow3(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'MOVE_BELOW_3'})
            smach.StateMachine.add('MOVE_BELOW_3',
                MoveBelow3(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_POSITION_3'})

            # ------ Phase 4 ----- (NEED to include part where the left arm retracts the gauze back)
                # ------ Phase 4-A - TOP -----

            smach.StateMachine.add('EXECUTE_PARTIAL_TRAJ_TOP',
                ExecutePartialTrajTop(self.davinciArmLeft, self.davinciArmRight),
                transitions={'looping':'CHECK_POSITION_TOP', 'complete':'EXECUTE_PARTIAL_TRAJ_BOTTOM'})
            smach.StateMachine.add('CHECK_POSITION_TOP',
                CheckPostionTop(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'EXECUTE_PARTIAL_TRAJ_TOP', 'left': 'WARP_LEFT_TOP', 'right': 'WARP_RIGHT_TOP'})
            smach.StateMachine.add('WARP_LEFT_TOP',
                WarpLeftTop(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'MOVE_LEFT_TOP'})
            smach.StateMachine.add('MOVE_LEFT_TOP',
                MoveLeftTop(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_POSITION_TOP'})
            smach.StateMachine.add('WARP_RIGHT_TOP',
                WarpRightTop(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'MOVE_RIGHT_TOP'})
            smach.StateMachine.add('MOVE_RIGHT_TOP',
                MoveRightTop(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_POSITION_TOP'})

                # ------ Phase 4-B - BOTTOM -----

            smach.StateMachine.add('EXECUTE_PARTIAL_TRAJ_BOTTOM',
                ExecutePartialTrajBottom(self.davinciArmLeft, self.davinciArmRight),
                transitions={'looping':'CHECK_POSITION_BOTTOM', 'complete':'SUCCESS'})
            smach.StateMachine.add('CHECK_POSITION_BOTTOM',
                CheckPostionBottom(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'EXECUTE_PARTIAL_TRAJ_BOTTOM', 'left': 'WARP_LEFT_BOTTOM', 'right': 'WARP_RIGHT_BOTTOM'})
            smach.StateMachine.add('WARP_LEFT_BOTTOM',
                WarpLeftBottom(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'MOVE_LEFT_BOTTOM'})
            smach.StateMachine.add('MOVE_LEFT_BOTTOM',
                MoveLeftBottom(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_POSITION_BOTTOM'})
            smach.StateMachine.add('WARP_RIGHT_BOTTOM',
                WarpRightBottom(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'MOVE_RIGHT_BOTTOM'})
            smach.StateMachine.add('MOVE_RIGHT_BOTTOM',
                MoveRightBottom(self.davinciArmLeft, self.davinciArmRight),
                transitions={'success':'CHECK_POSITION_BOTTOM'})

    def run(self):
        # self.davinciArmRight.start()
        # self.davinciArmLeft.start()
        rospy.sleep(2)

        rate = rospy.Rate(1)
        try:
            self.state_machine.execute()
        except Exception, e:
            print e

        while not rospy.is_shutdown():
            if not self.state_machine.is_running():
                print "State Machine stopped"
                rospy.sleep(0.5)
                rospy.signal_shutdown('state machines finished')
                break
            rate.sleep()

        # self.davinciArmRight.stop()
        # self.davinciArmLeft.stop()

def main():
    smach.loginfo("Starting main...")
    master = MasterClass()
    master.run()


if __name__ == '__main__':
    main()