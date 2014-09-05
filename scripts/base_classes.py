#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import time
import tfx
import smach
import rospy

from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm

from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped

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

    def execute(self, userdata):        
        smach.loginfo(self.__class__.__name__ + " must be subclassed. It does nothing.")
        time.sleep(SLEEP_TIME)
        self.counter += 1
        if self.outcomes is None:
            self.outcomes = self.get_registered_outcomes()
        return self.outcomes[self.counter%len(self.outcomes)]

# -----------------------   Phase 1  -------------------------------------

class Start(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class DetectCircle(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class Warp(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class ExecuteRegrasp(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'


class GraspGauze(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class CheckGrasp(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class WarpGrasp(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'
class ExecuteNotchCut(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class CheckNotch(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

# -----------------------   Phase 2  -------------------------------------

class ExecutePartialTraj2(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['looping', 'complete'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.counter = 0

    def execute(self, userdata):
        self.counter += 1
        if self.counter == 2:
            return 'complete'
        return 'looping'

class CheckPostion2(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'below', 'above'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class WarpAbove2(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class MoveAbove2(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class WarpBelow2(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class MoveBelow2(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'


# -----------------------   Phase 3  -------------------------------------

class ExecutePartialTraj3(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['looping', 'complete'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.counter = 0

    def execute(self, userdata):
        self.counter += 1
        if self.counter == 2:
            return 'complete'
        return 'looping'

class CheckPostion3(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'below', 'above'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class WarpAbove3(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class MoveAbove3(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class WarpBelow3(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class MoveBelow3(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

# -----------------------   Phase 4A - TOP  -------------------------------------

class ExecutePartialTrajTop(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['looping', 'complete'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.counter = 0

    def execute(self, userdata):
        self.counter += 1
        if self.counter == 2:
            return 'complete'
        return 'looping'

class CheckPostionTop(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'left', 'right'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class WarpLeftTop(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class MoveLeftTop(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class WarpRightTop(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class MoveRightTop(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'
# -----------------------   Phase 4A - BOTTOM  -------------------------------------

class ExecutePartialTrajBottom(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['looping', 'complete'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.counter = 0

    def execute(self, userdata):
        self.counter += 1
        if self.counter == 2:
            return 'complete'
        return 'looping'

class CheckPostionBottom(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'left', 'right'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class WarpLeftBottom(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class MoveLeftBottom(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class WarpRightBottom(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'

class MoveRightBottom(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        return 'success'
