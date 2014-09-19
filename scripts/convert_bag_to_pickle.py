#!/usr/bin/env python

import rosbag
import pickle
import tfx
import IPython
import rospy
import pickle
import sys

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from rospy import Time

def main():
    bag = rosbag.Bag(sys.argv[1])

    output_bag_name = sys.argv[2]

    # pose_topic = '/dvrk_psm1/joint_position_cartesian'
    # gripper_angle_topic = '/dvrk_psm1/gripper_position'

    # pose_and_angle_tuples = {}
    # i = 0

    pose_topic_R = '/dvrk_psm1/joint_position_cartesian'
    gripper_angle_topic_R = '/dvrk_psm1/gripper_position'

    pose_topic_L = '/dvrk_psm2/joint_position_cartesian'
    gripper_angle_topic_L = '/dvrk_psm2/gripper_position'

    pose_and_angle_tuples_R = {}
    pose_and_angle_tuples_L = {}
    i = 0
    j = 0
    curr_angle_R = None
    curr_pose_R = None
    curr_angle_L = None
    curr_pose_L = None

    for topic, msg, t in bag.read_messages(topics=[pose_topic_R, gripper_angle_topic_R, 
        pose_topic_L, gripper_angle_topic_L]):
        if topic == gripper_angle_topic_R:
            curr_angle_R = msg.data
        elif topic == pose_topic_R:
            curr_pose_R = tfx.pose(msg)
        elif topic == gripper_angle_topic_L:
            curr_angle_L = msg.data
        else:
            curr_pose_L = tfx.pose(msg)

        if curr_pose_R != None and curr_angle_R != None:
            pair = (curr_pose_R, curr_angle_R)
            pose_and_angle_tuples_R[i] = pair
            i += 1
            curr_angle_R = None
            curr_pose_R = None

        if curr_pose_L != None and curr_angle_L != None:
            pair = (curr_pose_L, curr_angle_L)
            pose_and_angle_tuples_L[j] = pair
            j += 1
            curr_angle_L = None
            curr_pose_L = None
    traj = (pose_and_angle_tuples_L.values(), pose_and_angle_tuples_R.values())
    # IPython.embed()
    pickle.dump(traj, open(output_bag_name, 'wb' ))

    bag.close()

if __name__ == '__main__':
    main()
