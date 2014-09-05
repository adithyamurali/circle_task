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
`
    bag = rosbag.Bag(sys.argv[1])

    output_bag_name = sys.argv[2]

    pose_topic = '/dvrk_psm1/joint_position_cartesian'
    gripper_angle_topic = '/dvrk_psm1/gripper_position'

    pose_and_angle_tuples = {}
    currTime = None
    i = 0

    full_traj = {}

    for topic, msg, t in bag.read_messages(topics=[pose_topic, gripper_angle_topic]):
        if topic == pose_topic:
            currPose = tfx.pose(msg)
        if topic == gripper_angle_topic:
            pair = (currPose, msg.data)
            pose_and_angle_tuples[i] = pair
            i += 1

    # Robot State is a list of tuple(Pose, Gripper Open Angle)
    list_pose_and_angle =  []
    for value in pose_and_angle_tuples.values():
        list_pose_and_angle.append(value)

    # rand_pose = (tfx.random_pose().msg.PoseStamped(), 1.0)
    # list_robot_state = [rand_pose, rand_pose]
    # IPython.embed()

    pickle.dump(list_pose_and_angle, open(output_bag_name, 'wb' ))

    bag.close()

if __name__ == '__main__':
    main()
