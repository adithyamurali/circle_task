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

    pose_topic_R = '/dvrk_psm1/joint_position_cartesian'
    gripper_angle_topic_R = '/dvrk_psm1/gripper_position'

    pose_topic_L = '/dvrk_psm2/joint_position_cartesian'
    gripper_angle_topic_L = '/dvrk_psm2/gripper_position'

    pose_and_angle_tuples_R = {}
    pose_and_angle_tuples_L = {}
    i = 0
    j = 0
    time_index = 0
    time_frames = create_time_frames()

    complete_pairing_L = True
    complete_pairing_R = True

    traj = {}

    curr_time_frame = time_frames[time_index]

    for topic, msg, t in bag.read_messages(topics=[pose_topic_R, gripper_angle_topic_R, pose_topic_L, gripper_angle_topic_L]):
        if topic == pose_topic_R:
            curr_pose_R = tfx.pose(msg)
            complete_pairing_R = False
        elif topic == gripper_angle_topic_R:
            pose_and_angle_tuples_R[i] = (curr_pose_R, msg.data)
            i += 1
            complete_pairing_R = True
        elif topic == pose_topic_L:
            curr_pose_L = tfx.pose(msg)
            complete_pairing_L = False
        else:
            pose_and_angle_tuples_L[j] = (curr_pose_L, msg.data)
            j += 1
            complete_pairing_L = True
        if not within_time_frame(t.to_sec(), curr_time_frame):
            print "X"
            IPython.embed()
            if complete_pairing_L and complete_pairing_R:
                print "Y"
                IPython.embed()
                assert len(pose_and_angle_tuples_L.values()) == len(pose_and_angle_tuples_R.values())
                partial_traj = (pose_and_angle_tuples_L.values(), 
                    pose_and_angle_tuples_R.values())
                traj[time_index] = partial_traj
                time_index += 1
                curr_time_frame = time_frames[time_index]
                pose_and_angle_tuples_R = {}
                pose_and_angle_tuples_L = {}
            else:
                continue
    if (len(pose_and_angle_tuples_L) != 0) or (len(pose_and_angle_tuples_R) != 0):
        partial_traj = (pose_and_angle_tuples_L.values(), pose_and_angle_tuples_R.values())
        traj[time_index] = partial_traj
    # # Robot traj is a list of tuple(Pose, Gripper Open Angle)
    print "End"
    IPython.embed()
    pickle.dump(traj, open(output_bag_name, 'wb' ))

    bag.close()

def within_time_frame(t, time_frame):
    if time_frame[0] is None:
        return t <= time_frame[1]
    elif time_frame[1] is None:
        return t >= time_frame[0]
    else:
        return (t >= time_frame[0]) and (t <= time_frame[1])

def create_time_frames():
    time_frames = {}
    time_0 = 1409163345.1170287
    time_1 = 1409163381.539586
    time_2 = 1409163416.626796
    time_3 = 1409163433.579204 
    time_4 = 1409163461.470884
    time_5 = 1409163496.336294
    time_6 = 1409163539.732363
    time_7 = 1409163542.867822
    time_8 = 1409163558.397697
    time_9 = 1409163566.177478

    time_limits = [time_0, time_1, time_2, time_3, time_4, time_5, time_6, time_7, time_8, time_9]
    for i in range(len(time_limits)):
        if (i + 1) == len(time_limits):
            time_frames[i] = (time_limits[i], None)
        else:
            time_frames[i] = (time_limits[i], time_limits[i+1])
    return time_frames

if __name__ == '__main__':
    main()
