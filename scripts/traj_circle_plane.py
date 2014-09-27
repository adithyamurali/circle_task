#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import rospy
import IPython
import pickle
import tfx


def get_full_traj():
    traj_L = {}
    traj_R = {}
    file_prefix = "traj/teleop5_"
    for i in range(8):
        if (i == 3) or (i == 5):
            pickle_file_name = file_prefix + str(i) + '.p'
            traj_R[i]= pickle.load(open(pickle_file_name, 'rb'))[1]
    return traj_R

def test_cut_threshold(traj, threshold):
    count = 0
    for sample in traj:
        if sample[1] < threshold:
            count += 1
    print count

def test_open_threshold(traj, threshold):
    count = 0
    for sample in traj:
        if sample[1] > threshold:
            count += 1
    print count

def main():
    rospy.init_node('circle_traj_sampler',anonymous=False)
    traj = get_full_traj()
    samples = []
    output_file_name = "circle_plane_samples.p"
    collect_sample = True
    cut_threshold_angle_3 = -2 # This value was found by sequential convex optimization
    cut_threshold_angle_5 = -2
    open_threshold_angle_3 = 0.8
    open_threshold_angle_5 = 1
    # IPython.embed()
    for sample in traj[3]:
        a = tfx.random_point()
        if (sample[1] < cut_threshold_angle_3) and collect_sample:
            sample_pose = sample[0]
            a.x = sample_pose.position.x
            a.y = sample_pose.position.y
            a.z = sample_pose.position.z
            samples.append(a)
            collect_sample = False
        if sample[1] > open_threshold_angle_3:
            collect_sample = True

    print "# of samples", len(samples)
    collect_sample = True
    for sample in traj[5]:
        a = tfx.random_point()
        if (sample[1] < cut_threshold_angle_5) and collect_sample:
            sample_pose = sample[0]
            a.x = sample_pose.position.x
            a.y = sample_pose.position.y
            a.z = sample_pose.position.z
            samples.append(a)
            collect_sample = False
        if sample[1] > open_threshold_angle_5:
            collect_sample = True

    print "# of samples", len(samples)
    IPython.embed()
    pickle.dump(samples, open(output_file_name, 'wb' ))

if __name__ == '__main__':
    main()