#!/usr/bin/env python

# Aim of this script is to read all data from saved sequence and its positions and apss it as simple disctionary

import argparse
import struct
import sys
import copy

import rospy
import rospkg
import intera_interface

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

class Read_Sequence(object):
    def __init__(self, limb = "right", hover_distance = 0.3, tip_name = "right_gripper_tip"):
        self._limb_name = limb
        self._tip_name = tip_name
        self._hover_distance = hover_distance
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()
        self._sequence = {}
        self._seq_location = ""
        self._pose_location = ""
        self._overhead_orientation = Quaternion(
                             x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936)

    def _read_file(self):
        with open(self._pose_location, "r") as f:
            content = f.readlines()
            content = [x.strip() for x in content]
        f.close()
        return content

    def _get_joint_angles(self):
        content = self._read_file()
        content = content[10:-3] #check if correct!!
        joint_angles = {}
        for i in range(len(content)):
            joint_angles['right_j'+str(6-i)] = float(content[i])
        return joint_angles

    def _get_pose (self):
        content = self._read_file()
        content = content[1:-11] # check if correct!!
        del content[3]
        for i in range(len(content)):
            item = content[i]
            content[i] = float(item[3:])

        pos = Pose(position = Point(x = content[0], y = content[1], z= content[2]), 
                    orientation = self._overhead_orientation)
        return pos

    def _get_grippers (self):
        content = self._read_file()
        return content[-2:] # check


    def _read_seq(self):
        name_seq = "Test"
        self._seq_location = "/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+name_seq+"_sequence.txt"
        f = open (self._seq_location, "r")
        content = f.readline()
        content = content.split()
        return content

    def _change_to_poses(self):
        #get sequence as list of strings/names of poses - transfer to poses
        seq = self._read_seq()
        #print str(seq)+"\n"
        seq_poses = []
        seq_grippers = []
        for i in seq:
            self._pose_location = "/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+i+".txt"
            seq_j6 = self._get_joint_angles()
            self._sequence[i] = [self._get_pose(),seq_j6['right_j6'],self._get_grippers()]
            

def main():
    rospy.init_node('read_sequence')
    while not rospy.is_shutdown():
        rs = Read_Sequence()
        rs._change_to_poses()
        print rs._sequence

if __name__ == '__main__':
    sys.exit(main())
