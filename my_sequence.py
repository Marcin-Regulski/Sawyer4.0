#!/usr/bin/env python

import argparse
import copy
import struct
import sys

import imp

read_seq = imp.load_source("Read_sequence", "/home/nitro/sawyer_ws/src/sawyer_gripper/src/read_sequence.py")

import rospkg

import intera_interface
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

class MySequence(object):
    def __init__(self,limb = 'right', hover_distance = 0.2, tip_name = "right_gripper_tip"):
        self._limb_name = limb
        self._tip_name = tip_name
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    #def _get_sequence()


    def _move_to_pose(self, )


def main():
    rospy.init_node ('my_sequence')
    rs = read_seq.Read_Sequence()

    while not rospy.is_shutdown():
        rs._change_to_poses()
        print rs._sequence

if __name__ == '__main__':
    sys.exit(main())