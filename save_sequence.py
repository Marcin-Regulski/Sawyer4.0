#!/usr/bin/env python

# Aim of this script is to detect current joint positions and x,y,z of 
# gripper and save it in dictionary as set up locations for a robot for later use

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

class Save_Positions(object):
    def __init__(self, limb="right", hover_distance = 0.3, tip_name = "right_gripper_tip"):
        self._limb_name = limb # string
        self._tip_name = tip_name # string
        self._hover_distance = hover_distance # in meters
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()
        self._sequence = []
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        #ik_pose = current_pose
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        return ik_pose

    def save_sequence(self,name_seq):
        while not rospy.is_shutdown():
            f = open("/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+name_seq+"_sequence.txt","w+")
            for i in range(0,len(self._sequence)):
                f.write(self._sequence[i]+" ")
            f.close()
            return

    def save_values_to_file(self,name,grip_in):
        grip_widths = []
        self._sequence.append(name)
        grip_widths = grip_in.split()
        current_angles = self._limb.joint_angles()
        cp = self._retract()

        f = open("/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+name+".txt","w+")
        f.write(str(cp)+"\n")
        f.write("Joint angles:\n")
        f.write(str(current_angles['right_j6'])+"\n")
        f.write(str(current_angles['right_j5'])+"\n")
        f.write(str(current_angles['right_j4'])+"\n")
        f.write(str(current_angles['right_j3'])+"\n")
        f.write(str(current_angles['right_j2'])+"\n")
        f.write(str(current_angles['right_j1'])+"\n")
        f.write(str(current_angles['right_j0'])+"\n")
        f.write("Gripper widths - before and after: \n")

        grip_widths = ['0' if x == 'c' else x for x in grip_widths]
        grip_widths = ['MAX_POSITION' if x == 'o' else x for x in grip_widths]

        f.write(grip_widths)
        f.close()

def get_info(sp):
    while not rospy.is_shutdown():
        name = raw_input("Name of position: ")
        grippers_in = raw_input ("Set up width of grippers for beggining and end of position - leave space between values (c- close /o - open): ")
        sp.save_values_to_file(name,grippers_in)
        break


def main():
    rospy.init_node("Save_sequence")

    sp = Save_Positions()
    while not rospy.is_shutdown():
        raw_input( "Hello! Move Sawyer to the first location and click enter to save it!")
        get_info(sp)
        while True:
            decision = raw_input("Would you like to add new position or save sequence? p/s ")
            if decision == 'p':
                get_info(sp)
            elif decision == 's':
                print sp._sequence
                name_seq = raw_input("Name of sequence: ")
                sp.save_sequence(name_seq)
                break
            

if __name__ == '__main__':
    sys.exit(main())


    