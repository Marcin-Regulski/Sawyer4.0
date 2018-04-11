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

class Save_Positions(self, limb = "right", hover_distance = 0.3, tip_name = "right_gripper_tip"):
    self._limb_name = limb # string
    self._tip_name = tip_name # string
    self._hover_distance = hover_distance # in meters
    self._limb = intera_interface.Limb(limb)
    self._gripper = intera_interface.Gripper()
    # verify robot is enabled
    print("Getting robot state... ")
    self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    self._init_state = self._rs.state().enabled
    print("Enabling robot... ")
    self._rs.enable()

def retract(self):
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

def save_values_to_file(name,grip_in):
    grip_widths = []
    grip_widths = grip_in.split()
    current_angles = self._limb.joint_angles()
    cp = self._retract()

    f = open("/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+name+".txt","w+")
    f.write(str(cp))

    f.write(str(current_angles['right_j6'])+"\n")
    f.write(str(current_angles['right_j5'])+"\n")
    f.write(str(current_angles['right_j4'])+"\n")
    f.write(str(current_angles['right_j3'])+"\n")
    f.write(str(current_angles['right_j2'])+"\n")
    f.write(str(current_angles['right_j1'])+"\n")
    f.write(str(current_angles['right_j0'])+"\n")
    f.write(grip_widths[0]+"\n"+grip_widths[1]+"\n")
    f.close()
 

'''
def save_gripper_width(name,gripper):
    gripper_beg,gripper_end = gripper.split()
    f = open("/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+name+"_gripper.txt","w+")
    f.write(gripper_beg+"\n"+gripper_end)
    f.close()
'''

def get_values(name):
    file_location = "/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+name+"_pose.txt"
            
    with open(file_location, "r") as f:
        content = f.readlines()
        content = [x.strip() for x in content]
        del content[0]
        del content[3]
        for i in range(len(content)):
            item = content[i]
            content[i] = float(item[3:])

    Pos = Pose(position = Point(x = content[0], y = content[1], z= content[2]), 
                    orientation = Quaternion(x=content[3],y=content[4],z=content[5],w=content[6]))
    f.close()

    return Pos


def main():



if __name__ == '__main__':
    sys.exit(main())


    