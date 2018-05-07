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
    
def retract(Limb,hover_distance):
    limb = intera_interface.Limb(Limb)
    # retrieve current pose from endpoint
    current_pose = limb.endpoint_pose()
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


def save_joints_to_file(Limb, name):
    limb = intera_interface.Limb(Limb)
    current_angles = limb.joint_angles()

    f = open("/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+name+"_joints.txt","w+")
    f.write(str(current_angles['right_j0'])+"\n")
    f.write(str(current_angles['right_j1'])+"\n")
    f.write(str(current_angles['right_j2'])+"\n")
    f.write(str(current_angles['right_j3'])+"\n")
    f.write(str(current_angles['right_j4'])+"\n")
    f.write(str(current_angles['right_j5'])+"\n")
    f.write(str(current_angles['right_j6'])+"\n")
    f.close()

def save_pose_to_file(name,cp):
    f = open("/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+name+"_pose.txt","w+")
    f.write(str(cp))
    f.close()

'''
def save_gripper_width(name,gripper):
    gripper_beg,gripper_end = gripper.split()
    f = open("/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+name+"_gripper.txt","w+")
    f.write(gripper_beg+"\n"+gripper_end)
    f.close()
'''

def get_pose(name):
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

def create_sequence():
    sequence = []
    inp = raw_input("Type the sequence using spacing between poses: ")
    str_sequence = inp.split()

    for i in str_sequence:
        sequence.append(get_pose(i))

    print sequence
    return sequence


def main():

    sequence = []
    rospy.init_node('save_positions')
    hover_distance = 0.135
    main_limb = "right"
    limb = intera_interface.Limb(main_limb)

    while not rospy.is_shutdown():
        if sequence == [] :
            print "Hello! Please set up a gripper to the first location to save it"
        else:
            print "Please add next step to your sequence"
        raw_input("Click enter to continue: ")
        cp = retract(main_limb, hover_distance)
        print str(cp)
        rospy.sleep(1.0)
        decision = raw_input("Would you like to save that location? (y/n) ")

        if decision == "y" :
            name = raw_input("Name the location: ")
            #gripper = raw_input("What width of a gripper at the beggining and end (separate with space)?")

            save_pose_to_file(name,cp)
            save_joints_to_file(main_limb,name)
            #save_gripper_width(name,gripper)

        elif decision == "n" :
            return
        elif decision == "check":
            
            create_sequence()

if __name__ == '__main__':
    sys.exit(main())


    