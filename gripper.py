#!/usr/bin/env python

import argparse
import sys
import rospy
import struct
import intera_interface
import intera_external_devices

rospy.init_node('Sequence_Test')
limb = intera_interface.Limb('right')
current_pose = limb.endpoint_pose()
gripper = intera_interface.Gripper()

#Function for operating gripper (opening/closing)
def gripper_action(object_width):
    current_width = gripper.get_position()
    if object_width > current_width:
        gripper.open(object_width)
    elif object_width <current_width:
        gripper.close(object_width)
    rospy.sleep(2.5)

def main():
    limb = intera_interface.Limb('right')
    print limb
    current_angles = limb.joint_angles()
    starting_joint_angles = current_angles
    #starting_joint_angles['right_j6'] = 0
    #grip = intera_interface.Gripper()
    #grip.calibrate()
                             
    
    while not rospy.is_shutdown():
        current_pose = limb.endpoint_pose()
        print current_pose
        limb.move_to_joint_positions(starting_joint_angles,timeout = 5.0)
        question = "What width would you like to set the gripper?"
        choose = float(raw_input(question))
        gripper_action(choose/100)
    return 0

if __name__ == '__main__':
    sys.exit(main())