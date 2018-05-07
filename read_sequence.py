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
    def __init__(self, name_seq):
        self._seq_location = ""
        self._pose_location = ""
        self._name_seq = name_seq
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

                    # orientation = Quaternion(x = content[3], y=content[4], z = content[5], w = content[6])
        return pos

    def _get_grippers (self):
        content = self._read_file()
        return content[-2:] # check


    def _read_seq(self):
        self._seq_location = "/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+self._name_seq+"_sequence.txt"
        f = open (self._seq_location, "r")
        content = f.readline()
        content = content.split()
        return content

    def _change_to_poses(self):
        #get sequence as list of strings/names of poses - transfer to poses
        seq = self._read_seq()
        sequence_dictionary = {}
        for i in seq:
            self._pose_location = "/home/nitro/sawyer_ws/src/sawyer_gripper/src/poses/"+i+".txt"
            seq_j6 = self._get_joint_angles()
            sequence_dictionary[i] = [self._get_pose(),seq_j6['right_j6'],self._get_grippers()]
        return sequence_dictionary




#       Class and function for moving to direct poses, according to sequence
#-----------------------------------------------------------------------------------------------------



class Pick_and_Place(object):
    def __init__(self, limb = "right", hover_distance = 0.135, tip_name = "right_gripper_tip"):
        self._limb_name = limb
        self._tip_name = tip_name
        self._hover_distance = hover_distance
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()
        self._current_pose = Pose()
        self._grippers = []
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if start_angles:
            self._limb.move_to_neutral()
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
            self._guarded_move_to_joint_position(start_angles)
        self._gripper_action(0.04)
    
    def _approach(self, pose, angle = 0):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self._limb.ik_request(approach, "right_hand")
        self._limb.set_joint_position_speed(0.22)
        #joint_angles['right_j6'] = angle
        self._guarded_move_to_joint_position(joint_angles)
        self._limb.set_joint_position_speed(0.3)

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        self._servo_to_pose(ik_pose)


    def _servo_to_pose(self, pose, time=4.0, steps=400.0):
        self._limb.set_joint_position_speed(0.22)
        ''' An *incredibly simple* linearly-interpolated Cartesian move '''
        r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
        current_pose = self._limb.endpoint_pose()
        #print str(current_pose['position'].z)+"\n hover distance: "+str(self._hover_distance)
        ik_delta = Pose()
        ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps
        ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
        ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
        ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
        ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
        ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
        ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps
        for d in range(int(steps), -1, -1):
            if rospy.is_shutdown():
                return
            ik_step = Pose()
            ik_step.position.x = d*ik_delta.position.x + pose.position.x
            ik_step.position.y = d*ik_delta.position.y + pose.position.y
            ik_step.position.z = d*ik_delta.position.z + pose.position.z
            ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
            ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
            ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
            ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
            joint_angles = self._limb.ik_request(ik_step, "right_hand")
            if joint_angles:
                self._limb.set_joint_positions(joint_angles)
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            r.sleep()
        rospy.sleep(1.0)

    def _gripper_action(self, object_width):
        if object_width == 'MAX_POSITION':
            self._gripper.open()
        elif object_width == '0':
            self._gripper.close()
        else:
            self._gripper.set_position(float(object_width))
        rospy.sleep(1.0)


    def action(self, sequence):
        while not rospy.is_shutdown():
            for i in range (len(sequence)):
                self._limb.set_joint_position_speed(0.22)
                self._grippers = self._sequence[sequence[i]][2]
                print self._grippers
                #self._move_mid_pose(self._current_pose)
                self._current_pose = self._sequence[sequence[i]][0]
                self._approach(self._current_pose)
                self._gripper_action(self._grippers[0])
                self._servo_to_pose(self._current_pose)
                rospy.sleep(1)
                self._gripper_action(self._grippers[1])
                self._retract()
                rospy.sleep(3)


def main():

    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4':  -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}

    rospy.init_node('read_sequence')

    name_seq = raw_input("What sequence would you like to run?: ")
    rs = Read_Sequence(name_seq)
    pnp = Pick_and_Place()
    sequence = rs._read_seq()
    pnp._sequence = rs._change_to_poses()


    while not rospy.is_shutdown():
        pnp.move_to_start(starting_joint_angles)
        #print pnp._sequence['Test']
        raw_input("Click enter to start sequence: ")
        pnp.action(sequence)

if __name__ == '__main__':
    sys.exit(main())
