#!/usr/bin/env python

# Aim of this script is to read all data from saved sequence and its positions and apss it as simple disctionary

import sys
import copy

from snap7.snap7types import *

import rospy
import rospkg
import intera_interface
import cartesian_pose as cp
import go_angles
import plc_connection
import quality_check
import server

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion
)
#CAMERA CHECK
# 
import cv2

decision = ''

class Station_Control(object):
    def __init__(self,limb = "right", hover_distance = 0.16, tip_name = "right_hand"):
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()
        self._limb_name = limb
        self._process = bool 
        self._hover_distance = hover_distance
        self._tip_name = tip_name
        self._starting_pose = Pose()
        self._gripper.set_cmd_velocity(0.16)

        self.speed_ratio = 0.3
        self.accel_ratio = 0.4
        self.linear_speed = 0.3
        self.linear_accel = 0.4
        self._Sequence = server.Sequence()
        self._camera_check = self._Sequence.poses['Camera_check']

        self._zero_pose = self._Sequence._zero_pose

        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def zero(self):
        print "Current values for zero_position are:\n x: "+str(self._zero_pose.position.x)+"\n y: "+str(self._zero_pose.position.y)+"\n z: "+str(self._zero_pose.position.z)
        decision = raw_input("Would you like to change them or move? y/n/m: ")
        if decision == "y":
            moving = True
            while moving:
                #NOTE Use the script for saving into text file and reading it!!!
                raw_input("Move to desired pose and click enter")
                print self._limb.endpoint_pose()
                decision2 = raw_input("Position correct? y/n: ")
                if decision2 == "y":
                    current_position = self._limb.endpoint_pose()
                    self._starting_pose = Pose(position = Point(x = current_position['position'].x,y = current_position['position'].y,z = current_position['position'].z ))
                    moving = False
                    rospy.sleep(1.5)
        elif decision == "m":
            self._pick(self._zero_pose,'0','0')

    def _approach(self, pose):
        if rospy.is_shutdown():
            return
        approach = copy.deepcopy(pose)
        print "approach going..."
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self._limb.ik_request(approach, self._tip_name)
        self._guarded_move_to_joint_position(joint_angles)
        print "Done"
        rospy.sleep(1)
        return

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            joint_list = list(joint_angles.values())
            go_angles.move_to_angles(joint_list, speed_ratio=self.speed_ratio, accel_ratio=self.accel_ratio)
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
        cp.go_to_cartesian(ik_pose.position, linear_speed=self.linear_speed, linear_accel=self.linear_accel, timeout= 1.0)
        rospy.sleep(1.0)
        return

    def _gripper_action(self, object_width):

        if object_width == 'M':
            self._gripper.open(0.05)
        elif object_width == '0':
            self._gripper.close(0)
            #print self._gripper.get_force()
        else:
            self._gripper.set_position(float(object_width))
        rospy.sleep(1.0)
        return

    def _pick(self,pose):
        if rospy.is_shutdown():
            return
        self._gripper_action(pose[1])
        self._approach(pose[0])
        cp.go_to_cartesian(pose[0].position,linear_speed=self.linear_speed, linear_accel=self.linear_accel, timeout = 1.0)
        rospy.sleep(1.0)
        self._gripper_action(pose[2])
        self._retract()
        return 

    def _pick_and_wait(self,pose):
        if rospy.is_shutdown():
            return
        self._gripper_action(pose[1])
        self._approach(pose[0])
        cp.go_to_cartesian(pose[0].position,linear_speed=self.linear_speed, linear_accel=self.linear_accel, timeout = 1.0)
        rospy.sleep(3.0)
        self._gripper_action(pose[2])
        self._retract()

    def start(self,stage):

        global decision

        while not rospy.is_shutdown() and self._process:
            if plc_connection.workpiece_avaiable() and not plc_connection.belt_off():
                plc_connection.process_ongoing()
                full_sequence = self._Sequence._create_sequence(stage)
                #loop through all items in a sequence list
                for i in range(0, len(full_sequence)):
                    target = full_sequence[i]
                    #print str(i) + "\n"+str(target)
                    #raw_input("OK")
                    if isinstance(target[0], Pose):

                        if target[0] == self._camera_check[0]:
                            print "its a Camera Check!"
                            cp.go_to_cartesian(self._camera_check[0].position, linear_speed=self.linear_speed, linear_accel=self.linear_accel, timeout = 1.0)
                            
                            #place for camera function
                            decision = quality_check.check()
                            print decision
                            cv2.destroyAllWindows()

                            self._retract()
                        else:
                            print "its a Pose!"
                            self._pick(target)


                    elif type(target) == list and type(target[0]) == float:
                        print "its a list!"
                        go_angles.move_to_angles(target, speed_ratio = self.speed_ratio, accel_ratio = self.accel_ratio)
                    else:
                        print "el problemo"
                        rospy.sleep(20)
                else:
                    print "Sequence finished"
                    plc_connection.process_finished()
                    self._process = False  
            else:
                print "Workpiece not avaiable. Waiting..."
                rospy.sleep(2.0)
        else:
            print "Process didnt start."

def main():
    rospy.init_node("station_control_sawyer")
    sc = Station_Control()

    while not rospy.is_shutdown():
        decision = raw_input("Would u like to check zero_position, start assembly or do testing? 0/a/t: ")
        sc._process = False
        
        if decision == "a":
            sc._process = True
            sc.start("assembly")

        elif decision == "0":
            sc.zero()
        elif decision == "t":
            sc._process = True
            sc.start("testing")

if __name__ == '__main__':
    sys.exit(main())


