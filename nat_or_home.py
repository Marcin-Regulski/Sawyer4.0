#!/usr/bin/env python
import rospy
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

def nat_or_home(position):
    if position == "n":
        limb.move_to_neutral()
        #print (limb.joint_angles())
        #current_pose = limb.endpoint_pose() # get current endpoint pose
        print 'Moved to natural pose: '+ current_pose 
        print 'X: '+ str(current_pose['position'].x) 
        rospy.sleep(0.5)
    elif position == "h":

        
        overhead_orientation = Quaternion(
                             x=0.7,
                             y=-0.7,
                             z=0.0,
                             w=0.0)

        new_pose = Pose(position= Point(x = 0.8, y= 0, z = 0.403), orientation = overhead_orientation)

        joint_angles = limb.ik_request(new_pose, "right_hand_camera")
        limb.move_to_joint_positions(joint_angles)
        print joint_angles
        #print (limb.joint_angles())
        #current_pose = limb.endpoint_pose() # get current endpoint pose
        #print 'Moved to home pose: ' + current_pos
    rospy.sleep(0.5)
    return 0




rospy.init_node('Hello_Sawyer')
limb = intera_interface.Limb('right')
current_pose = limb.endpoint_pose() 
print (current_pose)

home_position = {'right_j6': 0.0, 'right_j5': 0.0, 'right_j4': 0.0, 'right_j3': 0.0, 'right_j2': 0.0, 'right_j1': -1.6, 'right_j0': -0.0}
current_pose = limb.endpoint_pose
print current_pose
question = "What position do you want to go to n or h ? "
chosen_position = raw_input(question).lower()
print(chosen_position)
nat_or_home(chosen_position)