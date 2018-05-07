#!/usr/bin/env python
from go_angles import *
import intera_interface
import numpy
import sys
import cartesian_pose as car
# import go_angles as ga
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)


def main():

    rospy.init_node('Hello_Sawyer1')
    limb = intera_interface.Limb('right')
    pos = Pose(position= Point(x=0.4495578615599669, y=0.16031226557067035, z=0.48001540843259894), orientation = Quaternion(x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936) )
    joint_angles = limb.ik_request(pos, "right_hand")

    if angles(joint_angles, speed_ratio=0.05, accel_ratio=0.05):
        return
        

if __name__ == '__main__':
    sys.exit(main())
