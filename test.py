#!/usr/bin/env python
import server
import argparse
import struct
import sys
import copy

import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

def main():
    cp = server.Sequence()
    target = cp.request_pose("assembly", 1)
    if isinstance(target[0],Pose):
        print "its a Pose!"
    elif type(target) == list:
        print "its a list!"
    else:
        print "faken"
    rospy.sleep(3.0)
    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())