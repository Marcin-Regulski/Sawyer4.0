#! /usr/bin/env python

# Copyright (c) 2016-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb
import rospy

def move_to_angles(r_joint_angles, speed_ratio=0.1, accel_ratio = 0.3, timeout = None, limb = "right_hand"):

    if not rospy.is_shutdown():

        limb = Limb()
        traj = MotionTrajectory(limb = limb)

        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=speed_ratio,
                                            max_joint_accel=accel_ratio)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        req_joint_angles = list(reversed(r_joint_angles))
        joint_angles = limb.joint_ordered_angles()
        waypoint.set_joint_angles(joint_angles = joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        if len(req_joint_angles) != len(joint_angles):
            rospy.logerr('The number of joint_angles must be %d', len(joint_angles))
            return None

        waypoint.set_joint_angles(joint_angles = req_joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=timeout)
        if result is None:
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                        result.errorId)