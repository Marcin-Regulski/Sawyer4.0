#! /usr/bin/env python

import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb


def go_to_cartesian(position=None,orientation= None, joint_angles = None,linear_speed = 0.1, rotational_speed = 0.57, linear_accel = 0.3,
                    tip_name = 'right_hand', relative_pose = None, rotational_accel = 0.57, timeout = None ):

    if not rospy.is_shutdown():
        try:
            limb = Limb()

            traj_options = TrajectoryOptions()
            traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
            traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

            wpt_opts = MotionWaypointOptions(max_linear_speed=linear_speed,
                                            max_linear_accel=linear_accel,
                                            max_rotational_speed=rotational_speed,
                                            max_rotational_accel=rotational_accel,
                                            max_joint_speed_ratio=1.0)
            waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

            joint_names = limb.joint_names()

            if joint_angles and len(joint_angles) != len(joint_names):
                rospy.logerr('len(joint_angles) does not match len(joint_names!)')
                return None

            if (position is None and orientation is None
                and relative_pose is None):
                if joint_angles:
                    # does Forward Kinematics
                    waypoint.set_joint_angles(joint_angles, tip_name, joint_names)
                else:
                    rospy.loginfo("No Cartesian pose or joint angles given. Using default")
                    waypoint.set_joint_angles(joint_angles=None, active_endpoint=tip_name)
            else:
                endpoint_state = limb.tip_state(tip_name)
                if endpoint_state is None:
                    rospy.logerr('Endpoint state not found with tip name %s', tip_name)
                    return None
                pose = endpoint_state.pose

                if relative_pose is not None:
                    if len(relative_pose) != 6:
                        rospy.logerr('Relative pose needs to have 6 elements (x,y,z,roll,pitch,yaw)')
                        return None
                    # create kdl frame from relative pose
                    rot = PyKDL.Rotation.RPY(relative_pose[3],
                                            relative_pose[4],
                                            relative_pose[5])
                    trans = PyKDL.Vector(relative_pose[0],
                                        relative_pose[1],
                                        relative_pose[2])
                    f2 = PyKDL.Frame(rot, trans)
                    # and convert the result back to a pose message
                    if in_tip_frame:
                    # end effector frame
                        pose = posemath.toMsg(posemath.fromMsg(pose) * f2)
                    else:
                    # base frame
                        pose = posemath.toMsg(f2 * posemath.fromMsg(pose))
                else:
                    if position is not None:
                        pose.position.x = position.x
                        pose.position.y = position.y
                        pose.position.z = position.z
                    if orientation is not None:
                        pose.orientation.x = orientation.x
                        pose.orientation.y = orientation.y
                        pose.orientation.z = orientation.z
                        pose.orientation.w = orientation.w
                poseStamped = PoseStamped()
                poseStamped.pose = pose
                waypoint.set_cartesian_pose(poseStamped, tip_name, joint_angles)

            #rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

            traj.append_waypoint(waypoint.to_msg())

            result = traj.send_trajectory(timeout=timeout)
            if result is None:
                return

            if result.result:
                rospy.loginfo('Motion controller successfully finished the trajectory!')
            else:
                rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                            result.errorId)
        except rospy.ROSInterruptException:
            rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')
