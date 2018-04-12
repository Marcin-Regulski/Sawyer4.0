#!/usr/bin/env python

import argparse
import copy
import struct
import sys

import rospkg

import intera_interface
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

class MySequence(object):
    def __init__(self,):