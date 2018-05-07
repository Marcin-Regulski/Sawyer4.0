#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
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

import argparse
import numpy as np
import sys
import glob

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface

decision = ''

def show_image_callback(img_data, (edge_detection, window_name)):
    """The callback function to show image by using CvBridge and cv
    """
    global decision
    template_data = []
    files1 = glob.glob('/home/nitro/sawyer_ws/src/sawyer_gripper/src/images/templates/no*')

    names = ['no1','no2','no3','no4','no5','green1','green2','green3','red1','red2','red3','red4','red5','empty']
    for myfile in names:
        image = cv2.imread('/home/nitro/sawyer_ws/src/sawyer_gripper/src/images/templates/'+myfile+'.png',0)
        template_data.append([myfile,image])

    head_display = intera_interface.HeadDisplay()  
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError, err:
        rospy.logerr(err)
        return
    for tmp in template_data:
        temp_name = tmp[0]
        if edge_detection == True:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            template = tmp[1]
            w, h = template.shape[::-1]

            res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
            threshold = 0.8
            loc = np.where( res >= threshold)
            for pt in zip(*loc[::-1]):
                cv2.rectangle(cv_image, pt, (pt[0] + w, pt[1] + h), (0,255,255), 2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image,temp_name,pt, font, 2,(0,255,255),2,cv2.LINE_AA)

                decision = temp_name[:-1]
    #slowing down the process too much
    #head_display.display_image(cv_image)
    cv2.imshow("Quality Check", cv_image)
    cv2.waitKey(3)

def check():
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    camera = "right_hand_camera"
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return
    cameras = intera_interface.Cameras()
    if not cameras.verify_camera_exists(camera):
        rospy.logerr("Could not detect the specified camera, exiting the example.")
        return
    rospy.loginfo("Opening camera '{0}'...".format(camera))
    cameras.start_streaming(camera)
    rectify_image = not None
    use_canny_edge = None
    cameras.set_callback(camera, show_image_callback,
        rectify_image=rectify_image, callback_args=(True, camera))
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.sleep(3.0)
    cv2.destroyAllWindows()
    return decision

if __name__ == '__main__':
    sys.exit(main())