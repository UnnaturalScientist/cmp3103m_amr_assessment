#!/usr/bin/env python

"""
____________________________________________________________________________________________________
Brief Description
____________________________________________________________________________________________________
Here
# (Quigley et al., 2015)
____________________________________________________________________________________________________
1. Imported Clients and Packages
____________________________________________________________________________________________________
"""

import rospy
import numpy as np
import sys
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

import math

class Colour_Id():
    def __init__(self):
        self.node_name = "Colour_Identification"
        self.cv_window_name = self.node_name
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber('/scan', LaserScan, self.sense)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        rospy.Timer(rospy.Duration(0.03), self.open_windows) # timer for displaying windows


    def open_windows(self,event):
        try:
            cv2.namedWindow("TB_view", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Segment", cv2.WINDOW_NORMAL)

            cv2.imshow("TB_view",self.cam_view)
            cv2.imshow("Segment",self.processed_image)

            cv2.waitKey(3)
        except:
            pass


    def image_callback(self, data):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            self.cam_view = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e: # try and catch called from Cv2
            print e
            pass

        cam_view = np.array(self.cam_view, dtype = np.uint8)
        self.processed_image = self.color_slice(cam_view)


    def rgb_id(self, cam_view):
        hsv_id = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)

        blue_min = np.array([100, 100, 100]) # This does actually pick up blue clues. Perfect
        blue_max = np.array([255, 255, 250])

    	hsv_confirmed = cv2.inRange(hsv_id, blue_min, blue_max)
        segment = cv2.bitwise_and(self.cam_view, self.cam_view, hsv_confirmed = hsv_confirmed)

        return segment

def main():

    rospy.init_node('reactive_behaviours')
    found_colour = Colour_Id()

    try:
        found_colour.segment()
    except rospy.ROSInterruptException:
        print("finish")

if __name__ == '__main__':
    main()

"""
____________________________________________________________________________________________________
References
____________________________________________________________________________________________________

https://www.w3schools.com/python/ref_math_radians.asp

ROS.org (2018) actionlib - ROS Wiki. Open Robotics. Available from
http://wiki.ros.org/actionlib [accessed 9 February 2020].

ROS.org (2019) move_base_msgs - ROS Wiki. Open Robotics. Available from
http://wiki.ros.org/move_base_msgs [accessed 9 February 2020].

Quigley, M., Gerkey, B. and Smart, W.D. (2015) Programming Robots with ROS.
Sebastopol, USA: O'Reilly Media, Inc.

Fairchild, C. and Harman, T.L. (2017) Ros Robotics By Example, 2nd Edition.
Birmingham, UK: Packt Publishing Ltd.

Packt Publishing (2017) Chapter04. Github. Available from
https://github.com/PacktPublishing/ROS-Robotics-By-Example-Second-Edition/tree/master/Chapter04
[accessed 9 February 2020].
"""