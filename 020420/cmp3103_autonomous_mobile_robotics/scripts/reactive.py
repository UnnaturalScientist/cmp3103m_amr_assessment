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
from numpy import mean
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_msgs.msg import String

import math

pub_vel = None
state_ = 0

img_ = {
    'R': [0, 0, 0],
    'G': [0, 0, 0],
    'B': [0, 0, 0],
}

ang_ = {
    '0_deg': 0, # 50
    '15_deg': 0, # 260
    '30_Deg': 0, # 20
    '45_deg': 0, # 260
    '60_deg': 0, # 50
}

state_dict_ = {

    # This is just some text output
    0: 'go forwards',
    1: 'corner turn left',
    2: 'corner turn right',
    3: 'turn left',
    4: 'turn right',
    5: 'adjust left',
    6: 'adjust right',
    7: 'reversing',
    8: 'one eighty degree turn',
}

class Reactive():
    def __init__(self):
        self.node_name = "color_contours"
        self.cv_window_name = self.node_name
        self.bridge = CvBridge()

        #self.pub = rospy.Publisher("/result_topic", String, queue_size=10)
        self.pub_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size= 10)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.sense_scan)
        self.sub_img = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        rospy.Timer(rospy.Duration(0.03), self.open_windows) # timer for displaying windows


    def open_windows(self,event):
        try:
            cv2.namedWindow("Cammy", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Slice", cv2.WINDOW_NORMAL)

            cv2.imshow("Cammy",self.cam_view)
            cv2.imshow("Slice",self.processed_image)

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
        self.processed_image = self.color_slice(cam_view) # This is our reference from the TB camera. IT is what we compare (eg)[0,255,255] to


    # At first ... E and W = 50, NW and NE = 150, N = 40
    # Change to... E and W = 250, NW and NE = 55 and N = 40?
    def sense_scan(self, scan):
        global ang_ # GLOBAL TO AND FROM
        ang_ = {
            #'0deg': min(min(scan.ranges[639]), 10), # 40%
            '0_deg': min(min(scan.ranges[439:639]), 10), # 40%
            '15_deg': min(min(scan.ranges[341:438]), 10), # appromixately 10%
            '30_deg': min(min(scan.ranges[300:340]), 10), # 1 range message dead centre
            '45_deg': min(min(scan.ranges[250:299]), 10), # approximately 10%
            '60_deg': min(min(scan.ranges[0:249]), 10), # 40%
            #'60deg': min(min(scan.ranges[0]), 10), # 40%c
        }

        self.state_conditions() #  allows the ranges to be associated with val


        """

        def sense_image(self, img):
            global img_ # GLOBAL TO AND FROM
            img_ = {
                'R': [147, 1, 1], #all other numbers are between 0 and 2
                'G': [1, 147, 1],
                'B': [1, 1, 102],
            }

            self.state_conditions() # creates and instance of the function here

        """


    def color_slice(self, cam_view):
        hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)

        blue = np.array([120, 255, 102]) # This cannot be HSV...? wait, it might be.....
        green = np.array([60, 253, 147]) # This cannot be HSV...? wait, it might be.....
        red = np.array([0, 253, 147]) # This cannot be HSV...? wait, it might be.....

        upper = np.array([0, 255, 147]) # 0 will become x, which can be 0(R), 60(G) or 120 (B)
        lower = np.array([0, 253, 102])

        #TODO return general variable "colour boundary" and input twice, like so...
        # mask = cv2.inRange(hsv, colour_id, colour_id)
        #TODO calculate and return open cv HSV array

        #TODO check difference between V of HSV (102 - 147)

        mask = cv2.inRange(hsv, lower, upper)
        masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask = mask)

        return masked # output can then be used in a state machine like thing


    def state_conditions(self):
        global ang_ # GLOBAL TO AND FROM
        ang = ang_ # sense scan ranges

        #global img_
        #img = img_

        state_description = ''

        c_prx = 0.4 # 65? 60? 55?
        m_prx = 0.8
        l_prx = 1.5
        # There is not one proxance, there is a range of proxances
        # Long range is 1.0m and above or between 1.0m and 0.5m
        # Short range is 0.5m and below

        # This is where we will establisht he behaviours
        # Take individual actions and combine them to create new behaviours

        #TODO remove some beams all together
        # State 1: 65cm from wall (front) --------------> 1m, 1m, 1m, 1m, 1m
        if ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 1 - Approaching wall, front side...'
            self.state_select(6)

        # State 2: Corridor -------------->  0m, 0m, 1m, 0m, 0m
        elif ang['0_deg'] > m_prx and ang['15_deg'] > m_prx and ang['30_deg'] < c_prx and ang['45_deg'] > m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 4 - Corridor, proceeding...'
            self.state_select(7)

        # State 3: Corridor -------------->  1m, 1m, 0m, 1m, 1m
        elif ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 4 - Corridor, proceeding...'
            self.state_select(0)

        # State 4: Corridor -------------->  1m, 0m, 0m, 0m, 1m
        elif ang['0_deg'] < m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] > m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 4 - Corridor, proceeding...'
            self.state_select(5)

        # State 5: Left rotation (major)   1m 1m 1m 0m 0m
        elif ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] > m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 2 - Tight corner. Rotating...'
            self.state_select(1)

        # State 6: Right rotation (major)   0m 0m 1m 1m 1m
        elif ang['0_deg'] > m_prx and ang['15_deg'] > m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 3 - Tight corner. Rotating...'
            self.state_select(2)

        # State 7: Left rotation --------------> 1m, 0m, 0m, 0m, 0m
        elif ang['0_deg'] < m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] > m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 5 - Nearing wall, left side...'
            self.state_select(3)

        # State 8: Right rotation --------------> 0m, 0m, 0m, 0m, 1m
        elif ang['0_deg'] > m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] > m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 6 - Nearing wall, right side...'
            self.state_select(4)

        # State 9: Left rotation (minor) --------------> 0m, 1m, 0m, 0m, 0m
        elif ang['0_deg'] < m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] > m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 5 - Nearing wall, left side...'
            self.state_select(5)

        # State 10: Right rotation (minor) --------------> 0m, 0m, 0m, 1m, 0m
        elif ang['0_deg'] > m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 6 - Nearing wall, right side...'
            self.state_select(6)

        # State 11: Left gap --------------> 0m, 1m, 1m, 1m, 1m
        elif ang['0_deg'] > m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 7 - Gap found, left side...'
            self.state_select(3)

        # State 12: Right gap --------------> 1m, 1m, 1m, 1m, 0m
        elif ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 8 - Gap found, right side...'
            self.state_select(4)

        # State 13: Left gap --------------> 0m, 1m, 0m, 1m, 1m
        elif ang['0_deg'] > m_prx and ang['15_deg'] < m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 7 - Gap found, left side...'
            self.state_select(3)

        # State 14: Right gap --------------> 1m, 1m, 0m, 1m, 0m
        elif ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 8 - Gap found, right side...'
            self.state_select(4)

        # State 15: Staggered corridor (Stairs) ---------------> 0m, 1m, 0m, 1m, 0m
        elif ang['0_deg'] > m_prx and ang['15_deg'] < m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 9 - Staggered corridor, proceeding...'
            self.state_select(0)

        # State 16: Nothing --------------> 0m, 0m, 0m, 0m, 0m
        elif ang['0_deg'] > m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] > m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 10 - Clear space...'
            self.state_select(0)

        else:
            state_description = 'Unknown...'

        """
        #!!!!!!!! COLOUR BASED REACTIONS
        # (DEADEND) --------------> 1, 1, 1, 1, 1
        elif ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'case 13 - Trap found!!!'
            self.state_select(4)

        # (LEFT GAP 1) --------------> 0, 1, 1, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif ang['0_deg'] > m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'case 14 - Clue found!!!'
            self.state_select(2)

        # (RIGHT GAP 1) --------------> 1, 1, 1, 1, 0 (rotate 90 deg/ 1.571 rad CW)
        elif ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] > m_prx:
            state_description = 'case 15 - Goal found!!!'
            self.state_select(3)
        """

    def state_select(self, state):
        global state_, state_dict_
        if state is not state_:
            print 'Reactive Behaviour - [%s] - %s' % (state, state_dict_[state])
            state_ = state


    def action(self):
        rate = rospy.Rate(10) # rate needs changing?

        # all 8 states will be needed here.
        while not rospy.is_shutdown():
            msg = Twist()
            #!!! Forward
            if state_ == 0:
                msg.linear.x = 0.5 # maybe reduce speed, just try it and see
            #!!! minor left turn
            elif state_ == 1:
                msg.angular.z = -0.785 # corner blocked (left)
            #!!! minor right turn
            elif state_ == 2:
                msg.angular.z = 0.785 # corner blocked (right)
            #!!! 90 deg turn left
            elif state_ == 3:
                msg.angular.z = -0.524 # (pi/4 r/s) ... do this for 2 seconds for 90 degree turn
            #!!! 90 deg turn right
            elif state_ == 4:
                msg.angular.z = 0.524 # (pi/4 r/s) ... do this for 2 seconds for 90 degree turn
            #!!! 0.5 rad turn at open side
            elif state_ == 5:
                msg.angular.z = -0.262
            elif state_ == 6:
	            msg.angular.z = 0.262
            elif state_ == 7:
                msg.linear.x = -0.3
            #!!! 1 x rad turn right at deadend
            elif state_ == 8:
                msg.angular.z = 1.047

            else:
                rospy.logerr('Unknown state!')

            self.pub_vel.publish(msg)

            rate.sleep()

# This does not have to be called main
def main():
    global pub_vel # can be passed with self instead?

    rospy.init_node('reactive_behaviours')

    pub_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1) # something mux/velocity?
    #sub = rospy.Subscriber('/scan', LaserScan, self.sense_scan) # this is perfectly fine

    react = Reactive()

    try:
        react.action()
    except rospy.ROSInterruptException:
        print("finish")

if __name__ == '__main__':
    main()

"""
____________________________________________________________________________________________________
References
____________________________________________________________________________________________________

#https://docs.opencv.org/trunk/df/d9d/tutorial_py_colorspaces.html

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