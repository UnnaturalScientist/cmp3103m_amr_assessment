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

#TODO Take these found RGB values and convert them to HSV? Nah, no need. They are already so precise.
# Is there even a need for upper or lower boundary?
# Try it


"""
#TODO IS there a requirement for empt values (like ang_?)
img_ = {
    'R_min': [0, 0, 0],
    'R_max': [0, 0, 0],
    'G_min': [0, 0, 0],
    'G_max': [0, 0, 0],
    'B_min': [0, 0, 0],
    'G_max': [0, 0, 0],

    #'Y': [102, 102, 0], # just in case these colours need to play a role
    #'floor': [62, 62, 62],
    #'walls': [102, 102, 102],
}
"""

pub_ = None

ang_ = {
    'E': 0, # 50
    'NE': 0, # 260
    'N': 0, # 20
    'NW': 0, # 260
    'W': 0, # 50
}

#TODO Keep this Dictionary???

#!!!! every state must be justified
state_ = 0
state_dict_ = {

    # This is just some text output
    0: 'go forwards',
    1: 'straighten up',
    2: 'turn left',
    3: 'turn right',
    4: 'one eighty degree turn',
    #5: 'trap found',
    #6: 'clue found',
    #7: 'goal found',
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
        self.processed_image = self.color_slice(cam_view)


    def color_slice(self, cam_view):
        hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([0, 0, 102]) # This cannot be HSV...
        mask = cv2.inRange(hsv, lower_blue, lower_blue)
        masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask = mask)

        self.masked = masked

        return self.masked

    """
    # At first ... E and W = 50, NW and NE = 150, N = 40
    # Change to... E and W = 250, NW and NE = 55 and N = 40?
    def sense_image(self, img):
        global img_ # GLOBAL TO AND FROM
        img_ = {
            'R_min': [147, 1, 1], #all other numbers are between 0 and 2
            'R_max': [148, 2, 2],
            'G_min': [1, 147, 1],
            'G_max': [2, 148, 2],
            'B_min': [0, 0, 101],
            'B_max': [1, 1, 102],
        }

        self.state_conditions() # creates and instance of the function here
    """

    # At first ... E and W = 50, NW and NE = 150, N = 40
    # Change to... E and W = 250, NW and NE = 55 and N = 40?
    def sense_scan(self, scan):
        global ang_ # GLOBAL TO AND FROM
        ang_ = {
            'E':  min(min(scan.ranges[0:50]), 10),
            'NE': min(min(scan.ranges[51:299]), 10),
            'N':  min(min(scan.ranges[300:339]), 10),
            'NW': min(min(scan.ranges[340:589]), 10),
            'W':  min(min(scan.ranges[590:639]), 10), #719
        }

        self.state_conditions() #  allows the ranges to be associated with val


    def state_select(self, state):
        global state_, state_dict_
        if state is not state_:
            print 'Reactive Behaviour - [%s] - %s' % (state, state_dict_[state])
            state_ = state


    def state_conditions(self):
        global ang_ # GLOBAL TO AND FROM
        ang = ang_ # sense scan ranges

        #global img_
        #img = img_

        state_description = ''

        prx = 0.8 # 65? 60? 55?

        # There is not one proxance, there is a range of proxances
        # Long range is 1.0m and above or between 1.0m and 0.5m
        # Short range is 0.5m and below

        # This is where we will establisht he behaviours
        # Take individual actions and combine them to create new behaviours

        # (DEADEND) --------------> 1, 1, 1, 1, 1
        if ang['W'] < prx and ang['NW'] < prx and ang['N'] < prx and ang['NE'] < prx and ang['E'] < prx:
            state_description = 'case 1 - walls detected - front, left and right sides'
            self.state_select(4)

        # (LEFT GAP 1) --------------> 0, 1, 1, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif ang['W'] > prx and ang['NW'] < prx and ang['N'] < prx and ang['NE'] < prx and ang['E'] < prx:
            state_description = 'case 2 - opening detected - left side'
            self.state_select(2)

        # (RIGHT GAP 1) --------------> 1, 1, 1, 1, 0 (rotate 90 deg/ 1.571 rad CW)
        elif ang['W'] < prx and ang['NW'] < prx and ang['N'] < prx and ang['NE'] < prx and ang['E'] > prx:
            state_description = 'case 3 - opening detected - right side'
            self.state_select(3)

        # (CORRIDOR) -------------->  1, 1, 0, 1, 1 (travel forwards at 0.5 m/s, while staying as straight as possible)
        elif ang['W'] < prx and ang['NW'] < prx and ang['N'] > prx and ang['NE'] < prx and ang['E'] < prx:
            state_description = 'case 4 - walls detected - left and right sides'
            self.state_select(1)

        # (LEFT GAP 2) --------------> 0, 1, 0, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif ang['W'] > prx and ang['NW'] < prx and ang['N'] > prx and ang['NE'] < prx and ang['E'] < prx:
            state_description = 'case 5 - opening detected - left side'
            self.state_select(2)

        # (RIGHT GAP 2) --------------> 1, 1, 0, 1, 0 (rotate 90 deg/ 1.571 rad CW)
        elif ang['W'] < prx and ang['NW'] < prx and ang['N'] > prx and ang['NE'] < prx and ang['E'] > prx:
            state_description = 'case 6 - opening detected - right side'
            self.state_select(3)

        # (RIGHT & FRONT WALL) -------> 0, 0, 1, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif ang['W'] > prx and ang['NW'] > prx and ang['N'] < prx and ang['NE'] < prx and ang['E'] < prx:
            state_description = 'case 7 - wall detected - right and front sides'
            self.state_select(2)

        # (FRONT WALL) --------------> 0, 1, 1, 1, 0 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif ang['W'] > prx and ang['NW'] < prx and ang['N'] < prx and ang['NE'] < prx and ang['E'] > prx:
            state_description = 'case 8 - wall detected - front side'
            self.state_select(2)

        # (LEFT & FRONT WALL) -------> 1, 1, 1, 0, 0 (rotate 90 deg/ 1.571 rad CW)
        elif ang['W'] < prx and ang['NW'] < prx and ang['N'] < prx and ang['NE'] > prx and ang['E'] > prx:
            state_description = 'case 9 - wall detected - left and front sides'
            self.state_select(3)

        # (FRONT GAP 1) ---------------> 0, 1, 0, 1, 0 (travel forwards at 0.5 m/s)
        elif ang['W'] > prx and ang['NW'] < prx and ang['N'] > prx and ang['NE'] < prx and ang['E'] > prx:
            state_description = 'case 10 - opening detected - front side'
            self.state_select(0)

        # (FRONT GAP 2) ---------------> 1, 0, 0, 0, 1 (travel forwards at 0.5 m/s)
        elif ang['W'] < prx and ang['NW'] > prx and ang['N'] > prx and ang['NE'] > prx and ang['E'] < prx:
            state_description = 'case 11 - opening detected - front side'
            self.state_select(1)

        # (NOTHING) --------------> 0, 0, 0, 0, 0 (travel forwards at 0.5 m/s)
        elif ang['W'] > prx and ang['NW'] > prx and ang['N'] > prx and ang['NE'] > prx and ang['E'] > prx:
            state_description = 'case 12 - nothing detected' #
            self.state_select(0)

        else:
            state_description = 'unknown case'
            rospy.loginfo(ang)

        """
        #!!!!!!!! COLOUR BASED REACTIONS
        # (DEADEND) --------------> 1, 1, 1, 1, 1
        elif ang['W'] < prx and ang['NW'] < prx and ang['N'] < prx and ang['NE'] < prx and ang['E'] < prx:
            state_description = 'case 13 - Trap found!!!'
            self.state_select(4)

        # (LEFT GAP 1) --------------> 0, 1, 1, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif ang['W'] > prx and ang['NW'] < prx and ang['N'] < prx and ang['NE'] < prx and ang['E'] < prx:
            state_description = 'case 14 - Clue found!!!'
            self.state_select(2)

        # (RIGHT GAP 1) --------------> 1, 1, 1, 1, 0 (rotate 90 deg/ 1.571 rad CW)
        elif ang['W'] < prx and ang['NW'] < prx and ang['N'] < prx and ang['NE'] < prx and ang['E'] > prx:
            state_description = 'case 15 - Goal found!!!'
            self.state_select(3)
        """


    def action(self):
        rate = rospy.Rate(5) # rate needs changing?

        # all 8 states will be needed here.
        while not rospy.is_shutdown():
            msg = Twist()

            if state_ == 0:
                msg.linear.x = 0.5 # maybe reduce speed, just try it and see
            elif state_ == 1:
                msg.angular.z =  0.524 # (pi/6 r/s) ... do this for 1 second to achieve a slight 30 degree correction
            elif state_ == 2:
                msg.angular.z = 0.785 # (pi/4 r/s) ... do this for 2 seconds for 90 degree turn
            elif state_ == 3:
                msg.angular.z = -0.785 # (pi/4 r/s) ... do this for 2 seconds for 90 degree turn
            elif state_ == 4:
                msg.angular.z = -1.047 # r/s ---------[LIMIT??]---------- (pi/3 r/s) .. rotate for 3 seconds for full 180 degree turn
                pass # else is no return, but elif keeps every state possibility open.
            else:
                rospy.logerr('Unknown state!')

            self.pub_.publish(msg)

            rate.sleep()
            """
            elif state_ == 5:
                pass
            elif state_ == 6:
                pass
            elif state_ == 7:
                pass
            """


# This does not have to be called main
def main():
    global pub_ # can be passed with self instead?

    rospy.init_node('reactive_behaviours')

    pub_ = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1) # something mux/velocity?
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