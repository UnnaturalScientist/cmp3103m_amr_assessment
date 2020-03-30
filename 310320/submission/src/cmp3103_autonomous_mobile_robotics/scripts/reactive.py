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

pub_ = None

angle_ = {
    'E': 0, # 50
    'NE': 0, # 260
    'N': 0, # 20
    'NW': 0, # 260
    'W': 0, # 50
}

#TODO Keep this Dictionary???

state_ = 0
state_dict_ = {
    # 0: 'find the wall',
    # 1: 'turn left', # 2. fine as is, but maybe change degrees
    # 2: 'follow the wall', # 3. turn right 90 degrees

    # This is just some text output
    0: 'go forwards',
    1: 'straighten up',
    2: 'turn left',
    3: 'turn right',
    4: 'one eighty degree turn',
}

class Reactive():
    def __init__(self):

        self.node_name = "color_contours"
        self.cv_window_name = self.node_name
        self.bridge = CvBridge()

        #self.pub = rospy.Publisher("/result_topic", String, queue_size=10)
        self.pub_ = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size= 10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.sense)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

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
        lower_blue = np.array([100, 100, 100]) # This does actually pick up blue clues. Perfect
        upper_blue = np.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask=mask)


        """
        #!!! For mean only
        self.pub.publish(str(np.mean(hsv[:, :, 0]))) #H?
        self.pub.publish(str(np.mean(hsv[:, :, 1]))) #S?
        self.pub.publish(str(np.mean(hsv[:, :, 2]))) #V?
        #print mean(hsv)
        """
        return masked


    # At first ... E and W = 50, NW and NE = 150, N = 40
    # Change to... E and W = 250, NW and NE = 55 and N = 40?
    def sense(self, msg):
        global angle_ # GLOBAL TO AND FROM
        angle_ = {
            'E':  min(min(msg.ranges[0:50]), 10),
            'NE': min(min(msg.ranges[51:299]), 10),
            'N':  min(min(msg.ranges[300:339]), 10),
            'NW': min(min(msg.ranges[340:589]), 10),
            'W':  min(min(msg.ranges[590:639]), 10), #719
        }

        self.think() # creates and instance of the function here


    def change_state(self, state):
        global state_, state_dict_
        if state is not state_:
            print 'Reactive Behaviour - [%s] - %s' % (state, state_dict_[state])
            state_ = state

    #https://www.w3schools.com/python/ref_math_radians.asp


    def think(self):
        global angle_ # GLOBAL TO AND FROM
        angle = angle_ # define as new variable to differ from sense.angle_

        state_description = ''

        prox = 0.8 # 65? 60? 55?

        # There is not one proxance, there is a range of proxances
        # Long range is 1.0m and above or between 1.0m and 0.5m
        # Short range is 0.5m and below

        # This is where we will establisht he behaviours
        # Take individual actions and combine them to create new behaviours

        # (DEADEND) --------------> 1, 1, 1, 1, 1
        if angle['W'] < prox and angle['NW'] < prox and angle['N'] < prox and angle['NE'] < prox and angle['E'] < prox:
            state_description = 'case 1 - Deadend reached'
            self.change_state(4)

        # (LEFT GAP 1) --------------> 0, 1, 1, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif angle['W'] > prox and angle['NW'] < prox and angle['N'] < prox and angle['NE'] < prox and angle['E'] < prox:
            state_description = 'case 2 - NE'
            self.change_state(2)

        # (RIGHT GAP 1) --------------> 1, 1, 1, 1, 0 (rotate 90 deg/ 1.571 rad CW)
        elif angle['W'] < prox and angle['NW'] < prox and angle['N'] < prox and angle['NE'] < prox and angle['E'] > prox:
            state_description = 'case 3 - NW'
            self.change_state(3)

        # (CORRIDOR) -------------->  1, 1, 0, 1, 1 (travel forwards at 0.5 m/s, while staying as straight as possible)
        elif angle['W'] < prox and angle['NW'] < prox and angle['N'] > prox and angle['NE'] < prox and angle['E'] < prox:
            state_description = 'case 4 - N and NW'
            self.change_state(1)

        # (LEFT GAP 2) --------------> 0, 1, 0, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif angle['W'] > prox and angle['NW'] < prox and angle['N'] > prox and angle['NE'] < prox and angle['E'] < prox:
            state_description = 'case 5 - NE'
            self.change_state(2)

        # (RIGHT GAP 2) --------------> 1, 1, 0, 1, 0 (rotate 90 deg/ 1.571 rad CW)
        elif angle['W'] < prox and angle['NW'] < prox and angle['N'] > prox and angle['NE'] < prox and angle['E'] > prox:
            state_description = 'case 6 - NW'
            self.change_state(3)

        # (RIGHT & FRONT WALL) -------> 0, 0, 1, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif angle['W'] > prox and angle['NW'] > prox and angle['N'] < prox and angle['NE'] < prox and angle['E'] < prox:
            state_description = 'case 7 - NE'
            self.change_state(2)

        # (FRONT WALL) --------------> 0, 1, 1, 1, 0 (rotate 90 deg/ 1.571 rad Anti-CW)
        elif angle['W'] > prox and angle['NW'] < prox and angle['N'] < prox and angle['NE'] < prox and angle['E'] > prox:
            state_description = 'case 8 - nothing'
            self.change_state(2)

        # (LEFT & FRONT WALL) -------> 1, 1, 1, 0, 0 (rotate 90 deg/ 1.571 rad CW)
        elif angle['W'] < prox and angle['NW'] < prox and angle['N'] < prox and angle['NE'] > prox and angle['E'] > prox:
            state_description = 'case 9 - N'
            self.change_state(3)

        # (FRONT GAP 1) ---------------> 0, 1, 0, 1, 0 (travel forwards at 0.5 m/s)
        elif angle['W'] > prox and angle['NW'] < prox and angle['N'] > prox and angle['NE'] < prox and angle['E'] > prox:
            state_description = 'case 10 - nothing'
            self.change_state(0)

        # (FRONT GAP 2) ---------------> 1, 0, 0, 0, 1 (travel forwards at 0.5 m/s)
        elif angle['W'] < prox and angle['NW'] > prox and angle['N'] > prox and angle['NE'] > prox and angle['E'] < prox:
            state_description = 'case 11 - nothing'
            self.change_state(1)

        # (NOTHING) --------------> 0, 0, 0, 0, 0 (travel forwards at 0.5 m/s)
        elif angle['W'] > prox and angle['NW'] > prox and angle['N'] > prox and angle['NE'] > prox and angle['E'] > prox:
            state_description = 'case 12 - NW' #
            self.change_state(0)

        else:
            state_description = 'unknown case'
            rospy.loginfo(angle)

    #TODO pretty sure this is not, at all, neccessary
    def go_forward(self):
        msg = Twist()
        msg.linear.x = 0.5 # maybe reduce speed, just try it and see
        return msg

    def bang_bang(self):
        msg = Twist()
        msg.angular.z =  0.524 # (pi/6 r/s) ... do this for 1 second to achieve a slight 30 degree correction
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.785 # (pi/4 r/s) ... do this for 2 seconds for 90 degree turn
        return msg

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.785 # (pi/4 r/s) ... do this for 2 seconds for 90 degree turn
        return msg

    def one_eighty(self):
        msg = Twist()
        msg.angular.z = -1.047 # r/s ---------[LIMIT??]---------- (pi/3 r/s) .. rotate for 3 seconds for full 180 degree turn
        return msg

    def action(self):
        rate = rospy.Rate(5) # rate needs changing?

        # all 8 states will be needed here.
        while not rospy.is_shutdown():
            msg = Twist()

            if state_ == 0:
                msg = self.go_forward()
            elif state_ == 1:
                msg = self.bang_bang()
            elif state_ == 2:
                msg = self.turn_left()
            elif state_ == 3:
                msg = self.turn_right()
            elif state_ == 4:
                msg = self.one_eighty()
                pass # else is no return, but elif keeps every state possibility open.
            else:
                rospy.logerr('Unknown state!')

            self.pub_.publish(msg)

            rate.sleep()

# This does not have to be called main
def main():
    global pub_ # can be passed with self instead?

    rospy.init_node('reactive_behaviours')

    pub_ = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1) # something mux/velocity?
    #sub = rospy.Subscriber('/scan', LaserScan, self.sense) # this is perfectly fine

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