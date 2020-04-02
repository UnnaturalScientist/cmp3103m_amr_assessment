#!/usr/bin/env python

import sys
import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

pub_vel = None
state_ = 0

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
    20: 'trap found',
    21: 'clue found',
    22: 'goal found',
}

class Reactive():
    """Docstring"""
    def __init__(self):
        self.node_name = "amr_assessment_item_one"
        self.cv_window_name = self.node_name
        self.bridge = CvBridge()

        self.msg = Twist()
        self.rate = rospy.Rate(10) # rate needs changing?

        self.state_description = ''

        #self.pub = rospy.Publisher("/result_topic", String, queue_size = 10)
        self.pub_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.sense_scan)
        self.sub_img = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        rospy.Timer(rospy.Duration(0.03), self.open_windows) # timer for displaying windows


    def open_windows(self, event):
        """Docstring"""
        try:
            cv2.namedWindow("TB_View", cv2.WINDOW_NORMAL)
            cv2.imshow("TB_View", self.cam_view)
            cv2.waitKey(3)
        except:
            pass


    def image_callback(self, data):
        """Docstring"""
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            self.cam_view = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e: # try and catch called from Cv2
            print e
            pass


    def colour_config(self, r, g, b):
        """
        When an RGB cube is viewed in the chromacity plane/as a hexacone,
        the colour values for hue become clear. It can be seen that each
        of the 6 colours is designated to a specific radiant/segment
        (red = 0, magenta = 60 blue = 120, cyan = 180, green = 240 and yellow = 360).
        This confirms that the caluclated values for R, G and B are correct.

        https://en.wikipedia.org/wiki/HSL_and_HSV

        This is the only value for blue, green and red. The min is 0.
        Therefore, the max cannot be subtracted from. The is no range of
        B, G or R. So there is not chroma either. The tone of B, G and R
        is defined by saturation and value.

        Below, blue (rgb) is used to determine the value for blue (hsv):

        4 Rad * 180/pi = 229.183 ...
        2.7462 is the difference between a 60 degrees and 57.2358 (an exact Rad)
        compensate for 2.7462 degrees * 4 Rad = 11.0568 degrees
        229.183 + 11.0568 = 240.2938 (or 240 degrees)
        240 is the hue value of blue. red and green are calculated simiarly.

        """
        r = 147
        g = 1
        b = 1

        # Logic of using max values for RGB then
        # R, G, B values are divided by 255
        # to change the range from 0..255 to 0..1:
        r, g, b = r / 255.0, g / 255.0, b / 255.0

        # h, s, v = hue, saturation, value
        cmax = max(r, g, b)    # maximum of r, g, b
        cmin = min(r, g, b)    # minimum of r, g, b
        diff = cmax-cmin       # diff of cmax and cmin.

        # if cmax and cmax are equal then h = 0
        if cmax == cmin:
            h = 0

        # if cmax equal r then compute h
        elif cmax == r:
            h = (60 * ((g - b) / diff) + 360) % 360

        # if cmax equal g then compute h
        elif cmax == g:
            h = (60 * ((b - r) / diff) + 120) % 360

        # if cmax equal b then compute h
        elif cmax == b:
            h = (60 * ((r - g) / diff) + 240) % 360

        # if cmax equal zero
        if cmax == 0:
            s = 0
        else:
            s = (diff / cmax) * 100

        # compute v
        v = cmax * 100

        print h / 2, s * 2.55 , v * 2.55

        """
        output is hsv (360, 100, 100) in open_cv format (180, 255, 255)

            ___________________________________
            OpenCV  |   H     S      V
            ___________________________________
                    |
            Red     |   0,    253,   147
            Green   |   60,   253,   147
            Blue    |   120,  255,   102


        As the S and V values are so similar, only the Hue will be passed.
        This will work across hues for r, g and b. For example:

            lower_hsv = (hue[0], 253, 102)
            upper_hsv = (hue[0], 255, 147)

        """

    def find_trap(self, cam_view):
        """Docstring"""
        hsv_trap = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)
        trap = np.array(0, 253, 147)
        trap_mask = cv2.inRange(hsv_trap, trap, trap)
        trap_output = cv2.bitwise_and(self.cam_view, self.cam_view, mask=trap_mask)

        h, w, d = cam_view.shape
        M = cv2.moments(trap_mask)

        if M['m00'] > 0:
            self.state_select(20)
            self.state_description = 'State 1 - Approaching wall, front side...'
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cam_view, (cx, cy), 20, (0, 0, 255), -1)


    def find_clue(self, cam_view):
        """Docstring"""
        hsv_clue = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)
        clue = np.array(60, 253, 147)
        clue_mask = cv2.inRange(hsv_clue, clue, clue)
        clue_output = cv2.bitwise_and(self.cam_view, self.cam_view, mask = clue_mask)

        h, w, d = cam_view.shape
        M = cv2.moments(clue_mask)

        if M['m00'] > 0:
            self.state_select(21)
            self.state_description = 'State 1 - Approaching wall, front side...'
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cam_view, (cx, cy), 20, (0, 0, 255), -1)


    def find_goal(self, cam_view):
        """Docstring"""
        hsv_goal = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)
        goal = np.array(120, 255, 102)
        goal_mask = cv2.inRange(hsv_goal, goal, goal)
        goal_output = cv2.bitwise_and(self.cam_view, self.cam_view, mask = goal_mask)

        h, w, d = cam_view.shape
        M = cv2.moments(goal_mask)

        if M['m00'] > 0:
            self.state_select(22)
            self.state_description = 'State 1 - Approaching wall, front side...'
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cam_view, (cx, cy), 20, (0, 0, 255), -1)


    def sense_scan(self, scan):
        """Docstring"""
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


    def state_conditions(self):
        """Docstring"""
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
        # State 1: 65cm from wall (front) --  --  --  --  --  --  --  > 1m, 1m, 1m, 1m, 1m
        if ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 1 - Approaching wall, front side...'
            self.state_select(6)

        # State 2: Corridor --  --  --  --  --  --  --  >  0m, 0m, 1m, 0m, 0m
        elif ang['0_deg'] > m_prx and ang['15_deg'] > m_prx and ang['30_deg'] < c_prx and ang['45_deg'] > m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 4 - Corridor, proceeding...'
            self.state_select(7)

        # State 3: Corridor --  --  --  --  --  --  --  >  1m, 1m, 0m, 1m, 1m
        elif ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 4 - Corridor, proceeding...'
            self.state_select(0)

        # State 4: Corridor --  --  --  --  --  --  --  >  1m, 0m, 0m, 0m, 1ms
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

        # State 7: Left rotation --  --  --  --  --  --  --  > 1m, 0m, 0m, 0m, 0m
        elif ang['0_deg'] < m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] > m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 5 - Nearing wall, left side...'
            self.state_select(3)

        # State 8: Right rotation --  --  --  --  --  --  --  > 0m, 0m, 0m, 0m, 1m
        elif ang['0_deg'] > m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] > m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 6 - Nearing wall, right side...'
            self.state_select(4)

        # State 9: Left rotation (minor) --  --  --  --  --  --  --  > 0m, 1m, 0m, 0m, 0m
        elif ang['0_deg'] < m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] > m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 5 - Nearing wall, left side...'
            self.state_select(5)

        # State 10: Right rotation (minor) --  --  --  --  --  --  --  > 0m, 0m, 0m, 1m, 0m
        elif ang['0_deg'] > m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 6 - Nearing wall, right side...'
            self.state_select(6)

        # State 11: Left gap --  --  --  --  --  --  --  > 0m, 1m, 1m, 1m, 1m
        elif ang['0_deg'] > m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 7 - Gap found, left side...'
            self.state_select(3)

        # State 12: Right gap --  --  --  --  --  --  --  > 1m, 1m, 1m, 1m, 0m
        elif ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] < m_prx and ang['45_deg'] < m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 8 - Gap found, right side...'
            self.state_select(4)

        # State 13: Left gap --  --  --  --  --  --  --  > 0m, 1m, 0m, 1m, 1m
        elif ang['0_deg'] > m_prx and ang['15_deg'] < m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] < m_prx:
            state_description = 'State 7 - Gap found, left side...'
            self.state_select(3)

        # State 14: Right gap --  --  --  --  --  --  --  > 1m, 1m, 0m, 1m, 0m
        elif ang['0_deg'] < m_prx and ang['15_deg'] < m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 8 - Gap found, right side...'
            self.state_select(4)

        # State 15: Staggered corridor (Stairs) --  --  --  --  --  --  -- - > 0m, 1m, 0m, 1m, 0m
        elif ang['0_deg'] > m_prx and ang['15_deg'] < m_prx and ang['30_deg'] > m_prx and ang['45_deg'] < m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 9 - Staggered corridor, proceeding...'
            self.state_select(0)

        # State 16: Nothing --  --  --  --  --  --  --  > 0m, 0m, 0m, 0m, 0m
        elif ang['0_deg'] > m_prx and ang['15_deg'] > m_prx and ang['30_deg'] > m_prx and ang['45_deg'] > m_prx and ang['60_deg'] > m_prx:
            state_description = 'State 10 - Clear space...'
            self.state_select(0)

        else:
            state_description = 'Unknown...'

    def state_select(self, state):
        """Docstring"""
        global state_, state_dict_
        if state is not state_:
            print 'Reactive Behaviour - [%s] - %s' % (state, state_dict_[state])
            state_ = state


    def action(self):
        """Docstring"""
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

    pub_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1) # something mux/velocity?
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

!!! Referencing

https://stackoverflow.com/questions/52083797/opencv-detecting-color-ranges-and-display-on-console

https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv

https://answers.ros.org/question/52549/navigation-stack-parameters-tuning-to-pass-through-a-narrow-space/

https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/

https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html

https://docs.opencv.org/trunk/df/d9d/tutorial_py_colorspaces.html

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