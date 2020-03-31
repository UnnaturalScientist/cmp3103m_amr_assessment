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
import cv2		# OpenCV library
import numpy as np 	# Used for image processing/matrices

from cv_bridge import CvBridge, CvBridgeError	# Bridge (for converting ROS image to OpenCV) and error log
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image


"""
____________________________________________________________________________________________________
2. Parent Class: Sense
____________________________________________________________________________________________________

    The purpose of this class is to ...

"""

class Sense():
    def __init__(self):

        self.node_name = "sense"
        rospy.init_node(self.node_name)

        self.cv_window_name = self.node_name
        self.bridge = CvBridge()

        # Publishes a custom string. It can say whatever you want.
        self.pub = rospy.Publisher("/result_topic", String, queue_size=1)
        # Subscribes to the image, which has just been converted into OpenCV from ROS Image
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.convert_data)
        rospy.Timer(rospy.Duration(0.03), self.visual_output) # timer for displaying windows
        # Publishes velocity commands. Like the name says.
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', t, queue_size=1)

        # An instance of t.
        self.t = Twist()


    """
    ____________________________________________________________________________________________________
    3. The preperation of the Windows to Display Visual Output
    ____________________________________________________________________________________________________

        This method...

    """

    # Exactly what the name states. A functional block to open windows to view images.
    def visual_output(self,event):
        try:
            # Just some naming conventions for each window. "WINDOW_NORMAL" makes the window more versatile (snaps to sides, enlarges etc)
            cv2.namedWindow("Pov", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Seg", cv2.WINDOW_NORMAL)

            # "Cammy" is just the default RGB (BGR) camera view.
            cv2.imshow("Pov",self.cam_view)
            # "Slice" displays the segmented colour (HSV) set within 'segment_colour'.
            cv2.imshow("Seg",self.processed_image)

            # This is not really needed, but meh. Just something to give a bit of pause.
            cv2.waitKey(3)
        except:
            pass # bad exception handling. It works. Screw it.

    """
    ____________________________________________________________________________________________________
    4. Convert the Raw Image Data into OpenCV Format
    ____________________________________________________________________________________________________

        This method...

    """
    def convert_data(self, data):
        try:
            # calling the class variable, bridge [CvBridge()]
             # This takes the ROS Image messages and convert them into OpenCV format.
            # "bgr8" is and 8-bit RGB Image.
             # The reason it is backwards (RGB) is because the image is perceived by the camera as inverse matrices. I think?
            self.cam_view = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e: # try and catch called from Cv2
            print 'e' # print whatever did/did not happen (error)
            pass #...

        cam_view = np.array(self.cam_view, dtype = np.uint8)
        self.processed_image = self.segment_colour(cam_view)

    """
    ____________________________________________________________________________________________________
    5. Perform a Visual Search for Colours within the Target Boundaries (?)
    ____________________________________________________________________________________________________

        This method...

    """
    def segment_colour(self, cam_view):
        hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)

        # define range of green color in HSV (upper and lower boundary)
        green_min = np.array([30,30,30])
        green_max = np.array([100,255,255])

        mask = cv2.inRange(hsv, green_min, green_max) # create a mask based on those boundaries (view shows hsv colours in range)
        masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask=mask) # mask the camera view to segment out the colour green

        h, w, d = cam_view.shape # height, width, depth of the camera display (for the mean stuff)
        M = cv2.moments(mask)

        #TODO the centroids will stay, but the majority of this will become a state in react.py
        # I will explain this properly tomorrow, but you probably get it
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cam_view, (cx, cy), 20, (0, 0, 255), -1)
            # !!! May not need anything from about here...
            # We just want the colour.
            # !!! there will need to be a conditional for R, G or B
            err = cx - w/2
            self.t.linear.x = 0.2
            self.t.angular.z = -float(err) / 100

            self.cmd_vel_pub.publish(self.t)
            #cv2.imshow("window", cam_view)
            # cv2.waitKey(3)

        # !!! This is the key. The segmented colour frame
        return masked

        # TODO even within this script, it will be neccessary to state...
        # above all else: avoid red. Then find blue, but if green is found before blue, Green is king.
        # The same would go for Green over red, unless red is between the green and the robot.
        rospy.spin()


"""
____________________________________________________________________________________________________
6. Execution
____________________________________________________________________________________________________

"""

if __name__ == '__main__':
    cv2.startWindowThread()
    Sense()

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

#TODO Reference this properly
# https://github.com/pirobot/rbx2/blob/indigo-devel/rbx2_nav/nodes/select_cmd_vel.py

"""
