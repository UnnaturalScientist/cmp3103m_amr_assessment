#!/usr/bin/env python

"""
__________________________________________________________________________________________________________
Brief Description
__________________________________________________________________________________________________________
Here
# (Quigley et al., 2015)
__________________________________________________________________________________________________________
1. Imported Clients and Packages
__________________________________________________________________________________________________________
"""
import cv2
import array
import math
import matplotlib.pyplot as plt # remove after use
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from math import pi, radians
from nav_msgs.msg import Odometry
from pprint import pformat
from sensor_msgs.msg import Image, LaserScan
from sklearn.cluster import KMeans # remove after use
from std_msgs.msg import Float32
from tf import transformations

"""
__________________________________________________________________________________________________________
2. Parent Class (Sense)
__________________________________________________________________________________________________________
"""

class Sense_and_Think():

    # TODO -- actually ensure that everything here is being used
    def __init__(self):
        self.node_name = "Sense_and_Think"
        rospy.init_node(self.node_name)

        self.cv_window_name = self.node_name # is this actually needed?
        self.bridge = CvBridge()

        self.hz = rospy.Rate(10)

        self.t = Twist

        self.pub_cmd_vel = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size = 10)
        self.sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.sub_odoom = rospy.Subscriber("/odom", Odometry, self.callback)

        # should this even be here?
        rospy.Timer(rospy.Duration(0.03), self.open_windows)

        rospy.spin()

    """
    __________________________________________________________________________________________________________
    4. Sense: Display Image Windows
    __________________________________________________________________________________________________________
    """

    # Exactly what the name states. A functional block to open windows to view images.
    def open_windows(self, event): # wtf is the event? check ws4 opt
        try:
            # Just some naming conventions for each window. "WINDOW_NORMAL"
               # makes the window more versatile (snaps to sides, enlarges etc)
            cv2.namedWindow("Cammy", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Slice", cv2.WINDOW_NORMAL)

            # "Cammy" is just the default RGB (BGR) camera view.
            cv2.imshow("Cammy", self.cam_view)
            # "Slice" displays the segmented colour (HSV) set within 'color_slice'.
            cv2.imshow("Slice", self.processed_image)

            # This is not really needed, but meh. Just something to give a bit of pause.
            cv2.waitKey(3)
        except:
            pass

    def image_callback(self, info):
        try:
            # calling the class variable, bridge [CvBridge()]
            # This takes the ROS Image messages and convert them into OpenCV format.
            # "bgr8" is and 8-bit RGB Image.
            # The reason it is backwards (RGB) is because the image is perceived by the camera as inverse matrices.
            self.cam_view = self.bridge.imgmsg_to_cv2(info, "bgr8")
        except CvBridgeError ("e"): # try and catch called from Cv2
            print ("e") # print whatever did/did not happen (error)
            pass #...

        cam_view = np.array(self.cam_view, dtype = np.uint8)
        self.processed_image = self.color_slice(cam_view)

    # TODO REF PackT Publishing pages

    """
    __________________________________________________________________________________________________________
    3. Think: Odmometry Track and Update
    __________________________________________________________________________________________________________
    """

    #TODO -- check the orientation is actually doing something
    def odom_orientation(self, q):
        # (y, p, r) yaw, pitch and roll
        # (q.w, q.y, q.z, q.w) our qauternions/orientation
        y, p, r = transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
        # yaw * 180 (degrees) divided by 3.141592... (pi)
        return y * 180 / pi


    def callback(self, data):
        print("odom pose: \n" + pformat(data.pose.pose))
        angle = self.odom_orientation(data.pose.pose.orientation)
        print("angle = %f" % angle)

    # TODO REF the UoL LCAS Github



    #TODO -- It would be much better to use this than "That looks like red"
    """
    def RGB2HEX(color):
	    return "#{:02x}{:02x}{:02x}".format(int(color[0]), int(color[1]), int(color[2]))

    def image_read(self, image):
        image = cv2.imread('hole_image')
        #image = cv2.imread('goal_image')
        #image = cv2.imread('clue_image')

        print("The type of this input is {}".format(type(image)))
        print("Shape: {}".format(image.shape))

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        return image

        #https://towardsdatascience.com/color-identification-in-images-machine-learning-application-b26e770c4c71

    __________________________________________________________________________________________________________
    5. Think: Converting RGB Values to Hue
    __________________________________________________________________________________________________________

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

    """
    def colour_config(self, rgb):
        rgb_upper = [255, 255, 255]
        rgb_lower = [255, 255, 255]

        print(rgb_upper)
        print(rgb_lower)

        print(rgb_upper[0])
        print(rgb_upper[1])
        print(rgb_upper[2])

        print(rgb_upper[0]) + (rgb_lower[1])

        red_hue = (60 * (rgb_upper[1]-rgb_upper[2]) + 360) % 360
        print(red_hue)

        green_hue = (60 * (rgb_upper[2]-rgb_upper[0]) + 120) % 360
        print(green_hue)

        blue_hue = (60 * (rgb_upper[0]-rgb_upper[1]) + 240) % 360
        print(blue_hue)

        hue_array = []

        hue_array.append(blue_hue)
        hue_array.append(green_hue)
        hue_array.append(red_hue)

        print(hue_array)

        print(hue_array[0]) # change comments to python 2.7 again
        print(hue_array[1])
        print(hue_array[2])

        #return (blue_hue, green_hue, red_hue)
        # array becomes [240, 120, 0]

        # it works

        self.hue_array = hue_array

        return self.hue_array
        """
    __________________________________________________________________________________________________________
    6. Think: Detect Colours within the Maze
    __________________________________________________________________________________________________________

    Below, blue (rgb) is used to determine the value for blue (hsv):

    4 Rad * 180/pi = 229.183 ...
    2.7462 is the difference between a 60 degrees and 57.2358 (an exact Rad)
    compensate for 2.7462 degrees * 4 Rad = 11.0568 degrees
    229.183 + 11.0568 = 240.2938 (or 240 degrees)

    240 is the hue value of blue. red and green are calculated simiarly.
    """

    def confirm_colour(self, cam_view):

        hue_array = self.hue_array

        cam_view_hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)

        for x in hue_array:

            # camera feed may be set to HSV, but the values must be set in HSV also.
            lower = [(hue_array[x]), 50, 25]
            upper = [(hue_array[x]), 100, 100]

            mask = cv2.inRange(cam_view_hsv, lower, upper) # create a mask based on those boundaries (view shows hsv colours in range)
            masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask = mask) # mask the camera view to segment out the colour green

            h, w, d = cam_view.shape # height, width, depth of the camera display (for the mean stuff)
            M = cv2.moments(mask)

            if x in hue_array[x] == 120:
                # I will explain this properly tomorrow, but you probably get it
                if M['m00'] > 0:
                    # centroids, x and y
                    # checking moments received by the camera for the specific colour
                    # this will be in the range of (REF PRwR book)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(cam_view, (cx, cy), 20, (255, 0, 0), -1)
                    # what does err do?
                    err = cx - w/2
                    # remember to use a precise number of steps to stop exactly
                    self.t.linear.x = 0.2
                    self.t.angular.z = -float(err) / 100

            elif x in hue_array[x] == 0:
                # I will explain this properly tomorrow, but you probably get it
                if M['m00'] > 0:
                    # centroids, x and y
                    # checking moments received by the camera for the specific colour
                    # this will be in the range of (REF PRwR book)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(cam_view, (cx, cy), 20, (255, 0, 0), -1)

                    # if this is turned into a method,
                    # then cmd_vel will be in the class below
                    err = cx - w/2
                    self.t.linear.x = 1.0
                    self.t.angular.z = -float(err) / 100

                    self.cmd_vel_pub.publish(self.t)

                    # However, masked will still be returned
                    return masked

            #(Fairchild et al., 2017, 189-191)

#TODO -- make a class that inherits the properties from the above

# Make a class to inherit the behaviour of of the colour/cam function

# class Act():

    #def __init__(self):

        # could actually publish velocity here as well

        # this can use super to inehrit all
        # of the class properties from Sense and Act

    #def colour_detected(self, object):
        # if red (call function)
        # 180 turn

        #elif green (call function)
        # cancel all nav goals
        # goto goal

    # return reaction

"""
__________________________________________________________________________________________________________
7. Act: Execution
__________________________________________________________________________________________________________
"""

if __name__ == '__main__':
    try:
        Sense_and_Think()
    except:
        rospy.ROSInterruptException
        pass

# (Fairchild et al., 2017, 189-191)
# (Packt Publishing, 2017)

"""
__________________________________________________________________________________________________________
References
__________________________________________________________________________________________________________

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

https://scikit-learn.org/stable/install.html

https://towardsdatascience.com/color-identification-in-images-machine-learning-application-b26e770c4c71

https://www.dataquest.io/blog/tutorial-advanced-for-loops-python-pandas/

https://learn.turtlebot.com/2015/02/04/2/

http://wiki.ros.org/sound_play#Playing_a_built_in_sound
"""