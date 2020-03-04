#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32

class Jareth_the_Goblin_King():

	wheel_radius = 0.03
	robot_radius = 0.18

	# All colour boundaries
	# Can I just make them all "bright" to factor out "upper" and "lower"
	red = [0, 0] # so red_upper[2], could just be 255 and this could be red [X, Y] instead.
	green = [0, 0] # third value is always 255? max brightness? maybe lower? still, it is fixed.
	blue = [0, 0]



	def __init__(self):
		self.node_name = "my_babeh"
		rospy.init_node(self.node_name)
  
		self.cv_window_name = self.node_name # is this actually needed?
		self.bridge = CvBridge()
  
		self.hz = rospy.Rate(10)
		self.t = Twist

		self.sub_wheel = rospy.Subscriber("/moving", Float32, self.turn_left)
		self.sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
		 
		self.pub_cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=1)

		rospy.Timer(rospy.Duration(0.03), self.open_windows)

	"""
	--------------------------------------------------------------------------------------------------------------------------------------------
	Step. 1:  SENSE
	--------------------------------------------------------------------------------------------------------------------------------------------	
	"""
 
	def scan_perimeter(self, detect):
		
		pass

	def capture_perimeter(self, shot):
		
		pass


	"""
	--------------------------------------------------------------------------------------------------------------------------------------------
	Step. 2:  THINK
	--------------------------------------------------------------------------------------------------------------------------------------------	
	"""

	def image_callback(self, data):
		pass

	def compare_images(self, recon):
		# Tensor flow shiz
		pass

	def compare_colour(self, data): # are you (ru) red (r), green (g) or blue (b)?
		try:
			self.cam_view = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e

		hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)

		# args for hue and sat?

		# define range of green color in HSV (upper and lower boundary)
		colour_array = np.array([hue, sat, 255])
		
		mask = cv2.inRange(hsv, colour_array) # create a mask based on those boundaries (view shows hsv colours in range)
		masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask= mask) # mask the camera view to segment out the colour green
		
		h, w, d = cam_view.shape # height, width, depth of the camera display (for the mean stuff)

		# check threshold? HERE?

 		M = cv2.moments(mask)
		# I will explain this properly tomorrow, but you probably get it
		if M['m00'] > 0:
			# centroids, x and y
			# checking moments received by the camera for the specific colour
			# this will be in the range of (REF PRwR book)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(cam_view, (cx, cy), 20, (255, 0, 0), -1)
			err = cx - w/2
			self.twist.linear.x = 0.2
			self.twist.angular.z = -float(err) / 100
			
			self.cmd_vel_pub.publish(self.twist)
			#cv2.imshow("window", cam_view)
			# cv2.waitKey(3)
		
		return masked


	def forward_kinematics(self, w_l, w_r):
		w_l = 1.0
		w_r = 0.0

		c_l = wheel_radius * w_l
		c_r = wheel_radius * w_r

		v = (c_l + c_r) / 2
		a = (c_l - c_r) / (2 * robot_radius)

		return (w_l, w_r)


	"""
	--------------------------------------------------------------------------------------------------------------------------------------------
	Step. 3:  ACT
	--------------------------------------------------------------------------------------------------------------------------------------------	
	"""

	# Exactly what the name states. A functional block to open windows to view images.
	def open_windows(self,event): # wtf is the event? check ws4 opt
		try:
			# Just some naming conventions for each window. "WINDOW_NORMAL" makes the window more versatile (snaps to sides, enlarges etc)
			cv2.namedWindow("Cammy", cv2.WINDOW_NORMAL) 
			cv2.namedWindow("Slice", cv2.WINDOW_NORMAL)
			
			# "Cammy" is just the default RGB (BGR) camera view.
			cv2.imshow("Cammy",self.cam_view)
			# "Slice" displays the segmented colour (HSV) set within 'color_slice'. 
			cv2.imshow("Slice",self.processed_image)	

			# This is not really needed, but meh. Just something to give a bit of pause.
			cv2.waitKey(3)
		except:
			pass # bad exception handling. It works. Screw it.
  
	def turn_left(self, left):
		(v, a) = self.forward_kinematics(left.data, 0)
		print("right wheel has been locked \t turning left...")

		#redefining the Twist variable
		lock_right = Twist()
		lock_right.linear.x = v
		lock_right.angular.z = a

		self.pub_cmd_vel.publish(lock_right)

	def turn_right(self, right):
		(v, a) = self.forward_kinematics(right.data, 0)
		print("left wheel has been locked \t turning right...")

		#redefining the Twist variable
		lock_left = Twist()
		lock_left.linear.x = v
		lock_left.angular.z = a

		self.pub_cmd_vel.publish(lock_left)

	def onward(self, forward):
		(v, a) = self.forward_kinematics(forward.data, 0)		
		print("proceding with forward velocity \t REMIND ME OF THE BABE...")

		free_wheel = Twist()
		free_wheel.linear.x = v
		free_wheel.angular.z = a

		self.pub_cmd_vel.publish(free_wheel)
