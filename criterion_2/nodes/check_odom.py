#!/usr/bin/env python

import rospy
from pprint import pformat
from tf import transformations #tf_conversions instead of tf originally. Eigen transforms are in the blue book
from math import pi


from nav_msgs.msg import Odometry


class odom_reader:

	def __init__(self):
		self.image_sub = rospy.Subscriber("/odom", Odometry, self.callback)

	"""
	convert an orientation given in quaternions to an actual
	angle in degrees for a 2D robot
	"""
	def odom_orientation(self, q):
		# (y, p, r) yaw, pitch and roll 
		# (q.w, q.y, q.z, q.w) our qauternions/orientation
		# 
		y, p, r = transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
		# yaw * 180 (degrees) divided by 3.141592... (pi)
		return y * 180 / pi


	def callback(self, data):
		print "odom pose: \n" + pformat(data.pose.pose)
		angle = self.odom_orientation(data.pose.pose.orientation)
		print "angle = %f" % angle

ic = odom_reader()
rospy.init_node('odom_reader')
rospy.spin()