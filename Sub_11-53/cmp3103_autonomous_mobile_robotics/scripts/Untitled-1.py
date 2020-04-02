#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
	print "Value at 0 degrees:"
	print msg.ranges[0]
	print "Value at 90 degrees:"
	print msg.ranges[360]
 	print "Value at 180 degrees:"
	print msg.ranges[719]

rospy.init_node("test_scan")
sub = rospy.Subscriber("kobuki/laser/scan", LaserScan, callback)
rospy.spin()
