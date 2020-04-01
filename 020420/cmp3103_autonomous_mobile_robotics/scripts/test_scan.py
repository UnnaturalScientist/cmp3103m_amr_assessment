#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print 'Length of msg ranges:'
    print len(msg.ranges)
    print 'Value at 0 degrees:'
    print msg.ranges[639]
    print 'Value at 15 degrees:'
    print msg.ranges[479]
    print 'Value at 30 degrees:'
    print msg.ranges[320]
    print 'Value at 45 degrees:'
    print msg.ranges[159]
    print 'Value at 60 degrees:'
    print msg.ranges[0]

"""
header:
  seq: 760
  stamp:
    secs: 123
    nsecs: 390000000
  frame_id: "/camera_depth_frame"
angle_min: -0.521567881107 (rad)
angle_max: 0.524276316166 (rad)
angle_increment: 0.00163668883033
time_increment: 0.0
scan_time: 0.0329999998212 (time between scans)
range_min: 0.0010000000475 (metres)
range_max: 10.0 (metres)

"""
rospy.init_node('test_scan')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()

#http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
#https://www.theconstructsim.com/read-laserscan-data/?utm_source=youtube&utm_medium=ros_qa&utm_campaign=31