#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    # 720 / 5 = 144
    regions = [
        min(min(msg.ranges[0:127]), 10),
        min(min(msg.ranges[128:254]), 10),
        min(min(msg.ranges[256:381]), 10),
        min(min(msg.ranges[382:508]), 10),
        min(min(msg.ranges[509:639]), 10),
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')

    rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
