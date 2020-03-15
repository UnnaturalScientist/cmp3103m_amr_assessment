#!/usr/bin/env python

import rospy
import tf

from geometry_msgs import PointStamped

if __name__ == '__main__':

    rospy.init_node('listener')
    Listener = tf.TransformListener()
    Rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        pointstamp = PointStamped()
        pointstamp.header.frame_id = 'camera_depth_frame'
        pointstamp.header.stamp = rospy.Time(0)
        pointstamp.point.x = 0.0
        pointstamp.point.y = 0.0
        pointstamp.point.z = 0.0

        try:
            Listener.transformPoint('base_link', pointstamp)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pass

        Rate.sleep()