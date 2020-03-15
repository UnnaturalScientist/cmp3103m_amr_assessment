#!/usr/bin/env python

import rospy
import geometry_msgs
import tf

def callback():

    tf_broadcast = tf.TransformBroadcaster()
    tf_stamped = geometry_msgs.msg.TransformStamped()
    tf_stamped.header.stamp = rospy.Time.now()

    tf_stamped.header.frame_id = 'base_link'
    tf_stamped.child_frame_id = 'camera_depth_frame'

    tf_stamped.transform.translation = (0.0, 0.0, 0.0)
    tf_stamped.transform.rotation = (0.0, 0.0, 0.0)

    tf_broadcast.sendTransform(tf_stamped)

if __name__ == '__main__':
    rospy.init_node("tf_talker")
    rospy.Subscriber('scan', 'LaserScan', callback)
    rospy.spin()
