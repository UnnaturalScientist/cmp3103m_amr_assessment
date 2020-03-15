#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class nav_maze():

    def ___init___(self):

        self.act = Twist()
        self.rate = rospy.Rate(10)

        self.time_stamp = rospy.Time.now()

        self.pub_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, self.search_and_destroy, queue_size=10)
        self.sub_scan = rospy.Subscriber('scan', LaserScan, search_and_destroy)

    def search_and_destroy(self, sense):

        if sense.ranges[320] >= 0.5:
            self.act.linear.x = 0.2
            self.time_stamp = rospy.Time.now() + rospy.Duration(2)

        elif sense.ranges[320] == 0.5 and sense.ranges[0] == 0.5:
            self.act.angular.z = 0.75
            self.time_stamp = rospy.Time.now() + rospy.Duration(2)

        else:
            if sense.ranges[320] == 0.5 and sense.ranges[0] == 0.5 and sense.ranges[639] == 0.5:
                self.act.angular.z = 0.75
                self.time_stamp = rospy.Time.now() + rospy.Duration(4)

        self.pub_vel.publish(self.act)

        self.rate.sleep()


if __name__ == '__main__':

    rospy.init_node('assessment_one')
    rospy.spin()

    try:
        nav_maze()
    except rospy.ROSInterruptException:
        pass
