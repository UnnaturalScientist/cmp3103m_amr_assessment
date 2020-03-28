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

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

pub_ = None

sample_angle_ = {
    'E': 0, # 50
    'NE': 0, # 260
    'N': 0, # 20
    'NW': 0, # 260
    'W': 0, # 50
}

#TODO Keep this Dictionary???

state_ = 0
state_dict_ = {
    # 0: 'find the wall',
    # 1: 'turn left', # 2. fine as is, but maybe change degrees
    # 2: 'follow the wall', # 3. turn right 90 degrees

    # This is just some text output
    0: 'go forwards',
    1: 'straighten up',
    2: 'turn left',
    3: 'turn right',
    4: 'one eighty degree turn',
}

# TODO change the laser ranges after tweaking turning and forward velocities
# At first ... E and W = 50, NW and NE = 150, N = 40
# Change to... E and W = 250, NW and NE = 55 and N = 40?
def sense(msg):
    global sample_angle_ # GLOBAL TO AND FROM
    sample_angle_ = {
        'E':  min(min(msg.ranges[0:50]), 10),
        'NE': min(min(msg.ranges[51:299]), 10),
        'N':  min(min(msg.ranges[300:339]), 10),
        'NW':  min(min(msg.ranges[340:589]), 10),
        'W':   min(min(msg.ranges[590:639]), 10),
    }

    think() # creates and instance of the function here

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Reactive Behaviour - [%s] - %s' % (state, state_dict_[state])
        state_ = state


#https://www.w3schools.com/python/ref_math_radians.asp
# convert to radians

#SENSE

def think():
    global sample_angle_ # GLOBAL TO AND FROM
    sample_angle = sample_angle_ # define as new variable to differ from sense.sample_angle_

    state_description = ''

    sample_range = 0.8 # 65? 60? 55?

    # There is not one sample_rangeance, there is a range of sample_rangeances
    # Long range is 1.0m and above or between 1.0m and 0.5m
    # Short range is 0.5m and below

    # This is where we will establisht he behaviours
    # Take individual actions and combine them to create new behaviours

    # (DEADEND) --------------> 1, 1, 1, 1, 1
    if sample_angle['W'] < sample_range and sample_angle['NW'] < sample_range and sample_angle['N'] < sample_range and sample_angle['NE'] < sample_range and sample_angle['E'] < sample_range:
        state_description = 'case 1 - Deadend reached'
        change_state(4)

    # (LEFT GAP 1) --------------> 0, 1, 1, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
    elif sample_angle['W'] > sample_range and sample_angle['NW'] < sample_range and sample_angle['N'] < sample_range and sample_angle['NE'] < sample_range and sample_angle['E'] < sample_range:
        state_description = 'case 2 - NE'
        change_state(2)

    # (RIGHT GAP 1) --------------> 1, 1, 1, 1, 0 (rotate 90 deg/ 1.571 rad CW)
    elif sample_angle['W'] < sample_range and sample_angle['NW'] < sample_range and sample_angle['N'] < sample_range and sample_angle['NE'] < sample_range and sample_angle['E'] > sample_range:
        state_description = 'case 3 - NW'
        change_state(3)

    # (CORRIDOR) -------------->  1, 1, 0, 1, 1 (travel forwards at 0.5 m/s, while staying as straight as possible)
    elif sample_angle['W'] < sample_range and sample_angle['NW'] < sample_range and sample_angle['N'] > sample_range and sample_angle['NE'] < sample_range and sample_angle['E'] < sample_range:
        state_description = 'case 4 - N and NW'
        change_state(1)

    # (LEFT GAP 2) --------------> 0, 1, 0, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
    elif sample_angle['W'] > sample_range and sample_angle['NW'] < sample_range and sample_angle['N'] > sample_range and sample_angle['NE'] < sample_range and sample_angle['E'] < sample_range:
        state_description = 'case 5 - NE'
        change_state(2)

    # (RIGHT GAP 2) --------------> 1, 1, 0, 1, 0 (rotate 90 deg/ 1.571 rad CW)
    elif sample_angle['W'] < sample_range and sample_angle['NW'] < sample_range and sample_angle['N'] > sample_range and sample_angle['NE'] < sample_range and sample_angle['E'] > sample_range:
        state_description = 'case 6 - NW'
        change_state(3)

    # (RIGHT & FRONT WALL) -------> 0, 0, 1, 1, 1 (rotate 90 deg/ 1.571 rad Anti-CW)
    elif sample_angle['W'] > sample_range and sample_angle['NW'] > sample_range and sample_angle['N'] < sample_range and sample_angle['NE'] < sample_range and sample_angle['E'] < sample_range:
        state_description = 'case 7 - NE'
        change_state(2)

    # (FRONT WALL) --------------> 0, 1, 1, 1, 0 (rotate 90 deg/ 1.571 rad Anti-CW)
    elif sample_angle['W'] > sample_range and sample_angle['NW'] < sample_range and sample_angle['N'] < sample_range and sample_angle['NE'] < sample_range and sample_angle['E'] > sample_range:
        state_description = 'case 8 - nothing'
        change_state(2)

    # (LEFT & FRONT WALL) -------> 1, 1, 1, 0, 0 (rotate 90 deg/ 1.571 rad CW)
    elif sample_angle['W'] < sample_range and sample_angle['NW'] < sample_range and sample_angle['N'] < sample_range and sample_angle['NE'] > sample_range and sample_angle['E'] > sample_range:
        state_description = 'case 9 - N'
        change_state(3)

    # (FRONT GAP 1) ---------------> 0, 1, 0, 1, 0 (travel forwards at 0.5 m/s)
    elif sample_angle['W'] > sample_range and sample_angle['NW'] < sample_range and sample_angle['N'] > sample_range and sample_angle['NE'] < sample_range and sample_angle['E'] > sample_range:
        state_description = 'case 10 - nothing'
        change_state(0)

    # (FRONT GAP 2) ---------------> 1, 0, 0, 0, 1 (travel forwards at 0.5 m/s)
    elif sample_angle['W'] < sample_range and sample_angle['NW'] > sample_range and sample_angle['N'] > sample_range and sample_angle['NE'] > sample_range and sample_angle['E'] < sample_range:
        state_description = 'case 11 - nothing'
        change_state(1)

    # (NOTHING) --------------> 0, 0, 0, 0, 0 (travel forwards at 0.5 m/s)
    elif sample_angle['W'] > sample_range and sample_angle['NW'] > sample_range and sample_angle['N'] > sample_range and sample_angle['NE'] > sample_range and sample_angle['E'] > sample_range:
        state_description = 'case 12 - NW' #
        change_state(0)

    else:
        state_description = 'unknown case'
        rospy.loginfo(sample_angle)

    #TODO cases for colour recognition / states even

# ACT

#!!!!!!!!!!!!!!!!!!!!!!!! IMPORTANT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# uses global sample_angle to ref laser for response

def go_forward():
    msg = Twist()
    msg.linear.x = 0.5 # maybe reduce speed, just try it and see
    return msg

def bang_bang():
    msg = Twist()
    msg.angular.z =  0.524 # (pi/6 r/s) ... do this for 1 second to achieve a slight 30 degree correction
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.785 # (pi/4 r/s) ... do this for 2 seconds for 90 degree turn
    return msg

def turn_right():
    msg = Twist()
    msg.angular.z = -0.785 # (pi/4 r/s) ... do this for 2 seconds for 90 degree turn
    return msg

def one_eighty():
    msg = Twist()
    msg.angular.z = -1.047 # r/s ---------[LIMIT??]---------- (pi/3 r/s) .. rotate for 3 seconds for full 180 degree turn
    return msg


# This does not have to be called main
def act():
    global pub_ # can be passed with self instead?

    rospy.init_node('reactive_behaviours')

    pub_ = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1) # something mux/velocity?

    sub = rospy.Subscriber('/scan', LaserScan, sense) # this is perfectly fine

    rate = rospy.Rate(20) # rate needs changing?

    # all 8 states will be needed here.
    while not rospy.is_shutdown():
        msg = Twist()

        if state_ == 0:
            msg = go_forward()
        elif state_ == 1:
            msg = bang_bang()
        elif state_ == 2:
            msg = turn_left()
        elif state_ == 3:
            msg = turn_right()
        elif state_ == 4:
            msg = one_eighty()
            pass # else is no return, but elif keeps every state possibility open.
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    act()

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

"""