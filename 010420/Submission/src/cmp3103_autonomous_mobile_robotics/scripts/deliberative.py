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

import actionlib
import rospy
# (ROS.org, 2018)

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# (ROS.org, 2019)

"""
___________________________________________________________________________________________________
2. Goal List: Topological Graph
___________________________________________________________________________________________________

        A. ( 0.0,  4.5)    E. (  0.0, -4.5)    K. (-2.25, -4.5)    R. ( 2.25,  4.5)
        B. ( 4.5,  4.5)    F. ( -4.5, -4.5)    L. (  0.0,-2.25)    S. (  0.0, 2.25)
        C. ( 4.5,  0.0)    G. ( -4.5,  0.0)    M. ( 2.25, -4.5)    T. (-2.25,  4.5)
        D. ( 4.5, -4.5)    H. ( -4.5,  4.5)    N. (  4.5,-2.25)    U. ( -4.5, 2.25)
        E. ( 0.0, -4.5)    I. (-2.25,  0.0)    P. ( 2.25,  0.0)
        F. (-4.5, -4.5)    J. (  4.5,-2.25)    Q. (  4.5, 2.25)

        Primary:   A, B, C, A, B, o, C, D, E, C, D, o, G, F, E, o, F, E, G, H, A, G, A
        Secondary: I, J, K, L, M, N, P, Q, R, S, T, U, U, I, S, P, L, I, o

        H ---- A ---- B       _T_   _R_
        |\_ _/ | \_ _/|      /   \ /   \.
        | _|_  |  _|_ |     U     S     Q
        |/   \ | /   \|      \_ _/ \_ _/
        G ---- o ---- C       _I_   _P_
        |\___/ | \___/|      /   \ /   \.
        | _|_  |  _|_ |     J     L     N
        |/   \ | /   \|      \_ _/ \_ _/
        F ---- E ---- D        K     M

    Above are two, simple, topological maps showing the nodes and connecting edges of the goals set
    below. These range from A - S, with the lower 'o' representing the origin of the turtlebot.
    If A - H were considered the primary route, the I - P would be considered the secondary. The
    weightings of each edge will vary from map to map as each layout is unique. Hoever, the pattern
    of travel will consistenly cover the majority of any given map using this method.

    Although the concept for this control architecture appears complex, it has been executed simply

"""

waypoints = [# START: Begin Primary Route
            [(0.0, 4.5), (0.0, 1.0)],
            [(4.5, 4.5), (0.7, 0.7)],
            [(4.5, 0.0), (0.7, 0.7)],
            [(0.0, 4.5), (0.7, 0.7)],
            [(4.5, 4.5), (0.7, 0.7)],
            [(0.0, 0.0), (0.7, 0.7)],
            [(4.5, 0.0), (0.7, 0.7)],
            [(4.5, -4.5), (0.7, 0.7)],
            [(-4.5, 0.0), (0.7, 0.7)],
            [(4.5, 0.0), (0.7, 0.7)],
            [(4.5, -4.5), (0.7, 0.7)],
            [(0.0, 0.0), (0.7, 0.7)],
            [(-4.5, 0.0), (0.7, 0.7)],
            [(-4.5, -4.5), (0.7, 0.7)],
            [(0.0, -4.5), (0.7, 0.7)],
            [(0.0, 0.0), (0.7, 0.7)],
            [(-4.5, -4.5), (0.7, 0.7)],
            [(0.0, -4.5), (0.7, 0.7)],
            [(-4.5, 0.0), (0.7, 0.7)],
            [(-4.5, 4.5), (0.7, 0.7)],
            [(0.0, 4.5), (0.7, 0.7)],
            [(-4.5, 0.0), (0.7, 0.7)],
            [(-4.5, 4.5), (0.7, 0.7)],
            # Switch to Seconday Route
            [(0.0, 0.0), (0.7, 0.7)],
            [(-2.25, 0.0), (0.7, 0.7)],
            [(4.5, -2.25), (0.7, 0.7)],
            [(-2.25, -4.5), (0.7, 0.7)],
            [(0.0, -2.25), (0.7, 0.7)],
            [(2.25, -4.5), (0.7, 0.7)],
            [(4.5, -2.25), (0.7, 0.7)],
            [(2.25, 0.0), (0.7, 0.7)],
            [(4.5, 2.25), (0.7, 0.7)],
            [(2.25, 4.5), (0.7, 0.7)],
            [(0.0, 2.25), (0.7, 0.7)],
            [(-2.25, 4.5), (0.7, 0.7)],
            [(-4.5, 2.25), (0.7, 0.7)],
            [(-2.25, 0.0), (0.7, 0.7)],
            [(0.0, 2.25), (0.7, 0.7)],
            [(2.25, 0.0), (0.7, 0.7)],
            [(0.0, -2.25), (0.7, 0.7)],
            [(-2.25, 0.0), (0.7, 0.7)],
            [(-4.5, -4.5), (0.7, 0.7)]
            ]# ORIGIN: Repeat Layer A


"""
___________________________________________________________________________________________________
3. Class: The Deliberative Control Architecture
___________________________________________________________________________________________________

    Reactive.py had several methods that acted like state machines. However, for deliberative.py,
    a ros package (smach) has been installed. The deliberative control architecture is better
    suited to a sequence of repeatative behaviours, whereas the more reactive script is dnyamic.
    The reactive behaviours are set to cancel all navigation goals if a blue clue is discovered.

"""

def goal_pose(pose):  # <2>
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0.0
    goal_pose.target_pose.pose.orientation.y = 0.0
    goal_pose.target_pose.pose.orientation.z = pose[1][0]
    goal_pose.target_pose.pose.orientation.w = pose[1][1]

    return goal_pose


if __name__ == '__main__':
    rospy.init_node('patrol')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    client.wait_for_server()

    while True:
        for pose in waypoints:   # <4>
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()

"""
____________________________________________________________________________________________________
References
____________________________________________________________________________________________________

#TODO reference this properly
http://wiki.ros.org/navigation/Tutorials/RobotSetup

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