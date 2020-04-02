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

import rospy
from geometry_msgs.msg import Twist
from topic_tools.srv import MuxSelect
import thread

class Coordinator:
    def __init__(self):
        rospy.init_node("coordinator")

        # The rate at which to update the input selection
        rate = rospy.get_param('~rate', 5) # 5hz?

        # Convert to a ROS rate
        r = rospy.Rate(rate) # Convert what? Oh, the above.

        # Get a lock for updating the selected cmd_vel input
        # Creates a new 'lock' on the enabled/disabled control method (My note)
        self.lock = thread.allocate_lock()

        #TODO do not worry, these are specified by me
        self.goalpose = True # hmmm. Move base and....move_base? shit...
        self.reaction = False # joystick? No, lol

        # Track the last input control
        self.prev_reaction = self.reaction
        self.prev_goalpose = self.goalpose

        #TODO Establish the exact velocity methods used
        #NOTE: This is not being published.
        #These would be the velocities in the launch files that are remapped
        rospy.Subscriber('reactive_behaviours', Twist, self.reaction_bc) # joystick_cmd_vel
        rospy.Subscriber('planned_behaviours', Twist, self.goalpose_cb) # move_base_cmd_vel

        # !!!! This will not change (not really), up to...
        # Wait for the mux select service
        rospy.loginfo("Preparing the multiplexer...")
        rospy.wait_for_service('mux_cmd_vel/select', 60) # 60 seconds before timeout

        #TODO mux_select becomes 'prioritise'
        # Create a proxy for the mux select service
        prioritise = rospy.ServiceProxy('mux_cmd_vel/select', MuxSelect)

        rospy.loginfo("Multiplexer is active.")

        rospy.loginfo("Awaiting colour detection...")
        # !!!! ....about here. It is just console output

        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! IMPORTANT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        #TODO Determine switching criteria

        # Main loop to switch inputs if user move joystick
        while not rospy.is_shutdown():
            # If the last input is equal to the previous input, then use this input..
            if self.reaction and self.reaction != self.prev_reaction: # last_joystick
                prioritise('reactive_behaviours')
            # Equally, the same treatment for planned behaviours
            elif self.goalpose and self.goalpose != self.prev_goalpose: # last_movebase
                prioritise('planned_behaviours')

            self.prev_reaction = self.reaction # joystick
            self.prev_goalpose = self.goalpose # movebase

            r.sleep()

    # If the joystick is moved, get the message here
    def reaction_bc(self, msg): # joystick_cb
        self.lock.acquire() # puts a hold on the current control system ...
        self.reaction = True
        self.goalpose = False
        self.lock.release() # reverts back to the control system...

    # If move_base is active, get the message here
    def goalpose_cb(self, msg): # move_base_cb
        self.lock.acquire() # ...while this one is put on hold ....
        self.reaction = False # while reative behaviours are not occuring..
        self.goalpose = True # planned behaviours are prioritised
        self.lock.release() # ... and is enabled once more.

if __name__ == '__main__':
    Coordinator()
    rospy.spin()

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

#TODO Reference this properly
# https://github.com/pirobot/rbx2/blob/indigo-devel/rbx2_nav/nodes/select_cmd_vel.py

"""
