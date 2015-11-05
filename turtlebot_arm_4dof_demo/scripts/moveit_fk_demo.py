#!/usr/bin/env python

"""
    moveit_fk_demo.py - Version 0.1.1 2015-08-26

    Use forward kinemtatics to move the arm to a specified set of joint angles

    Copyright 2014 by Patrick Goebel <patrick@pirobot.org, www.pirobot.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_OPEN = [-0.2, 0, 0]
GRIPPER_CLOSED = [1.25, 0, 0]
GRIPPER_NEUTRAL = [0.0, 0, 0]

class MoveItFKDemo:
    def __init__(self):
        # Initialize the move_group API and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_fk_demo', anonymous=True)

        # Use the groups of arm and gripper
        arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        gripper = moveit_commander.MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Set a goal joint tolerance
        arm.set_goal_joint_tolerance(0.04)
        gripper.set_goal_joint_tolerance(0.1)

        # Use the joint positions with FK
        # 1. Set the target joint values
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory
        joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(3)

        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(3)

        joint_positions = [0.1, -1.5707, -1.5707, 1.5707, 0.0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(5)

        gripper.set_joint_value_target(GRIPPER_CLOSED)
        gripper.go()
        rospy.sleep(5)

        joint_positions = [0.0, 0.0, 0.0, 1.5707, 0.0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(5)

        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFKDemo()
    except rospy.ROSInterruptException:
        pass
