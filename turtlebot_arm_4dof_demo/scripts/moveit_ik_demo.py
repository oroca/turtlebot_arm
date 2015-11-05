#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1.1 2015-08-26

    Use inverse kinemtatics to move the end effector to a specified pose

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
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_OPEN = [-0.2, 0, 0]
GRIPPER_CLOSED = [1.25, 0, 0]
GRIPPER_NEUTRAL = [0.0, 0, 0]

REFERENCE_FRAME = '/base_link'

class MoveItIKDemo:
    def __init__(self):
        # Initialize the move_group API and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_ik_demo', anonymous=True)

        # Use the groups of SmartPal5
        arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        gripper = moveit_commander.MoveGroupCommander(GROUP_NAME_GRIPPER)
        end_effector_link = arm.get_end_effector_link()

        # Set a goal joint tolerance
        arm.set_goal_joint_tolerance(0.04)
        gripper.set_goal_joint_tolerance(0.1)

        # Set the option related IK solution
        arm.allow_replanning(True)

        # Use the joint positions with FK
        joint_positions = [0.0, 0.0, 0.0, 1.5707, 0.0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(5)

        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(5)

        # Use the joint pose with IK
        # 1. Set the target pose for IK
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory

        # Move the arm to left side
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x =  0.0
        target_pose.pose.position.y =  0.194
        target_pose.pose.position.z =  0.147
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.707
        target_pose.pose.orientation.w = 0.707

        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(5)

        # Grasp
        gripper.set_joint_value_target(GRIPPER_CLOSED)
        gripper.go()
        rospy.sleep(5)

        # Move the arm to right side
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x =  0.0
        target_pose.pose.position.y = -0.194
        target_pose.pose.position.z =  0.147
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = -0.707
        target_pose.pose.orientation.w =  0.707

        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(5)

        # Place
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(5)

        # Move the arm to center
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x =  0.0
        target_pose.pose.position.y =  0.0
        target_pose.pose.position.z =  0.341
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = -0.706
        target_pose.pose.orientation.z =  0.0
        target_pose.pose.orientation.w =  0.707

        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(5)

        gripper.set_joint_value_target(GRIPPER_CLOSED)
        gripper.go()
        rospy.sleep(5)

        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItIKDemo()
    except rospy.ROSInterruptException:
        pass
