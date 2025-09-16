#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose

rospy.init_node('move_group_test', anonymous=True)

# Initialize MoveIt commander
moveit_commander.roscpp_initialize(sys.argv)

# Create a robot commander object
robot = moveit_commander.RobotCommander()

# Create a move group commander object
right_arm_group = moveit_commander.MoveGroupCommander('right_arm')
print('joints values {0}'.format(right_arm_group.get_current_joint_values()))
# Set the planner and planning time
right_arm_group.set_planner_id('RRTConnectkConfigDefault:')
right_arm_group.set_planning_time(50)

# Set the target pose (in this example, we're setting a target pose of all zeros)
target_pose = Pose()
target_pose.position.x = -0.440
target_pose.position.y = -0.30
target_pose.position.z = 0.64
# target_pose.orientation.x = 0.0
# target_pose.orientation.y = 0.0
# target_pose.orientation.z = 0.0
# target_pose.orientation.w = 1.0
#  x: 4011368751526
#   y: 689990520477295
#   z: 84317779541016
# ---

# Set the target pose for the right arm group

print('set pose {0}'.format(right_arm_group.set_pose_target(target_pose)))
# Plan and execute the motion
plan = right_arm_group.go(wait=True)
print('set plan {0}'.format(plan))

# Clean up MoveIt commander
moveit_commander.roscpp_shutdown()