#!/usr/bin/env python3
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg

rospy.init_node('move_to_joint_positions')

# Create a MoveIt Move Group interface object for the arm planning group
group = moveit_commander.MoveGroupCommander("right_arm")

# Get the current joint positions
current_joint_positions = group.get_current_joint_values()

# Set the target joint positions
target_joint_positions = [1.0, 0.2, 0.3, 0.4, 0.5]
group.set_joint_value_target(target_joint_positions)

# Plan and execute the movement
plan = group.plan()
group.execute(plan)

# # Alternatively, you can use the MoveIt Action interface to plan and execute the movement
# move_group = actionlib.SimpleActionClient('move_group', moveit_msgs.msg.MoveGroupAction)
# move_group.wait_for_server()

# goal = moveit_msgs.msg.MoveGroupGoal()
# goal.request.group_name = "arm"
# goal.request.allowed_planning_time = 5.0
# goal.request.num_planning_attempts = 1

# goal.request.goal_constraints.joint_constraints = []
# for i, joint_position in enumerate(target_joint_positions):
#     joint_constraint = moveit_msgs.msg.JointConstraint()
#     joint_constraint.joint_name = f"joint_{i+1}"
#     joint_constraint.position = joint_position
#     joint_constraint.tolerance_above = 0.1
#     joint_constraint.tolerance_below = 0.1
#     joint_constraint.weight = 1.0
#     goal.request.goal_constraints.joint_constraints.append(joint_constraint)

# move_group.send_goal(goal)
# move_group.wait_for_result()

# result = move_group.get_result()
# if result:
#     print("Movement successful!")
# else:
#     print("Movement failed :(")