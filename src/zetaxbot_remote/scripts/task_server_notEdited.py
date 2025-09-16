#! /usr/bin/env python3
import rospy
import actionlib
from zetaxbot_remote.msg import ZetaxbotTaskAction, ZetaxbotTaskResult
import sys
import moveit_commander
import csv
from std_msgs.msg import UInt16MultiArray
import math
"""
  zetaxbot - task_server

  This script implements an Action Server that manages the execution
  of goals of the robot interfacing with moveit.
  Given a goal, it sends and execute a moveit trajectory

  Copyright (c) 2021 Antonio Brandi.  All right reserved.
"""
def parse_string(s):
    # Split the string into words
    words = s.split('/')

    # Remove any empty or whitespace-only words
    words = [w.strip() for w in words if w.strip()]

    # Return the list of words
    return words

def find_movement(movement_name,file_name):
    # Open the CSV file and search for the matching movement name
    with open('/home/pizetaxbot/src/zetaxbot_remote/scripts/'+file_name+'.csv', 'r') as file:
        reader = csv.reader(file)
        row_number = 0
        for row in reader:
            if row[0] == movement_name:
                # Print the row number and angles for the matching movement
                print(f"Row {row_number}: {row}")
                return row
                break
            row_number += 1
        else:
            # If no matching movement was found, print an error message
            print("No matching movement found.")

class TaskServer(object):
    # create messages that are used to publish feedback/result
    result_ = ZetaxbotTaskResult()
    right_arm_goal_ = []
    left_arm_goal_ = []
    head_goal_ = []
    gripper_goal_ = []

    def __init__(self, name):
        # Constructor
        # function that inizialize the ZetaxbotTaskAction class and creates 
        # a Simple Action Server from the library actionlib
        # Constructor that gets called when a new instance of this class is created
        # it basically inizialize the MoveIt! API that will be used throughout the script
        # initialize the ROS interface with the robot via moveit
        moveit_commander.roscpp_initialize(sys.argv)

        # create a move group commander object that will be the interface with the robot joints
        self.right_arm_move_group_ = moveit_commander.MoveGroupCommander('right_arm') 
        # create a move group commander object that will be the interface with the robot joints
        self.left_arm_move_group_ = moveit_commander.MoveGroupCommander('left_arm')
        # create a move group commander object that will be the interface with the robot joints
        # self.head_move_group_ = moveit_commander.MoveGroupCommander('head')

        # create a move group commander object for the gripper
        self.gripper_move_group_ = moveit_commander.MoveGroupCommander('head')

        self.action_name_ = name
        self.as_ = actionlib.SimpleActionServer(self.action_name_, ZetaxbotTaskAction, execute_cb=self.execute_cb, auto_start = False)
        self.as_.start()
      

    def execute_cb(self, goal):
        global right_hand_pub
        global left_hand_pub
        success = True     
        splitted_str=parse_string(goal.movement_name)
        # start executing the action
        # based on the goal id received, send a different goal 
        # to the robot
        if splitted_str[1]=='right_arm':
            movement=find_movement(splitted_str[2], 'right_arm')
            # print(movement)
            self.right_arm_goal_ = [float(int(x)*math.pi/180) for x in movement[1:]]
            # Sends a goal to the moveit API
            self.right_arm_move_group_.go(self.right_arm_goal_, wait=True)
            # Make sure that no residual movement remains
            self.right_arm_move_group_.stop()

        elif splitted_str[1]=='left_arm':
            movement=find_movement(splitted_str[2], 'left_arm')
            # print(movement)
            self.left_arm_goal_ = [float(int(x)*math.pi/180) for x in movement[1:]]
            self.left_arm_move_group_.go(self.left_arm_goal_, wait=True)
            self.left_arm_move_group_.stop()

        # if splitted_str[1]=='left_arm':
        elif splitted_str[1]=='head':
            movement=find_movement(splitted_str[2], 'head')
            # print(movement)
            self.gripper_goal_ = [float(int(x)*math.pi/180) for x in movement[1:]]
            self.gripper_move_group_.go(self.gripper_goal_, wait=True)
            self.gripper_move_group_.stop()

            # if splitted_str[2]=='0':
            #     self.gripper_goal_ = [0.5, 0.0]
            # elif splitted_str[2]=='1':
            #     self.gripper_goal_ = [-0.7, 0.7]

        elif splitted_str[1]=='right_hand':
            movement=find_movement(splitted_str[2],'right_hand')
            publishedAngles = [int(x)+90 for x in movement[1:]]
            msg = UInt16MultiArray()
            msg.data=publishedAngles
            print('published angles{0}'.format(publishedAngles))
            # ROS_INFO(str(publishedAngles))
            right_hand_pub.publish(msg)

        elif splitted_str[1]=='left_hand':
            movement=find_movement(splitted_str[2],'left_hand')
            publishedAngles = [int(x)+90 for x in movement[1:]]
            msg = UInt16MultiArray()
            msg.data=publishedAngles
            print('published angles',publishedAngles)
            # ROS_INFO(str(publishedAngles))
            left_hand_pub.publish(msg)
        # if goal.movement_name == '0':
        #     self.right_arm_goal_ = [0.0 ,0.0 ,0.0 ,0.0 ,0.0]
            
        # elif goal.movement_name == '1':
            
            
        # elif goal.movement_name == '2':
        #     self.right_arm_goal_ = [0.8 ,1.2 ,0.9 ,0.4 ,0.2]
        #     self.gripper_goal_ = [0.9, 1]
        else:
            rospy.logerr('Invalid goal')
            return

        # # Sends a goal to the moveit API
        # self.right_arm_move_group_.go(self.right_arm_goal_, wait=True)
        # # Make sure that no residual movement remains
        # self.right_arm_move_group_.stop()

        
        # check that preempt has not been requested by the client
        if self.as_.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.action_name_)
            self.as_.set_preempted()
            success = False
       
        # check if the goal request has been executed correctly
        if success:
            self.result_.success = True
            rospy.loginfo('%s: Succeeded' % self.action_name_)
            self.as_.set_succeeded(self.result_)        


if __name__ == '__main__':
    global right_hand_pub
    global left_hand_pub
    # Inizialize a ROS node called task_server
    rospy.init_node('task_server')

    server = TaskServer(rospy.get_name())
    right_hand_pub = rospy.Publisher('/arduino/right_hand_actuate', UInt16MultiArray, queue_size=10)
    left_hand_pub = rospy.Publisher('/arduino/left_hand_actuate', UInt16MultiArray, queue_size=10)
    
    # keeps the node up and running
    rospy.spin()
