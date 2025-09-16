#!/usr/bin/env python3
from zetaxbot_remote.msg import ZetaxbotTaskAction, ZetaxbotTaskGoal
import rospy
import threading
import actionlib
import csv
from std_msgs.msg import String,UInt16

# threading.Thread(target=lambda: rospy.init_node('alexa_interface', disable_signals=True)).start()

client = actionlib.SimpleActionClient('task_server', ZetaxbotTaskAction)


def parse_string(s):
    # Split the string into words
    words = s.split('/')

    # Remove any empty or whitespace-only words
    words = [w.strip() for w in words if w.strip()]

    # Return the list of words
    return words

def find_movement(movement_name,file_name):
    # Open the CSV file and search for the matching movement name
    with open(file_name, 'r') as file:
        reader = csv.reader(file)
        row_number = 0
        for row in reader:
            if row[0] == movement_name:
                # Print the row number and angles for the matching movement
                print(f"Row {row_number}: {row}")
                break
            row_number += 1
        else:
            # If no matching movement was found, print an error message
            print("No matching movement found.")
# # Ask the user for the movement name
# movement_name = input("Enter the movement name: ")
# find_movement(movement_name=movement_name,file_name='right_arm.csv')

# goal_no=input('enter goal number:0-3')
# goal = ZetaxbotTaskGoal(task_number=goal_no)
# client.send_goal(goal)
class TaskClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('task_server', ZetaxbotTaskAction)
        self.client.wait_for_server()

        # Subscribe to the /movement_name topic
        self.subscriber = rospy.Subscriber('/movement_name', String, self.on_movement_name)
        print('subscried successfully')
        # rospy.spin()
    def on_movement_name(self, message):
        # When a movement name is received, send it as a goal to the task server
        # string1 =parse_string(message.data)
        # find_movement(string1[2],string1[1]+'.csv')
        # ZetaxbotTaskGoal(task_client   )
        print('data {0}'.format(message.data))
        goal = ZetaxbotTaskGoal(movement_name=message.data)
        self.client.send_goal(goal)


if __name__ == '__main__':
    rospy.init_node('alexa_interface', disable_signals=True)
    # Create an instance of the TaskClient class
    task_client = TaskClient()

    # Spin the ROS node to handle events
    rospy.spin()
    # while True:
    #     goal_no=int(input('enter goal number:0-3'))
    #     if goal_no == 5:
    #         break
    #     goal = ZetaxbotTaskGoal(task_number=goal_no)
    #     client.send_goal(goal)
        
        