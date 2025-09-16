# #!/usr/bin/env python3

# import rospy
# import csv
# import math

# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64
# from tkinter import Tk, Button, Label, Entry


# class JointStateGUI:
#     def __init__(self):
#         rospy.init_node('joint_state_gui')

#         # Define joint names
#         self.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5','J7', 'J10', 'J13', 'J16', 'J20','J24', 'J25', 'J26', 'J27', 'J28','J29', 'J32', 'J35', 'J38', 'J42']
        
#         # Initialize joint angles dictionary
#         self.joint_angles = {joint: 0.0 for joint in self.joint_names}

#         # Create a subscriber to get the joint states
#         rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)

#     def joint_state_callback(self, msg):
#         # Update joint angles dictionary
#         for i, joint in enumerate(msg.name):
#             if joint in self.joint_names:
#                 self.joint_angles[joint] = msg.position[i]


# def save_to_csv(joint_angles, name):
#     # Convert joint angles to degrees
#     joint_angles_degrees = {joint: math.degrees(angle) for joint, angle in joint_angles.items()}

#     # Write joint angles to a CSV file
#     with open('joint_angles.csv', 'a') as csvfile:
#         writer = csv.writer(csvfile)
#         writer.writerow(['Name'] + list(joint_angles_degrees.keys()))
#         writer.writerow([name] + list(joint_angles_degrees.values()))


# def gui_callback():
#     # Get name from the entry field
#     name = name_entry.get()

#     # Save joint angles to a CSV file
#     save_to_csv(joint_state_gui.joint_angles, name)


# if __name__ == '__main__':
#     # Create a GUI window
#     root = Tk()
#     root.title('Joint State GUI')

#     # Create a label and entry field for the name
#     name_label = Label(root, text='Name:')
#     name_label.pack()
#     name_entry = Entry(root)
#     name_entry.pack()

#     # Create a button
#     button = Button(root, text='Save to CSV', command=gui_callback)
#     button.pack()

#     # Initialize the JointStateGUI class
#     joint_state_gui = JointStateGUI()

#     # Start the GUI event loop
#     root.mainloop()

# #!/usr/bin/env python3

# import rospy
# import csv
# import math

# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64
# from tkinter import Tk, Button, Label, Entry, Checkbutton, IntVar

# file_path='/home/pi/zetaxbot/src/zetaxbot_remote/scripts/movements/'
# class JointStateGUI:
#     def __init__(self):
#         rospy.init_node('joint_state_gui')

#         # Define joint names
#         self.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5','J7', 'J10', 'J13', 'J16', 'J20','J24', 'J25', 'J26', 'J27', 'J28','J29', 'J32', 'J35', 'J38', 'J42','J46','J47']

#         # Initialize joint angles dictionary
#         self.joint_angles = {joint: 0.0 for joint in self.joint_names}

#         # Create a subscriber to get the joint states

#     def joint_state_callback(self, msg):
#         # Update joint angles dictionary
#         for i, joint in enumerate(msg.name):
#             if joint in self.joint_names:
#                 self.joint_angles[joint] = msg.position[i]


# def save_to_csv(joint_angles, name, write_mode):
#     # Convert joint angles to degrees
#     joint_angles_degrees = {joint: math.degrees(angle) for joint, angle in joint_angles.items()}
#     print(joint_angles_degrees)
#     # Determine the file mode (write or append)
#     file_mode = 'w' if write_mode else 'a'

#     # Write or append joint angles to the CSV file
#     with open(file_path+name, file_mode) as csvfile:
#         writer = csv.writer(csvfile)
        
#         # Write header if it's a new file
#         if file_mode == 'w':
#             writer.writerow(['Name'] + list(joint_angles_degrees.keys()))
        
#         writer.writerow([name] + list(joint_angles_degrees.values()))
#         # print('written '.format([name] + list(joint_angles_degrees.values())))


# def gui_callback():
#     # Get name from the entry field
#     name = name_entry.get()

#     # Get write mode from the checkbox
#     write_mode = write_var.get()

#     # Save joint angles to a CSV file
#     save_to_csv(joint_state_gui.joint_angles, name, write_mode)


# if __name__ == '__main__':
#     # Create a GUI window
#     root = Tk()
#     root.title('Joint State GUI')

#     # Create a label and entry field for the name
#     name_label = Label(root, text='File Name:')
#     name_label.pack()
#     name_entry = Entry(root)
#     name_entry.pack()

#     # Create a checkbox for write mode
#     write_var = IntVar()
#     write_checkbox = Checkbutton(root, text='Write Mode', variable=write_var)
#     write_checkbox.pack()

#     # Create a button
#     button = Button(root, text='Save to CSV', command=gui_callback)
#     button.pack()

#     # Initialize the JointStateGUI class
#     joint_state_gui = JointStateGUI()

#     # Start the GUI event loop
#     root.mainloop()

#!/usr/bin/env python3

import rospy
import csv
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tkinter import Tk, Button, Label, Entry, Checkbutton, IntVar

file_path='/home/pi/zetaxbot/src/zetaxbot_remote/scripts/movements/'

class JointStateGUI:
    def __init__(self):
        rospy.init_node('joint_state_gui')

        # Define joint names
        self.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5','J16', 'J20', 'J13', 'J10', 'J7','J24', 'J25', 'J26', 'J27', 'J28','J42', 'J38', 'J35', 'J32', 'J29','J46','J47']

        # Initialize joint angles dictionary
        self.joint_angles = {joint: 0.0 for joint in self.joint_names}

        # Create a subscriber to get the joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)

    def joint_state_callback(self, msg):
        # Update joint angles dictionary
        for i, joint in enumerate(msg.name):
            if joint in self.joint_names:
                self.joint_angles[joint] = msg.position[i]

def save_to_csv(joint_angles, name, write_mode,delay):
    # Convert joint angles to degrees
    joint_angles_degrees = {joint: math.floor(math.degrees(angle)) for joint, angle in joint_angles.items()}

    # Determine the file mode (write or append)
    file_mode = 'w' if write_mode else 'a'

    # Write or append joint angles to the CSV file
    with open(file_path+name+'.csv', file_mode) as csvfile:
        writer = csv.writer(csvfile)
        
        # Write header if it's a new file
        if file_mode == 'w':
            writer.writerow(['Name'] + list(joint_angles_degrees.keys())+['Delay'])
        
        writer.writerow([name] + list(joint_angles_degrees.values())+[delay])


def gui_callback():
    # Get name from the entry field
    name = name_entry.get()

    # Get delay from the entry field
    delay = delay_entry.get()
    # Get write mode from the checkbox
    write_mode = write_var.get()

    # Save joint angles to a CSV file
    save_to_csv(joint_state_gui.joint_angles, name, write_mode,delay)


if __name__ == '__main__':
    # Create a GUI window
    root = Tk()
    root.title('Joint State GUI')

    # Create a label and entry field for the name
    name_label = Label(root, text='File Name:')
    name_label.pack()
    name_entry = Entry(root)
    name_entry.pack()

    # Create a label and entry field for the name
    delay_label = Label(root, text='movement delay:')
    delay_label.pack()
    delay_entry = Entry(root)
    delay_entry.pack()
    # Create a checkbox for write mode
    write_var = IntVar()
    write_checkbox = Checkbutton(root, text='Write Mode', variable=write_var)
    write_checkbox.pack()

    # Create a button
    button = Button(root, text='Save to CSV', command=gui_callback)
    button.pack()

    # Initialize the JointStateGUI class
    joint_state_gui = JointStateGUI()

    # Start the GUI event loop
    root.mainloop()