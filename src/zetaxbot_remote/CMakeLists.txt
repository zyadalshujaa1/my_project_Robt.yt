cmake_minimum_required(VERSION 3.0.2)
project(zetaxbot_remote)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib             
  std_msgs                      
  message_generation 
  moveit_ros_planning_interface
)

add_action_files(
  FILES 
  ZetaxbotTask.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    scripts/task_server.py 
    scripts/alexa_interface.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
