cmake_minimum_required(VERSION 2.8.3)
project(zeta_chat)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SynthesizeSpeech.srv
  RecognizeSpeech.srv
  GetGptResponse.srv
  ClassifyInput.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  gpt_node.py
  speech_recognition_node.py
  voice_node.py
  classification_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
