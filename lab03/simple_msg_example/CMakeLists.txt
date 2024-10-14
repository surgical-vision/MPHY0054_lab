cmake_minimum_required(VERSION 3.0.2)
project(simple_msg_example)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  CustomMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/msg_publisher.py src/msg_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
