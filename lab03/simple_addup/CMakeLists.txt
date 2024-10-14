cmake_minimum_required(VERSION 3.0.2)
project(simple_addup)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_service_files(
  FILES
  AddUp.srv
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

catkin_install_python(PROGRAMS src/addup_server.py src/addup_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
