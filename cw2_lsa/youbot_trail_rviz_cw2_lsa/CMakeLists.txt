cmake_minimum_required(VERSION 3.5)
project(youbot_trail_rviz_cw2_lsa)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/youbot_trail_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME youbot_trail_node
)

install(DIRECTORY
  launch
  bags
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
