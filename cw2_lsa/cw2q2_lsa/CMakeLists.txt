cmake_minimum_required(VERSION 3.5)
project(cw2q2_lsa)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/cw2q2_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cw2q2_node
)

install(FILES
  src/youbot_kdl_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  bags
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
