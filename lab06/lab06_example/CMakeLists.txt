cmake_minimum_required(VERSION 3.5)
project(lab06_example)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/lab06_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lab06_example
)

install(FILES
  src/urdf_kdl_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
