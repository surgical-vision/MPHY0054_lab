cmake_minimum_required(VERSION 3.5)
project(lab05_example)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/lab05_example.py
  src/lab05_task.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/urdf_kdl_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  my_robot.urdf
  my_robot_template.urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
