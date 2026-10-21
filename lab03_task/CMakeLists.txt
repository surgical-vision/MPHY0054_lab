cmake_minimum_required(VERSION 3.8)
project(lab03_task)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Rotmat2quat.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

install(PROGRAMS server.py DESTINATION lib/${PROJECT_NAME} RENAME server)
install(PROGRAMS client.py DESTINATION lib/${PROJECT_NAME} RENAME client)
install(PROGRAMS self_check.py DESTINATION lib/${PROJECT_NAME} RENAME check)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES CHECKING.md DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
