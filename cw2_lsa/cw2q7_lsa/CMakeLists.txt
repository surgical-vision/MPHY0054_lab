cmake_minimum_required(VERSION 3.5)
project(cw2q7_lsa)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

install(PROGRAMS
  src/cw2q7.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cw2q7
)

install(DIRECTORY launch config urdf bag
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(FILES hardware_interface_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES hardware_interface_plugins.xml
  DESTINATION share/ament_index/resource_index/hardware_interface__pluginlib__plugin
  RENAME ${PROJECT_NAME}
)

ament_package()
