cmake_minimum_required(VERSION 3.8)
project(open_manipulator_description)

# ---- Find required ROS 2 packages ----
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# ---- Install resource directories ----

install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# ---- Export package ----
ament_package()
