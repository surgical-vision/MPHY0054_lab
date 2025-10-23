cmake_minimum_required(VERSION 3.8)
project(youbot_description)

# Find ament_cmake to handle ROS 2 build logic
find_package(ament_cmake REQUIRED)

# Install the description, mesh, and urdf files to the package's share directory
install(DIRECTORY
  controller
  meshes
  robots
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Add standard ROS 2 linters for testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package
ament_package()