cmake_minimum_required(VERSION 3.5)
project(rosoomba_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  meshes
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
