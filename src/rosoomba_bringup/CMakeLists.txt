cmake_minimum_required(VERSION 3.8)
project(rosoomba_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
