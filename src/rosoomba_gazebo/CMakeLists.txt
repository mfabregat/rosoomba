cmake_minimum_required(VERSION 3.5)
project(rosoomba_gazebo)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(create_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
