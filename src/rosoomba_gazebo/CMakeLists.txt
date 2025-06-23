cmake_minimum_required(VERSION 3.5)
project(rosoomba_gazebo)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# gazebo
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)


ament_target_dependencies(gazebo_tf_publisher_node
  rclcpp
  geometry_msgs
  tf2_msgs
)

target_link_libraries(gazebo_tf_publisher_node
  gz-transport::core
)



install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)
