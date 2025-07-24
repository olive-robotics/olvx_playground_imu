cmake_minimum_required(VERSION 3.5)
project(sync_republish_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Declare the executable
add_executable(sync_republish_node src/sync_republish_node.cpp)
ament_target_dependencies(sync_republish_node
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

# Install executable
install(TARGETS
  sync_republish_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
