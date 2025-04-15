cmake_minimum_required(VERSION 3.8)
project(sensor_rig_transform_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python module(s)
install(
  PROGRAMS
    src/sensor_rig_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
