add_executable(ant_imu_control olive_app_main.cpp MainNode.cpp)

set_target_properties(ant_imu_control PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(ant_imu_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROTOBUF_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ant_imu_control
pthread
rt
)

ament_target_dependencies(
  ant_imu_control
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

install(TARGETS
ant_imu_control
DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY
# ${PROJECT_SOURCE_DIR}/launch
#   DESTINATION share/${PROJECT_NAME}
# )
