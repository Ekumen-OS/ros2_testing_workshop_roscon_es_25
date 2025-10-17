find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_laser_detector_node
  test_laser_detector_node.cpp
)
target_link_libraries(test_laser_detector_node
  laser_detector_node
)
ament_target_dependencies(test_laser_detector_node
  rclcpp
  std_msgs
  sensor_msgs
)
