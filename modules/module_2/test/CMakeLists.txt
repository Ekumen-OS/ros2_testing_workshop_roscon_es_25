find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_laser_detector
  test_laser_detector.cpp
)
target_link_libraries(test_laser_detector
  laser_detector
)
