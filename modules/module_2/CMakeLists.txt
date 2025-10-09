cmake_minimum_required(VERSION 3.8)
project(module_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Libraries
add_library(bad_laser_detector
  src/bad_laser_detector.cpp
)
target_include_directories(bad_laser_detector
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(bad_laser_detector
  PUBLIC
    rclcpp
    std_msgs
    sensor_msgs
)

add_library(laser_detector
  src/laser_detector.cpp
)
target_include_directories(laser_detector
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.clang-format")
  set(ament_cmake_cpplint_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../CPPLINT.cfg")
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

# Installation
install(
  TARGETS bad_laser_detector
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  # Requires lib destination to be found by ros2 run package binary.
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)

ament_package()
