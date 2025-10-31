# Copyright 2025 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(module_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(module_2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Libraries
add_library(laser_detector_node SHARED
  src/laser_detector_node.cpp
)
target_include_directories(laser_detector_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(laser_detector_node
  PUBLIC
    module_2
    rclcpp
    rclcpp_components
    std_msgs
    sensor_msgs
)
rclcpp_components_register_node(
    laser_detector_node
    PLUGIN module_3::LaserDetectorNode
    EXECUTABLE laser_detector_main
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
  TARGETS laser_detector_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  module_2
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

ament_package()
