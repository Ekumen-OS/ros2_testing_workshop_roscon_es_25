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
project(module_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Example executable
add_executable(bad_example src/bad_example.cpp)
ament_target_dependencies(bad_example
  rclcpp
  std_msgs
)

# Install the executable
install(TARGETS bad_example
  DESTINATION bin/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.clang-format")
  set(ament_cmake_cpplint_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../CPPLINT.cfg")

  # This is the key to execute the tests with all the linters included as test dependencies.
  ament_lint_auto_find_test_dependencies()
  # NOTE: if any of the packages specified as test dependencies is missing, this won't fail. 
  # Instead, it will just skip that linter and execute the rest.
endif()

ament_package()
