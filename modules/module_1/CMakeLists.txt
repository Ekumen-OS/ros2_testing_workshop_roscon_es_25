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

  # This is the key to execute the tests with all the linters included as test dependencies
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
