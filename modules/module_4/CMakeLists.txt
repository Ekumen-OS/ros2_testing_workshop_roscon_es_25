cmake_minimum_required(VERSION 3.8)
project(module_4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Libraries
add_library(safety_light
  src/safety_light.cpp
)
target_include_directories(safety_light
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(safety_light
  PUBLIC
    rclcpp
    std_msgs
)

# Executables
add_executable(safety_light_node src/safety_light_node.cpp)
target_link_libraries(safety_light_node
  safety_light
)
ament_target_dependencies(safety_light_node
  rclcpp
  std_msgs
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
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS safety_light
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS safety_light_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION bin/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
)

ament_package()
