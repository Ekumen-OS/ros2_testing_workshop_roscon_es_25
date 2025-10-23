find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(module_3 REQUIRED)

# Function used to isolate the integration tests and avoid cross-talk between them
function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
endfunction()

add_ros_isolated_launch_test(./test_detection_launch.py)
