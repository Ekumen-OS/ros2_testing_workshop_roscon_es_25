# Module 4 – Integration Testing

In this module, the focus is to explore **integration testing in ROS 2** to verify that multiple nodes work together as expected.

- [Module 4 – Integration Testing](#module-4--integration-testing)
  - [Objectives](#objectives)
  - [Motivation](#motivation)
  - [The launch\_testing Framework](#the-launch_testing-framework)
    - [Alternatives](#alternatives)
    - [Registering the tests](#registering-the-tests)
  - [Exercises](#exercises)
    - [Exercise 1](#exercise-1)
      - [Definition of Success](#definition-of-success)
  - [References](#references)


## Objectives

By the end of the module, participants will be able to:

- Understand the role of integration testing in a robotics software stack.
- Write a basic integration test using the **launch_testing framework**.
- Create a ROS 2 launch file embedded within a Python test script.
- Use launch_testing actions to **start nodes and verify their behavior**.
- Execute integration tests with colcon test and interpret the results.

## Motivation

While linters and unit tests are essential for verifying the quality of individual code files and functions, they can't guarantee that the **different parts of the robotic system will work correctly when they communicate with each other**. Integration tests fill this critical gap.

Think of a robotic system as a team of specialists. A unit test ensures each specialist (a single node) can do their job correctly in isolation. An integration test ensures they can communicate and collaborate effectively as a team to achieve a larger goal.

The importance of integration testing in robotics includes:

- **Verifying Communication**: They confirm that nodes are using the correct topic names, message types, and service definitions. A mismatch here is a common bug that unit tests won't catch.
- **Validating System Behavior**: They enable testing a sequence of interactions. For example: "Does the camera driver node correctly publish an image that the perception node can process?"
- **Catching Runtime Errors**: They can uncover issues related to timing, network configuration, and race conditions that only appear when multiple nodes are running concurrently.
- **Automating System-Level Checks**: Integration tests provide an automated way to perform "sanity checks" on a multi-node application, ensuring that the core functionalities are working before deploying to a physical robot.

In ROS 2, this is primarily achieved using the `launch_testing` framework, which cleverly **combines the power of ROS 2 launch files with standard Python testing libraries** like unittest. This makes possible the definition, launching, and testing of an entire multi-node system from a single, automated script.

## The launch_testing Framework

`launch_testing` is the standard tool for writing integration tests in ROS 2. It is possible to write a Python script that both launches a set of nodes and runs tests to verify their behavior.

A launch_testing script typically has two main parts:

- **generate_test_description()**: This function is the entry point. It works just like a standard ROS 2 launch file, defining which nodes to start, their parameters, and any other launch actions.
- **A unittest.TestCase Class**: This is where the actual tests are written. These tests run after the nodes have been launched and the system is ready.

Here are the key components that will be used:

| Component                                       | What it does                                                                                                       | Example Usage                                                                               |
| :---------------------------------------------- | :----------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------ |
| `launch.actions`<br>`.ExecuteProcess`           | The standard launch action to execute a command-line process, such as a ROS 2 node.                                | `ExecuteProcess(cmd=`<br>`['my_package', 'my_node'])`                                       |
| `launch_testing.actions`<br>`.WaitForProcesses` | An action that pauses the launch until a specific process emits a certain line of output.                          | `WaitForProcesses(`<br>`process_to_wait_for=node_process,`<br>` on_output='Node is ready')` |
| `launch_testing.actions`<br>`.ReadyToTest`      | A special action that signals to the test framework that the system setup is complete and tests can begin.         | `return LaunchDescription(`<br>`[..., ReadyToTest()])`                                      |
| `launch_testing.asserts`<br>`.assertIn`         | An assertion helper that checks if a specific string appears in the captured output of a process.                  | `self.assertIn(`<br>`b'Node initialized',`<br>` proc_output.stdout)`                        |
| `proc_info`,<br> `proc_output`                  | Handlers passed to the test methods that enable inspecting the exit codes and stdout/stderr of launched processes. | `def test_node_output(`<br>`self, proc_output): ...`                                        |

These are instructions used in the core `launch_testing` framework. For ROS 2 integration, `launch_ros` is used, providing support for directly launching nodes or lifecycle nodes, instead of relying in the `ExecuteProcess` action.

The workflow for running integration tests is identical to running linter or unit tests. You use colcon to execute them as part of the package's test suite.

It's important to notice that even though the `launch_testing` framework is written in Python, tests can execute nodes written in any language, because they interact with them as system processes. This enforces the testing process to be language agnostic, and to test the nodes using the ROS API and external behavior, treating them as black boxes.

### Alternatives

While using `launch_testing` with `unittest` is the classic approach used, support for more modern approaches like using `pytest` is also available. `Pytest` is a powerful and modern third party framework (`unittest` is part of the Python standard library) that has become the most used option for Python testing in the community. It is also gaining popularity within the ROS ecosystem.

That's why `launch_pytest` was created, as a bridge between the `launch_testing` and `pytest` frameworks. As `launch_testing` is a standalone testing tool, it lacks many features, such as being able to mark tests as skipped or xfail (expected to fail).

In this case, `launch_testing` is going to be used in the exercise for the sake of simplicity and for learning about the core concepts, as using `launch_pytest` requires more advanced knowledge.

### Registering the tests

After developing the tests, it's necessary to register them in order to be executed when `colcon test` is called. One thing to be extra careful about, as it was presented in Module 3, is test isolation in ROS 2 to avoid the cross-talk between different tests being run in parallel.

`ament_cmake_ros` provides the `run_test_isolated.py` special test runner to ensure the isolation with unique domain ids for each test.

To ease adding several integration tests, the CMake function `add_ros_isolated_launch_test` is usually defined, so the instructions needed in your CMakeLists.txt are similar to this:

```cmake
if(BUILD_TESTING)
  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_integration.py)
endif()
```

And in the package.xml:

```xml
<test_depend>ament_cmake_ros</test_depend>
<test_depend>launch</test_depend>
<test_depend>launch_ros</test_depend>
<test_depend>launch_testing</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
```

## Exercises

### Exercise 1

The objective is to write an integration test that launches 2 nodes (the `laser_detector` from previous module and a new `safety_light` node) and verifies that an action in the first node triggers a reaction in the second node. This is the most common and powerful way to test integration with ROS 2 nodes.

The package already contains the [safety_light_node.cpp](./src/safety_light_node.cpp). The task is to complete the Python test script [test/test_detection_launch.py](./test/test_detection_launch.py). This test will use `rclpy` to create a temporary subscriber within the test itself to listen for messages from the C++ node.

First, build and source the workspace to ensure the talker node is available:

<!-- TODO Add info about building Module 3 -->

```bash
cd ~/ws
colcon build --packages-select module_4
source install/setup.bash
```

Now, run the tests. This will fail because the test script is incomplete:

```bash
colcon test --packages-select module_4
colcon test-result --verbose
```

The incomplete test script already handles launching the nodes. You need to fill in the logic inside the `unittest.TestCase` to verify its behavior.

The additions to the Python test script must:

1. Initialize `rclpy` and create a temporary node.        
2. Create a publisher to the `/scan` topic.
3. Create and publish a `LaserScan` message that will trigger the detector.
4. Use `proc_output.assertWaitFor` to check for the "RED LIGHT" message.
5. Shutdown `rclpy`.

#### Definition of Success
The task is complete when tests are run again, and the output of `colcon test-result --verbose` shows **0 errors and 0 failures** for the tests in module_4, with the new requirements in the test script.

## References

- [ROS 2 Documentation: Launch Testing (official tutorial)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Integration.html)
- [launch_testing GitHub Repository](https://github.com/ros2/launch/tree/jazzy/launch_testing)
- [Example launch_testing file](https://github.com/abaeyens/ros2-integration-testing-examples/blob/main/src/app/test/test_integration.py)
- [launch_ros Github repository](https://github.com/ros2/launch_ros/tree/jazzy)
