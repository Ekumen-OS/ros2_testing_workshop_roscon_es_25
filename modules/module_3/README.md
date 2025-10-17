# Module 3 – ROS Unit Testing

This module introduces the development of **unit tests for ROS 2 nodes** and interfaces, focusing on how to validate node behavior, parameters, and message exchanges while ensuring test isolation.

- [Module 3 – ROS Unit Testing](#module-3--ros-unit-testing)
  - [Objectives](#objectives)
  - [Motivation](#motivation)
  - [ROS Unit Tests](#ros-unit-tests)
    - [Test Fixtures](#test-fixtures)
    - [Parameters](#parameters)
    - [Topics](#topics)
    - [Services](#services)
    - [Node Lifecycle Management](#node-lifecycle-management)
    - [Node Pipeline](#node-pipeline)
  - [Test Isolation](#test-isolation)
  - [Alternative: The Rtest Framework](#alternative-the-rtest-framework)
  - [Exercises](#exercises)
    - [Exercise 1](#exercise-1)
      - [Definition of success](#definition-of-success)
    - [Exercise 2](#exercise-2)
      - [Definition of success](#definition-of-success-1)
  - [References](#references)

## Objectives

By the end of this module, participants will be able to:

- Implement **ROS-aware unit tests** using fixtures to manage node initialization and shutdown.
- Validate **ROS 2 node behavior and interfaces**, including topics, services, and parameters.
- Apply **domain isolation** to prevent node cross-talks during testing.

## Motivation

In the previous module, the core algorithm was successfully decoupled from ROS and validated with unit tests. Now that the algorithm behaves as expected, the next step is to **build a ROS 2 node** that wraps this logic, exposing it through standard ROS interfaces like subscribers and publishers so it can interact with other components in the system.

This wrapper node requires its own validation. **ROS unit testing** ensures that the interface layer is correct: that the node subscribes to the right topic, handles the incoming message correctly, executes the core logic, and publishes the expected results.

This layer of testing serves as a bridge between algorithmic validation and system-level integration. It confirms the correctness of the ROS 2 interface before proceeding to integration tests (Module 4). By testing each node in isolation, correctness can be verified without the complexity and non-determinism of a full multi-node system.

## ROS Unit Tests

Before any ROS-dependent test can run, the `rclcpp` system must be initialized and properly shut down. Doing this manually in every test quickly becomes repetitive and error-prone. To avoid this, **test fixtures** are used.

### Test Fixtures

A **test fixture** in GoogleTest defines a common test environment shared by multiple test cases. It handles the setup and teardown logic required to prepare that environment before each test and to clean it up afterward.

A fixture is created by deriving a class from `testing::Test`. The class body should begin with `protected` so that its members and methods are accessible from subclasses. The static lifecycle methods, however, must be declared as public, since the GoogleTest framework invokes them directly.

To manage the lifecycle:

- `SetUpTestCase`/`TearDownTestCase`: Static methods called once per test suite, before the first test and after the last one. These are ideal for global resources like `rclcpp::init()` and `rclcpp::shutdown()`, which should not be called repeatedly between tests.
- `SetUp`/`TearDown`: Instance methods executed before and after each individual test, respectively. These are used to create and destroy per-test resources such as nodes, publishers, or subscribers.

Example:

```cpp
class TestMyClass : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

protected:
  const rclcpp::NodeOptions default_node_options;
};
```

### Parameters

Parameters define node configuration and can be tested directly using `rclcpp::NodeOptions`. Parameter overrides allow verifying that parameters are correctly declared and propagated.

```cpp
TEST_F(TestMyClass, ParameterOverride)
{
  rclcpp::NodeOptions custom_node_options;
  custom_node_options.append_parameter_override("my_param", false);

  const MyClass dut(custom_node_options);

  ASSERT_TRUE(dut.has_parameter("my_param"));
  ASSERT_EQ(false, dut.get_parameter("my_param").as_bool());
}
```

This approach ensures parameters are initialized correctly before the node spins, mirroring how ROS 2 parameters are passed during node construction.

> [!TIP]
> Define parameter names and default values as class constants inside the ROS 2 class e.g. `MyClass`. This centralizes parameter definitions, documents them in the header, and prevents typos or mismatches across nodes and tests.
>
> ```cpp
> static inline constexpr char MY_PARAM_NAME[] = "my_param";
> static inline constexpr bool MY_PARAM_DEFAULT_VALUE{false};
> ```

### Topics

Publishers and subscribers can be verified using `rclcpp::Node` API `get_topic_names_and_types()`. This confirms that topics are correctly declared and registered within the ROS graph.

```cpp
TEST_F(TestMyClass, TopicRegistration)
{
  const MyClass dut(default_node_options);

  const auto topic_map = dut.get_topic_names_and_types();
  const std::string topic_name = "topic_name";
  const std::string expected_type = "std_msgs/msg/String";

  ASSERT_TRUE(topic_map.find(topic_name) != topic_map.end());
  ASSERT_FALSE(topic_map.at(topic_name).empty());
  ASSERT_EQ(expected_type, topic_map.at(topic_name)[0]);
}
```

This test confirms that the node correctly registers its topics and associates them with the expected message types, ensuring the interface is properly defined.

### Services

Similarly to topics, services can be verified using the `get_service_names_and_types()` API. This check ensures that the node correctly registers its services with the expected names and types in the ROS graph.

```cpp
TEST_F(TestMyClass, ServiceRegistration)
{
  const MyClass dut(default_node_options);

  const auto service_map = dut.get_service_names_and_types();
  const std::string service_name = "reset_service";
  const std::string expected_type = "std_srvs/srv/Empty";

  ASSERT_TRUE(service_map.find(service_name) != service_map.end());
  ASSERT_FALSE(service_map.at(service_name).empty());
  ASSERT_EQ(expected_type, service_map.at(service_name)[0]);
}
```

### Node Lifecycle Management

Lifecycle nodes define a managed execution model in which resources such as publishers, subscribers, and timers are created and destroyed as the node transitions between states. Unit tests can directly trigger and validate these state transitions using the rclcpp_lifecycle API, verifying that transitions occur correctly and that side effects happen as expected.

```cpp
TEST_F(TestMyLifecycleNode, LifecycleTransitions)
{
  auto dut = std::make_shared<MyLifecycleNode>(default_node_options);

  EXPECT_EQ(dut->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  dut->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(dut->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  dut->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  EXPECT_EQ(dut->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}
```

Testing lifecycle transitions in this way ensures that the node’s initialization, activation, and cleanup logic behaves consistently, supporting predictable and reliable operation when deployed in a managed system.

### Node Pipeline

Node pipeline tests verify that data flows correctly through a node from input subscription to output publication. These tests simulate the node’s full behavior without involving external processes.

A typical pipeline test setup includes:

- A test node that publishes inputs and subscribes to outputs.
- The DUT (Device Under Test) node instance.
- An executor to spin both nodes.
- A deterministic wait/spin utility.

When dealing with time and synchronization in pipeline tests, **determinism is key**. The following practices help ensure tests remain reliable and repeatable:

- **Drive the executor explicitly** — Instead of relying on arbitrary delays, use short loops that call `executor.spin_some()` until a specific condition is met or a timeout expires. This guarantees that callbacks are processed precisely when needed.
- **Wait on futures for blocking operations** — For services and actions, use `executor.spin_until_future_complete(fut, timeout)` to process executor work while waiting for responses in a controlled, timeout-bound manner.
- **Use simulated time for timer-driven logic** — Enable `use_sim_time=true` and publish `/clock` messages from the test node to advance time deterministically. Each clock tick can trigger timers without waiting in real time.
- **Avoid arbitrary sleeps** — While short sleeps like `std::this_thread::sleep_for()` may occasionally be used, they should never replace condition-based or event-driven waits.

A complete example of a pipeline test (including message publication, synchronization, and assertions) is provided in the Exercises section.

## Test Isolation

When running unit tests in ROS 2, especially in large workspaces, one might encounter the **cross-talk** issue where nodes receive unintended messages from other tests.

By default, all ROS 2 nodes operate in domain ID 0 (`ROS_DOMAIN_ID=0`), sharing the same DDS communication space. Nodes within a domain automatically discover each other and can exchange messages, even if they belong to different packages or test executors.

Since tests are run in parallel by default, nodes publishing or subscribing to identical topic names can interfere across tests. This results in flaky and nondeterministic behavior (e.g., a subscriber receiving unexpected messages) or immediate failures (e.g., when a service server attempts to instantiate a name already in use).

There are two ways to solve the issue:

- **Sequential Execution**: Run tests one after another using `colcon test --executor sequential`. By running tests sequentially, even if nodes in different tests are publishing to the same topic they should not affect subsequent tests. However, this dramatically increases the total execution time for large projects.

- **Domain Isolation with ROS_DOMAIN_ID (Recommended)**: This approach leverages the DDS domain mechanism in ROS 2 by assigning a unique `ROS_DOMAIN_ID` to each test. This confines all communication within the test’s scope, preventing cross-talk and ensuring deterministic behavior.

The `ament_cmake_ros` package provides a convenient solution for domain isolation. It includes CMake functions that automatically assign an available domain ID to each test executor.

To enable this, add the following dependency to `package.xml`:

```xml
<test_depend>ament_cmake_ros</test_depend>
```

Then, replace the standard test command in `CMakeLists.txt` with the isolated version:

```cmake
find_package(ament_cmake_ros REQUIRED)
ament_add_ros_isolated_gtest(test_my_node test/test_my_node.cpp)
```

## Alternative: The Rtest Framework

While this module focuses on standard GoogleTest-based ROS 2 testing using `rclcpp` and `ament_cmake_gtest`, an emerging framework called Rtest provides an alternative approach.

Rtest offers tools to mock and introspect ROS 2 entities such as publishers, subscribers, services, timers, and actions. It allows direct message injection, timer triggering, and simulated time control—enabling precise and fully deterministic tests without spinning an executor.

At this stage, Rtest remains **experimental** and has a key limitation: it can only test ROS 2 components whose source code is available in the workspace, since it relies on compile-time template substitution to mock and intercept ROS entities. This prevents testing against precompiled or system-installed packages. Despite this, Rtest shows strong potential as a unified, deterministic testing layer that could **complement the approach presented in this module**.

## Exercises

In this module, participants will debug and complete a ROS 2 node and its corresponding unit tests, fixing intentional defects and ensuring all tests pass deterministically.

### Exercise 1

Start from the provided `LaserDetectorNode` ROS2 node implementation and unit tests. The goal is to identify and fix the issues that currently prevent the tests from passing, ensuring the node can be compiled, tested deterministically, and behaves as expected.

What to review:

- Source: [laser_detector_node.cpp](src/laser_detector_node.cpp) — intentional defects are present.
- Tests: [test_laser_detector.cpp](test/test_laser_detector.cpp) — contains `BEGIN EDIT / END EDIT` blocks and an intentional failing check.

Tasks:

- Build the package:
  
  ```bash
  colcon build --packages-up-to module_3 --event-handlers console_direct+
  ```

- Run the tests and examine the output to identify which parts of the node or tests are failing.

  ```bash
  colcon test --packages-up-to module_3 --event-handlers console_direct+
  ```

- Review the node’s source files to find and correct issues such as typos and mismatched parameter or topic names that cause the tests to fail.
- Locate the `BEGIN EDIT / END EDIT` blocks in the test file and replace the placeholder or failing statements with the appropriate assertions. Remove the intentional failure once the expected logic is implemented.
- Use the provided utilities (`spin_until`, `make_scan`) for deterministic execution for the missing test instead of arbitrary sleeps or background spinning threads.

#### Definition of success

The task is complete when tests are run and the output shows **0 errors and 0 failures**, demonstrating that:

- The node declares and retrieves parameters correctly.
- Publishers and subscribers are registered under the correct topic names and message types.
- The node processes incoming LaserScan messages without exceptions.
- The obstacle detection logic produces correct Boolean results for both obstacle-present and obstacle-absent scenarios

### Exercise 2

In this exercise, the build configuration will be modified to ensure that the unit test for this package runs **in isolation,** without interference from other tests or nodes using the same topics.

When running multiple ROS 2 tests in parallel, shared topic names can cause cross-talk between nodes, leading to flaky or nondeterministic results. ROS provides a mechanism to automatically assign unique domain IDs to each test, preventing this issue. Although this problem does not occur in this module, enabling isolation is considered a good practice to avoid future headaches as the software stack grows.

Tasks:

- Update the [CMakeLists.txt](CMakeLists.txt) and [package.xml](package.xml) files to enable test isolation.
- Rebuild the package and run the tests again.

  ```bash
  colcon build --packages-up-to module_3 --event-handlers console_direct+
  colcon test --packages-up-to module_3 --event-handlers console_direct+
  ```

#### Definition of success

The task is complete when the tests run successfully and the console shows `Running with ROS_DOMAIN_ID X`, where `X` is an automatically assigned domain ID (its value may vary depending on how many tests are running in parallel). This confirms that the test is being executed in isolation.

> [!NOTE]
> Refer back to the [Test Isolation](#test-isolation) section of this module to identify which package and macro are used to achieve isolation.

## References

- [ROS 2 Test Isolation](https://medium.com/@yifeicheng/test-isolation-and-avoiding-cross-talk-in-ros-2-3731f54e9ccf)
- [Test Fixtures in GoogleTest](https://google.github.io/googletest/primer.html#same-data-multiple-tests)
- [Rtest Unit Testing Framework](https://github.com/Beam-and-Spyrosoft/rtest/)
- [rclcpp Node header](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/node.hpp)
- [rclcpp_lifecycle Node header](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp)