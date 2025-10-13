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
    - [Node Pipeline](#node-pipeline)
  - [Test Isolation](#test-isolation)
  - [Exercises](#exercises)
  - [References](#references)

## Objectives

By the end of this module, participants will be able to:

- Implement **ROS-aware unit tests** using fixtures to manage node initialization and shutdown.
- Validate **ROS 2 node behavior and interfaces**, including topics, services, and parameters.
- Apply **domain isolation** to prevent node cross-communication during testing.

## Motivation

In the previous module, the core algorithm was successfully isolated and validated with unit tests. Now that the algorithm behaves as expected, the next step is to **build a ROS 2 node** that wraps this logic, exposing it through standard ROS interfaces like subscribers and publishers so it can interact with other components in the system.

This wrapper node requires its own validation. **ROS unit testing** ensures that the interface layer is correct: that the node subscribes to the right topic, handles the incoming message correctly, executes the core logic, and publishes the expected results.

This layer of testing serves as a bridge between algorithmic validation and system-level integration. It confirms the correctness of the ROS 2 interface before proceeding to integration tests (Module 4), which focus on end-to-end behavior across multiple communicating nodes.
By testing each node in isolation, correctness can be verified without the complexity and non-determinism of a full multi-node system.

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
> Define parameter names and default values as class constants inside your ROS 2 class e.g. `MyClass`. This centralizes parameter definitions, documents them in the header, and prevents typos or mismatches across nodes and tests.
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

<!-- @todo: What about actions?? -->
<!-- @todo: Test lifecycle management. Ping Jesus -->

### Node Pipeline

## Test Isolation

When running unit tests in ROS 2, especially in large workspaces, one might encounter the **cross-talk** issue where nodes receive unintended messages from other tests.

By default, all ROS 2 nodes operate in domain ID 0 (`ROS_DOMAIN_ID=0`), sharing the same DDS communication space. Nodes within a domain automatically discover each other and can exchange messages, even if they belong to different packages or test executors.

Since tests are run in parallel by default, nodes publishing or subscribing to identical topic names can interfere across tests. This results in flaky and nondeterministic behavior (e.g., a subscriber receiving unexpected messages) or immediate failures (e.g., when a service server attempts to instantiate a name already in use).

There are two ways to solve the issue:

- **Sequential Execution**: You can run tests one after another using `colcon test --executor sequential`. By running tests sequentially, even if nodes in different tests are publishing to the same topic they should not affect subsequent tests. However, this dramatically increases the total execution time for large projects.

- **Domain Isolation with ROS_DOMAIN_ID (Recommended)**: This approach leverages the fact that ROS 2 relies on DDS domains, which are isolated by an integer ID. By assigning a unique `ROS_DOMAIN_ID` to each test, you strictly confine communication within that test's boundaries.

The `ament_cmake_ros` package provides a convenient solution for domain isolation. It includes CMake functions that automatically assign an available domain ID to each test executor.

To enable this, add the following dependency to your `package.xml`:

```xml
<test_depend>ament_cmake_ros</test_depend>
```

Then, in your `CMakeLists.txt`, replace the standard test command with the isolated version:

```cmake
find_package(ament_cmake_ros REQUIRED)
ament_add_ros_isolated_gtest(test_my_node test/test_my_node.cpp)
```

## Exercises

## References

- [ROS 2 Test Isolation](https://medium.com/@yifeicheng/test-isolation-and-avoiding-cross-talk-in-ros-2-3731f54e9ccf)
- [Test Fixtures in GoogleTest](https://google.github.io/googletest/primer.html#same-data-multiple-tests)
