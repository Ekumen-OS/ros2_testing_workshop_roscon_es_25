# Module 2 – Unit Testing

In this module, the focus is on **unit tests** in ROS 2, with an emphasis on designing testable code and validating core algorithms using gtest and gmock.

- [Module 2 – Unit Testing](#module-2--unit-testing)
  - [Objectives](#objectives)
  - [Motivation](#motivation)
  - [Testable Design](#testable-design)
    - [Example](#example)
  - [GoogleTest](#googletest)
  - [Ament Integration](#ament-integration)
  - [How to Write Tests](#how-to-write-tests)
  - [Exercises](#exercises)
    - [Exercise 1](#exercise-1)
      - [Definition of success](#definition-of-success)
  - [References](#references)

## Objectives

By the end of this module, participants will be able to:

- Structure ROS 2 nodes so that **core logic** can be tested independently of ROS interfaces.
- Apply **SOLID design principles** to create modular, maintainable, and testable code.
- Write unit tests using GoogleTest (`gtest`) and GoogleMock (`gmock`).
- Integrate tests into the ROS 2 build system using `ament_cmake_gtest` and `ament_cmake_gmock`.
- Run unit tests with `colcon test` and analyze the results.

## Motivation

Unit tests validate the smallest building blocks of a system (functions and classes) in isolation. They provide the fastest and most reliable feedback loop for developers and form the foundation of the testing pyramid.

In ROS 2, this is especially important because nodes often combine algorithmic logic (e.g., obstacle detection, planning) with middleware interfaces (publishers, subscribers, services). Testing such nodes directly requires spinning up ROS infrastructure, which slows feedback and introduces non-determinism.

Unit tests solve this by keeping the focus on algorithms alone: they run **faster**, produce **deterministic** results, and allow **refactoring** with confidence. Higher-level integration and simulation tests can then concentrate on validating communication and system behavior, rather than re-checking algorithm correctness.

> In short: Unit testing in ROS 2 protects your algorithms. It reduces complexity, speeds up development, and prevents subtle bugs from leaking into higher layers where they are harder to detect and debug.

## Testable Design

Testable design means writing code that can be exercised in isolation, with well-defined inputs and outputs and minimal hidden dependencies. Such code does not require the full system to be running in order to be validated.

A well-known guide for achieving this is the set of **SOLID principles** from object-oriented programming:

- **Single Responsibility Principle (SRP)**: a class should only have one reason to change. Keeping algorithms separate from ROS plumbing ensures each part can be tested independently.
- **Open/Closed Principle (OCP)**: entities should be open for extension but closed for modification. Well-defined interfaces allow extensions without rewriting existing tests.
- **Liskov Substitution Principle (LSP)**: subtypes must be substitutable for their base types. This makes it possible to replace real components with mocks or fakes in unit tests.
- **Interface Segregation Principle (ISP)**: favor small, specific interfaces over large, general ones. Narrow interfaces are easier to mock and lead to more focused tests.
- **Dependency Inversion Principle (DIP)**: depend on abstractions, not on concretions. This allows unit tests to supply fake dependencies instead of requiring live ROS publishers or hardware.

In ROS 2, **SRP** and **DIP** are the most relevant. Separating algorithmic logic from middleware allows algorithms to be validated independently, while abstracting dependencies makes it straightforward to substitute real inputs with mocks during tests.

A recommended practice is to implement core algorithms using ROS-agnostic libraries such as `Eigen` for linear algebra or `PCL` for point cloud processing. This reduces coupling to ROS distribution APIs, ensures portability across ROS versions, and makes the algorithm easier to test and maintain over time.

### Example

How could the filtering logic be tested in the example below? The callback mixes computation, state, logging, and publishing: does this follow the Single Responsibility Principle (SRP)?

```cpp
void callback(const std_msgs::msg::Float64 & msg) {
  double value = msg.data;

  double filtered = 0.9 * prev_ + 0.1 * value;
  prev_ = filtered;

  RCLCPP_INFO(this->get_logger(), "Filtered = %f", filtered);
  publisher_->publish(std_msgs::msg::Float64{filtered});
}
```

## GoogleTest

Once code is structured for testability, a framework is required to define and run the tests. The industry standard is [GoogleTest](https://github.com/google/googletest), commonly referred to as `gtest`.

GoogleTest provides a simple and expressive way to define tests using macros such as `TEST`, `EXPECT_EQ`, or `ASSERT_TRUE`. It produces structured output that integrates cleanly with continuous integration systems. Beyond readability, the biggest reason to use gtest in ROS 2 is its seamless integration, which make it trivial to add tests that are executed automatically.

A few core concepts are especially useful:

- **EXPECT vs. ASSERT**:
  - `EXPECT_*` records a failure but allows the test to continue.
  - `ASSERT_*` aborts the test immediately on failure.

    Consider testing that a function returns a `std::vector<int>` with the right length and expected contents:

    ```cpp
    std::vector<int> vec = get_vector();
    // If the length is wrong, further checks (indexing) would be invalid -> abort test
    ASSERT_EQ(3u, vec.size());   // stop the test immediately if size != 3
    // Now it is safe to check contents; these can be EXPECT so we see all mismatches at once
    EXPECT_EQ(10, vec[0]);
    EXPECT_EQ(20, vec[1]);
    EXPECT_EQ(30, vec[2]);
    ```

    Use `ASSERT_*` for preconditions that must hold for remaining assertions to make sense (avoid crashes and meaningless failures). Use `EXPECT_*` for value checks where continuing to run the test to collect multiple failures is useful

- **Fixtures**: allows code to be reused across multiple tests. Define a test class deriving from `::testing::Test` and use `TEST_F` instead of `TEST`.
- **Parameterized tests**: The same test logic can be executed against multiple input values with `TEST_P`. This reduces duplication and is especially helpful when validating algorithms across many corner cases

See [Macros](https://google.github.io/googletest/reference/testing.html) and [Assertions](https://google.github.io/googletest/reference/assertions.html) in the official documentation for a more in-depth explanation.

In addition to `gtest`, the same framework also provides [GoogleMock](https://google.github.io/googletest/gmock_for_dummies.html), or `gmock`, which is a library for creating mock objects. Mocks are fake implementations of classes that behave in controlled ways, defined by the test. This is particularly useful in robotics, where real data often comes from sensors or hardware that is not always available during testing. By mocking a sensor interface, it is possible to test how an algorithm reacts to predefined inputs without requiring hardware.

Minimal example:

```cpp
#include <gtest/gtest.h>

TEST(TestBasicMath, Addition)
{
  ASSERT_EQ(4, 2 + 2);
}
```

> [!NOTE]
> GoogleTest provides a default `main()`. A custom main is only necessary if special initialization is required before running tests.

## Ament Integration

ROS 2 wraps GoogleTest/GoogleMock with lightweight CMake helpers so tests build and run with `colcon test`. The integration has three pieces: `package.xml`, `CMakeLists.txt`, and how tests are executed.

- Add the following lines to `package.xml`:

    ```xml
    <test_depend>ament_cmake_gtest</test_depend>
    <!-- add ament_cmake_gmock only if using gmock -->
    <test_depend>ament_cmake_gmock</test_depend> 
    ```

- Add to `CMakeLists.txt`:

    ```CMake
    if(BUILD_TESTING)
      find_package(ament_cmake_gtest REQUIRED)
      # Define test executable
      ament_add_gtest(test_module_2_algorithm test/test_algorithm.cpp)
      # Link against internal libraries
      target_link_libraries(test_module_2_algorithm algorithm)
      # Link against external libraries
      ament_target_dependencies(test_module_2_algorithm Eigen3)

      # Same for mocks
    endif()
    ```

    The `if(BUILD_TESTING)` block keeps tests out of non-testing builds.

- Run the tests with `colcon`:

    ```bash
    colcon build --packages-select module_2
    colcon test  --packages-select module_2
    colcon test-result --verbose
    ```

## How to Write Tests

A good unit test is clear, concise, and focused. The best way to achieve this is by following the Arrange-Act-Assert (AAA) pattern, which provides a simple mental model for structuring each test:

- **Arrange**: prepare the environment, inputs, and objects needed for the test.
- **Act**: execute the function or behavior being tested.
- **Assert**: verify that the observed result matches the expected outcome.

Following this pattern leads to tests that are consistent, self-explanatory, and easy to debug when they fail.

Beyond how tests are written, it’s also important to consider when they are written. This leads to a popular development workflow known as **Test-Driven Development (TDD)**. TDD follows an iterative approach where tests are written before the actual code. Each cycle begins by defining a small, failing test that expresses a desired behavior. The minimal code needed to make the test pass is then implemented, followed by a short refactoring step to clean up or generalize the design. This rhythm of red → green → refactor encourages clear requirements, modular code, and continuous verification.

While TDD helps drive better design decisions and encourages modular, testable architectures, the same testing principles can be applied in traditional “test-after” workflows. The key takeaway is that **testability should guide design**, regardless of whether tests come before or after the code.

## Exercises

The exercises for this module focus on transforming non-testable code into testable code and validating the core logic using the GoogleTest framework.

### Exercise 1

The objective of this exercise is to identify and refactor code that violates the Single Responsibility Principle (SRP) and is therefore hard to test.

Begin by examining [bad_laser_detector.cpp](src/bad_laser_detector.cpp) node that processes LiDAR scans, combining publishers, subscribers, and algorithmic logic in a single callback. This coupling between ROS interfaces and computation makes the algorithm untestable in isolation, as every test would require launching ROS infrastructure.

Next, review the `LaserDetector` class defined in [laser_detector.cpp](src/laser_detector.cpp), which isolates the core obstacle detection algorithm from the ROS 2 node. The task is to complete the missing parts of this class so that the algorithm becomes fully testable and all provided unit tests pass.

The following components require completion:

- Constructor: Complete the missing input validation checks to make tests pass.
- `roi_filter`: Implement the logic to iterate through the input ranges, check the angle against the ROI and return a new vector with the filtered data.
- `points_inside_footprint`: Implement the logic to count how many ranges are finite and less than or equal to `footprint_radius_`.
- `detect_obstacle`: Implement the final comparison logic. A detection is true if `num_points≥min_points_`.

> [!NOTE]
> This exercise demonstrates Test-Driven Development (TDD) in practice: using predefined unit tests to guide the implementation of a clear, modular, and testable design.

#### Definition of success

The task is complete when tests are run and the output shows **0 errors** and **0 failures** for the `TestLaserDetector` suite defined in [test_laser_detector.cpp](test/test_laser_detector.cpp).

## References

- [SOLID principles](https://en.wikipedia.org/wiki/SOLID)
- [Writing Basic Tests with C++ with GTest](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Cpp.html)
- [Google Test Repo](https://github.com/google/googletestl)
- [Google Test Macros](https://google.github.io/googletest/reference/testing.html)
- [Google Test Assertions](https://google.github.io/googletest/reference/assertions.html)
- [Google Mock Basics](https://google.github.io/googletest/gmock_for_dummies.html)
