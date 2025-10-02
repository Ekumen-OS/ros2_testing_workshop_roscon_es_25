# Module 2: Unit Testing

In this module, the focus is on **unit tests** in ROS 2, with an emphasis on designing testable nodes and validating core algorithms using gtest and gmock.

- [Module 2: Unit Testing](#module-2-unit-testing)
  - [Objectives](#objectives)
  - [Motivation](#motivation)
  - [Testable Design](#testable-design)
    - [Example](#example)
  - [GoogleTest](#googletest)
  - [Exercises](#exercises)
  - [References](#references)

## Objectives

By the end of this module, participants will be able to:

- Structure ROS 2 nodes so that **core logic** can be tested independently of ROS interfaces.
- Apply **SOLID design principles** to create modular, maintainable, and testable code.
- Write unit tests using GoogleTest (`gtest`) and GoogleMock (`gmock`).
- Integrate tests into the ROS 2 build system using `ament_cmake_gtest` and `ament_cmake_gmock`.
- Run unit tests with `colcon test` and analyze the results.

## Motivation

Unit tests validate the smallest building blocks of a system (functions and classes) in complete isolation. They are the fastest and most reliable feedback loop for developers, and they form the foundation of the testing pyramid.

In ROS 2, unit testing is especially important because nodes often mix algorithmic logic (e.g., obstacle detection, planning) with ROS middleware (publishers, subscribers, services). This tight coupling makes it difficult to test algorithms without spinning up ROS infrastructure.

By applying unit testing:

- **Algorithms remain correct regardless of ROS glue code**: core logic can be validated independently of message passing.
- **Tests are fast and deterministic**: unit tests execute in milliseconds and avoid the non-determinism of ROS communication or timing.
- **Refactoring is safer**: developers can confidently change ROS interfaces or restructure algorithms knowing the core logic is still validated.
- **Design quality improves**: unit testing enforces separation of concerns, with ROS interfaces in one layer and algorithms in another, in line with principles such as SRP and DIP.
- **Higher-level tests are simplified**: integration and simulation tests can focus on validating communication and system behavior, not algorithm correctness.

In short:

> Unit testing in ROS 2 protects your algorithms. It reduces complexity, speeds up development, and prevents subtle bugs from leaking into higher layers where they are harder to detect and debug.

## Testable Design

Testable design means writing code that can be exercised in isolation, with well-defined inputs and outputs and minimal hidden dependencies. Such code does not require the full system to be running in order to be validated.

A useful guide for achieving this is the set of **SOLID principles** from object-oriented programming, which encourage modularity, abstraction, and separation of concerns:

- **Single Responsibility Principle (SRP)**: a class should only have one reason to change. Keeping algorithms separate from ROS plumbing ensures each part can be tested independently.
- **Open/Closed Principle (OCP)**: entities should be open for extension but closed for modification. Well-defined interfaces allow extensions without rewriting existing tests.
- **Liskov Substitution Principle (LSP)**: subtypes must be substitutable for their base types. This makes it possible to replace real components with mocks or fakes in unit tests.
- **Interface Segregation Principle (ISP)**: favor small, specific interfaces over large, general ones. Narrow interfaces are easier to mock and lead to more focused tests.
- **Dependency Inversion Principle (DIP)**: depend on abstractions, not on concretions. This allows unit tests to supply fake dependencies instead of requiring live ROS publishers or hardware.

In ROS 2, **SRP** and **DIP** are the most critical. Separating algorithmic logic from middleware allows algorithms to be validated independently, while abstracting dependencies makes it straightforward to substitute real inputs with mocks during tests.

A recommended practice is to implement core algorithms using ROS-agnostic libraries such as `Eigen` for linear algebra or `PCL` for point cloud processing. This reduces coupling to ROS distribution APIs, increases portability, and ensures that the algorithm can be maintained and tested independently of the ROS version in use.

### Example

## GoogleTest

With code structured for testability, a framework is needed to define and run the tests. For C++ projects, the industry standard is [GoogleTest](https://google.github.io/googletest/primer.html), commonly referred to as `gtest`.

GoogleTest provides a simple and expressive way to define tests using macros like `TEST`, `EXPECT_EQ`, or `ASSERT_TRUE`. It produces structured output that integrates cleanly with continuous integration systems. Beyond readability, the biggest reason to use gtest in ROS 2 is its seamless integration: the build system provides wrappers such as `ament_cmake_gtest`, which make it trivial to add tests that are executed automatically when you run `colcon test`.

In addition to `gtest`, the same framework also provides [GoogleMock](https://google.github.io/googletest/gmock_for_dummies.html), or `gmock`, which is a library for creating mock objects. Mocks are fake implementations of classes that behave in controlled ways, defined by the test. This is particularly useful in robotics, where real data often comes from sensors or hardware that is not always available during testing. By mocking a sensor interface, it is possible to test how an algorithm reacts to predefined inputs without requiring hardware.

In summary:

> `gtest` is used to validate algorithm correctness, and `gmock` is used to simulate external dependencies, both of which are essential for reliable unit testing in ROS 2.

## Exercises

## References

- [SOLID principles](https://en.wikipedia.org/wiki/SOLID)
- https://wiki.ros.org/Quality/Tutorials/UnitTesting
- https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Cpp.html