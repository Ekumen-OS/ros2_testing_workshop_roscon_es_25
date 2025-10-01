# Module 2: Unit Testing

In this module, we will work with **unit tests** in ROS 2, focusing on designing testable nodes and validating core algorithms with gtest and gmock.  

## Objectives

By the end of this module, you will be able to:

- Structure ROS 2 nodes so that **core logic** can be tested independently of ROS interfaces.
- Apply **SOLID design principles** to create modular, maintainable, and testable code.
- Write unit tests using GoogleTest (`gtest`) and GoogleMock (`gmock`).
- Integrate tests into the ROS 2 build system using `ament_cmake_gtest` and `ament_cmake_gmock`.
- Run unit tests with `colcon tes`t and analyze the results.

## Motivation

Unit tests validate the smallest building blocks of a system (functions and classes) in complete isolation. They are the fastest and most reliable feedback loop for developers, and they form the foundation of the testing pyramid.

In ROS 2, unit testing is especially important because nodes often mix algorithmic logic (e.g., obstacle detection, planning) with ROS middleware (publishers, subscribers, services). This tight coupling makes it difficult to test algorithms without spinning up ROS infrastructure.

By applying unit testing:

- Algorithms stay correct regardless of ROS glue code
We can verify obstacle detection, control laws, or planners independently of message passing.
- Tests remain fast and deterministic
Unit tests run in milliseconds and avoid the non-determinism of ROS communication or timing.
- Refactoring becomes safe
Developers can confidently change ROS interfaces or restructure algorithms knowing the core logic is still validated.
- Design quality improves
Writing unit tests forces separation of concerns: ROS interfaces in one layer, algorithms in another. This follows SOLID principles like SRP and DIP.
- Higher-level tests become simpler
Since algorithm correctness is already validated at the unit level, integration and simulation tests can focus solely on ROS plumbing and system behavior, reducing brittleness.

In short:

> Unit testing in ROS 2 protects your algorithms. It reduces complexity, speeds up development, and prevents subtle bugs from leaking into higher layers where they are harder to detect and debug.

## Testable Design

<!-- Explain what SOLID principles are and reference -->

When we talk about testable design, we mean writing code that can be easily and reliably exercised in isolation. A testable component has clear inputs and outputs, few hidden dependencies, and does not require a full system to be running in order to be validated.

The **SOLID principles** from object-oriented programming are a useful guide for achieving this. They are not specific to ROS, but they help us avoid tangled designs where business logic, framework code, and external dependencies are mixed together in ways that make testing slow or brittle. By following SOLID, we make our code easier to test, but also easier to extend, reuse, and maintain.

The five SOLID principles are:

- Single Responsibility Principle (SRP): a class should only have one reason to change. Keeping algorithms separate from ROS plumbing ensures each part can be tested independently.
- Open/Closed Principle (OCP): entities should be open for extension but closed for modification. With well-defined interfaces, we can extend functionality without rewriting existing tests.
- Liskov Substitution Principle (LSP): subtypes must be substitutable for their base types. This makes it possible to replace real components with mocks or fakes in unit tests.
- Interface Segregation Principle (ISP): favor small, specific interfaces over large, general ones. Narrow interfaces are easier to mock and lead to more focused tests.
- Dependency Inversion Principle (DIP): depend on abstractions, not on concretions. This allows unit tests to supply fake dependencies instead of requiring live ROS publishers or hardware.

In the context of ROS 2, **SRP** and **DIP** are especially important: separating algorithmic logic from communication logic makes algorithms independently testable, and depending on abstractions allows us to swap real ROS inputs for mocks during unit tests. Together, these principles help us build software that is not only more testable, but also more robust and adaptable in the long term.

### Example

## GoogleTest

With our code structured to be testable, we need the right tools to actually write and run the tests. For C++ projects, the industry standard is [GoogleTest](https://google.github.io/googletest/primer.html), commonly referred to as `gtest`.

GoogleTest provides a simple and expressive way to define tests using macros like `TEST`, `EXPECT_EQ`, or `ASSERT_TRUE`. It produces structured output that integrates cleanly with continuous integration systems. Beyond readability, the biggest reason to use gtest in ROS 2 is its seamless integration: the build system provides wrappers such as `ament_cmake_gtest`, which make it trivial to add tests that are executed automatically when you run `colcon test`.

In addition to `gtest`, the same framework also provides [GoogleMock](https://google.github.io/googletest/gmock_for_dummies.html), or `gmock`, which is a library for creating mock objects. A mock is a fake implementation of a class that behaves the way you tell it to. This is particularly useful in robotics, where real data often comes from sensors or hardware that is not always available during testing. By mocking a sensor interface, you can test how your algorithm reacts to predefined inputs without requiring a physical device.

In summary:
> `gtest` is used to validate algorithm correctness, and `gmock` is used to simulate external dependencies, both of which are essential for reliable unit testing in ROS 2.

## Exercises

## References

- https://wiki.ros.org/Quality/Tutorials/UnitTesting
- https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Cpp.html