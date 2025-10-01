# Module 2: Unit Testing

In this module, we will work with **unit tests** in ROS 2, focusing on designing testable nodes and validating core algorithms with gtest and gmock.  

## Objectives

By the end of this module, you will be able to:

- Structure ROS 2 nodes so that **core logic** can be tested independently of ROS interfaces.
- Apply **SOLID design principles** to create modular, maintainable, and testable code.
- Write unit tests using [GoogleTest](https://github.com/google/googletest) (`gtest`) and GoogleMock (`gmock`).
- Integrate tests into the ROS 2 build system using `ament_cmake_gtest` and `ament_cmake_gmock`.
- Run unit tests with `colcon tes`t and analyze the results.

## Motivation

Unit tests validate the smallest building blocks of a system — functions and classes — in complete isolation. They are the fastest and most reliable feedback loop for developers, and they form the foundation of the testing pyramid.

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

Unit testing in ROS 2 protects your algorithms. It reduces complexity, speeds up development, and prevents subtle bugs from leaking into higher layers where they are harder to detect and debug.

## Testable Design

Decoupling Logic from ROS 2 Interfaces is a fundamental principle for building reliable ROS 2 systems is to decouple the core algorithm/logic from the ROS 2 communication interfaces. This design pattern allows you to test the most critical part of your software (the algorithms) without the overhead and complexity of the ROS 2 environment (nodes, topics, services, etc.).

### Example

## Using googletest in ROS 2

## Exercises

## References

- https://wiki.ros.org/Quality/Tutorials/UnitTesting
- https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.htmllation, or hardware.
