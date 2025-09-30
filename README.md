# ROS 2 Testing: A Practical Survival Guide

This workshop aims to introduce the **best practices for designing, testing, and maintaining ROS 2 nodes in C++**, ensuring code quality, ease of maintenance, and confidence in deployments.

The workshop combines theory with practical exercises so that attendees can apply the concepts directly in their own workflow.

## 🎯 Knowledge objectives

By the end of the workshop, participants will be able to:

- Configure static analysis in their ROS 2 projects to enforce quality standards.
- Design ROS 2 nodes so that the main logic is **independently testable** from the ROS interfaces.
- Create and run **unit tests** in C++ using `ament_cmake_gtest` and `ament_cmake_gmock`.
- Implement **ROS 2 interface tests** (publishers, subscribers, parameters, services).
- Perform **integration tests** between multiple nodes.
- Know how to perform **end-to-end tests** with `rosbag` and simulation.
- Integrate all these steps into a **Continuous Integration** workflow with GitHub Actions.

---

## 🖥️ Technical requirements

- Laptop with Linux, Docker, and an IDE (of your choice) installed.
- A configured GitHub account.
- Use of the Linux terminal and basic commands.
- Fundamental concepts of ROS 2 and C++.

👉 It is recommended to first review the following official tutorials on how to work with ROS 2:

- [ROS 2 Basic Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)

---

## 📋 Workshop content

<!-- TODO Revisar cuando tengamos todo preparado, añadir links a los ejercicios practicos -->

> [!IMPORTANT]
> Before the workshop, it is recommended to build and test the `Docker` image that contains everything. For this, there is a prepared [guide](./docker/README.md).

1. **Linters**

   - Introduction to the different linters and tools available, configuration of `ament_lint_auto` to integrate with CI.
   - Practical example with `colcon lint`.

2. **Unit Testing**

   - Principles for decoupling logic and ROS 2 communication.
   - Use of **Dependency Injection** for publishers, subscribers, and services.
   - Configuration of `ament_cmake_gtest` and `ament_cmake_gmock`.
   - Example: testing an algorithm in isolation.

3. **ROS 2 Unit Testing**

   - How to test publishers/subscribers/services/parameters.
   - Using `ament_add_ros_isolated_gtest` to avoid interference between tests.
   - Practical exercise: testing a simple node.

4. **Integration Testing**

   - Validate communication and behavior between multiple nodes.
   - Example: interaction between a producer and a consumer node.

5. **End-to-End Testing**

   - Complete system validation with `rosbag` and simulation environments.
   - Examples of testing pipelines.
   - Theoretical content only.

6. **Continuous Integration**
   - Add static analysis and tests to a GitHub Actions workflow.
   - Example of a minimal workflow.

---

## 📦 Tools and resources

(Rellenar con links y recursos)
