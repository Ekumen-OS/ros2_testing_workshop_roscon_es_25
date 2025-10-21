# ROS 2 Testing: A Practical Survival Guide

This workshop aims to introduce the **best practices for designing, testing, and maintaining ROS 2 nodes in C++**, ensuring code quality, ease of maintenance, and confidence in deployments.

The workshop combines theory with practical exercises so that attendees can apply the concepts directly in their own workflow.

## 🚀 Motivation

<!-- TODO: Review and refine this section -->

ROS projects are complex. They combine algorithms, drivers, middleware, and hardware interfaces into large, interdependent systems. In such an environment, even a small code change can have unintended effects. Without tests, these effects are only discovered late (often on a robot, at a demo, or by another developer) when the cost of fixing them is highest.

Automated testing addresses this by providing fast, repeatable feedback. A well-designed test suite is not just about catching bugs; it fundamentally changes how you work:

- **Confidence in change**. Tests let you make incremental updates and refactor with reduced fear of breaking something hidden. This gives you this wonderful freedom from change fear!

- **Better design**. Writing testable code naturally leads to cleaner separation between core logic and ROS interfaces. This pays off in long-term maintainability.

- **Bug prevention**. Regression tests ensure that once a bug is fixed, it stays fixed. They also make it easier to prove to reviewers that a patch solves the problem.

- **Living documentation**. Tests encode the expected behavior of your code. They serve as executable documentation that guides both current and future contributors.

- **Collaboration at scale**. In a distributed, open-source ecosystem like ROS, tests enable many developers to work together without stepping on each other’s toes. They reduce guesswork for newcomers and ease the burden on maintainers.

- **Continuous Integration synergy**. Automated tests unlock the full value of CI, catching regressions early and ensuring compatibility across the evolving ROS ecosystem.

The benefits are clear, but they don’t come for free. Automated testing requires deliberate investment in two main areas:

- **Development cost**. Writing a test takes time, and making it automatic is not always trivial. Special care is needed if tests involve hardware or external environments. The general strategy is to simulate, mock, or reduce the scope of the test to keep it reliable and repeatable.

- **Maintenance cost**. Tests evolve with the code. When APIs change or components are redesigned, tests may break not because of a bug but because they need to be updated. In some cases, old regression tests must be retired if they no longer reflect the current design.

Despite these costs, the return on investment is substantial. Tests reduce debugging time, increase software quality, and provide long-term stability in a fast-moving ecosystem. In practice, the time you spend writing and maintaining tests is far less than the time you save by avoiding regressions, broken builds, and late-night bug hunts on hardware.

## 🎯 Learning objectives

By the end of the workshop, participants will be able to:

- Configure static analysis in their ROS 2 projects to enforce quality standards.
- Design ROS 2 nodes so that the main logic is **independently testable** from the ROS interfaces.
- Create and run **unit tests** in C++ using `ament_cmake_gtest` and `ament_cmake_gmock`.
- Implement **ROS 2 interface tests** (publishers, subscribers, parameters, services).
- Perform **integration tests** between multiple nodes.
- Know how to perform **end-to-end tests** with `rosbag` and simulation.
- Integrate all these steps into a **Continuous Integration** workflow with GitHub Actions.

## 🖥️ Technical requirements

- Laptop with Linux, Docker, and an IDE (of your choice) installed.
- At least 5GB of free disk space (for the Docker image).
- A configured GitHub account.
- Use of the Linux terminal and basic commands.
- Fundamental concepts of ROS 2 and C++.

👉 It is recommended to first review the following official tutorials on how to work with ROS 2:

- [ROS 2 Basic Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)

## 📋 Workshop content

<!-- TODO: Link here the different READMEs? Refine the contents -->

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

## 📦 Tools and resources
<!-- TODO: I don't think this is necessary at all, given that we are including the resources in each of the modules -->

(Fill with links and resources)
