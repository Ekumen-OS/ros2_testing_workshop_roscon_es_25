# ROS 2 Testing: A Practical Survival Guide

This workshop introduces the **best practices for designing, testing, and maintaining ROS 2 nodes in C++** to ensure code quality, maintainability, and confidence in deployments.

It combines theory with hands-on exercises so that participants can directly apply the concepts in their own workflow.

## 🚀 Motivation

ROS projects are complex. They combine algorithms, drivers, middleware, and hardware interfaces into large, interdependent systems. In such an environment, even a small code change can have unintended effects. Without tests, these effects are only discovered late (often on a robot, at a demo, or by another developer) when the cost of fixing them is highest.

Automated testing addresses this by providing fast, repeatable feedback. A well-designed test suite does much more than just catching bugs:

- **Confidence in change**. Safely refactor and evolve your code without fear of breaking hidden functionality.

- **Better design**. Writing testable code naturally leads to cleaner separation between logic and ROS interfaces, improving long-term maintainability.

- **Bug prevention**. Regression tests ensure that once a bug is fixed, it stays fixed.

- **Living documentation**. Tests describe expected behavior and serve as executable documentation for future contributors.

- **Collaboration at scale**. In a distributed, open-source ecosystem like ROS, tests enable many developers to work together without stepping on each other’s toes.

- **Continuous Integration synergy**. Automated tests unlock the full value of CI, catching regressions early and ensuring compatibility across the evolving ROS ecosystem.

The benefits are clear, but they don’t come for free. Automated testing requires deliberate investment in two main areas:

- **Development cost**. Writing a test takes time, and making it automatic is not always trivial. Special care is needed if tests involve hardware or external environments.

- **Maintenance cost**. Tests evolve with the code. When APIs change, the tests must adapt. Outdated tests can become misleading if not maintained.

Despite these costs, the **return on investment is substantial**. Projects that embrace testing enjoy shorter debugging times, more reliable deployments, and greater confidence in every release. For robotics, where failures can have physical consequences, this confidence is not optional; it’s essential.

## 🎯 Learning objectives

By the end of the workshop, participants will be able to:

- Understand the role of testing in the ROS 2 software lifecycle.
- Differentiate between unit, integration, and end-to-end tests.
- Apply good design practices that make ROS 2 nodes easier to test and maintain.
- Use automated tools to enforce code quality and prevent regressions.
- Integrate all these practices into a continuous integration pipeline.

## 🖥️ Technical requirements

- Laptop with Linux, Docker, and an IDE (of your choice) installed.
- At least **1.5GB** of free disk space (for the Docker image).
- A configured GitHub account.
- Basic command-line skills and familiarity with ROS 2 and C++.

👉 It is recommended to review the official [ROS 2 Basic Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) beforehand.

## 📋 Workshop structure

This workshop is organized into six modules that progressively develop the participant’s understanding of **testing in ROS 2**, from code quality fundamentals to complete Continuous Integration pipelines.

Each module combines conceptual material with practical exercises that apply the ideas directly to real ROS 2 code. All exercises are designed to be executed in a consistent environment using the provided Docker setup.

> [!IMPORTANT]
> Before starting, build the Docker environment provided for this workshop. It includes all dependencies and tools required for the exercises. Follow the detailed instructions in the [Docker README](./docker/README.md).

1. **[Module 1 – Code Quality and Static Analysis](modules/module_1/README.md)**

   Understand how automated formatters, linters and static analysis tools enforce consistency, readability, and safety across ROS 2 codebases.  

2. **[Module 2 – Unit Testing](modules/module_2/README.md)**

   Learn how to design testable code by separating algorithmic logic from ROS interfaces, applying SOLID principles, and using Google Test and Mock effectively.  

3. **[Module 3 – ROS Unit Testing](modules/module_3/README.md)**

   Explore how to validate ROS 2 nodes through interface-level tests that verify topics, parameters, and services while ensuring deterministic isolation.  

4. **[Module 4 – Integration Testing](modules/module_4/README.md)**

   Discover how to verify multi-node interactions using the `launch_testing` framework and ensure end-to-end communication correctness.

5. **[Module 5 – End-to-End Testing](modules/module_5/README.md)**

   Examine system-level testing strategies with rosbag playback and simulation environments to validate full robot behavior.  

6. **[Module 6 – Continuous Integration](modules/module_6/README.md)**

   Learn how to automate builds, linters, and tests with GitHub Actions to maintain code quality and enforce review standards.

---

For detailed explanations and references, see the individual module READMEs in the [modules](./modules) directory.
