# ROS 2 Testing AI Guidelines

This document defines testing practices for ROS 2 software used in the workshop “ROS 2 Testing: A Practical Survival Guide.”

AI assistants generating ROS 2 code or tests should follow these rules.

## Core Principles

- Design ROS 2 nodes to be testable.

- Separate application logic from ROS communication.

- Core algorithms should live in plain C++ classes, independent of ROS.

ROS nodes should primarily act as adapters between ROS communication and application logic.

## Design Principles

ROS 2 nodes should be designed so that core logic can be tested
independently from ROS. Follow SOLID principles to achieve this:

- Single Responsibility — classes should do one thing.

- Dependency Inversion — depend on abstractions, not concrete implementations.

- Dependency Injection — inject dependencies instead of creating them internally.

This enables mocking and isolated unit testing.

## Testing Strategy

Use a **testing pyramid**:

1. Unit tests (majority)

2. ROS interface tests

3. Integration tests

4. End-to-end tests (few)

Unit tests should focus on core logic, not ROS infrastructure.

## Unit Testing

- Use gtest (`ament_cmake_gtest`)

- Test algorithms without ROS dependencies

- Tests must be:

    - deterministic

    - fast

    - independent

Structure tests using Arrange–Act–Assert (AAA).

## ROS Interface Testing

ROS nodes should be tested by validating their communication behavior.

Typical tests include:

- publishers emit correct messages

- subscribers process incoming messages

- services return correct responses

- parameters affect behavior correctly

Guidelines:

- Use `ament_add_ros_isolated_gtest` when possible.

- Ensure tests are isolated from each other.

## Integration Testing

Integration tests validate interaction between multiple ROS nodes.

Typical scenarios:

- producer node publishes data

- consumer node processes the data

- system behavior is verified

Tools commonly used:

- `launch_testing`

- ROS launch files

Integration tests should verify system behavior, not internal implementation.

## Deterministic Tests

Tests must be **deterministic** and **reliable**.

Avoid:

- arbitrary sleeps

- timing assumptions

- race conditions

Prefer:

- waiting for expected messages

- synchronization mechanisms

- proper timeouts

## Test Isolation

Tests should not interfere with each other.

Guidelines:

- Avoid shared global state.

- Ensure ROS nodes shut down properly after tests.

- Use separate ROS domains if necessary.

## Static Analysis

Testing should be complemented with static analysis tools.

Common tools:

- `ament_lint_auto`

- `clang-tidy`

- `cppcheck`

Static analysis helps detect:

- style violations

- memory issues

- potential bugs

before runtime.

## Continuous Integration

All tests should run automatically in CI pipelines.

Typical CI workflow:

1. Build the workspace with `colcon build`
2. Run the test suite:

    ```bash
    colcon test
    colcon test-result --verbose
    ```

In ROS 2 projects using `ament_lint_auto`, static analysis and linters are typically executed as part of the test suite.

The pipeline should fail if any test fails.

## Test Coverage

Aim for high coverage of core application logic.

Guidelines:

- Focus coverage on algorithms and business logic.

- ROS glue code does not require exhaustive coverage.

Typical targets: 90-100% coverage for core logic

Coverage metrics should complement **meaningful tests**, not replace them.
