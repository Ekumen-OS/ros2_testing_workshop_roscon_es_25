# AI ROS 2 Testing Guide

This document defines professional engineering and testing standards for ROS 2 software development. AI assistants generating ROS 2 code or tests must strictly adhere to these rules to ensure safety, maintainability, and reliability in production-ready systems.

## Testable Design

To enable professional-grade testing, algorithmic logic must be decoupled from the ROS 2 middleware.

Follow these relevant SOLID principles:

- **Single Responsibility (SRP)**: Build ROS 2 nodes to have a single responsibility. Separate application logic into its own class or library, and keep ROS 2 nodes as thin wrappers responsible only for communication between ROS interfaces and the core application logic.

- **Dependency Injection (DI)**: Inject dependencies and configurations into the logic class constructor rather than creating them internally. This enables the use of mocks to isolate functionality during testing.

- **Interface Segregation (ISP)**: Depend on abstractions, not concrete implementations, to allow for seamless substitution of real components with fakes or mocks in unit tests.

## Testing Strategy

Maintain a balanced testing pyramid to ensure high-quality software:

- **Static Analysis (Foundation)**: Use linters and formatters to catch style, naming, and memory issues before runtime.

- **Unit Tests (Majority)**: Target the ROS-independent logic class using GoogleTest (`gtest`) and the **Arrange-Act-Assert (AAA)** pattern. Aim for 90-100% coverage on core algorithms. These must be ROS-agnostic, fast, and deterministic.

- **ROS Unit/Component Tests**: Validate node interfaces (topics, services, parameters) in isolation using test fixtures to manage the `rclcpp` lifecycle.

- **Integration Tests**: Verify multi-node interactions and communication behavior using the `launch_testing` framework.

- **End-to-End (E2E)**: Validate complete system behavior in realistic environments, such as simulation or on target hardware.

## Determinism and Reliability

- **Avoid Arbitrary Sleeps**: Never use arbitrary sleeps in tests, as they make them non-deterministic and flaky.

- **Synchronization Mechanisms**: Instead of sleeping, use synchronization mechanisms or wait for the expected result with a proper timeout.

- **Test Isolation**: Always use `ament_add_ros_isolated_gtest` to prevent cross-talk between parallel tests on the same network by assigning unique domain IDs.

## Local Development

- **Pre-commit Hooks**: Use pre-commit hooks to automatically run formatting, linters, and other fast checks before committing code. This helps catch issues early and reduces CI failures.

## Continuous Integration

The CI pipeline (e.g., **GitHub Action**s) serves as an enforceable quality gate.

1. **Build**: Run colcon build to ensure the package and its dependencies compile correctly.

2. **Test and Lint**: Execute colcon test. This triggers both the Static Analysis and the functional tests (Unit, ROS Unit, Integration).

3. **Verification**: Use `colcon test-result --verbose` to interpret results.

4. **Enforcement**: Configure branch protection rules to require these status checks pass before code can be merged.
