---
name: ros2-testing
description: Testable design and testing standards for ROS 2 C++ (rclcpp) packages (gtest/gmock unit tests, node tests, launch_testing integration tests, ament linters). Use this skill whenever writing, refactoring or reviewing rclcpp nodes or their tests, adding tests, linters or test registration (CMakeLists.txt, package.xml) to a ROS 2 package, or debugging failing or flaky ROS 2 tests, even if the user doesn't explicitly ask for tests.
---

# ROS 2 Testing

This document defines professional engineering and testing standards for ROS 2 software development, as taught in the ROS 2 Testing Workshop. AI assistants generating ROS 2 code or tests must adhere to these rules to ensure safety, maintainability, and reliability in production-ready systems.

## Workflow

1. **Read the package first**: its existing tests and test registration, and how the project builds and tests (e.g., a Docker environment or helper scripts, which keep results consistent with CI).

2. **Check testability**: if logic lives in callbacks or hardware can't be injected, propose the refactor before writing tests (see [Testable Design](#testable-design)). Otherwise every test of that logic needs a running ROS graph, or real hardware.

3. **Choose the lowest test level that can verify the behavior**, since lower levels are faster and more reliable:

   | What to verify | Test type |
   |---|---|
   | Algorithm/logic, edge cases, invalid input | Unit (gtest, gmock for hardware) |
   | Node parameters, topics, services, callbacks | ROS unit (isolated gtest fixture + executor) |
   | Several nodes working together | Integration (isolated `launch_testing`) |

4. **Write and register the tests** following the rules below.

5. **Run them and report the actual results** (see [Verification](#verification)), using the project's own build setup if it has one. When a test fails, fix the root cause, and never weaken the test to get a green run.

For bug fixes, write the regression test first and check that it fails, then fix the code and check that it passes. A test that never failed doesn't prove the fix.

## Testable Design

To enable professional-grade testing, algorithmic logic must be decoupled from the ROS 2 middleware.

Follow these relevant SOLID principles:

- **Single Responsibility (SRP)**: Build ROS 2 nodes to have a single responsibility. Separate application logic into its own class or library, and keep ROS 2 nodes as thin wrappers responsible only for communication between ROS interfaces and the core application logic. Build the node class into a library separate from `main()`, so that tests can link against it.

- **Dependency Inversion (DIP)**: Depend on abstractions, not concrete implementations, and inject dependencies and configurations into the logic class constructor rather than creating them internally. This allows hardware drivers and other external systems to be replaced with mocks in unit tests.

- **Interface Segregation (ISP)**: Favor small, specific interfaces over large, general ones. Narrow interfaces are easier to mock and lead to more focused tests.

## Testing Strategy

Maintain a balanced testing pyramid to ensure high-quality software:

- **Static Analysis (Foundation)**: Use linters and formatters to catch style, naming, and memory issues before runtime. Enable them with `ament_lint_auto` so they run as part of `colcon test`, and use a single formatter: `ament_cmake_clang_format` and the `ament_cmake_uncrustify` linter included in `ament_lint_common` conflict with their default configurations, so list the linters explicitly when using `clang-format`.

- **Unit Tests (Majority)**: Target the ROS-independent logic class using GoogleTest (`gtest`) and the **Arrange-Act-Assert (AAA)** pattern. Cover every branch of the core algorithms: nominal cases, boundaries, invalid data (e.g., `NaN`, `inf`, or out-of-range sensor readings), and invalid configuration (e.g., constructors that must throw). These must be ROS-agnostic, fast, and deterministic.

- **Mocking**: Hide hardware drivers behind abstract interfaces injected through the constructor, and replace them with GoogleMock (`gmock`) mocks in unit tests. Mock only hardware and external systems; test pure logic with its real implementation. Use plain mocks by default, and `StrictMock` only when any unexpected call is itself a bug (e.g., commands sent to hardware). Set expectations only on the interactions the test is about, so that tests don't break on unrelated changes.

- **ROS Unit/Component Tests**: Validate node interfaces (topics, services, parameters) in isolation using test fixtures that manage the `rclcpp` lifecycle (`rclcpp::init`/`rclcpp::shutdown`), an auxiliary test node, and an explicitly driven executor.

- **Integration Tests**: Verify multi-node interactions and communication behavior using the `launch_testing` framework. Signal readiness with `ReadyToTest`, wait for events (e.g., `proc_output.assertWaitFor`) instead of fixed delays, and check process exit codes in post-shutdown tests.

- **End-to-End (E2E)**: Validate complete system behavior in realistic environments, such as simulation or on target hardware.

Every bug fix must include a regression test that fails without the fix.

## Test Registration

- Register every test in `CMakeLists.txt` inside `if(BUILD_TESTING)`, otherwise `colcon test` never runs it. Use `ament_add_gtest` for unit tests, or `ament_add_gmock` when they use mocks.

- Declare as `<test_depend>` in `package.xml` every package the tests need: the ones found with `find_package` inside `if(BUILD_TESTING)`, and the ones imported by Python launch tests (e.g., `rclpy`, `launch`, `launch_ros`, `launch_testing`, and message packages). A missing test dependency can work locally but fail in a clean CI environment.

## Determinism and Reliability

- **Avoid Arbitrary Sleeps**: Never use arbitrary sleeps in tests, as they make them non-deterministic and flaky.

- **Synchronization Mechanisms**: Instead of sleeping, wait for the expected result with a proper timeout:
  - Drive the executor explicitly, spinning it in a loop until the expected condition holds or a timeout expires.
  - Use `spin_until_future_complete(future, timeout)` for services and actions.
  - Wait for publishers/subscriptions to be matched before publishing.
  - Use simulated time (`use_sim_time` and `/clock`) for timer-driven logic.

- **Test Isolation**: Isolate every test that creates ROS nodes to prevent cross-talk between parallel tests (e.g., by assigning unique domain IDs). Use `ament_add_ros_isolated_gtest` (or `ament_add_ros_isolated_gmock`) from `ament_cmake_ros` for gtests, and its `run_test_isolated.py` runner for launch tests. Pure logic unit tests can use `ament_add_gtest`. Launch tests are registered with the runner like this:

  ```cmake
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  add_ros_isolated_launch_test(test/test_my_system_launch.py)
  ```

- **Never Weaken Tests**: When a test fails, fix the code or the test's wrong assumption. Do not delete, skip, or loosen assertions, nor add sleeps or retries, just to make it pass. Treat intermittent failures as determinism problems (a missing wait, shared state, or missing isolation).

## Working on Existing Code

- Follow the package's existing test conventions, unless they conflict with these rules.

- When refactoring for testability, keep the node's external interface (topics, services, parameters) unchanged.

- When changing code in a package that already has tests, add or update tests for the changed behavior. Otherwise, and for larger changes such as restructuring code for testability or enabling linters on existing code, propose them instead of applying them unasked.

## Verification

- **Verify Before Done**: Build and run the tests before considering a change complete: `colcon build`, `colcon test`, and `colcon test-result --verbose`. `colcon test` succeeds even when tests fail, so always check the results with `colcon test-result`.

- **Pre-commit Hooks**: If the project uses pre-commit hooks, run them before committing.

- **Continuous Integration**: CI (e.g., **GitHub Actions**) should run the same three commands, and branch protection rules should require it to pass before merging.

## Reviewing ROS 2 code or tests

Review against the rules above. The most common issues are:

- Logic inside callbacks or tightly coupled to `rclcpp`.
- `sleep`-based waits, missing timeouts, or tests depending on execution order.
- Node or launch tests without isolation.
- Tests not registered in CMake, or missing `<test_depend>` entries.
- Assertions that can't fail (e.g., only checking that no exception is thrown when an output can be checked).
- Over-mocking: mocks of the project's own logic, or expectations on every call.
