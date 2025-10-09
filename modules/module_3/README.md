# Module 3 – ROS Unit Testing

This module introduces the development of **unit tests for ROS 2 nodes** and interfaces, focusing on how to validate node behavior, parameters, and message exchanges while ensuring test isolation.

- [Module 3 – ROS Unit Testing](#module-3--ros-unit-testing)
  - [Objectives](#objectives)
  - [Motivation](#motivation)
  - [Test Isolation](#test-isolation)
  - [Exercises](#exercises)
  - [References](#references)

## Objectives

By the end of this module, participants will be able to:

- Implement **ROS-aware unit tests** using fixtures to manage node initialization and shutdown.
- Validate **ROS 2 node behavior and interfaces**, including topics, services, and parameters.
- Apply **domain isolation** to prevent node cross-communication during testing.

## Motivation

In the previous module, the core algorithm was successfully isolated and validated with unit tests. Now that the algorithm behaves as expected, the next step is to **build a ROS 2 node** that wraps this logic, exposing it through standard ROS interfaces like subscribers and publishers so it can interact with other components in the system.

This wrapper node requires its own validation. **ROS unit testing** ensures that the interface layer is correct: that the node subscribes to the right topic, handles the incoming message correctly, executes the core logic, and publishes the expected results.

This layer of testing serves as a bridge between algorithmic validation and system-level integration. It confirms the correctness of the ROS 2 interface before proceeding to integration tests (Module 4), which focus on end-to-end behavior across multiple communicating nodes.
By testing each node in isolation, correctness can be verified without the complexity and non-determinism of a full multi-node system.

## Test Isolation

## Exercises

## References

- [ROS 2 Test Isolation](https://medium.com/@yifeicheng/test-isolation-and-avoiding-cross-talk-in-ros-2-3731f54e9ccf)