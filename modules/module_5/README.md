# Module 5 – End-to-End Testing

In this module, the goal is to explore End-to-End (E2E) testing. This is the highest level of testing, designed to verify that the entire robotic system can successfully perform a complete "mission" or task from start to finish, just as it would in the real world.

- [Module 5 – End-to-End Testing](#module-5--end-to-end-testing)
  - [Objectives](#objectives)
  - [Motivation](#motivation)
  - [The rosbag Toolset](#the-rosbag-toolset)
    - [Recording Data](#recording-data)
    - [Inspecting Data](#inspecting-data)
    - [Playing Back Data](#playing-back-data)
  - [Manual End-to-End Testing](#manual-end-to-end-testing)
    - [Using Pre-recorded Rosbags](#using-pre-recorded-rosbags)
    - [Using Simulation Data](#using-simulation-data)
  - [Automated End-to-End Testing](#automated-end-to-end-testing)
    - [Replay\_testing](#replay_testing)
  - [References](#references)

## Objectives

By the end of the module, participants will be able to:

- Understand the purpose of E2E testing and how it differs from integration testing.
- Use the `ros2 bag` command-line interface to record, inspect, and play data.
- Perform a manual E2E test by launching a system and replaying a rosbag.
- Understand the concept and workflow for automating E2E tests for CI/CD.

## Motivation

While integration tests (Module 4) confirm that a subset of nodes can collaborate (like checking the topics or services communication), they don't verify the entire "plot". End-to-end testing runs the entire system, bringing simulation into the picture.

Its purpose is to validate the robot's ability to meet a high-level requirement, such as "Given a map, navigate to the kitchen" or "Inspect all waypoints in the warehouse".

The importance of E2E testing in robotics includes:

- **Validating the "Mission"**: It's the only test level that answers the question: "Does the robot actually achieve its goal?"
- **Testing Against Reality**: By using data recorded from the real world (or a high-fidelity simulator), rosbags provide a "ground truth" scenario. This makes possible to test complex, emergent behaviors and edge cases that are impossible to script in a simple integration test.
- **Ultimate Regression-Proofing**: An E2E test is the ultimate safety net. If a change in any package (perception, control, navigation) breaks the robot's ability to complete its mission, a good E2E test will catch it.

## The rosbag Toolset

The most important tool for E2E testing is rosbag (specifically, rosbag2 in ROS 2). It makes possible to capture and replay the entire message-passing state of a ROS system.

A rosbag is essentially a database (in ROS 2, from the Iron distro onwards, MCAP is used by default, in older distros it's a sqlite3 file) that stores all messages published on specific topics, along with their timestamps.

### Recording Data
To record data, use the `ros2 bag record` command. It's possible to either specify topics or record everything:

```bash
# Record all topics on the system
ros2 bag record -a

# Record only specific topics
ros2 bag record /scan /odom /tf

# Record to a specific bag file (directory)
ros2 bag record -o my_mission_bag /scan /odom
```

This last command creates a directory (for example, `my_mission_bag/`) containing the database file and metadata.

### Inspecting Data

Before using a bag, it's necessary to know what's in it.

```bash
# Get a summary of the bag file
ros2 bag info my_mission_bag/

# Example output:
# Files:             my_mission_bag.mcap
# Bag size:          15.8 MiB
# Storage id:        mcap
# Duration:          1m 10s
# Start:             Oct 17 2025 15:30:01.000
# End:               Oct 17 2025 15:31:11.000
# Messages:          3013
# Topic information: 
   # Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
   # Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr
```

### Playing Back Data

This is the core of the workflow. `ros2 bag play` republishes all the messages in the bag exactly as they were recorded. If the `--clock` flag is passed, it publishes `/clock` messages that makes the playback follow the original timing via simulated time, so nodes with the parameter `use_sim_time = True` experience the same timeline as when the bag was recorded, but those without this parameter set will still see real time.

```bash
# Play a bag
ros2 bag play my_mission_bag/

# Play in a loop (great for repeated testing)
ros2 bag play -l my_mission_bag/

# Play at 2.0x speed
ros2 bag play --rate 2.0 my_mission_bag/
```

This command acts like a "data simulator", providing a perfectly repeatable stream of inputs to the system.

## Manual End-to-End Testing

This is the most common and intuitive form of E2E testing. It involves a human operator launching the system, providing a scenario (usually via a rosbag), and visual or log-based verification of the result.

This is perfect for a final "sanity check" before merging a major feature.

### Using Pre-recorded Rosbags

A typical manual test session looks like this:
 
1. Launch the System: Start the core nodes of the robot, along with visualization tools like `Rviz` to be able to monitor the progress.
2. Provide Input: Instead of launching the drivers (like the camera or lidar node), use `ros2 bag play`. This feeds the recorded data (for example, `/scan`, `/camera/image_raw`) into the system.
3. Observe and Verify: The engineer watches the output:
    - In `RViz`: "Does the robot's navigation visualization show it reaching the goal?"
    - In the terminal: "Did the mission control node log 'MISSION_COMPLETE'?"

This workflow is incredibly powerful but has one major drawback: it's not automated. It relies on a human to launch, observe, and judge success.

> [!NOTE]
> There will be cases where the rosbag won't be enough or where it's necessary to test other features in a known environment. In these cases, starting a simulator like `Gazebo` along with playing the rosbag is a good idea, but it's not common.

### Using Simulation Data

While rosbags are invaluable for reproducing real-world scenarios, simulation-based E2E testing provides complementary advantages. Instead of replaying fixed data, the simulator generates live sensor streams (for example, LiDAR, camera) and physics interactions in real time.

This is really useful specially for testing certain features:

- Test dynamic conditions such as moving obstacles or lightning changes.
- Parameterize worlds and robot configurations to explore edge cases.
- Ideal for CI, since there is the possibility of running simulations in headless mode on a server (using `--headless-rendering` flag in `Gazebo` for example).

Example workflow:

1. Launch the robot’s navigation and perception stacks inside the simulator.
2. Define an automated mission (for example, "Navigate to the charging dock").
3. Let the simulator generate data live, instead of replaying a rosbag.
4. Use `launch_testing` (presented in Module 4) to check that the final goal is achieved (for example, via `/mission_status` or `/odom`).

This approach complements rosbag replay testing: the former focuses on realism and variability, the latter on repeatability and regression detection.

## Automated End-to-End Testing

The "holy grail" of robotics testing is to automate the manual workflow. This is key to build a robust Continuous Integration (CI) pipeline, as it will be covered in Module 6.

This approach combines everything learned so far:

1. Test Framework: Use `launch_testing` as the test runner.
2. `generate_test_description()`: This function is now responsible for launching the entire system:
   - The robot's core nodes (navigation, perception, control).
   - A simulator (for example, Gazebo) with a specific world file.
   - A `ros2 bag play` command (using ExecuteProcess) to provide the test scenario.
3. `unittest.TestCase`:
   - The test case can no longer "watch" RViz. It must check for success programmatically.
   - It creates its own temporary rclpy node.
   - It subscribes to a "result" topic (for example, `/mission_status`).
   - It waits for the rosbag to finish playing.
   - It asserts the final state. For example, it might subscribe to `/odom` and assert that the robot's final position is within 10cm of the desired goal.

This creates a fully self-contained test that can be run on a server with `colcon test`. It is the most complex type of test to write, but it provides the highest possible confidence in the system's stability.

It's important to highlight that using both the simulator and the rosbag can be misleading. One might think: "If I'm already using a simulation, why would I need a rosbag?" And this is true in some cases, but there are special cases when you might want to combine both, for example, with a probllematic trajectory pre-recorded that needs to be tested again but without assuming risks in the real world.

### Replay_testing

Polymath Robotics `replay_testing` tool provides a convenient wrapper for End-to-End testing with rosbags. It automates conventions like:

- Launching the system and a rosbag together.
- Synchronizing `/clock` and simulated time.
- Checking for topic availability and test completion conditions.

This makes it an excellent starting point for teams who want automated mission playback without writing custom `launch_testing` code from scratch.

## References

- [ROS 2 Documentation: ros2 bag CLI](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [ROS 2 rosbag2 GitHub Repository](https://github.com/ros2/rosbag2)
- [replay_testing Polymath Robotics](https://github.com/PolymathRobotics/replay_testing)
- [Example of system tests in Nav2](https://github.com/ros-navigation/navigation2/blob/main/nav2_system_tests/src/system/README.md)
