# Module 5 – End-to-End Testing

In this final testing module, we explore End-to-End (E2E) testing. This is the highest level of testing, designed to verify that the entire robotic system can successfully perform a complete "mission" or task from start to finish, just as it would in the real world.

## Objectives

By the end of the module, participants will be able to:

- Understand the purpose of E2E testing and how it differs from integration testing.
- Use the `ros2 bag` command-line interface to record, inspect, and play data.
- Perform a manual E2E test by launching a system and replaying a rosbag.
- Understand the concept and workflow for automating E2E tests for CI/CD.

## Motivation

While integration tests (Module 4) confirm that a subset of nodes can collaborate (like checking the topics or services communication), they don't verify the entire "plot". End-to-end testing runs the entire system, bringing, for example, simulation into the table.

Its purpose is to validate the robot's ability to meet a high-level requirement, such as "Given a map, navigate to the kitchen" or "Inspect all waypoints in the warehouse".

The importance of E2E testing in robotics includes:

- **Validating the "Mission"**: It's the only test level that answers the question: "Does the robot actually achieve its goal?"
- **Testing Against Reality**: By using data recorded from the real world (or a high-fidelity simulator), rosbags provide a "ground truth" scenario. This allows you to test complex, emergent behaviors and edge cases that are impossible to script in a simple integration test.
- **Ultimate Regression-Proofing**: An E2E test is the ultimate safety net. If a change in any package (perception, control, navigation) breaks the robot's ability to complete its mission, a good E2E test will catch it.
- **Debugging Complex Failures**: When a robot fails in the field, a rosbag of that failure is invaluable. It can be replayed in a simulator over and over until the root cause (for example, a race condition, a state machine logic error) is found.

## The rosbag Toolset

The most important tool for E2E testing is rosbag (specifically, rosbag2 in ROS 2). It allows you to capture and replay the entire message-passing state of a ROS system.

A rosbag is essentially a database (in ROS 2, it's a sqlite3 file) that stores all messages published on specific topics, along with their timestamps.

### Recording Data
To record data, you use the `ros2 bag record` command. You can either specify topics or record everything:

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

Before using a bag, you will want to know what's in it.

```bash
# Get a summary of the bag file
ros2 bag info my_mission_bag/

# Example output:
# Files:             my_mission_bag.db3
# Bag size:          15.8 MiB
# Storage id:        sqlite3
# Duration:          1m 10s
# Start:             Oct 17 2025 15:30:01.000
# End:               Oct 17 2025 15:31:11.000
# Topics with Type:
#   - /scan: sensor_msgs/msg/LaserScan [1000 msgs]
#   - /odom: nav_msgs/msg/Odometry    [700 msgs]
```

### Playing Back Data

This is the core of the workflow. ros2 bag play republishes all the messages in the bag exactly as they were recorded, with the original timing.

```bash
# Play a bag
ros2 bag play my_mission_bag/

# Play in a loop (great for repeated testing)
ros2 bag play -l my_mission_bag/

# Play at 2.0x speed
ros2 bag play --rate 2.0 my_mission_bag/
```

This command acts like a "data simulator", providing a perfectly repeatable stream of inputs to your system.

## Manual End-to-End Testing

This is the most common and intuitive form of E2E testing. It involves a human operator launching the system, providing a scenario (usually via a rosbag), and visual or log-based verification of the result.

This is perfect for debugging, or for a final "sanity check" before merging a major feature.

### The Manual E2E Workflow

A typical manual test session looks like this:
 
1. Launch the System: Start the core nodes of your robot. This might be in a simulator like `Gazebo`, or it might be just the processing stack. You would also launch visualization tools like `RViz`.
2. Provide Input: Instead of launching the drivers (like the camera or lidar node), you use ros2 bag play. This feeds the recorded data (e.g., /scan, /camera/image_raw) into your system.
3. Observe and Verify: The engineer watches the output:
    - In `RViz`: "Does the robot's navigation visualization show it reaching the goal?"
    - In `Gazebo`: "Does the simulated robot arm move to the correct object?"
    - In the terminal: "Did the mission control node log 'MISSION_COMPLETE'?"
4. Analyze: If it fails, you can now debug the running nodes, knowing the input data is identical every single time.

This workflow is incredibly powerful but has one major drawback: it's not automated. It relies on a human to launch, observe, and judge success.

## Automated End-to-End Testing

The "holy grail" of robotics testing is to automate the manual workflow. This is how you build a robust Continuous Integration (CI) pipeline.

This approach combines everything we've learned so far:

1. Test Framework: Use launch_testing (from Module 4) as the test runner.
2. generate_test_description(): This function is now responsible for launching the entire system:
   - The robot's core nodes (navigation, perception, control).
   - A simulator (for example, Gazebo) with a specific world file.
   - A ros2 bag play command (using ExecuteProcess) to provide the test scenario.
3. unittest.TestCase:
   - The test case can no longer "watch" RViz. It must check for success programmatically.
   - It creates its own temporary rclpy node.
   - It subscribes to a "result" topic (for example, `/mission_status`).
   - It waits for the rosbag to finish playing.
   - It asserts the final state. For example, it might subscribe to `/odom` and assert that the robot's final position is within 10cm of the desired goal.

This creates a fully self-contained test that can be run on a server with `colcon test`. It is the most complex type of test to write, but it provides the highest possible confidence in your system's stability.

## References

- [ROS 2 Documentation: ros2 bag CLI](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [ROS 2 rosbag2 GitHub Repository](https://github.com/ros2/rosbag2)
- [Example of system tests in Nav2](https://github.com/ros-navigation/navigation2/blob/main/nav2_system_tests/src/system/README.md)
