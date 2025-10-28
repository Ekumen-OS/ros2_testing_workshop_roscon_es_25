import math
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.clock import Clock
from sensor_msgs.msg import LaserScan


# Helper function to create the LaserScan message
def create_triggering_scan_msg():
    """Creates a LaserScan message with a close obstacle to trigger the detector."""
    scan_msg = LaserScan()
    scan_msg.header.stamp = Clock().now().to_msg()
    scan_msg.header.frame_id = "laser_frame"
    scan_msg.angle_min = -math.pi / 2
    scan_msg.angle_max = math.pi / 2
    scan_msg.angle_increment = math.pi / 180.0
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.1
    scan_msg.range_min = 0.05
    scan_msg.range_max = 10.0

    num_readings = (
        int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
    )
    scan_msg.ranges = [10.0] * num_readings

    # Place a close obstacle directly in front
    scan_msg.ranges[num_readings // 2] = 0.1
    return scan_msg


def generate_test_description():
    """
    Generates the launch description for the integration test.
    This function finds and prepares the laser_detector and safety_light nodes.
    """
    # Prepare the nodes for execution using the Node action
    composable_nodes_container = ComposableNodeContainer(
        name="safety_light_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="module_3",
                plugin="module_3::LaserDetectorNode",
                name="laser_detector_node",
                parameters=[
                    {"footprint_radius": 0.2},  # meters
                    {"min_points": 1},  # require only 1 point to trigger
                    {"roi_min_angle": -1.57},  # -90 degrees
                    {"roi_max_angle": 1.57},  # +90 degrees
                ],
            ),
            ComposableNode(
                package="module_4",
                plugin="module_4::SafetyLightNode",
                name="safety_light_node",
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            composable_nodes_container,
            # This action tells launch_testing that the system is ready for testing
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDetectionSystem(unittest.TestCase):
    """
    The actual test class for the detection system.
    This test will publish a fake LaserScan to trigger the system
    and verify that the safety_light_node reacts correctly.
    """

    # Use setUpClass and tearDownClass for robust resource management
    @classmethod
    def setUpClass(cls):
        """Initialize rclpy and create a node once for all tests."""
        # ===================== BEGIN EDIT =================================

        # ====================== END EDIT ==================================
        pass

    @classmethod
    def tearDownClass(cls):
        """Shutdown rclpy and destroy the node after all tests are done."""
        # ===================== BEGIN EDIT =================================

        # ====================== END EDIT ==================================
        pass

    def test_obstacle_triggers_red_light(self, proc_output):
        """
        Tests that a close laser scan reading triggers the 'RED LIGHT' status.
        """
        # ===================== BEGIN EDIT =================================
        #
        # 1. Create a publisher to the /scan topic.
        #
        # 2. Wait for the subscriber to be ready (THIS IS THE CRITICAL PART!)
        #    - Create a loop that checks `self.scan_publisher.get_subscription_count()`
        #    - Use a timeout (for example 10 seconds) to prevent an infinite loop.
        #    - Inside the loop, spin the node (`rclpy.spin_once`)
        #    - After the loop, use `self.assertGreater` to fail the test if
        #      no subscriber appeared.
        #
        # 3. Create and publish a LaserScan message that will trigger the detector.
        #    Use the helper function above.
        #
        # 4. Use 'proc_output.assertWaitFor' to check for the "RED LIGHT" message.
        #    - Give it a timeout (for example, 5 seconds).
        #
        # 5. (Optional but good practice) Add a try/except block around
        #    `assertWaitFor` to provide a clearer failure message if it times out.
        #
        assert False  # Replace this 'assert' with the necessary code for the test
        # ====================== END EDIT ==================================


# Post-shutdown tests
@launch_testing.post_shutdown_test()
class TestDetectionSystemShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
