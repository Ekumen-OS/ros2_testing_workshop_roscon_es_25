import math
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
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
        rclpy.init()
        cls.node = rclpy.create_node("test_node")
        # ====================== END EDIT ==================================
        pass

    @classmethod
    def tearDownClass(cls):
        """Shutdown rclpy and destroy the node after all tests are done."""
        # ===================== BEGIN EDIT =================================
        cls.node.destroy_node()
        rclpy.shutdown()
        # ====================== END EDIT ==================================
        pass

    def test_obstacle_triggers_red_light(self, proc_output):
        """
        Tests that a close laser scan reading triggers the 'RED LIGHT' status.
        """
        # ===================== BEGIN EDIT =================================
        # Create a publisher to the /scan topic.
        scan_publisher = self.node.create_publisher(LaserScan, "scan", 10)

        # Create a LaserScan message using the helper function.
        scan_msg = create_triggering_scan_msg()

        # Add a small delay to ensure the publisher is ready
        time.sleep(0.5)

        # Publish the message.
        scan_publisher.publish(scan_msg)

        # Use 'proc_output.assertWaitFor' to check for the "RED LIGHT" message.
        # This will check the output of all launched processes.
        proc_output.assertWaitFor(
            "STATUS: RED LIGHT - OBSTACLE DETECTED!",
            process=None,
            timeout=5,
        )
        # ====================== END EDIT ==================================


# Post-shutdown tests
@launch_testing.post_shutdown_test()
class TestDetectionSystemShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
