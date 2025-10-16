// Copyright 2025 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <gtest/gtest.h>

#include <chrono>
#include <module_3/laser_detector_node.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

namespace module_3 {
namespace test {

using namespace std::chrono_literals;

static constexpr float LASER_RANGE_MIN = 0.1f;
static constexpr float LASER_RANGE_MAX = 40.0f;
static constexpr char LASER_FRAME_ID[] = "laser_frame";

// Spin the given executor until predicate is true or timeout elapses.
// Returns true if the predicate became true, false on timeout.
template <class Predicate, class Rep, class Period>
inline bool spin_until(rclcpp::Executor& exec, Predicate&& predicate,
                       const std::chrono::duration<Rep, Period>& timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && !predicate()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;
    const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);
    exec.spin_once(remaining);
    exec.spin_some(remaining);
  }
  return predicate();
}

class TestLaserDetectorNodeConstructor : public ::testing::Test {
 public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

 protected:
  const rclcpp::NodeOptions default_node_options_;
};

TEST_F(TestLaserDetectorNodeConstructor, CreatingANodeDoesNotThrow) {
  ASSERT_NO_THROW({ const LaserDetectorNode dut(default_node_options_); });
}

TEST_F(TestLaserDetectorNodeConstructor, EvaluateConstructorDeclaresParameters) {
  // Create laser detector node object (dut)
  const LaserDetectorNode dut(default_node_options_);

  // Assert dut has parameters
  ASSERT_TRUE(dut.has_parameter("footprint_radius"));
  ASSERT_TRUE(dut.has_parameter("min_points"));
  ASSERT_TRUE(dut.has_parameter("roi_min_angle"));
  ASSERT_TRUE(dut.has_parameter("roi_max_angle"));
}

TEST_F(TestLaserDetectorNodeConstructor, EvaluateConstructorReadsParameters) {
  // Set node parameter values via node options
  rclcpp::NodeOptions custom_node_options;
  custom_node_options.append_parameter_override("footprint_radius", 1.5);
  custom_node_options.append_parameter_override("min_points", 15);
  custom_node_options.append_parameter_override("roi_min_angle", 0.5);
  custom_node_options.append_parameter_override("roi_max_angle", 2.5);

  // Create laser detector node object (dut)
  const LaserDetectorNode dut(custom_node_options);

  // Assert dut has the same parameter values as specified via node options
  ASSERT_EQ(1.5, dut.get_parameter("footprint_radius").as_double());
  ASSERT_EQ(15, dut.get_parameter("min_points").as_int());
  ASSERT_EQ(0.5, dut.get_parameter("roi_min_angle").as_double());
  ASSERT_EQ(2.5, dut.get_parameter("roi_max_angle").as_double());
}

TEST_F(TestLaserDetectorNodeConstructor, EvaluateTopicInterface) {
  // Create laser detector node object (dut)
  const LaserDetectorNode dut(default_node_options_);

  // Get a map of topic name and types
  const std::map<std::string, std::vector<std::string>> topic_names_and_types =
      dut.get_topic_names_and_types();

  // Assert topic name and types match to the expected values
  ASSERT_TRUE(topic_names_and_types.find("/obstacle_detection") != topic_names_and_types.end());
  ASSERT_TRUE(topic_names_and_types.find("/scan") != topic_names_and_types.end());
  ASSERT_FALSE(topic_names_and_types.at("/obstacle_detection").empty());
  ASSERT_FALSE(topic_names_and_types.at("/scan").empty());
  ASSERT_EQ("std_msgs/msg/Bool", topic_names_and_types.at("/obstacle_detection")[0]);
  ASSERT_EQ("sensor_msgs/msg/LaserScan", topic_names_and_types.at("/scan")[0]);
}

class TestLaserDetectorNode : public ::testing::Test {
 public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

 protected:
  void SetUp() override {
    // Initialize node options
    node_options_.append_parameter_override("footprint_radius", 1.5);
    node_options_.append_parameter_override("min_points", 5);
    node_options_.append_parameter_override("roi_min_angle", -0.5);
    node_options_.append_parameter_override("roi_max_angle", 0.5);

    // Create auxiliary node
    test_node_ = rclcpp::Node::make_shared("auxiliary_node");

    // Publisher
    scan_pub_ =
        test_node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

    // Create DUT
    dut_ = std::make_shared<LaserDetectorNode>(node_options_);

    // Setup subscriber
    detection_sub_ = test_node_->create_subscription<std_msgs::msg::Bool>(
        "obstacle_detection", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { latest_detection_ = *msg; });

    // Create executor and add both nodes
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);
    executor_->add_node(dut_);
  }
  void TearDown() override {
    // Reset shared pointers
    detection_sub_.reset();
    scan_pub_.reset();
    dut_.reset();
    test_node_.reset();
    executor_.reset();
  }

  // Node options
  rclcpp::NodeOptions node_options_;

  // Auxiliary node for publishing scan msgs and subscribe to obhect detection
  rclcpp::Node::SharedPtr test_node_;
  // Publishers, Subscribers and Services
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_sub_;

  std::optional<std_msgs::msg::Bool> latest_detection_;

  // Executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  // The Device Under Test
  std::shared_ptr<LaserDetectorNode> dut_;
};

TEST_F(TestLaserDetectorNode, SubscribingToLaserScanDoesNotThrow) {
  // Create LaserScan message
  auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  msg->header.frame_id = LASER_FRAME_ID;
  msg->header.stamp = test_node_->get_clock()->now();
  msg->angle_min = -0.6;
  msg->angle_max = 0.6;
  msg->angle_increment = 0.2;
  msg->range_min = LASER_RANGE_MIN;
  msg->range_max = LASER_RANGE_MAX;
  msg->ranges = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6};
  msg->intensities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Publish msg
  scan_pub_->publish(*msg);

  // Deterministic wait for at least one callback to process
  ASSERT_TRUE(spin_until(*executor_, [&] { return latest_detection_.has_value(); }, 500ms));
  // No explicit assertion needed here beyond "no throw" and a callback arrived.
}

// TEST_F(TestLaserDetectorNode, ObstacleDetectionIsTrue) {}

// TEST_F(TestLaserDetectorNode, ObstacleDetectionIsFalse) {}

}  // namespace test
}  // namespace module_3
