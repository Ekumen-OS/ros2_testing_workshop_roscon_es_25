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

#include <module_3/laser_detector_node.hpp>

namespace module_3 {
namespace test {

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

}  // namespace test
}  // namespace module_3
