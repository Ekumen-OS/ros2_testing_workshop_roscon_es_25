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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace module_3 {
namespace test {

using namespace std::chrono_literals;

static sensor_msgs::msg::LaserScan make_scan(const std::vector<float>& ranges) {
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lascaner";
  scan.angle_min = -0.6f;
  scan.angle_max = 0.6f;
  scan.range_min = 0.05f;
  scan.range_max = 40.0f;
  scan.angle_increment = (ranges.size() > 1) ? (scan.angle_max - scan.angle_min) /
                                                   static_cast<float>(ranges.size() - 1)
                                             : 0.f;
  scan.ranges = ranges;
  scan.intensities.assign(ranges.size(), 0.0f);
  return scan;
}

TEST(CrossTalkInterferer, SpamNearScansOnDefaultDomain) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("interferer_node");
  auto pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // For ~500ms, publish scans likely to trigger detection (near ranges).
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    auto msg = make_scan({0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 0.7f, 0.6f});
    msg.header.stamp = node->get_clock()->now();
    pub->publish(msg);
    exec.spin_once(5ms);
  }

  rclcpp::shutdown();
}

}  // namespace test
}  // namespace module_3
