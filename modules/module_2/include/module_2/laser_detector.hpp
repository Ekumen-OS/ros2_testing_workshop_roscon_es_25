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
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

namespace module_2
{

class LaserDetector : public rclcpp::Node
{
public:
  LaserDetector();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Parameters
  double footprint_radius_{};
  int    min_points_{};
  double roi_min_angle_{};
  double roi_max_angle_{};

  // Publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_detection_;
};

}  // namespace module_2
