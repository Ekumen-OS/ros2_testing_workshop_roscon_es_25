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

#include "module_2/bad_laser_detector.hpp"

namespace module_2 {

LaserDetectorROS::LaserDetectorROS() : Node("laser_detector_ros") {
  // Declare parameters
  this->declare_parameter("footprint_radius", 0.5);
  this->declare_parameter("min_points", 5);
  this->declare_parameter("roi_min_angle", -0.5);
  this->declare_parameter("roi_max_angle", 0.5);

  // Get parameters
  footprint_radius_ = this->get_parameter("footprint_radius").as_double();
  min_points_ = this->get_parameter("min_points").as_int();
  roi_min_angle_ = this->get_parameter("roi_min_angle").as_double();
  roi_max_angle_ = this->get_parameter("roi_max_angle").as_double();

  // Publishers
  pub_detection_ = this->create_publisher<std_msgs::msg::Bool>("obstacle_detected", 10);

  // Subscribers
  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&LaserDetectorROS::scan_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialized");
}

void LaserDetectorROS::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (msg->ranges.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty scan received");
    return;
  }

  double current_angle = msg->angle_min;
  unsigned int count_inside_footprint = 0;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float r = msg->ranges[i];

    // Skip invalid data
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
      continue;
    }

    // Skip points outside ROI
    current_angle += msg->angle_increment;
    if (current_angle < roi_min_angle_ || current_angle > roi_max_angle_) {
      continue;
    }

    // Count points inside the footprint radius
    if (r <= footprint_radius_) {
      count_inside_footprint++;
    }
  }

  // Check if the number of points inside the footprint is greater or equal than the limit
  bool detection = static_cast<int>(count_inside_footprint) >= min_points_;

  // Publish if obstacle is inside the footprint
  std_msgs::msg::Bool detection_msg;
  detection_msg.data = detection;
  pub_detection_->publish(detection_msg);
}

}  // namespace module_2
