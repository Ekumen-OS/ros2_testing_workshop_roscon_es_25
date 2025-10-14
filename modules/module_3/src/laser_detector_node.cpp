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
#include "module_3/laser_detector_node.hpp"

#include <cmath>
#include <stdexcept>

namespace module_3 {

LaserDetectorNode::LaserDetectorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("laser_detector_node", options) {
  // Declare parameters
  this->declare_parameter("footprint_radius", 1.0);
  this->declare_parameter("min_points", 20);
  this->declare_parameter("roi_min_angle", 0.0);
  this->declare_parameter("roi_max_angle", 2.0 * M_PI);

  // Get parameters
  footprint_radius_ = this->get_parameter("footprint_radius").as_double();
  min_points_ = this->get_parameter("min_points").as_int();
  roi_min_angle_ = this->get_parameter("roi_min_angle").as_double();
  roi_max_angle_ = this->get_parameter("roi_max_angle").as_double();

  // Ensure footprint radius is positive
  if (footprint_radius_ <= 0) {
    throw std::runtime_error("Footprint radius must be positive");
  }

  // Ensure min points is positive
  if (min_points_ <= 0) {
    throw std::runtime_error("Min points must be positive");
  }

  // Ensure ROI configuration is valid
  if (roi_min_angle_ >= roi_max_angle_) {
    throw std::runtime_error("Invalid ROI! Maximum angle must be greater than minimum angle");
  }

  // Publishers
  obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("obstacle_detection", 10);

  // Subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&LaserDetectorNode::scan_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LaserDetectorNode initialized");
}

void LaserDetectorNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!detector_) {
    const module_2::LaserOptions laser_options{
        msg->angle_min, msg->angle_max, msg->angle_increment, msg->range_min, msg->range_max,
    };

    // Construct detector
    detector_ = std::make_unique<module_2::LaserDetector>(
        laser_options, footprint_radius_, min_points_, roi_min_angle_, roi_max_angle_);

    RCLCPP_INFO(this->get_logger(), "LaserDetector initialized from first scan");
  }

  // ROI filter
  const auto filtered_roi = detector_->roi_filter(msg->ranges);

  // Count points inside footprint
  const int points_inside = detector_->points_inside_footprint(filtered_roi);

  // Detect obstacle
  const bool obstacle_detection = detector_->detect_obstacle(points_inside);

  // Publish output
  std_msgs::msg::Bool obstacle;
  obstacle.data = obstacle_detection;
  obstacle_pub_->publish(obstacle);
}

}  // namespace module_3
