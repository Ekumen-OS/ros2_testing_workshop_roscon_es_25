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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "module_4/safety_light.hpp"

namespace module_4 {

/**
 * @class SafetyLightNode
 * @brief The ROS 2 wrapper that subscribes to detection status and logs the light state.
 */
class SafetyLightNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Safety Light Node object.
   */
  explicit SafetyLightNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  /**
   * @brief Callback for the /obstacle_detected topic.
   * @param msg The received boolean message.
   */
  void detection_callback(const std_msgs::msg::Bool::SharedPtr msg) const;

  std::unique_ptr<SafetyLight> safety_light_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detection_sub_;
};

}  // namespace module_4
