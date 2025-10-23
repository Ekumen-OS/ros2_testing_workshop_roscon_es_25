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
#include "module_4/safety_light_node.hpp"

namespace module_4 {

SafetyLightNode::SafetyLightNode(const rclcpp::NodeOptions& options)
    : Node("safety_light_node", options) {
  safety_light_ = std::make_unique<SafetyLight>();

  obstacle_detection_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/obstacle_detection", 10,
      std::bind(&SafetyLightNode::detection_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Safety Light node has been initialized.");
}

void SafetyLightNode::detection_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
  safety_light_->update_status(msg->data);
  std::string status = safety_light_->get_status_message();
  RCLCPP_INFO_STREAM(this->get_logger(), status);
}

}  // namespace module_4

// This macro registers the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(module_4::SafetyLightNode)
