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

#include <string>

namespace module_4 {

/**
 * @class SafetyLight
 * @brief ROS-agnostic core logic for determining the safety light status.
 */
class SafetyLight {
 public:
  /**
   * @brief Construct a new Safety Light Logic object.
   */
  SafetyLight() = default;

  /**
   * @brief Updates the light's status based on obstacle detection.
   * @param obstacle_detected True if an obstacle is detected, false otherwise.
   */
  void update_status(bool obstacle_detected);

  /**
   * @brief Generates a status message based on the current light state.
   * @return A string representing the light's color and status.
   */
  std::string get_status_message() const;

 private:
  bool is_red_{false};
};

}  // namespace module_4
