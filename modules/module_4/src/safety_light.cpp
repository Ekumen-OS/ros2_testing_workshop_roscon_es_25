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
#include "module_4/safety_light.hpp"

namespace module_4 {

void SafetyLight::update_status(bool obstacle_detected) { is_red_ = obstacle_detected; }

std::string SafetyLight::get_status_message() const {
  if (is_red_) {
    return "STATUS: RED LIGHT - OBSTACLE DETECTED!";
  } else {
    return "STATUS: GREEN LIGHT - CLEAR";
  }
}

}  // namespace module_4
