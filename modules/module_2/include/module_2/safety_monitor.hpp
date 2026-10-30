// Copyright 2026 Ekumen, Inc.
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

#include "module_2/laser_detector.hpp"
#include "module_2/laser_scanner_interface.hpp"

namespace module_2 {

/**
 * @class SafetyMonitor
 * @brief ROS-agnostic logic that checks whether the path in front of the robot is clear.
 *
 * The laser scanner driver is injected through an abstract interface, so it
 * can be replaced by a mock in unit tests.
 */
class SafetyMonitor {
 public:
  /**
   * @brief Construct a new Safety Monitor object.
   * @param scanner Laser scanner driver used to sense the surroundings.
   * @param detector Obstacle detection algorithm.
   * @throws std::runtime_error If the scanner is null.
   */
  SafetyMonitor(std::shared_ptr<LaserScannerInterface> scanner, const LaserDetector& detector);

  /**
   * @brief Reads a scan from the driver and checks it for obstacles.
   * @return True if the robot can move safely, false otherwise.
   */
  bool is_path_clear();

 private:
  std::shared_ptr<LaserScannerInterface> scanner_;
  LaserDetector detector_;
};

}  // namespace module_2
