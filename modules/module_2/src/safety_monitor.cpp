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

#include "module_2/safety_monitor.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

namespace module_2 {

SafetyMonitor::SafetyMonitor(std::shared_ptr<LaserScannerInterface> scanner,
                             const LaserDetector& detector)
    : scanner_{std::move(scanner)}, detector_{detector} {
  // Ensure the laser scanner driver is provided
  if (!scanner_) {
    throw std::runtime_error("Laser scanner must not be null");
  }
}

bool SafetyMonitor::is_path_clear() {
  const std::vector<float> ranges = scanner_->read_ranges();

  /// BEGIN EDIT ------------------------------------------------------

  // Fail-safe: an empty scan means the driver did not provide any data
  // (e.g., the scanner is disconnected), so there is no way to know
  // whether the path is clear. Return false in that case.

  /// END EDIT --------------------------------------------------------

  // Run the obstacle detection algorithm
  const std::vector<float> ranges_in_roi = detector_.roi_filter(ranges);
  const int num_points = detector_.points_inside_footprint(ranges_in_roi);

  return !detector_.detect_obstacle(num_points);
}

}  // namespace module_2
