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
#include "module_2/laser_detector.hpp"

#include <stdexcept>

namespace module_2 {

LaserDetector::LaserDetector(const LaserOptions& laser_options, const double& footprint_radius,
                             const int& min_points, const double& roi_min_angle,
                             const double& roi_max_angle)
    : laser_options_(laser_options),
      footprint_radius_{footprint_radius},
      min_points_{min_points},
      roi_min_angle_{roi_min_angle},
      roi_max_angle_{roi_max_angle} {
  // Sanity checks
  if (((laser_options_.angle_max - laser_options_.angle_min) / laser_options_.angle_increment) <
      0) {
    throw std::runtime_error("Wrong scan angle option");
  }
}

void LaserDetector::roi_filter(const std::vector<double> scan) {
  if (scan.empty()) {
    return;
  }
  // Implement
}

int LaserDetector::points_inside_footprint(const std::vector<double> scan) {
  if (scan.empty()) {
    return 0;
  }
  // Implement
}

bool LaserDetector::detect_obstacle(const int num_points) {
  // Implement
}

}  // namespace module_2
