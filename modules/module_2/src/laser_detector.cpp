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

#include <cmath>
#include <stdexcept>

namespace module_2 {

LaserDetector::LaserDetector(const LaserOptions& laser_options, const float& footprint_radius,
                             const int& min_points, const float& roi_min_angle,
                             const float& roi_max_angle)
    : laser_options_(laser_options),
      footprint_radius_{footprint_radius},
      min_points_{min_points},
      roi_min_angle_{roi_min_angle},
      roi_max_angle_{roi_max_angle} {
  // Sanity checks

  // Ensure angle_max is strictly greater than angle_min
  if (laser_options_.angle_max <= laser_options_.angle_min) {
    throw std::runtime_error("Laser angle_max must be strictly greater than angle_min");
  }

  // Ensure the angle increment is positive
  if (laser_options_.angle_increment <= 0.0) {
    throw std::runtime_error("Laser angle increment must be positive");
  }

  // Ensure min points is positive
  if (min_points_ <= 0) {
    throw std::runtime_error("Min points must be positive");
  }

  // Ensure ROI configuration is valid
  if (roi_min_angle_ >= roi_max_angle_) {
    throw std::runtime_error("Invalid ROI! Maximum angle must be greater than minimum angle");
  }

  /// BEGIN EDIT ------------------------------------------------------

  // Add check for a missing configuration value and throw based on tests

  // Footprint radius must be positive.
  if (footprint_radius_ <= 0.0) {
    throw std::runtime_error("footprint_radius must be > 0");
  }

  /// END EDIT --------------------------------------------------------
}

std::vector<float> LaserDetector::roi_filter(const std::vector<float>& scan) {
  if (scan.empty()) {
    return {};
  }

  /// BEGIN EDIT ------------------------------------------------------

  // Return a filtered scan that fits within the angle ROI provided in
  // the constructor.

  std::vector<float> scan_out;
  scan_out.reserve(scan.size());  // upper bound

  float current_angle = laser_options_.angle_min;
  for (size_t i = 0; i < scan.size(); i++) {
    // Keep only beams whose angle lies within the ROI (inclusive)
    if (current_angle >= roi_min_angle_ && current_angle <= roi_max_angle_) {
      scan_out.push_back(scan[i]);
    }
    // Update angle
    current_angle += laser_options_.angle_increment;
  }

  return scan_out;

  /// END EDIT --------------------------------------------------------
}

int LaserDetector::points_inside_footprint(const std::vector<float>& scan) {
  if (scan.empty()) {
    return 0;
  }

  /// BEGIN EDIT ------------------------------------------------------

  // Return the number of points that are inside of the footprint.
  // Remember to exclude invalid data such as nans or values
  // outside of the range limits.

  int count = 0;
  for (const auto& r : scan) {
    if (!std::isfinite(r) || r < laser_options_.range_min || r > laser_options_.range_max) {
      continue;
    }
    if (r <= footprint_radius_) {
      count++;
    }
  }
  return count;

  /// END EDIT --------------------------------------------------------
}

bool LaserDetector::detect_obstacle(const int& num_points) {
  /// BEGIN EDIT ------------------------------------------------------

  // Return whether the number of points inside the footprint (input)
  // is lower than the limit (min_points_)

  return num_points >= min_points_;

  /// END EDIT --------------------------------------------------------
}

}  // namespace module_2
