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

LaserDetector::LaserDetector(const LaserOptions& laser_options, const double& footprint_radius,
                             const int& min_points, const double& roi_min_angle,
                             const double& roi_max_angle)
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

  /// END EDIT --------------------------------------------------------
}

std::vector<double> LaserDetector::roi_filter(const std::vector<double>& scan) {
  if (scan.empty()) {
    return {};
  }

  /// BEGIN EDIT ------------------------------------------------------

  // Return a filtered scan that fits within the angle ROI provided in
  // the constructor.

  /// END EDIT --------------------------------------------------------
}

int LaserDetector::points_inside_footprint(const std::vector<double>& scan) {
  if (scan.empty()) {
    return 0;
  }

  /// BEGIN EDIT ------------------------------------------------------

  // Return the number of points that are inside of the footprint.
  // Remember to exclude invalid data such as nans or values
  // outside of the range limits.

  /// END EDIT --------------------------------------------------------
}

bool LaserDetector::detect_obstacle(const int& num_points) {
  /// BEGIN EDIT ------------------------------------------------------

  // Return whether the number of points inside the footprint (input)
  // is lower than the limit (min_points_)

  /// END EDIT --------------------------------------------------------
}

}  // namespace module_2
