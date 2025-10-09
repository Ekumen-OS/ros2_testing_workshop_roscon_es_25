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

#include <vector>

namespace module_2 {
struct LaserOptions {
  float angle_min{};
  float angle_max{};
  float angle_increment{};
  float range_min{};
  float range_max{};
};

class LaserDetector {
 public:
  LaserDetector(const LaserOptions& laser_options, const double& footprint_radius,
                const int& min_points, const double& roi_min_angle, const double& roi_max_angle);
  std::vector<double> roi_filter(const std::vector<double>& scan);
  int points_inside_footprint(const std::vector<double>& scan);
  bool detect_obstacle(const int& num_points);

 private:
  LaserOptions laser_options_;
  double footprint_radius_{};
  int min_points_{};
  double roi_min_angle_{};
  double roi_max_angle_{};
};

}  // namespace module_2
