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
#include <gtest/gtest.h>

#include <vector>

#include "module_2/laser_detector.hpp"

namespace module_2 {
namespace test {

TEST(LaserDetector, TestROIFilter) {
  // Define laser options
  const LaserOptions laser_options{
      -0.4,  // angle_min
      0.4,   // angle_max
      0.2,   // angle_increment
      0.1,   // range_min
      40.0,  // range_max
  };

  // Define parameters
  const double footprint_radius{1.5};
  const

      // Define input data
      const std::vector input {}

  // Create dut
  LaserDetector dut(laser_options, );

  dut.filter();
}

TEST(LaserDetector, TestPointsInFootprint) {
  // Use parametrized test
}

TEST(LaserDetector, TestObstacleDetection) {}

}  // namespace test
}  // namespace module_2
