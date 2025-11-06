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

#include <stdexcept>
#include <vector>

#include "module_2/laser_detector.hpp"

namespace module_2 {
namespace test {

// Define tolerance for tests
static constexpr double TOL = 1e-6;

TEST(TestLaserDetector, ConstructorDoesNotThrow) {
  // Define laser options
  const LaserOptions laser_options{
      -0.6,  // angle_min
      0.6,   // angle_max
      0.2,   // angle_increment
      0.1,   // range_min
      40.0,  // range_max
  };

  // Define parameters
  const double footprint_radius{1.5};
  const int min_points{5};
  const double roi_min_angle{-0.4};
  const double roi_max_angle{0.4};

  // Create dut and expect no throw
  ASSERT_NO_THROW({
    const LaserDetector dut(laser_options, footprint_radius, min_points, roi_min_angle,
                            roi_max_angle);
  });
}

struct InvalidConstructorParams {
  LaserOptions laser_options{};
  double footprint_radius{};
  int min_points{};
  double roi_min_angle{};
  double roi_max_angle{};
};

class TestInvalidConstructor : public ::testing::TestWithParam<InvalidConstructorParams> {};

TEST_P(TestInvalidConstructor, InvalidArguments) {
  const auto& params = GetParam();

  // Create dut and expect throw
  ASSERT_THROW(LaserDetector dut(params.laser_options, params.footprint_radius, params.min_points,
                                 params.roi_min_angle, params.roi_max_angle),
               std::runtime_error);
}

INSTANTIATE_TEST_SUITE_P(
    InvalidConstructorTests, TestInvalidConstructor,
    ::testing::Values(
        // Invalid laser options (angle_min = angle_max)
        InvalidConstructorParams{{0.6, 0.6, 0.2, 0.1, 40.0}, 1.5, 5, -0.4, 0.4},
        // Invalid laser options (angle_increment < 0)
        InvalidConstructorParams{{-0.6, 0.6, -0.2, 0.1, 40.0}, 1.5, 5, -0.4, 0.4},
        // Invalid footprint radius (footprint_radius = 0)
        InvalidConstructorParams{{-0.6, 0.6, 0.2, 0.1, 40.0}, 0.0, 5, -0.4, 0.4},
        // Invalid footprint radius (footprint_radius < 0)
        InvalidConstructorParams{{-0.6, 0.6, 0.2, 0.1, 40.0}, -1.5, 5, -0.4, 0.4},
        // Invalid min points (min_points = 0)
        InvalidConstructorParams{{-0.6, 0.6, 0.2, 0.1, 40.0}, 1.5, 0, -0.4, 0.4},
        // Invalid min points (min_points < 0)
        InvalidConstructorParams{{-0.6, 0.6, 0.2, 0.1, 40.0}, 1.5, -5, -0.4, 0.4},
        // Invalid ROI (angle_min = angle_max)
        InvalidConstructorParams{{-0.6, 0.6, 0.2, 0.1, 40.0}, 1.5, 5, 0.4, 0.4}));

TEST(TestLaserDetector, TestROIFilter) {
  // Define laser options
  const LaserOptions laser_options{
      -0.6,  // angle_min
      0.6,   // angle_max
      0.2,   // angle_increment
      0.1,   // range_min
      40.0,  // range_max
  };

  // Define parameters
  const double footprint_radius{1.5};
  const int min_points{5};
  const double roi_min_angle{-0.5};
  const double roi_max_angle{0.5};

  // Define input data
  const std::vector<float> input{1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6};

  // Define expected output
  const std::vector<float> expected_output{1.1, 1.2, 1.3, 1.4, 1.5};

  // Create dut
  LaserDetector dut(laser_options, footprint_radius, min_points, roi_min_angle, roi_max_angle);

  // Filter ROI
  const std::vector<float> output = dut.roi_filter(input);

  // Assert
  ASSERT_EQ(output.size(), 5u);
  for (size_t i = 0; i < output.size(); ++i) {
    EXPECT_NEAR(output[i], expected_output[i], TOL) << "Mismatch at index " << i;
  }
}

TEST(TestLaserDetector, TestPointsInFootprint) {
  // Define laser options
  const LaserOptions laser_options{
      -0.6,  // angle_min
      0.6,   // angle_max
      0.2,   // angle_increment
      0.1,   // range_min
      40.0,  // range_max
  };

  // Define parameters
  const double footprint_radius{1.5};
  const int min_points{5};
  const double roi_min_angle{-0.6};
  const double roi_max_angle{0.6};

  // Define input data
  const std::vector<float> input{1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6};

  // Define expected output (points inside)
  const int expected_output{6};

  // Create dut
  LaserDetector dut(laser_options, footprint_radius, min_points, roi_min_angle, roi_max_angle);

  // Note: we are not applying any ROI

  // Calculate num points inside the footprint
  const int output = dut.points_inside_footprint(input);

  // Assert
  EXPECT_EQ(output, expected_output);
}

TEST(TestLaserDetector, TestPointsInFootprintOutsideRange) {
  // Define laser options
  const LaserOptions laser_options{
      -0.6,  // angle_min
      0.6,   // angle_max
      0.2,   // angle_increment
      0.1,   // range_min
      40.0,  // range_max
  };

  // Define parameters
  const double footprint_radius{1.5};
  const int min_points{5};
  const double roi_min_angle{-0.6};
  const double roi_max_angle{0.6};

  // Define input data
  const std::vector<float> input{0.05, 1.1, 1.2, 45.3, 1.4, 1.5, 1.6};

  // Define expected output (points inside)
  const int expected_output{4};

  // Create dut
  LaserDetector dut(laser_options, footprint_radius, min_points, roi_min_angle, roi_max_angle);

  // Note: we are not applying any ROI

  // Calculate num points inside the footprint
  const int output = dut.points_inside_footprint(input);

  // Assert
  EXPECT_EQ(output, expected_output);
}

struct ObstacleDetectionParams {
  int points{};
  bool detection{};
};

class TestObstacleDetection : public ::testing::TestWithParam<std::pair<int, bool>> {};

TEST_P(TestObstacleDetection, DetectObstacleThresholds) {
  // Get params
  const auto& [points, detection] = GetParam();

  // Define laser options
  const LaserOptions laser_options{
      -0.6,  // angle_min
      0.6,   // angle_max
      0.2,   // angle_increment
      0.1,   // range_min
      40.0,  // range_max
  };

  // Define parameters
  const double footprint_radius{1.5};
  const int min_points{5};
  const double roi_min_angle{-0.4};
  const double roi_max_angle{0.6};

  // Create dut
  LaserDetector dut(laser_options, footprint_radius, min_points, roi_min_angle, roi_max_angle);

  // Assert obstacle detection
  EXPECT_EQ(dut.detect_obstacle(points), detection);
}

INSTANTIATE_TEST_SUITE_P(ObstacleThresholdCases, TestObstacleDetection,
                         ::testing::Values(
                             // below threshold
                             std::pair{2, false},
                             // exactly at min_points (boundary)
                             std::pair{5, true},
                             // above threshold
                             std::pair{7, true}));

}  // namespace test
}  // namespace module_2
