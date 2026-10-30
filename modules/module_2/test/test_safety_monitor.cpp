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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <vector>

#include "module_2/laser_detector.hpp"
#include "module_2/laser_scanner_interface.hpp"
#include "module_2/safety_monitor.hpp"

namespace module_2 {
namespace test {

using ::testing::Return;

// Mock of the laser scanner driver
class MockLaserScanner : public LaserScannerInterface {
 public:
  MOCK_METHOD(std::vector<float>, read_ranges, (), (override));
};

LaserDetector make_detector() {
  // Define laser options (7 beams from -0.6 to 0.6 rad)
  const LaserOptions laser_options{
      -0.6,  // angle_min
      0.6,   // angle_max
      0.2,   // angle_increment
      0.1,   // range_min
      40.0,  // range_max
  };

  // Define parameters
  const double footprint_radius{1.5};
  const int min_points{3};
  const double roi_min_angle{-0.5};
  const double roi_max_angle{0.5};

  return LaserDetector(laser_options, footprint_radius, min_points, roi_min_angle, roi_max_angle);
}

TEST(TestSafetyMonitorConstructor, NullScannerThrows) {
  // Create dut without a scanner and expect throw
  ASSERT_THROW(SafetyMonitor dut(nullptr, make_detector()), std::runtime_error);
}

class TestSafetyMonitor : public ::testing::Test {
 protected:
  // Mock injected into the dut instead of the real driver
  std::shared_ptr<MockLaserScanner> scanner_{std::make_shared<MockLaserScanner>()};
  SafetyMonitor dut_{scanner_, make_detector()};
};

TEST_F(TestSafetyMonitor, ObstacleDetected) {
  // This test verifies that the path is not clear when the scanner sees an
  // obstacle inside the footprint.

  // Arrange: the scanner returns three beams hitting an obstacle at 1.0 m,
  // inside the footprint (1.5 m)
  const std::vector<float> ranges{5.0, 1.0, 1.0, 1.0, 5.0, 5.0, 5.0};
  EXPECT_CALL(*scanner_, read_ranges()).WillOnce(Return(ranges));

  // Act
  const bool path_clear = dut_.is_path_clear();

  // Assert
  EXPECT_FALSE(path_clear);
}

TEST_F(TestSafetyMonitor, PathClear) {
  // This test verifies that the path is clear when the scanner does not see
  // any obstacle inside the footprint.

  /// BEGIN EDIT ------------------------------------------------------

  // Make the scanner return ranges far away from the footprint (e.g., all
  // of them at 5.0 m), call is_path_clear() and expect it to return true.

  EXPECT_FALSE(true) << "Intentional failure: logic not implemented yet.";

  /// END EDIT --------------------------------------------------------
}

TEST_F(TestSafetyMonitor, EmptyScanIsNotClear) {
  // This test simulates a driver failure where no data is received, which
  // is hard to reproduce on purpose with the real hardware.

  /// BEGIN EDIT ------------------------------------------------------

  // Make the scanner return an empty scan (std::vector<float>{}), call
  // is_path_clear() and expect it to return false.

  EXPECT_FALSE(true) << "Intentional failure: logic not implemented yet.";

  /// END EDIT --------------------------------------------------------
}

}  // namespace test
}  // namespace module_2
