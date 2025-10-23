#!/usr/bin/env bash
set -euo pipefail

if [ $# -eq 0 ]; then
  echo "No packages specified, skipping build."
  exit 0
fi

echo "🔧 Building and testing changed ROS 2 packages: $@"

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build
colcon build --packages-up-to "$@" --symlink-install --event-handlers console_direct+

# Test
colcon test --packages-up-to "$@" --event-handlers console+
