#!/usr/bin/env bash
set -e

# Get the absolute directory of this script
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
# Get the repository root
REPO_ROOT=$( dirname "$SCRIPT_DIR" )

# Navigate to the repo root to ensure all paths are correct
cd "$REPO_ROOT"

if [ $# -eq 0 ]; then
  echo "No packages specified, skipping build."
  exit 0
fi

echo "Building and testing changed ROS 2 packages: $@"

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep update
rosdep install --from-paths modules --ignore-src -y

# Build
colcon build --packages-up-to "$@" --symlink-install --event-handlers console_direct+

# Test
colcon test --packages-up-to "$@" --event-handlers console_direct+ --return-code-on-test-failure
