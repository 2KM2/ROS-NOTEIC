#!/usr/bin/env bash
# Build the custom messages locally for ROS Noetic.
# This package is an overlay workspace; do not install it into /opt/ros.

set -euo pipefail

source /opt/ros/noetic/setup.bash
cd "$(dirname "$0")"
catkin_make -DCMAKE_BUILD_TYPE=Release

echo "Build complete. Before building or running LaserUndistortion, run:"
echo "  source $(pwd)/devel/setup.bash"
