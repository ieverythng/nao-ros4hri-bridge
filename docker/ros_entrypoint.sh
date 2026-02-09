#!/bin/bash
set -e

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace if exists
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

exec "$@"
