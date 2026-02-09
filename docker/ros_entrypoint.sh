# docker/ros_entrypoint.sh
#!/bin/bash
set -e

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace if exists
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
  echo "✅ Workspace sourced successfully"
else
  echo "⚠️  Workspace not built yet"
fi

# Print ROS environment
echo "ROS_DISTRO: $ROS_DISTRO"
echo "Available packages:"
ros2 pkg list | grep -E "(naoqi|nao_chatbot)" || echo "No naoqi packages found"

exec "$@"
