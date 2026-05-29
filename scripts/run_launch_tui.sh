#!/usr/bin/env bash
set -eo pipefail

PACKAGE="${1:-nao_chatbot}"
LAUNCH_FILE="${2:-nao_chatbot_sim.launch.py}"
shift 2 || true

if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if [ -f /home/ubuntu/ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/ubuntu/ws/install/setup.bash
fi

if ! ros2 launch_tui --help >/dev/null 2>&1; then
  echo "launch_tui is not installed. Install socialminds-ros-jazzy-launch-tui and rebuild the image." >&2
  exit 1
fi

exec ros2 launch_tui "${PACKAGE}" "${LAUNCH_FILE}" "$@"
