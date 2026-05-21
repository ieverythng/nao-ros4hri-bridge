#!/bin/bash
set -e

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace if exists
if [ -f /home/ubuntu/ws/install/setup.bash ]; then
  source /home/ubuntu/ws/install/setup.bash
  echo "Workspace sourced successfully"
else
  echo "Workspace not built yet"
fi

maybe_build_optional_detector_stack() {
  if [ "${AUTO_BUILD_OPTIONAL_WS_PACKAGES:-1}" = "0" ]; then
    return
  fi

  local detector_packages=(
    emorobcare_cv_msgs
    my_game_interface
    emorobcare_cv_object_detection
  )
  local source_present=()
  local install_missing=()

  for pkg in "${detector_packages[@]}"; do
    if [ -d "/home/ubuntu/ws/src/${pkg}" ]; then
      source_present+=("${pkg}")
      if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
        install_missing+=("${pkg}")
      fi
    fi
  done

  if [ "${#source_present[@]}" -eq 0 ] || [ "${#install_missing[@]}" -eq 0 ]; then
    return
  fi

  echo "Optional detector sources are present but not installed: ${install_missing[*]}"
  echo "Building detector workspace slice: ${source_present[*]}"
  (
    cd /home/ubuntu/ws
    colcon build --symlink-install --packages-select "${source_present[@]}"
  )
  source /home/ubuntu/ws/install/setup.bash
  echo "Optional detector workspace slice built and sourced"
}

maybe_build_optional_detector_stack

# Print ROS environment
echo "ROS_DISTRO: $ROS_DISTRO"
echo "Available packages:"
ros2 pkg list | grep -E "(asr_vosk|chatbot_llm|dialogue_manager|interaction_skills|interaction_trace_viewer|naoqi|nao_(chatbot|look_at|orchestrator|replay_motion|say_skill|scene_grounding|skills)|planner_(common|llm)|fake_skills|simple_audio_capture|emorobcare_cv_(msgs|object_detection)|my_game_interface)" || \
  echo "No expected nao-related packages found"

exec "$@"
