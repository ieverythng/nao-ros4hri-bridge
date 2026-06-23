#!/usr/bin/env bash
set -euo pipefail

# Full-stack seam launch helper for planner/chatbot/orchestrator validation.
# Runs the same runtime seams used in live operator sessions.

source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash

echo "[trace] Launching full stack with planner seam validation profile..."
echo "[trace] Expected seam: /planner/dialogue_act -> /nao_orchestrator/planner_dialogue_act -> dialogue_manager"

ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  sim_use_laptop_tts:=false \
  posture_bridge_wake_up_on_connect:=true \
  start_naoqi_driver:=true \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv \
  nao_ip:=172.26.112.130 \
  network_interface:=wlp1s0 \
  start_planner_llm:=true \
  chatbot_planner_mode_enabled:=true \
  chatbot_server_url:=http://10.7.138.215:8004/v1/chat/completions \
  planner_llm_provider:=openai_compatible \
  planner_llm_base_url:=http://10.7.138.215:8004 \
  planner_llm_model:=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ \
  planner_llm_api_key_env:=VLLM_API_KEY \
  start_fake_skills:=true \
  start_interaction_trace_viewer:=true \
  start_demo_log_window:=true \
  "$@"
