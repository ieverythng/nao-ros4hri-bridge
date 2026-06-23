#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash

pkill -f 'interaction_trace_viewer trace_node' >/dev/null 2>&1 || true

ros2 run interaction_trace_viewer trace_node --ros-args \
  -p compact_mode:=false \
  -p include_raw_payloads:=false \
  -p enable_scene_summary_channel:=false \
  -p rosout_min_level:=warn \
  -p include_channels_csv:='planner/request,intents,planner/execution_feedback,planner/dialogue_act,nao_orchestrator/planner_dialogue_act,chatbot_llm/turn_trace,fake_skills/events,world_model/enriched_snapshot,world_model/enriched_text' \
  -p include_event_types_csv:='planner_request,planner_output,execution_feedback,planner_dialogue_act,chatbot_turn_trace,skill_result,kb_snapshot'
