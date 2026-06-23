# Trace Workflow (Full Stack)

This folder defines the operator workflow to validate the entire planner/chatbot/orchestrator runtime seam in one live run.

## What We Validate

- planner bring-up is live (`/planner_llm` node + preflight logs)
- execution authority is orchestrator-owned
- planner dialogue seam is relayed through orchestrator:
  - `/planner/dialogue_act` (planner-owned publish)
  - `/nao_orchestrator/planner_dialogue_act` (orchestrator-owned relay)
  - `dialogue_manager` consumes relay topic
- planner dialogue contract is direct-mode by default; completion wording is
  relayed through `chatbot_llm` when chatbot client wiring is available.

## 1) Launch the Full Stack (Exact Profile)

Run this exactly as the live seam test entrypoint:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
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
  start_demo_log_window:=true
```

Shortcut helper:

```bash
scripts/run_full_stack_planner_seam_session.sh
```

## 2) Required Startup Checks

```bash
ros2 node list | grep -E 'planner_llm|chatbot_llm|nao_orchestrator|dialogue_manager|interaction_trace_viewer'
ros2 param get /dialogue_manager planner_dialogue_act_topic
ros2 param get /nao_orchestrator planner_dialogue_act_topic
ros2 param get /nao_orchestrator planner_dialogue_relay_topic
ros2 topic info /nao_orchestrator/planner_dialogue_act -v
```

Expected:

- `dialogue_manager.planner_dialogue_act_topic = /nao_orchestrator/planner_dialogue_act`
- `nao_orchestrator.planner_dialogue_act_topic = /planner/dialogue_act`
- `nao_orchestrator.planner_dialogue_relay_topic = /nao_orchestrator/planner_dialogue_act`

## 3) Drive the Stack Like an Operator

- Use `rqt` speech input flow as in normal operation.
- Or inject one fixture request directly:

```bash
ros2 topic pub --once /nao_orchestrator/planner_request chatbot_msgs/msg/Intent \
'{intent: planner_request, data: "{\"goal_id\":\"trace_goal_01\",\"turn_id\":\"trace_turn_01\",\"goal_text\":\"scan the room\",\"kind\":\"new_goal\",\"source\":\"operator_probe\"}", data_raw: "", person_id: "", intent_type: "", priority: 0, confidence: 0.0}'
```

## 4) Force Success and Failure Modes

```bash
scripts/fake_skill_scenario_menu.sh /fake_skill_server /head_motion_skill_server
```

Recommended sequence:

- `success_default`
- `ambiguous_cup`
- `path_blocked`
- `head_motion_strict` (open-loop fallback disabled)

## 5) Optional Trace Capture

```bash
scripts/run_interaction_trace_viewer_compact_json.sh
```

Use `scripts/run_live_fake_skill_scenario_probe.py` only as a quick smoke pass.
It is not a replacement for full operator-in-the-loop validation.
