# TFM Defence Demonstration Runbook

## Presentation role

The live demonstration is illustrative. The frozen questionnaire, retained traces, and reported denominators remain the scored thesis evidence.

The recommended demonstration lasts no more than 90 seconds and exercises three easily distinguished routes:

1. `Hey, how are you?`
   Expected observation: one dialogue response, no planner request, and no skill dispatch.
2. `What can you see now?`
   Expected observation: a knowledge response based on the preloaded symbolic fixture, with no robot-action plan.
3. `Move your head up.`
   Expected observation: an execution request, an admitted plan, one motion result, and one terminal response.

If the physical NAO is available, the third turn can use the real head-motion adapter and robot TTS. Without the robot, use the deterministic motion adapter and show the typed result in the trace viewer. Do not add detector-dependent person or object selection to the timed demonstration.

## How to present the complex traces

Keep the failure and replanning evidence separate from the live demonstration.

- Slide 13 presents the retained middle-stage pick failure as a static lineage: successful find, failed pick, typed feedback, plan version 2, and terminal report.
- Prepare one 20 to 30 second screen recording of that retained interaction. The visible sequence should contain the user request, failed skill result, correlated feedback, revised plan version, and final closure.
- Keep one beginning-stage navigation recovery screenshot or recording as a question-period backup. It should show that the plan stopped, the target set survived, the plan version changed, and the revised execution closed.
- Do not show long raw JSON in the prepared talk. Keep the JSONL and complete trace available for tribunal questions.

This arrangement gives the live demonstration a narrow operational purpose while the stronger recovery claims remain tied to reproducible evidence.

## Window layout

Use one display and avoid switching between desktops:

- left side: the dialogue input window;
- right side: the interaction trace viewer with full planner, feedback, dialogue-act, and skill-result channels;
- robot visible beside the display if physical head motion and TTS are used;
- PowerPoint remains ready to return immediately to slide 15 after the third turn.

Do not expose container build logs, source editors, or unrelated ROS traffic.

## September 2 freeze gate

1. Choose the exact machine, image, model endpoint, network interface, and robot/no-robot mode.
2. Rehearse only from the frozen image. Do not patch source, prompts, fixtures, or expected behaviour after the final rehearsal.
3. Verify the endpoint model before launching ROS. The endpoint must advertise the model used in the demonstration.
4. Start one clean container and one integrated launch process. Do not hot-copy packages or restart individual nodes.
5. Preload `baseline_table` and set the fake-skill policy to `always_success`.
6. Run the three exact demonstration turns three times. All three repetitions must preserve route uniqueness and exactly-once speech.
7. Record the third complete repetition as the fallback video.
8. Stop. Do not use the defence morning for another behavioural iteration.

## Recommended launch, no physical robot

Run inside the rebuilt container after sourcing the ROS and workspace overlays. Set `VLLM_HOST` to the endpoint used for the frozen model.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash

ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  posture_bridge_wake_up_on_connect:=false \
  start_naoqi_driver:=false \
  start_object_detection:=false \
  start_scene_grounding:=true \
  preloaded_environment_ids:=baseline_table \
  preloaded_environment_lifespan_sec:=3600 \
  start_planner_llm:=true \
  chatbot_planner_mode_enabled:=true \
  chatbot_turn_pipeline_mode:=response_first \
  chatbot_grounded_context_digest_enabled:=false \
  chatbot_server_url:=http://${VLLM_HOST}:8004/v1/chat/completions \
  planner_llm_provider:=openai_compatible \
  planner_llm_base_url:=http://${VLLM_HOST}:8004 \
  planner_llm_model:=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ \
  planner_llm_api_key_env:=VLLM_API_KEY \
  start_fake_skills:=true \
  fake_skill_global_mode:=always_success \
  perform_motion_execution_mode:=fake \
  look_at_execution_mode:=fake \
  start_interaction_trace_viewer:=true \
  interaction_trace_compact_mode:=false \
  interaction_trace_include_raw_payloads:=true
```

## Recommended launch, physical NAO present

Use the same profile, but connect the NAOqi driver and select real motion execution. Keep look-at fake during the timed demonstration because its physical target-frame and reset paths remain outside the frozen v1 qualification.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash

ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  posture_bridge_wake_up_on_connect:=true \
  start_naoqi_driver:=true \
  nao_ip:=${NAO_IP} \
  network_interface:=${NAO_NETWORK_INTERFACE} \
  start_object_detection:=false \
  start_scene_grounding:=true \
  preloaded_environment_ids:=baseline_table \
  preloaded_environment_lifespan_sec:=3600 \
  start_planner_llm:=true \
  chatbot_planner_mode_enabled:=true \
  chatbot_turn_pipeline_mode:=response_first \
  chatbot_grounded_context_digest_enabled:=false \
  chatbot_server_url:=http://${VLLM_HOST}:8004/v1/chat/completions \
  planner_llm_provider:=openai_compatible \
  planner_llm_base_url:=http://${VLLM_HOST}:8004 \
  planner_llm_model:=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ \
  planner_llm_api_key_env:=VLLM_API_KEY \
  start_fake_skills:=true \
  fake_skill_global_mode:=always_success \
  perform_motion_execution_mode:=real \
  look_at_execution_mode:=fake \
  start_interaction_trace_viewer:=true \
  interaction_trace_compact_mode:=false \
  interaction_trace_include_raw_payloads:=true
```

## Preflight before every rehearsal

```bash
ros2 node list | grep -E 'chatbot_llm|dialogue_manager|planner_llm|nao_orchestrator|knowledge_core|fake_skill_server' | sort | uniq -c
ros2 lifecycle get /dialogue_manager
ros2 param get /chatbot_llm turn_pipeline_mode
ros2 param get /nao_orchestrator perform_motion_execution_mode
ros2 param get /fake_skill_server global_mode
ros2 service call /kb/query kb_msgs/srv/Query \
  "{patterns: ['?subject rdf:type ?type'], vars: ['?subject'], models: ['default']}"
```

The core nodes must appear once, the dialogue manager must be active, the pipeline must be `response_first`, the execution mode must match the chosen profile, the fake policy must be `always_success`, and the KB query must return `success=true`.

Collect one runtime snapshot after the final rehearsal:

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/collect_runtime_snapshot.py \
  --container nao_ros2 \
  --since 30m \
  --out /tmp/nao_defence_rehearsal_snapshot.json
```

## On-stage stop conditions

Abandon the live path immediately if any of the following occurs:

- the model endpoint is unavailable or advertises a different model;
- a core node is missing or duplicated;
- KnowledgeCore is not ready;
- the first dialogue turn exceeds its rehearsed bound;
- the system produces contradictory dialogue and execution routes;
- speech is duplicated;
- the robot adapter is unavailable.

Use one sentence: “The live path is illustrative rather than the scored evidence, so I will continue with the retained trace.” Continue to the next slide. Do not relaunch, switch models, or debug during the timed defence.

## Files to carry

- `TFM_DEFENCE_FINAL_2026-09-03.pptx`
- `TFM_DEFENCE_FINAL_2026-09-03.pdf`
- the original `TFM_PRESENTATION_TEMPLATE.html`
- the 20 to 30 second recovery recording
- the final rehearsal runtime snapshot
- the retained recovery JSONL and one static screenshot
