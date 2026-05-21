# interaction_trace_viewer

Simple ROS4HRI interaction trace viewer.

## Purpose

This package implements the initial observability deliverable from
`docs/plans/ros4hri_integration_master_plan_2026-05-18.md` (OBS-1).

It records and displays end-to-end interaction events across:

- user speech topics (`*/speech`)
- `/planner/request`
- `/intents`
- `/planner/execution_feedback`
- `/planner/dialogue_act`
- `/scene/summary`
- `/rosout`

## Run

```bash
ros2 run interaction_trace_viewer trace_node
```

Planner-focused low-noise mode (recommended):

```bash
ros2 run interaction_trace_viewer trace_node --ros-args \
  -p enable_scene_summary_channel:=false \
  -p rosout_min_level:=warn \
  -p rosout_node_allowlist_csv:="chatbot_llm,planner_llm,nao_orchestrator,scan_skill_server,report_result_skill_server,fake_skill_server,dialogue_manager,nao_say_skill,head_motion_skill_server,replay_motion_skill_server,nao_look_at,robot_speech_debug"
```

Verbose payload output:

```bash
ros2 run interaction_trace_viewer trace_node --ros-args -p compact_mode:=false
```

Launch file:

```bash
ros2 launch interaction_trace_viewer interaction_trace_viewer.launch.py
```

## Output

- JSONL event logs at `~/.ros/nao_ros4hri_traces` (configurable)
- HTML report on shutdown at `~/.ros/nao_ros4hri_trace_reports` (configurable)

## Static HTML render

```bash
ros2 run interaction_trace_viewer render_html \
  --input ~/.ros/nao_ros4hri_traces/<trace>.jsonl \
  --output ~/.ros/nao_ros4hri_trace_reports/<trace>.html
```

## Notes

- `/scene/summary` tracing is disabled by default to avoid high-frequency object/perception flood.
- `/rosout` tracing is filtered by node allowlist and severity by default (`warn` and above).
- Payloads are stored in full by default.
- Missing optional topics are handled by discovery and do not crash the node.
- Event model stays compatible with future dashboard and fake-skill event feeds.
