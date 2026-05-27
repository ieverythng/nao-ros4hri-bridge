# Scripts Index

## Runtime seam helpers

- `run_full_stack_planner_seam_session.sh`
  - launches the full live stack with the canonical planner/chatbot/orchestrator seam args
  - use this for end-to-end operator-in-the-loop seam validation

- `run_interaction_trace_viewer_compact_json.sh`
  - launches interaction trace viewer with JSON-focused filters
  - includes planner/chatbot/orchestrator/fake-skills/world-model channels

- `run_launch_tui.sh`
  - launches a stack profile through SocialMinds `launch_tui` (terminal GUI)
  - default target: `nao_chatbot nao_chatbot_sim.launch.py`

- `fake_skill_scenario_menu.sh`
  - interactive scenario selector for `/fake_skill_server`
  - supports optional head-motion node arg to toggle strict open-loop behavior

- `run_live_fake_skill_scenario_probe.py`
  - scripted smoke probe runner for quick scenario sweeps
  - writes markdown report + JSONL captures under `docs/artifacts/`
  - use as regression smoke only, not as replacement for full-stack human-in-loop tests

## Validation helpers

- `check_skill_registry_consistency.py`
- `sync_skill_registry_views.py`
- `run_precommit.sh`
- `run_tests.sh`

See `docs/traces/README.md` for the full-stack trace workflow.
