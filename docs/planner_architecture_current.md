# Planner Architecture Current State

Last updated: 2026-05-04

This file consolidates the Cursor guardrail review, the supervisor feedback
architecture note, and the demo planner handoff into one implementation-facing
status reference.

## Current Runtime Direction

The stack remains layered:

```text
dialogue_manager -> chatbot_llm -> planner_llm -> nao_orchestrator -> skills
                                      ^                 |
                                      |                 v
                              /planner/dialogue_act <- /planner/execution_feedback
```

Current demo path:

- `chatbot_llm` publishes execution-oriented planner requests on `/planner/request`.
- `planner_llm` plans over abstract skills and supervises execution feedback.
- `nao_orchestrator` validates and executes structured plan steps.
- `dialogue_manager` remains the speech/TTS realization owner.

Target architecture from supervisor feedback:

- dialogue-only turns should produce no executable intents.
- `nao_orchestrator` should become the deterministic planner gate for task
  intents and publish `/planner/request`.
- `chatbot_llm` should keep task metadata in `Intent.data`, not own executable
  planning or robot routing.

The target planner-gate migration is intentionally not mixed into the current
demo-hardening pass; it changes runtime ownership and needs its own validation.

## Implemented In Current Worktree

- Planner contract drift was reduced by exporting shared intent labels, step
  types, and failure policies from `planner_common`.
- `on_failure=continue` is preserved by `nao_orchestrator` instead of being
  silently coerced to `fail`.
- Dead orchestrator dispatch wrappers were removed and motion execution uses
  the live `_execute_*_step` paths.
- `scan` is now a first-party NAO composite skill export and a planner-visible
  derived skill, replacing the earlier `mock_scan_scene` registry entry.
- Scan/demo success summaries are carried through planner execution feedback as
  `result_summary`.
- Planner skill prompt manifests expose aliases, params, expected effects,
  observable success, safety flags, and robot adapter mapping.
- Unsupported generated planner steps are treated as invalid whole-plan output,
  avoiding silent partial plans.
- `step_succeeded` feedback now uses `status=succeeded` with
  `event_type=step_succeeded`.
- Chatbot/dialogue lifecycle launch sequencing uses launch-native lifecycle
  events, avoiding shell `ros2 lifecycle` hangs in loaded sim profiles.
- Head-motion convergence timeout handling is simpler and the plain-node
  lifecycle stance is documented.

## Remaining Architectural Gaps

- Move planner handoff ownership from direct `chatbot_llm -> /planner/request`
  to `nao_orchestrator -> /planner/request`.
- Stop emitting dialogue-only intents (`greet`, `identity`, `wellbeing`,
  `help`) into the execution path; keep only a temporary ignore shim in
  `nao_orchestrator`.
- Add deterministic planner-routing policy in `nao_orchestrator` for direct
  skill execution versus planner-eligible tasks.
- Generate planner prompt packs from the skill registry instead of maintaining
  prompt-only architecture notes.
- Add validation scenarios for atomic action, two-step action, scan/report,
  unsupported step, and clarification.

## Ollama Model Evaluation

Ollama's public pricing currently separates Free, Pro, and Max usage. Free can
access cloud models, while Pro adds larger/more powerful cloud models and 50x
more cloud usage. The model library marks cloud-capable model families such as
`gpt-oss`, `qwen3.5`, `gemma4`, and `qwen3-coder`.

Use the repeatable probe:

```bash
python3 scripts/benchmark_ollama_models.py --markdown \
  qwen3-coder:480b-cloud \
  gemma4:31b-cloud \
  glm-5.1:cloud \
  kimi-k2.6:cloud \
  deepseek-v4-flash:cloud \
  qwen3.5:cloud
```

The probe scores whether each model returns non-empty planner JSON with
supported skill names for a scan/report request. It is a fast filter, not a full
HRI evaluation. The best candidate should then be tested through the live ROS
stack using the demo scenarios.

Probe results from 2026-05-04 against the current local Ollama endpoint:

| Model | Result | Latency | Notes |
| --- | --- | ---: | --- |
| `qwen3-coder:480b-cloud` | preferred | 0.983-1.054s | Valid compact planner JSON with supported skill names after pulling the cloud manifest and tightening the prompt to require top-level `steps`. |
| `gemma4:31b-cloud` | usable | 2.969-3.686s | Valid planner JSON, but wrapped in a fenced block. |
| `gpt-oss:120b-cloud` | not recommended | 2.021s | Passed the shallow JSON probe but produced poor stack responses in user testing. |
| `gpt-oss:20b-cloud` | weak | 1.890s | Responded, but not valid JSON in the quick planner probe. |
| `llama3.2:1b` | weak/local fallback | 10.655s | Responded but produced no executable steps. |
| `qwen3.5:cloud` | blocked | 0.265s | Ollama returned subscription-required HTTP 403. |
| `qwen3.5:397b-cloud` | blocked | 0.154s | Ollama returned subscription-required HTTP 403. |
| `glm-5.1:cloud` | blocked | 0.133s | Ollama returned subscription-required HTTP 403. |
| `kimi-k2.6:cloud` | blocked | 0.127s | Ollama returned subscription-required HTTP 403. |
| `deepseek-v4-flash:cloud` | blocked | 0.117s | Ollama returned subscription-required HTTP 403. |
| `deepseek-v4-pro:cloud` | blocked | 0.119s | Ollama returned subscription-required HTTP 403. |

Current recommendation: use `qwen3-coder:480b-cloud` for planner testing first
and `gemma4:31b-cloud` as the next cloud fallback. The launch defaults now use
`qwen3-coder:480b-cloud` for `planner_llm_model` and `gemma4:31b-cloud` for
`ollama_model`/chatbot response generation. Keep
`planner_llm_think:=false` and low temperature. The chatbot side now defaults
to `gemma4:31b-cloud` because `qwen3.5:*cloud` is subscription-blocked in this
environment.

If cloud access is quota-limited or paywalled, fall back to a llama.cpp
OpenAI-compatible endpoint by launching with:

```bash
ros2 launch nao_chatbot nao_chatbot_sim_demo.launch.py \
  planner_llm_provider:=openai_compatible \
  planner_llm_base_url:=http://HOST:PORT \
  planner_llm_model:=MODEL_NAME \
  planner_llm_think:=false
```

For chatbot turns, point the Ollama-compatible frontend at the same model only
after the planner path is stable; planner JSON validity is the first priority.
