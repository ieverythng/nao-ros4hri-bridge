# LLM Model Ablations

Date: 2026-07-11

This ledger records model-backend ablations for the NAO ROS4HRI runtime. It is
intended to keep model capability, launch wiring, prompt compatibility, and
runtime postconditions separate.

## Scope

The current ablations focus on four supervisor-critical seams:

- KB mutation when facts are placed, queried, revised, and removed.
- `report_result` wording after combined skill usage.
- Human/object distinction and grouped location effects.
- Fake-skill execution, recovery, replanning, and final reporting.

The active launch profile for the completed Qwen and mixed runs used
`response_first` and `chatbot_grounded_context_digest_enabled:=false`.

## Ablation Matrix

| Run | Chatbot response/intent | Planner | Evidence | Result |
|---|---|---|---|---|
| Qwen personal stack | `qwen36-turbo-hermes` via `http://10.88.140.94:4000` | `qwen36-turbo-hermes` via `http://10.88.140.94:4000` | `/tmp/nao_qwen_*_20260711.json`, `../artifacts/runtime_review_2026-07-11_qwen_personal_stack.md` | Fluent wording, but degraded fake-deep recovery and KB mutation. Score recorded as 5.8/10. |
| Mixed Watson/lab attempt | `qwen36-turbo-hermes` via `http://10.88.140.94:4000` | Intended lab Qwen3-VL via `http://10.7.138.215:8004` | `/tmp/nao_mixed_qwen_chat_lab_plan_*_20260711.json` | Planner JSON failures dropped to zero in the post snapshot, but KB mutation and fake-deep handoff still failed. Runtime label is weaker than a clean relaunch because dynamic planner params do not prove a new client was constructed. |
| Lab-only baseline attempt | Intended `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` for chatbot and planner | Intended `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` | `/tmp/codex_lab_stack_relaunch.log`, host/container `curl` checks | Blocked. `10.7.138.215:8004` was unreachable from host and container, so a scored lab baseline would be invalid. |
| Restored Watson stack | `qwen36-turbo-hermes` via `http://10.88.140.94:4000` | `qwen36-turbo-hermes` via `http://10.88.140.94:4000` | `/tmp/codex_watson_stack_restore.log` | Restored reachable runtime after the failed lab relaunch. Do not treat this restore as a scored baseline because the graph had auxiliary duplicate-name warnings during cleanup. |

## Mixed Run Findings

Artifacts:

- `/tmp/nao_mixed_qwen_chat_lab_plan_snapshot_pre_20260711.json`
- `/tmp/nao_mixed_qwen_chat_lab_plan_arch_20260711.json`
- `/tmp/nao_mixed_qwen_chat_lab_plan_main_20260711.json`
- `/tmp/nao_mixed_qwen_chat_lab_plan_environment_20260711.json`
- `/tmp/nao_mixed_qwen_chat_lab_plan_fake_deep_all_success_20260711.json`
- `/tmp/nao_mixed_qwen_chat_lab_plan_fake_deep_fail_once_navigation_20260711.json`
- `/tmp/nao_mixed_qwen_chat_lab_plan_snapshot_post_20260711.json`

The mixed post snapshot reported no planner invalid JSON, no invalid executable
plan, and no planner gate rejection events. It did report nine route repair
events, 81 planner requests, and 109 `report_result` references in the sampled
window.

The architecture sweep kept failing the KB postconditions:

- `kb_add` passed for `codex_arch_marker` with color green, name NOVA, and type
  Cube.
- `kb_revise` failed. The expected blue color was not present, and green
  remained.
- The non-mutating query observed the stale green marker state.
- `kb_remove` failed. Facts for `codex_arch_marker` remained.

The preloaded grouped-delivery sweep also showed stale spatial facts. The cup
and book retained kitchen predicates while also gaining recipient predicates.
The recipient accumulated object-like inverse relations such as `placeOf` and
object spatial predicates. This remains a post-effect contract issue, not a
cosmetic wording issue.

The active mixed questionnaires had an instrumentation limitation: main and
environment cases degraded for missing speech evidence even when the stack
logged chatbot/planner activity. Fake-deep all-success and fail-once-navigation
both stopped at `fake_deep_ordered_walk_report` with missing planner request and
execution feedback. Because the recovery profile failed with the same signature
as the all-success profile, the mixed failure is upstream of fake-skill recovery
policy.

## Launch And Harness Lessons

Runtime model changes by parameter are not sufficient evidence for a scored
ablation. During the lab switch, `/chatbot_llm` parameters changed to the lab
model, but later logs still showed response-stage calls using
`qwen36-turbo-hermes`. Scored model runs must be launched with explicit role
arguments:

```bash
chatbot_model:=<model>
chatbot_intent_model:=<model>
chatbot_server_url:=<chat-completions-url>
planner_llm_model:=<model>
planner_llm_base_url:=<base-url>
```

The lab-only baseline was not run because `http://10.7.138.215:8004/v1/models`
was unreachable from both host and container at the time of the relaunch. A
future lab baseline should first pass endpoint reachability, then launch a fresh
graph, verify node uniqueness, verify startup `STACK READY` model names, and
only then run the full and fake-deep suites.

## Next Ablations

- Clean lab-only baseline after the lab endpoint is reachable.
- Mixed role split with a fresh relaunch, not dynamic params:
  Watson chatbot plus lab planner, and lab chatbot plus Watson planner.
- Intent-only split: Watson response model with a stricter intent model.
- Digest split: JSON-only grounded context versus compact digest.
- Postcondition-heavy fake-deep run focused on grouped object moves,
  recipient/human distinction, and `report_result` after partial failures.
