# CRITIC Runtime Hardening Session

Date: 30 June 2026

This handoff records the CRITIC runtime hardening pass that followed the
response-first 8.7 baseline and the harder deep fake/replan run. The work
focuses on keeping the healthy response-first stack intact while closing the
validation gaps that appeared in preloaded environments, fake-skill failures,
location-scoped delivery, and active-goal lifecycle management.

## Session Objective

The target is not to raise the score by adding deterministic shortcuts. The
target is to make the runtime evidence clearer and more truthful:

- missing targets must not be hidden by stale KnowledgeCore fixtures
- grouped locations must resolve to deliverable objects, not support surfaces
  or people
- terminal planner dialogue acts must release the planner gate
- report-result wording must be driven by structured execution evidence
- the runtime questionnaire must show where each case failed

Prompt wording was deliberately left untouched in this pass. Any later prompt
mutation must go through the SkillOpt ledger requirement in `AGENTS.md`.

## What Changed

| Area | Action | Status | Evidence |
|---|---|---|---|
| Active planner goal lifecycle | Updated the planner gate so terminal planner dialogue acts clear the active goal. Clarifications that wait for the user still keep lineage. | Source-patched | `src/nao_orchestrator/nao_orchestrator/planner_gate.py`; `src/nao_orchestrator/test/test_planner_gate.py` |
| Runtime questionnaire observability | Added per-case phase breadcrumbs for turn injection, route, planner request, execution feedback, terminal outcome, and speech. Artifacts are written immediately after injection, then updated during long waits. The runner only short-circuits after terminal and speech evidence are both present. | Source-patched | `.codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py`; `test_run_active_questionnaire.py` |
| Stale-world guard | Added absence preflights for missing-object and missing-recipient cases. Contaminated cases are skipped before speech injection. | Source-patched | Questionnaire `stale_world_guard` fields |
| Grounded context object filtering | Filtered ontology, support, place, and person entries from user-facing location members while preserving raw RDF facts for trace/debug. | Source-patched | `src/planner_common/planner_common/contracts.py`; `src/planner_common/test/test_contracts.py` |
| Chatbot scene digest filtering | Applied the same user-facing ontology filter to chatbot scene digests, including compact location groups. | Source-patched | `src/chatbot_llm/chatbot_llm/knowledge_snapshot.py`; `src/chatbot_llm/test/test_knowledge_snapshot.py` |
| Grouped delivery fallback | Ensured the grouped-delivery fallback iterates only concrete object members from a matched location group. | Source-patched | `src/planner_llm/planner_llm/planner_engine.py`; `src/planner_llm/test/test_planner_engine.py` |
| Missing named recipient | Added grouped-delivery and chatbot admission holdouts where BLAKE is requested while only ALEX is grounded. Response-first and intent-first admission now clarify before planner handoff when a named person is absent from grounded context. | Source-patched; live proof pending | `src/planner_llm/test/test_planner_engine.py`; `src/chatbot_llm/test/test_turn_engine.py` |
| Execution outcome summaries | Added structured `plan_outcome_summary` data to execution feedback so later report and replan analysis can separate completed, failed, and pending targets. | Source-patched | `src/nao_orchestrator/nao_orchestrator/orchestrator.py`; `src/planner_common/planner_common/contracts.py` |
| SVG fixture availability | Packaged preloaded environment SVGs into the `nao_chatbot` install share so rqt can select them after a rebuild. | Source-patched | `src/nao_chatbot/setup.py`; `src/nao_chatbot/config/preloaded_environment_svgs/` |

## Validation Completed

Focused source validation was run before this handoff:

- `90 passed` for planner contracts, planner gate, planner engine, and the
  runtime questionnaire helper tests
- `64 passed` for orchestrator intent-rule and outcome-summary tests
- `81 passed` for chatbot turn-engine route/admission tests, including the
  named-recipient grounding guard
- `21 passed` for chatbot knowledge snapshot digest tests
- `3 passed` for interaction trace viewer tests
- `py_compile` passed for the touched Python entrypoints
- `git diff --check` passed
- `scripts/ros4hri_change_audit.py --mode working` passed

These checks prove source consistency. They do not replace the required rebuilt
container run.

## Remaining Live Proof

A pre-rebuild readiness snapshot was captured at
`/tmp/nao_runtime_snapshot_critic_pre_rebuild_check.json`. It confirms the
running stack is usable for manual QA: `/dialogue_manager` is `active [3]`,
`/chatbot_llm` is in `response_first`, `/interaction_trace_viewer` exists, and
`mirror_trace_lines_to_rosout` is enabled. It also confirms this is not the
acceptance proof for the local source patches, because the container still needs
to be rebuilt or relaunched from the patched workspace.

The later 30 June real-image pass rebuilt `iiia:nao`, removed the temporary
critic image, and launched a clean `nao_ros2` response-first stack. The graph
contained `/chatbot_llm`, `/dialogue_manager`, `/planner_llm`,
`/nao_orchestrator`, `/kb/knowledge_core`, `/fake_skill_server`,
`/report_result_skill_server`, `/scan_skill_server`, `/interaction_trace_viewer`,
and the HRI person/visualization nodes. `/chatbot_llm turn_pipeline_mode` matched
`response_first`, `mirror_trace_lines_to_rosout` was enabled, and
`nao_chatbot` exposed `preloaded_environment_viewer`.

The rebuilt pass used the patched harness and produced these artifacts:

```bash
/tmp/nao_fake_deep_all_success_real_20260630.json
/tmp/nao_fake_deep_grouped_delivery_real_20260630.json
/tmp/nao_fake_deep_fail_once_navigation_real_20260630.json
```

The duplicate active-goal rejection did not recur. Independent speech dialogues
now produce runtime goal ids instead of reusing `goal_default___1`. Isolated
grouped work-table delivery passed with route, planner request, three
`bring_object` successes, `report_result`, final speech, and `plan_completed`.
The `fail_once_navigation` kitchen case produced the desired recovery lineage:
`plan_version=1` failed `navigate_to(codex_iiia_kitchen)`, then
`plan_version=2` executed and resolved a final report. The final spoken result
reported that `codex_iiia_cup` was brought to `codex_iiia_alex`.

Keep the score qualified until the rebuilt stack proves the revised harness
under the same fake-deep cases. Source now records terminal evidence separately
from speech and keeps observing until both have appeared or the configured wait
expires.

## Current Risk Register

| Risk | Why it matters | Next action |
|---|---|---|
| Harness late-phase accounting can underreport success | The `fail_once_navigation` artifact marked route/planner/speech false because the case wrote `wait_sec=0.0`, while launch logs showed replan and final report. | Source now keeps observing until the configured wait or terminal plus speech evidence. Rerun the same case after a clean rebuild and compare the JSON to launch logs. |
| Missing-recipient speech guard needs live proof | Source now blocks named-person execution admission when the grounded context cannot confirm that recipient. | Rerun the BLAKE case in isolation; expected result is clarification before planner request, no invented recipient, and no duplicate active planner goal. |
| Grounded-context digest count mismatch | One fail-once run logged `Objects (0): none currently grounded` while the JSON contained visible cups, books, and tables. | Fix the digest count/filter path so the summary and JSON agree before using this view as thesis-facing contract evidence. |
| Manual vague KB add remains unsafe | “Add one cup” can still become an invalid mutation instead of asking for structured facts. | Add a clarification gate for under-specified KB mutations in a later pass. |
| Intent-first remains an ablation | It improves some route locks but regresses ordinary dialogue wording. | Keep response-first as the demo profile until intent-first passes the same holdouts. |
| Spatial/proximity proof is pending | The thesis needs a clear boundary between symbolic facts and metric spatial evidence. | Run a trusted simulator pose overlay and verify scene summary, KB, grounded context, and answer. |

## Actionable Next Steps

1. Patch harness late-phase accounting, then rerun the same isolated grouped
   delivery and fail-once navigation cases to confirm the artifact matches the
   launch logs.
2. Run missing-recipient, absent-object, delivery-blocked, and fail-once-pick
   speech cases in isolation before returning to the full ladder.
3. Fix the grounded-context digest count mismatch and add a contract holdout that
   compares summary counts against the JSON entity list.
4. Keep the deep fake/replan score at `7.4/10 qualified` until the harness and
   missing-recipient proof are clean.
