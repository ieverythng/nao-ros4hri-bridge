# Deep Fake JSON-Only Review (2026-07-07)

## Scope

This artifact records the 7 July strict JSON-only review of the deep fake and
replan validation suite. It separates three layers:

- base response-first health
- strict JSON-only fake-deep behavior
- source fixes that have now received a rebuilt fake-deep proof

## Evaluation Rule

Clarification is useful when information is genuinely missing. It is a
regression when the preloaded fixture already provides the source location,
recipient, and grounded member objects. The active questionnaire now records
that distinction with `expected_outcome` and `all_required_context`.

This follows the same evaluation posture recommended for agentic systems:
measure the final outcome and the action trajectory, keep the tool/environment
interface clear, and avoid reactive prompt churn when the failure is structural.
See Anthropic's
[Building effective agents](https://www.anthropic.com/engineering/building-effective-agents)
and
[Demystifying evals for AI agents](https://www.anthropic.com/engineering/demystifying-evals-for-ai-agents).

## Runtime Evidence

- Digest-enabled control: `/tmp/nao_fake_deep_questionnaire_20260707_digest_enabled_current.json`
  passed 8/8 cases. This remains a control run because the compact scene digest
  was enabled.
- JSON-only strict run: `/tmp/nao_fake_deep_all_success_json_only_scored_20260707.json`
  ran with `grounded_context_digest_enabled=false`.
- Focused grouped-location rerun: `/tmp/nao_fake_deep_grouped_delivery_json_only_resliced_20260707.json`
  confirmed that the grouped work-table clarification was real after fixing
  log slicing.
- Rebuilt all-success proof: `/tmp/nao_fake_deep_round2_all_success_shared_20260707.json`
  passed 7/8 with `grounded_context_digest_enabled=false`. The single failed
  BLAKE recipient case was a harness wording miss, not a runtime false
  execution: the robot asked which person to use, emitted no planner request,
  and produced no execution feedback.
- Rebuilt failure-profile proof: `/tmp/nao_fake_deep_round3_fail_once_navigation_shared_20260707.json`
  passed 8/8 under `fail_once_navigation`. The run recorded route-repair
  pressure, but zero duplicate active-goal, invalid-plan, invalid-JSON,
  backend-unreachable, or rules-response fallback markers.

## Findings

1. The harness previously leaked older ROS events into later cases because it
   assumed the first whitespace token in every log line was the timestamp.
   ROS lines beginning with `[INFO]` broke that assumption. The runner now
   extracts the first Unix-style timestamp from the line body.
2. The planner matched generic `table` labels at the same confidence as the
   specific `work_table` label. With several fixtures loaded, this produced an
   unnecessary location clarification.
3. Some deliverable objects were dropped from `locations.contains` when their
   compact class was `cyc:SpatialThing-Localized`, even when RDF type facts
   identified them as books, phones, or cups.
4. Repeated named people such as multiple ALEX fixtures need disambiguation.
   The source patch uses relation scope first, then a stable fixture namespace
   tie-breaker.

## Source Gate

The following checks passed after the source patch:

```bash
python3 -m py_compile \
  src/planner_llm/planner_llm/planner_engine.py \
  src/planner_common/planner_common/contracts.py \
  .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py

PYTHONPATH=src/planner_common:src/planner_llm \
python3 -m pytest src/planner_llm/test/test_planner_engine.py -q

PYTHONPATH=src/planner_common \
python3 -m pytest src/planner_common/test/test_contracts.py -q

python3 -m pytest \
  .codex/skills/robot-runtime-performance-review/scripts/test_run_active_questionnaire.py -q
```

Results:

- `planner_llm`: 40 passed
- `planner_common`: 35 passed
- runtime-review harness: 7 passed
- `py_compile`: passed

## Rebuilt Runtime Gate

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py \
  --container nao_ros2 \
  --case-set fake_deep \
  --speech-voice-scope group \
  --fake-policy-profile all_success \
  --max-case-wait-sec 90 \
  --global-timeout-sec 900 \
  --out /tmp/nao_fake_deep_round2_all_success_shared_20260707.json

python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py \
  --container nao_ros2 \
  --case-set fake_deep \
  --speech-voice-scope shared \
  --fake-policy-profile fail_once_navigation \
  --expected-turn-pipeline-mode response_first \
  --max-case-wait-sec 120 \
  --global-timeout-sec 1200 \
  --out /tmp/nao_fake_deep_round3_fail_once_navigation_shared_20260707.json
```

Confirmed proof:

- `grounded_context_digest_enabled=false` in the live `/chatbot_llm` node.
- grouped work-table and IIIA kitchen delivery expanded concrete members
  without the old location clarification.
- ordered walk/report no longer exported support locations as completed
  targets in the rebuilt fake-deep run.
- missing-recipient behavior is now a truthful clarification before planner
  handoff.
- `fail_once_navigation` passed the full fake-deep ladder, showing recovery
  without duplicate active-goal rejection or invalid planner JSON.

Still pending:

- `fail_once_pick` and `delivery_blocked` profiles need the same rebuilt proof.
- Subjective report wording still needs a speech holdout, but the rebuilt
  artifacts show the report path asks `chatbot_llm` first and only uses the
  deterministic chain fallback when the chatbot service does not provide text.
