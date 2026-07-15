# Deep Fake/Replan Stabilization Pass (2026-07-01)

## Scope

This pass checked the live `response_first` container after the recent
`chatbot_llm` restructuring, rebuilt a fresh `iiia:nao-critic` overlay, then
reran smoke, environment, and fake-deep probes. The focus was not prompt
wording. The focus was structural evidence: preloaded semantic environments,
SVG operator maps, planner goal lineage, and the first rebuilt fake-deep
baseline.

## Runtime Baseline

The first live container `nao_ros2` was running `iiia:nao` with the expected graph:
`/chatbot_llm`, `/dialogue_manager`, `/planner_llm`, `/nao_orchestrator`,
`/fake_skill_server`, `/report_result_skill_server`, `/kb/knowledge_core`,
`/nao_scene_grounding`, and trace-viewer nodes were visible.

`/chatbot_llm` reported:

```text
turn_pipeline_mode: response_first
```

The initial caveat was that Python imported:

```text
/home/ubuntu/ws/build/chatbot_llm/chatbot_llm/node_impl.py
```

That meant the active process was still using the built copy from the image, not
the patched source tree. A fresh overlay was then rebuilt from `iiia:nao` and
launched as `iiia:nao-critic`. The rebuilt stack imported the new
`dialogue_turn_id` helper, exposed a single `/interaction_trace_viewer`, and
ran in `response_first`.

## Evidence Artifacts

| Artifact | What it shows |
| --- | --- |
| `/tmp/nao_baseline_smoke_20260701.json` | `response_first` smoke run. Simple dialogue and KB query behavior remain usable. Nearby stale planner-gate warnings appear in excerpts, so do not use simple text flags as the final score. |
| `/tmp/nao_environment_baseline_20260701.json` | Semantic fixtures are injected through `/kb/revise`; inventory questions work. Grouped delivery/person binding still needs a rebuilt proof. |
| `/tmp/nao_fake_deep_success_20260701.json` | `fake_policy_profile=all_success` was applied, but the ladder still hit `duplicate active planner goal` because the running container used the old built `chatbot_llm`. |
| `/tmp/nao_smoke_after_rebuild_20260701.json` | Fresh `iiia:nao-critic` smoke run. Five cases completed without duplicate active-goal rejection. |
| `/tmp/nao_environment_after_rebuild_20260701.json` | Rebuilt environment run. Baseline, lab table, and lab-sections inventory questions answered from semantic fixtures. Lab-sections delivery still clarified on the collection location. |
| `/tmp/nao_fake_deep_success_after_rebuild_20260701.json` | Rebuilt fake-deep all-success probe. Duplicate active-goal did not recur. Ordered walk/report failed because planner output did not contain a JSON object. |

## Source Fixes Accepted

### Dialogue-Scoped Goal Lineage

`src/chatbot_llm/chatbot_llm/node_impl.py` now derives the turn id from role,
dialogue id, and request count. This replaces the repeated `__default__:1`
shape that made unrelated questionnaire dialogues collide at planner admission.

Expected effect after rebuild:

- independent user turns should not share `goal_default___1`;
- stale active goals from a prior questionnaire case should not block a new
  unrelated execution request;
- the orchestrator gate remains strict, but receives unique lineage.

### Grounded-Context Test Alignment

`src/chatbot_llm/test/test_planner_handoff_grounded_context.py` now checks the
current canonical location shape, including `role`, `member_count`,
`object_count`, and `person_count`.

### Rqt-Friendly SVG Environments

The preloaded environment SVGs in `src/nao_chatbot/config/preloaded_environment_svgs`
and `docs/artifacts/preloaded_environments` were normalized to positive
centimeter canvases. Text labels were removed from the SVG bodies so the rqt
radar view does not show overlapping names or "operator station" wording.

Scoreable semantic state still comes from `preloaded_environments.json` and
`/kb/revise`; SVGs are operator visual aids.

### Incremental Harness Evidence

`run_active_questionnaire.py` now flushes each active case during long waits
with updated `phase_observations` and log excerpts. This keeps fake-deep runs
inspectable while execution, replan, or speech evidence is still arriving.

## Validation Run Locally

```text
python3 -m py_compile src/chatbot_llm/chatbot_llm/node_impl.py \
  src/chatbot_llm/chatbot_llm/planner_request_adapter.py

PYTHONPATH=src/chatbot_llm:src/planner_common:src/kb_skills \
python3 -m pytest \
  src/chatbot_llm/test/test_turn_engine.py \
  src/chatbot_llm/test/test_planner_handoff_grounded_context.py \
  src/chatbot_llm/test/test_planner_request_adapter.py -q
```

Result:

```text
110 passed
```

## Remaining Gaps

| Gap | Current state | Next gate |
| --- | --- | --- |
| Duplicate active planner goal | Passed after rebuild. Smoke, environment, and fake-deep probes did not reproduce the previous rejection. | Keep as a holdout in future full-ladder runs. |
| SVG radar background | Files are installed, valid, and now rqt-oriented. | Select `baseline_table.svg` or `lab_sections.svg` in rqt after rebuild and confirm the radar background changes. |
| Grouped delivery from named locations | Improved but still mixed. The 2 Jul clean rebuilt focused run no longer reproduced the old "which location" clarification, but the accepted model plan targeted `codex_lab_table_section` as the delivered object. A post-run KB query showed cup/manual/phone still on the table section while the table section gained `isAt codex_lab_alex`. | Source now treats grounded "every object from location to recipient" as a member-expansion planner contract before model planning. Rerun the exact work-table delivery prompt after a clean rebuild, then query cup/manual/phone for stale source relations. |
| Ordered multi-object planner JSON | New rebuilt fake-deep blocker. "Walk to every object on the table..." reached planner admission, then failed because model output did not contain a JSON object. | Source now has a bounded grounded ordered-walk fallback after invalid planner JSON. It still uses the LLM first and only falls back to grounded object ids after parser failure. |
| Failure-induced replan | Prior 30 Jun logs show one fail-once navigation replan, but the artifact underreported late evidence. | Rerun `fail_once_navigation`, then add `fail_once_pick` and `delivery_blocked` with clean per-case lineage. |
| Manual KB add format | Structured triples pass; vague natural adds can still fail. | Add a clarification gate later, without prompt churn, for under-specified add/remove/revise requests. |
| Grounded-context digest counts | One run showed `Objects (0)` while JSON contained visible domain objects. | Source now keeps domain objects that also carry spatial materialization RDF types. Rerun a screenshot-quality digest after rebuild. |
| Skill-bound KB post-effects | Source-patched, live proof still pending. The 2 Jul run showed the current effect path can verify mutations, but the wrong target was selected before effects were applied. | Fake `bring_object` now removes known-source `oro:isOn`, `oro:isAt`, and `oro:isIn` relations before adding the recipient relation. The orchestrator verifies skill-emitted KB effects through `/kb/query` when available and fails the skill step if remove/add/update post-conditions do not hold. Rerun grouped delivery after the planner member-expansion rebuild, then query each delivered object. |

## Next Runtime Command

After the next source patch:

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py \
  --container nao_ros2 \
  --case-set fake_deep \
  --speech-voice-scope group \
  --fake-policy-profile all_success \
  --expected-turn-pipeline-mode response_first \
  --global-timeout-sec 420 \
  --max-case-wait-sec 20 \
  --out /tmp/nao_fake_deep_success_post_rebuild_20260701.json
```

Then repeat with:

```text
fail_once_navigation
fail_once_pick
delivery_blocked
recipient_missing
```

Do not raise the deep fake/replan score until the rebuilt runtime passes the
all-success ladder without duplicate active-goal rejection.
