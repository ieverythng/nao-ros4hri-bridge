# Runtime Seam Deslop Review

**Date:** 2026-07-14  
**Profile:** `response_first`, grounded-context digest disabled, Qwen3-VL 30B  
**Candidate base:** `iiia:nao` (`bb62ec1a4589`)  
**Verdict:** Main runtime accepted; complete deep-fake score pending endpoint recovery

## Executive Verdict

The main SV-facing seams are materially more reliable. Natural-language KB
mutation, strict spatial effects, grouped delivery, people/object separation,
head motion, ordered visits, and chatbot-owned completion wording all passed
live probes. No `report_result` emergency renderer was observed in accepted
success cases. The rebuilt base image matched all 56 changed runtime files and
the inspected colcon-built modules.

The review is not a complete deep-fake acceptance. The last source change adds
retry-exhausted provenance for quantified object intent failures. Its 156-test
source gate passed, but the clean v7 runtime could not start `planner_llm`
because the external vLLM endpoint refused connections.

## Scores

| Area | Score | Evidence |
| --- | ---: | --- |
| Dialogue and route safety | 8.2/10 | Main dialogue passed; residual route repair remained measurable |
| KB query and mutation | 9.0/10 | Query isolation and add/revise/remove passed; cleanup uncontaminated |
| Grounded people and objects | 8.6/10 | Grouped delivery and ordered visits passed; synthetic `me` correctly clarified |
| Planning and execution | 8.4/10 | Main execution passed; bounded retries work; full forced-failure rerun pending |
| `report_result` | 8.8/10 | Natural grouped closure, structured evidence, zero accepted-case report fallback |
| Qualified overall | **8.6/10** | Main runtime accepted, deep-fake completion withheld |

## Accepted Evidence

- `/tmp/nao_deslop_v4_main_full_valid_20260714.json`: 14 cases passed before
  the suite-level timeout marker.
- `/tmp/nao_deslop_v4_main_tail_20260714.json`: five direct passes; the two
  original failures were isolated to an incorrect synthetic-recipient oracle
  and an over-strict visible-object set comparison.
- `/tmp/nao_sv_image_targeted_regressions_20260714.json`: both corrected cases
  passed on the canonical image.
- `/tmp/nao_deslop_v4_architecture_holdout_20260714.json`: add, revise, query,
  remove, missing-recipient, and grounded-recipient behavior passed.
- `/tmp/nao_deslop_v4_environment_holdout_20260714.json`: 3/3 environment and
  grouped-delivery cases passed with natural closure.
- `/tmp/nao_deslop_v4_kb_mutation_20260714.json`: natural-language KB add passed
  with zero fallback markers.

## Findings

1. The questionnaire retained transient-local publishers for prior voices,
   causing false speech-ingress failures. It now retires prior publishers while
   preserving same-group continuity.
2. `intent_max_tokens=64` constrained the expanded target-selection schema.
   The runtime default is now 256.
3. Planner KB mutations now reject malformed triples before dispatch and retry
   with exact validation errors.
4. Quantified target selection remains LLM-owned. Deterministic derivation is
   eligible only after the single intent retry is exhausted and is traced as
   semantic salvage.
5. "Bring me" is not executable when the synthetic voice has no grounded HRI
   person. Early clarification is the accepted behavior; partial navigation or
   pickup before that clarification is rejected.

## Residual Risks

- Route repair still appeared in several semantically successful dialogue and
  motion turns. It is visible but not yet eliminated.
- The complete `fake_deep` `all_success` profile and
  `fail_once_navigation` recovery profile must be rerun.
- Physical NAO convergence was unavailable. Head motion used the declared
  open-loop path; NAOqi connection failures were not scored as semantic faults.

## Qwen3-Coder Cloud Supplemental Run

The lab vLLM host remained reachable by ICMP but refused TCP connections on
port 8004. The unchanged v7 image was therefore relaunched against the host
Ollama endpoint with `qwen3-coder:480b-cloud` for chatbot response, intent, and
planner stages. Both required model preflights passed on the first attempt.

The six-case smoke set passed dialogue, direct KB visibility, injected-object
recall, simple motion, composite head-motion plus waving, and reflective
follow-up. Both execution cases reached terminal feedback. No chatbot,
planner, route-repair, report, or unreachable-model fallback event was counted.
Cloud inference added several seconds per LLM stage but remained within the
configured 60-second model timeout.

The architecture mutation sweep passed all deterministic postconditions:

- natural-language add produced the requested type, name, and green color;
- revise replaced green with blue;
- the subsequent question returned NOVA as a blue cube without mutating it;
- remove left no facts for the marker;
- grouped kitchen delivery moved both selected objects to the named recipient.

A differential spatial test exposed a remaining mutation defect. A book was
inserted one fact at a time, while a cup and phone were inserted atomically.
The model correctly reported their initial shelf, table, and lab relations.
Adding `oro:isAt park` for all three did not retract the book and cup's base
`oro:isOn` assertions. KnowledgeCore continued to infer their old `isAt` and
`placeOf` relations, and the model consequently reported combinations such as
"on the shelf in the park".

An explicit natural-language relocation reached the planner. It added all
three park relations and attempted to remove the prior locations. The remove
postcondition failed because the plan targeted inferred `isAt` facts rather
than the owning `isOn` assertions. Replanning repeated the same semantic shape
and then issued a truthful request for help. Retracting the two base `isOn`
assertions removed all stale spatial closure, after which the unchanged model
answered that all three objects were in the park.

The accepted remediation is canonical spatial replacement at the KB mutation
boundary. Explicit `kb_revise` now resolves and retracts existing base and
derived relations across the `isOn`, `isIn`, and `isAt` family before asserting
the new location. This reuses the cleanup contract already applied to successful
skill post-effects and does not add prompt wording or response fallbacks.

The focused orchestrator gate passed 92 tests. A clean
`iiia:nao-deslop-20260714-v8-kb-spatial` rebuild then replayed the former
three-object failure against Qwen3-Coder Cloud. The planner admitted one
version-1 `kb_revise` plan and completed it without a failed step, replan, or
help request. Direct queries found one park relation for each object and no
remaining shelf, table, lab, `isOn`, `isIn`, or derived `placeOf` relation. The
follow-up answer was: "ATLAS, ORBIT, and PULSE are currently at the park."

Supplemental artifacts:

- `/tmp/nao_qwen3_coder_smoke.json`
- `/tmp/nao_qwen3_coder_kb_architecture_sweep.json`
- `/tmp/nao_qwen3_coder_kb_staggered_stress.json`
- `/tmp/nao_qwen3_coder_kb_explicit_relocation.json`
- `/tmp/nao_qwen3_coder_kb_base_relation_retract.json`
- `/tmp/nao_qwen3_coder_post_kb_stress_snapshot.json`
- `/tmp/nao_qwen3_coder_v8_spatial_relocation.json`

## Reopen Command

After rebuilding `iiia:nao` with the final source and confirming `/v1/models`:

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py \
  --container nao_ros2 --case-set fake_deep --speech-voice-scope group \
  --expected-turn-pipeline-mode response_first \
  --fake-policy-profile all_success --global-timeout-sec 1800 \
  --out /tmp/nao_fake_deep_success_final.json
```

Repeat with `--fake-policy-profile fail_once_navigation`. Acceptance requires
the ordered-walk target selection, failure/replan lineage, truthful closure,
zero duplicate speech, and uncontaminated fixture cleanup.

## 15 July Frozen-Image Completion Audit

A newly spawned `nao_ros2` container was confirmed to use the exact frozen
image id `b02fe04c12b9`. The v8 alias and `iiia:naofrozen` resolved to the same
id, the container had zero
restarts, one integrated launch owner, the expected Qwen/Ollama arguments, and
matching critical source hashes. The observed behavior was therefore not stale
or divergent container code.

The turn "Walk to all the entities" exposed an aggregate completion regression.
Three selected object-navigation steps succeeded and the final execution
feedback contained a populated `plan_outcome_summary`. The `PlannerSupervisor`
retained only the latest step summary and payload when constructing
`notify_completion`. Its dialogue act consequently described only
`phone_dzmgf`, and the chatbot naturally repeated that
last-step evidence.

Source now preserves `ExecutionFeedback.plan_outcome_summary` in
goal-scoped supervisor state and includes it in planner dialogue-act context.
The audit found two further losses in the same vertical path. The dialogue
manager's planner-completion projection and the chatbot's system-turn extractor
both omitted the aggregate field. Focused red-green tests now require the field
to survive each relay. The resulting source change does not affect planning,
execution, KnowledgeCore mutation, prompts, or fallback policy. The widened
planner/common/dialogue/chatbot gate passes 192 tests. Live acceptance remains
pending a clean rebuild; the running frozen container was not modified or
restarted during diagnosis.

The mounted-source validation also reproduced a stale-bytecode hazard. A test
container initially imported an older cached function body despite resolving
the module path to the mounted workspace. Running with an isolated
`PYTHONPYCACHEPREFIX` loaded the current source and passed. This does not change
the frozen image diagnosis, but overlay-based development runs should isolate
or clear Python bytecode and verify imported module hashes before comparing
runtime behavior.

Fresh containers from the same image remain behaviorally variable because they
start with new KnowledgeCore contents, regenerated HRI entity ids, new dialogue
history, and stochastic remote-model responses. Image identity guarantees code
and installed assets, not identical external state or model samples. Scored
comparisons must freeze launch arguments, fixtures, model configuration, and
case order in addition to the image digest.
