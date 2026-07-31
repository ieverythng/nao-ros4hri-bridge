# Runtime Review - `iiia:nao-runtime-v34-final-frozen-review`

**Standing:** improving, with a qualified primary model and bounded residual model variance.

**Score: 8.4/10**

The score applies to the response-first, JSON-only grounded-context campaign using `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ`. The active v34 snapshot was `ready_for_semantic_scoring`, every required lifecycle owner was active, KnowledgeCore was ready, both LLM preflights had zero failures, and the deduplicated fallback count was zero.

Deterministic source tests and live runtime stress cases are reported separately. Unit tests establish contract behavior. They do not count as robot-runtime stress evidence.

## Runtime tuple

| Field | Locked value |
| --- | --- |
| Image | `iiia:nao-runtime-v34-final-frozen-review` |
| Image digest | `sha256:bb82c158092a69870dae48272b3b20fd8cd4ecfe97a0e61a2253a2cca00c663e` |
| Chatbot / intent / planner model | `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` |
| Endpoint | `http://10.7.138.215:8004` |
| Pipeline | `response_first` |
| Grounding projection | JSON only, digest disabled |
| Chatbot sampling | temperature 0.2, top-p 0.9, top-k 0, min-p 0, neutral penalties |
| Planner sampling | temperature 0.1, top-p 1.0, top-k 0, min-p 0, neutral penalties |
| Token budgets | response 192, intent 256, planner 800 |
| Timeouts | first 60 s, response 30 s, intent 20 s, planner 45 s |
| Object detection | disabled intentionally for the cool profile |

## Bounded ablation result

| Cell | Result | Decision |
| --- | --- | --- |
| A0 application sampling | 5/6 smoke semantics; median chatbot response 3.89 s | Selected |
| A1 Qwen published sampling | 5/6; median 7.28 s | Rejected, slower without semantic gain |
| A2 deterministic sampling | 5/6; median 4.16 s | Rejected, no semantic gain |
| A3 256/256/1024 tokens | Maximal coverage still failed and latency increased | Rejected |
| A4 60/30/20/45 timeouts | Smoke completed without transport fallback | Selected |

The shared smoke failure was Qwen3-VL saying that three directions had been used after four head motions. The execution trace itself contained all four motions.

## Critical findings

1. No critical false-success, fabricated KB effect, duplicate active goal, invalid planner JSON, or backend-unreachable event occurred in the final snapshot.
2. Missing-recipient behavior is variable. One run asked the correct question. Another planner-routed run initially spoke an execution commitment before clarification. A later direct-dialogue run proposed substituting the currently visible person for absent MORGAN. The tightened harness now rejects a planner clarification unless correlated clarification wording is spoken.

## Grounding and KnowledgeCore

1. Environment fixtures passed 11/11. This includes grouped location delivery, IIIA kitchen delivery, gold-apple handoff, role separation, and follow-up state queries.
2. The stateful KB chain passed 7/7. Add/revise/query, delivery post-effects, moving the remaining object, and final mixed queries remained coherent.
3. Ordered deep stress omitted the phone once from an otherwise complete `every object` selection. The same four-target case selected all members under fail-once navigation. This is model variance, not missing fixture evidence.

## Planner and execution findings

1. A deterministic planner defect was fixed: a validated `operation=visit` selection paired with model-derived `look_at` and `inspect_area` intents could not enter target-selection recovery. v34 normalizes those aliases to navigation only when the authoritative visit operation lacks a navigation intent. Rich objectives that explicitly include navigation and look-at remain protected.
2. Fail-once navigation passed with observed failure, plan version advancement, four targets, recovery execution, and final closure speech.
3. Missing-object recovery truthfully performed two failed searches and asked for help, but did not produce terminal closure. This remains a supervision/closure degradation.
4. Two maximal capability composites were rejected before execution because model-authored plans did not cover the requested actions. The stack spoke truthful failure and did not partially execute them.

## Checks passed

- Deterministic source gates: planner `103 passed, 1 skipped`; runtime-review harness `72 passed`; chatbot behavioral suite from the same source campaign `291 passed, 1 skipped`.
- Compilation, skill registry consistency, ROS4HRI ownership audit, and `git diff --check` passed.
- Live main harness: 21/21, with two manual wording/closure downgrades retained below.
- Live environment: 11/11.
- Live KB stress: 7/7.
- Live v34 robustness: 12/15 across three frozen repetitions (5/5, 4/5, 3/5). Failures were one dropped report policy and three incomplete universal target selections. All were rejected or executed only over explicit selected members; no fabricated completion crossed the executor.
- Live fake-deep all-success: 7 pass, 1 degraded, 1 fail.
- Live fail-once navigation: pass with replan.
- Live capability-extreme: 5 pass, 2 fail.
- Final snapshot: semantic preflight ready and zero deduplicated fallback events.

## E2E questionnaire

- Simple dialogue: **pass**.
- KB query dialogue: **pass**.
- Simple skill execution: **pass**.
- Composite skill execution: **degraded**, because grouped delivery lacked terminal closure in one main run and maximal clarification wording did not match the planner act.
- Simple fake-skill scenarios: **pass**.
- Composite fake-skill scenarios: **degraded**, with 7/9 all-success cases passing and truthful handling of both residual failures.

## Ranked model table

| Rank | Model | Role | Evidence-backed status |
| ---: | --- | --- | --- |
| 1 | `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` | Primary | Qualified at 8.4/10 on vLLM. Strong KB, delivery, recovery, and ordinary composites. Reserve caution for universal selection, maximal plans, and absent-recipient wording. |
| 2 | `QuantTrio/Qwen3.6-35B-A3B-AWQ` | vLLM reserve | Startup preflight passed during its valid endpoint window, but no complete locked runtime score exists because the endpoint changed models. Requalify before use. |
| 3 | `qwen3-coder:480b-cloud` | Ollama reserve | Strong prior stack evidence, but the formal run ended at explicit HTTP 429. Resume after account limits reset. |
| 4 | `nemotron-3-super:cloud` | Ollama emergency reserve | Both preflights passed and intents were preserved, but maximal planner validation failed. Suitable for narrow dialogue/intent probes, not qualified planning. |
| 5 | `gemma4:31b-cloud` | Ollama compatibility reserve | Historically usable and fast, but the recent maximal test produced malformed intent JSON and lost target selection. Not qualified for the planner role. |
| 6 | `gemma4:cloud` | Ollama last resort | Preflight passed, but route repair reduced a maximal request to partial navigation/report behavior. Do not use for scored execution. |

## Next probes

- Rerun the missing-recipient case three times with the tightened speech observer when evaluating another model.
- Require one terminal user-facing closure after grouped delivery and after missing-object help/failure.
- Requalify Qwen3.6 only when `/v1/models` and named completion both prove the model continuously for the full run.
- Run the same frozen capability-extreme set against the next candidate. Do not tune prompts between candidates.

## Evidence

- `final_snapshot.json`
- `main_v33.json`
- `environment_v33.json`
- `kb_stress_v33.json`
- `robustness_v34_r1.json`
- `robustness_v34_r2.json`
- `robustness_v34_r3.json`
- `missing_recipient_v34.json`
- `fake_deep_all_success_v34.json`
- `fail_once_navigation_v34.json`
- `capability_extreme_v34.json`
- `v34_launch.log.gz`

Summary: 6 findings - critical 0, grounding/KB 1, model behavior 3, runtime closure 2, observability 0 unresolved after harness repair.
