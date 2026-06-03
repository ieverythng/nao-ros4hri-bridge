# TFM Validation and Experiment Protocol

Date: 2026-06-03
Audience: TFM writing, supervisor review, implementation handoff
Scope: Validation methodology, experiment design, metrics, ablations, and evidence capture.

---

## Purpose

This document converts the implementation into a thesis validation protocol. It is intended to support the methodology, results, and discussion chapters by making experiments repeatable and by naming the evidence that each run must collect.

## Validation Thesis

The system should be evaluated not only by whether the robot completes a task, but by whether the architecture preserves the intended boundaries while doing so. A successful planner-mediated run should show correct routing, valid contract generation, deterministic admission, skill dispatch, feedback publication, and a single user-facing speech authority.

## Research Questions Covered

| Research question | Observable evidence |
|---|---|
| Can planner-mediated execution improve multi-step task handling? | Plan validity, step completion, replanning events |
| Does the skill registry reduce invalid robot-specific outputs? | Invalid-step rate, unsupported-skill rejection rate |
| Does execution feedback improve recovery? | Replan success, clarification success, failure explanation quality |
| Do architecture boundaries remain inspectable? | Trace completeness, contract validity, ownership violations |
| What limitations remain under real HRI constraints? | Latency, duplicate speech, grounding staleness, model failure cases |

## Scenario Matrix

| Scenario group | Example command | Expected behavior | Primary metrics |
|---|---|---|---|
| Dialogue-only | hello | Remains dialogue, no planner request | false planner-route rate |
| Knowledge query | what can you see? | Uses KB/scene answer, no unnecessary execution | route accuracy, answer groundedness |
| Atomic execution | look left | Single validated skill step | completion, latency |
| Grounded look-at | look at the person | Uses visible person target and `look_at` step | target correctness, person/object separation |
| Multi-step task | scan for a cup and tell me what you found | Planner emits ordered steps | plan validity, step success |
| Failure recovery | target disappears before execution | Feedback triggers replan or failure dialogue | recovery rate |
| Clarification | ambiguous target | Planner asks for missing slot | clarification precision |
| Cancellation/update | stop or changed goal | Goal transition is explicit | supersede/cancel correctness |

## Runtime Evidence to Capture

- user utterance and timestamp;
- chatbot route and `user_intent` JSON;
- planner request envelope and `Intent.data`;
- grounded context snapshot (`knowledge_snapshot`, `scene_summary`, `state_t0`);
- planner output JSON;
- orchestrator validation result;
- dispatched action server and step id;
- execution feedback JSON;
- planner dialogue act, if emitted;
- final user-facing utterance and speech owner.

## Quantitative Metrics

| Metric | Definition | Interpretation |
|---|---|---|
| Route accuracy | Fraction of turns routed to the intended class | Tests chatbot planner gating |
| Plan validity | Fraction of planner outputs accepted by orchestrator | Tests planner contract discipline |
| Unsupported step rate | Fraction of steps rejected as unknown or invalid | Tests registry grounding |
| Task completion | Fraction of scenarios completed | Tests end-to-end behavior |
| Recovery success | Fraction of failures resolved by replan or clarification | Tests closed-loop supervision |
| Duplicate speech rate | Fraction of turns with multiple competing utterances | Tests dialogue ownership |
| Grounding correctness | Fraction of selected targets matching scene/KB evidence | Tests perception-to-planner context |
| Median latency | Time from user utterance to first action or answer | Tests HRI usability |

## Qualitative Analysis Dimensions

Each trace should explain why the chatbot selected the route, what grounded context was available at T0, why the planner selected the steps, whether the orchestrator accepted or rejected each step, how feedback changed planner state, whether the final user-facing response had one owner, and what the trace reveals about limitations of the architecture.

## Ablation Plan

| Ablation | Change | Hypothesis |
|---|---|---|
| Direct execution only | Disable planner-mediated path | Multi-step and failure recovery become less robust |
| No grounded context | Remove `knowledge_snapshot`, `scene_summary`, `state_t0` | Target selection and perception tasks degrade |
| No execution feedback | Suppress feedback to planner | Replanning and failure explanations degrade |
| Minimal registry | Hide some skill metadata | Unsupported or underspecified plan steps increase |
| Chatbot wording disabled | Direct planner dialogue only | Completion wording may become less conversational but easier to attribute |

## Validation Gates Before Experiments

1. Run contract unit tests for `planner_common`, `chatbot_llm`, `planner_llm`, and `nao_orchestrator`.
2. Run registry consistency checks so planner-visible skills match canonical AB entries.
3. Run ROS4HRI structural audit to confirm changed packages and ownership-sensitive files.
4. Launch the sim profile and verify expected action servers are available.
5. Execute one dialogue-only, one knowledge-query, one atomic action, and one failure case before collecting formal results.

## Experiment Log Template

| Field | Value |
|---|---|
| Experiment ID | `EXP-YYYYMMDD-###` |
| Scenario group |  |
| User utterance |  |
| Expected route |  |
| Observed route |  |
| Planner request published | yes/no |
| Planner output valid | yes/no |
| Executed steps |  |
| Feedback events |  |
| Final speech owner |  |
| Outcome | success / failure / partial |
| Notes |  |

## Threats to Validity

The main internal threats are prompt sensitivity, incomplete test coverage for live ROS timing, and possible mismatch between simulated and robot execution. External threats include model availability, cloud/local model differences, and the fact that the validation environment is centered on the NAO stack. Construct validity depends on whether metrics such as plan validity and route accuracy genuinely capture user-perceived task success. The thesis should therefore combine quantitative tables with trace-based qualitative analysis.

## Expected Results Narrative

A strong thesis result would show that planner-mediated execution does not merely add complexity. It should demonstrate that the planner path improves inspectability, makes multi-step execution more explicit, and creates recovery opportunities through feedback. Even when the planner fails, the architecture should make the failure attributable: wrong route, stale grounding, invalid plan, missing skill, execution failure, or dialogue ownership issue.
