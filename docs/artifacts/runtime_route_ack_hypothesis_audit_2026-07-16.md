# Runtime Route and Acknowledgement Hypothesis Audit, 16 July 2026

## 1. Target contract

This audit evaluates the response-first path for requests whose wording can
sound like a capability question while still being admitted as an immediate
robot action. The target contract is:

- the chatbot may acknowledge an accepted action before execution;
- the acknowledgement must not claim that the action has already completed;
- the final route and planner request must preserve enough structured intent to
  explain why execution was admitted;
- the orchestrator must provide execution evidence before the chatbot produces
  the user-facing report;
- the final report must remain chatbot-owned and must describe observed results;
- ambiguity must be measured and surfaced before it is considered a fallback
  defect.

The audit does not add a fallback, alter the response prompt, or change the
route policy. It is specifically bounded to the apparent mismatch in the
interaction containing “Can you move your head in all directions?”.

## 2. Baseline evidence

- Runtime container: `nao_ros2`.
- Runtime image: `iiia:nao-deslop-20260716-v15-target-recovery-posture-open-loop-ready`.
- Runtime image ID: `sha256:feb0b7227db4415e680e68146b6cc0f870708cb05873afadbfa54334797df0a4`.
- Launch profile: the canonical `response_first` profile, with the vLLM
  Qwen3-VL endpoint and fake skills enabled.
- Preflight: required nodes were unique, lifecycle nodes were active,
  KnowledgeCore was ready, and chatbot/planner LLM preflight failures were
  zero before scoring.
- Main questionnaire: 21/21 cases passed their configured trajectories.
  The head case had one `route_repair` marker, but no planner, execution,
  report, or speech failure marker.
- Deep fake questionnaire: 9 cases completed, 5 strict passes, 1 degraded
  recovery case, and 3 strict failures. The strict failures were associated
  with missing target-selection evidence or unnecessary clarification in
  multi-step cases. `report_result_fallback` remained zero in the recorded
  cases.
- Direct posture probe on this image succeeded through `open_loop` while the
  robot was disconnected. The later speech posture case also produced a
  planner-owned `perform_motion` step followed by a chatbot-resolved
  `report_result`.

The exact head trace is:

1. User input: `Can you move your head in all directions?`
2. Chatbot acknowledgement: `Yes, I can move my head in all directions. How
   can I assist you?`
3. Chatbot trace: `ROUTE_RESOLVED route=execution intent=-
   source=llm_response_route_repair confidence=0.95`.
4. Chatbot trace: a planner request was published for the same dialogue turn,
   with the goal text `move your head in all directions?` and an empty
   `normalized_intents` list.
5. Planner and orchestrator executed the head-motion sequence successfully.
6. The orchestrator requested a system execution-report turn and logged
   `Resolved report_result text via chatbot`.
7. Final speech: `I have moved my head in all directions and centered it back.`

The source flow supports this ordering. `response_first` first produces a
verbal acknowledgement, then resolves route and intent, then admits planner
execution. A later system turn owns execution-report wording. The
acknowledgement is therefore not evidence that the robot has already moved.

The remaining defect is visible in step 3 and step 4: execution was admitted
after route repair without a normalized intent label. The planner inferred the
action from goal text. The desired action succeeded, but the handoff does not
fully explain which structured intent authorized it.

## 3. Approach registry

| ID | Hypothesis or approach | Discriminating evidence | Status |
|---|---|---|---|
| H1 | The two utterances are an intended response-first acknowledgement and post-execution report pair | Same dialogue lineage, planner request between them, successful execution feedback, and a later system report turn | Supported; acknowledgement ordering is not a defect |
| H2 | The model intended a capability-only answer and execution was admitted accidentally | The raw intent-stage payload is not retained in the current live evidence, so the model's private distinction cannot be proven from wording alone | Blocked pending raw structured stage capture |
| H3 | Route repair converted an executable-looking user request to `execution` but failed to preserve a normalized intent | Live trace reports `route=execution`, `intent=-`, `source=llm_response_route_repair`, and the planner request has `normalized_intents=[]` | Supported |
| H4 | Planner free-text inference masks the missing chatbot intent and makes the runtime appear healthier than its contract is | The planner executed from the goal text despite the empty normalized-intent list; the main case passed while the handoff evidence was incomplete | Supported |
| H5 | The apparent mismatch is caused by duplicate speech or wrong-turn correlation | The trace has one acknowledgement, one execution lineage, one report turn, and one final report; the harness captured both utterances in order | Rejected for this case |
| H6 | More route or acknowledgement fallbacks are required | Main response-first execution and report behavior already passed; deep fake failures were target-selection/clarification cases, not report fallback failures | Rejected |
| H7 | The wording alone can consistently distinguish a trivial capability question from an immediate action | “Can you move...” can naturally be either a capability query or a polite command; the current route policy already treats explicit capability formulations separately | Rejected as a sufficient rule; use structured admission evidence |

## 4. Discriminating probes and results

1. **Exact live trace reconstruction.** The head case was reconstructed from
   dialogue-manager speech, chatbot route, planner request, planner feedback,
   orchestrator report resolution, and closed captions. This confirms the
   acknowledgement/report phase boundary and exposes the empty intent handoff.

2. **Source-flow inspection.** `turn_engine.py` applies response-first in two
   stages and invokes a separate system execution-report path. The route
   repair path can set the final route to execution while retaining no usable
   `resolved_intent`. `planner_request_adapter.py` then derives
   `normalized_intents` from the turn result, allowing an empty list to reach
   the planner.

3. **Focused chatbot tests.** Existing tests cover capability-only dialogue,
   missing-route execution repair, route-safe acknowledgement sanitization,
   repeat-action promotion, and prior-execution questions. They establish
   that the stack intentionally preserves conversational capability answers
   while protecting explicit execution routes. They do not yet assert a
   diagnostic event for `execution` with an empty normalized intent.

4. **Main runtime holdout.** The head sequence passed with six successful
   motion steps and a chatbot-authored final report. This is evidence against
   changing the natural acknowledgement style solely because it precedes
   execution.

5. **Deep fake holdout.** Multi-step failures clustered around missing
   `target_selection` evidence and complete-context clarification. No
   `report_result_fallback` was observed. This separates the current head
   concern from the existing deep-fake planning weaknesses.

## 5. Adversarial audit

- **Ownership:** dialogue-manager speech ownership, chatbot route/wording,
  planner execution planning, orchestrator dispatch, and chatbot report
  wording remain separate in the observed head path.
- **No fabricated completion:** the initial acknowledgement does not say the
  head has already moved. The completion claim appears only after execution
  feedback and the system report turn.
- **No duplicate speech:** the trace contains one pre-execution acknowledgement
  and one post-execution report. These are different phases, not duplicate
  reports.
- **Hidden policy risk:** allowing an empty `normalized_intents` list means the
  planner can infer an executable action from free text. That is a real
  contract-visibility weakness even when the inferred action is correct.
- **Fallback pressure:** adding another lexical fallback would make the
  distinction less observable and could turn a natural acknowledgement into a
  forced template. No such change is justified by this evidence.
- **Prompt discipline:** no prompt mutation is accepted from this audit. A
  prompt change would require a bounded SkillOpt baseline, mutation batch,
  train/holdout comparison, and explicit acceptance record.

## 6. Decision: accept the acknowledgement contract, retain a bounded handoff

The response-first acknowledgement pattern is accepted for the demonstrated
case. The user-facing wording should not be penalized merely because it says
“I can...” before the action. The final report is the completion statement.

The `execution + intent=- + route_repair` combination is retained as a bounded
handoff defect. It should be made visible and measured before any behavior
change is considered. The preferred next implementation is an
observability-only contract check that records:

- the model-provided route;
- the final route;
- the route-repair source;
- the normalized intent list handed to the planner;
- the dialogue/goal lineage; and
- whether execution feedback and a post-execution report followed.

This check must not block or rewrite an otherwise valid execution in the first
iteration. A later admission policy can be evaluated only after paired
capability/action holdouts show that the missing intent is harmful rather than
merely incomplete telemetry.

## 7. Residual risk and next probe

The remaining uncertainty is whether the empty intent came from the response
stage, the intent stage, route repair, or the planner-request projection. The
next probe should capture redacted structured payloads at the existing trace
boundary for paired prompts:

- `What can you do with your head?` (capability question);
- `Can you move your head in all directions?` (ambiguous polite request);
- `Please move your head in all directions now.` (explicit action);
- `How many directions did you move your head?` (prior-execution question).

For each pair, require the evidence tuple `(route, normalized_intents,
goal_id, plan_id, execution feedback, report turn)`. Score the acknowledgement
and final report separately. The runtime should classify the ambiguous case as
acceptable when the tuple is complete and the action was admitted once, while
reporting an intent-handoff warning when execution occurs with no normalized
intent. No new fallback is needed unless this holdout demonstrates an actual
wrong action or fabricated completion.

## 8. v18/v19 clean-overlay evidence

The first correction added non-blocking `ROUTE_INTENT_HANDOFF` and
`ROUTE_INTENT_GAP` traces. The second correction addressed the
target-selection omission exposed by those traces. When the chatbot has
repaired the route but has not supplied a selection, the existing grounded
selection derivation is now allowed to run. Healthy LLM-declared selections
remain authoritative, and the derived selection still passes grounded-context
and planner admission validation.

### v18 checkpoint

- Image: `iiia:nao-deslop-20260716-v18-motion-recovery-route-observability`
- Digest: `sha256:dee96e09e27ae8d102aab8626d22a1e3958227a0b1a45cf7b414eb0dcb7eb09d`
- Image tests: 261 focused chatbot/planner tests passed.
- Route-ack matrix: 4/4 passed. The previously failing explicit all-directions
  request recovered to five validated motion steps followed by `report_result`.
- Main suite: 20/21 passed. The only failure was missing `target_selection`
  evidence in the ordered-walk case.
- Deep fake: 6/9 passed, 1 degraded, 2 failed. Both strict failures were
  complete-context grouped deliveries that clarified before execution for the
  same missing-selection reason.

### v19 acceptance overlay

- Image: `iiia:nao-deslop-20260716-v19-route-repair-target-selection`
- Digest: `sha256:41e682cdd9da48e92f072687e3c82a97e38d367a3aef08d63075ca684bf7d95e`
- Image tests: 262 focused chatbot/planner tests passed.
- Preflight: all required nodes unique and active, KnowledgeCore ready, and
  chatbot/planner LLM preflight failures zero. Host/container source hashes
  matched for `planner_request_adapter.py` and `planner_engine.py`.
- Route-ack matrix: 4/4 passed. The polite request retained its natural
  acknowledgement and final report; the explicit request executed and
  reported correctly. `report_result_fallback` was zero in every case.
- Environment suite: 11/11 passed. Work-table, kitchen, IIIA-kitchen, and
  gold-apple delivery selections carried exact grounded member and recipient
  ids, and post-effect follow-ups passed.
- Main suite: 21/21 passed. The ordered walk carried grounded target
  selection, completed per-target reporting, and no report-result fallback.
- Deep fake: 9/9 strict pass in the corrected rerun. Both prior delivery
  failures passed, and `fake_deep_missing_object_recovery` now applies a
  case-local `find_object=always_fail` policy. The trace shows failed target
  evidence, planner replan, and chatbot-owned recovery speech asking the user
  to confirm the scene. `report_result_fallback` remained zero.
- Posture ablation: 4/4 passed for stand, sit, kneel, and sit-then-stand.
  With the robot disconnected, head and posture execution used the open-loop
  path. The NAOqi driver later exited with its known disconnected-driver
  failure, without preventing the fake/open-loop motion results.

The v19 overlay is the current `iiia:nao` checkpoint. The named v18 tag,
`iiia:naofrozen`, `iiia:base`, and
`iiia:nao-deslop-20260714-v8-kb-spatial` remain preserved. The container was
used for the clean runtime evidence and is closed after the run.

## 9. Remaining probe

The absence-recovery seam is now covered by a case-local fake policy rather
than the general success-oriented fake policy. Future failure-profile runs
should preserve this distinction and retain the post-terminal speech wait.
