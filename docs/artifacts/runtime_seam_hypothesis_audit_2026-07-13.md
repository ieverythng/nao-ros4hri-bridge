# Runtime Seam Hypothesis Audit, 13 July 2026

## Target contract

The live stack must preserve ROS4HRI speech ingress, chatbot-owned routing and
wording, planner-owned executable plans, orchestrator-owned deterministic
dispatch and feedback, AB=1 execution evidence, and exactly one coherent
user-facing closure. Fresh KnowledgeCore entities and relations must remain
usable after long dialogue contexts. Complete grounded requests must execute
without unnecessary clarification, while missing targets must clarify rather
than fabricate success.

This audit is bounded to the main questionnaire, environment/KB cases, and deep
fake-skill recovery cases. It does not change prompts or introduce new
deterministic phrase recognizers.

## Baseline evidence

- Runtime image: `iiia:nao`, image ID
  `sha256:d0dd959a870ef3fdf75da683bd535084b20e3dce29501f61b56555d71b5f9854`.
- The live imports for orchestrator, planner engine, and report outcome match
  the pending deslop checkout at `/home/juanbeck/nao-ros4hri-bridge`.
- The container was recreated as a complete stack. Node uniqueness and LLM
  preflight passed before scoring.
- Main suite: 20/20 assessed pass. The five KB cases passed without KB timeout,
  model fallback, report fallback, or duplicate-goal markers.
- Environment suite: 9/11 pass. Complete-context failures were the IIIA kitchen
  delivery and gold-apple handoff.
- Deep fake all-success: 7/9 pass. Ordered walk/report and the gold-apple
  multi-turn case clarified despite complete fixtures.
- `pick_object=fail_once` recovered through planner version 2 and produced the
  chatbot-authored closure, "I picked up the phone."
- `navigate_to=fail_once` was not reached because ordered-walk planning
  clarified first.
- `delivery_blocked` exposed a repeated identical work-table replan and the
  awkward kitchen closure, "I have brought the kitchen book to ALEX and placed
  it on them."
- Alternating fake outcomes failed both complete-context delivery cases;
  deterministic seeded-random outcomes passed both.
- Focused source-contract tests in the runtime image: 112 passed.

Primary artifacts are under `/tmp/nao_*_20260713_seam_audit*.json` and
`/tmp/nao_runtime_snapshot_20260713_final.json`.

## Approach registry

| ID | Mechanism | Discriminating evidence | Status |
|---|---|---|---|
| H1 | General KB transport or freshness failure | Fresh insertion, relation, name, count, and environment inventory cases passed after dialogue context | Rejected as a general cause |
| H2 | Selective grounded recipient resolution failure | Gold-apple handoff clarified although ALEX was present; other ALEX deliveries succeeded | Supported, scope not yet isolated |
| H3 | Planner phrase fallback fails to map complete ordered/location requests | Ordered walk reached planner request but no execution feedback; source uses `_looks_like_*` phrase markers before grounded fallbacks | Supported |
| H4 | Fake skill server or action wiring is unavailable | `pick_object=fail_once` replanned successfully and seeded-random delivery cases completed | Rejected as a general cause |
| H5 | Replan policy repeats a failed plan without a materially changed strategy | Work-table `delivery_blocked` emitted the same `bring_object` step in plan versions 1 and 2 | Accepted defect |
| H6 | Execution-report semantics misclassify recipient/place relations | Recovery closure said the book was placed "on" ALEX | Accepted defect |
| H7 | Review correlation overstates successful closure | The harness accepts any terminal event plus any speech in the window; an initial acknowledgement can satisfy a case with no post-terminal closure | Accepted test defect |
| H8 | Long context alone causes KB loss | Main KB sequence and location follow-ups passed; failures correlate with specific recipient/ordered constructs | Rejected with current evidence |

## Discriminating probes and results

1. **Fresh KB and relation sequence:** all five main KB cases passed. This
   narrows the problem from ingestion to selective semantic projection or
   admission.
2. **Same fake server, different policy:** fail-once pick and seeded-random
   deliveries succeeded. The fake server and replan transport are operational.
3. **Ordered walk with fail-once navigation:** no navigation action was
   dispatched. The blocking failure is upstream of the executor.
4. **Blocked delivery:** one case repeated an equivalent failed plan; another
   recovered but produced semantically weak report wording. Recovery and report
   quality are separate seams.
5. **Source inspection:** planner fallbacks use fixed phrase markers for grouped
   delivery and ordered walks. The questionnaire grades terminal and speech
   independently rather than requiring speech after the terminal outcome.

## Adversarial audit

- Ownership remains correct in the observed healthy paths: chatbot chooses the
  route and words speech, planner creates plans, orchestrator dispatches, and
  fake AB=1 actions provide execution evidence.
- No duplicate speech, raw planner leakage, fabricated fresh KB fact, or hidden
  chatbot plan was observed in the valid main run.
- The planner phrase fallbacks are architecture-compatible in ownership but
  brittle in policy. Adding more lexical markers would increase reward hacking
  and is rejected.
- The repeated replan violates the practical recovery requirement because the
  second plan does not address the failure cause.
- The kitchen recovery closure is not safe evidence of correct placement. A
  person recipient must not be rendered as a support surface.
- Prompt edits are not accepted from this audit because no bounded SkillOpt
  baseline, mutation, and holdout gate was run.

## Decision: bounded handoff

The stack is operational and the KB transport baseline is healthy, but the
deep recovery and selective recipient/ordered-request seams are not frozen.
No broad runtime fix should be accepted yet. The smallest safe next work is
test-first at the existing contracts:

1. Make questionnaire closure correlation require user-facing speech after the
   terminal event for execution and recovery cases.
2. Add a planner supervisor test that rejects an unchanged replan after the
   same blocking failure, or requires an explicit terminal help/failure act.
3. Add grounded-context fixtures proving that ALEX remains a person recipient
   in gold-apple and kitchen requests.
4. Add report-outcome tests that a person recipient cannot become a placement
   support and that partial recovery reports only evidenced completed effects.
5. Replace phrase-specific planner fallback dependence with a structured
   request/grounding mechanism only after those tests identify the minimal
   contract change.

## Residual risk and next probe

The next discriminating probe should capture the exact planner request and
grounded-context JSON for the failing gold-apple handoff and compare it with the
passing grouped delivery. This will distinguish chatbot projection loss from
planner recipient matching. Then rerun ordered walk with a valid structured
location group while recording the planner model output and fallback mode.

The current questionnaire can over-score recovery cases until temporal closure
correlation is fixed. Scores based on those cases must be qualified manually.
