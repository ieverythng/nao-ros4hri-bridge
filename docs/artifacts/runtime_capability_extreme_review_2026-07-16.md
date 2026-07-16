# Runtime Capability-Extreme Review, 16 July 2026

## Run identity

- Container: `nao_ros2`, id `44eddcfd093e`, image `sha256:41e682cdd9da`
- Container start: `2026-07-16T09:51:02Z`
- Review checkout: `feat/eval_suite` at `a39bf6e`
- Pipeline: `response_first`
- Model endpoint: laboratory OpenAI-compatible endpoint, Qwen3-VL-30B-A3B-Instruct-AWQ
- Grounded-context digest: disabled; structured grounded context remained the world-state input
- Success artifact: `nao_capability_extreme_all_success_2026-07-16.json`
- Recovery artifact: `nao_capability_extreme_fail_once_pick_2026-07-16.json`

The live container contains planner code newer than this review checkout. In
particular, `/home/ubuntu/ws/src/planner_llm/planner_llm/planner_engine.py`
contains `_validated_target_selection_recovery`, while the method is absent from
the checkout. A corrective patch must target the branch used to build the
container before it is brought into `feat/eval_suite`.

## Runtime review

**Score: 8.4/10 (valid capability-extreme run, not ready to freeze)**

### 🔴 Critical

1. Quantified target recovery preserved the three selected objects but discarded
   other requested actions. For “Stand up, walk to each visible object on the
   work table, look at each one, wave to ALEX, sit down, and give one final
   summary,” chatbot produced all six intents and an exact `target_selection`
   containing ATLAS, MIDAS, and TITAS. Planner mode
   `validated_target_selection_recovery` emitted only three `navigate_to` steps
   and `report_result`. It omitted stand, all `look_at` steps, wave, and sit.
   The spoken acknowledgement therefore promised actions that were never
   dispatched. Fix the recovery compiler so it cannot replace a richer
   composite objective with only the quantified operation. Prefer rejecting the
   failed model result and returning a structured clarification/failure over
   silently compiling a semantically incomplete plan.

### 🟡 Route and planning ambiguity

1. Two valid composite turns required chatbot route repair with confidence
   `0.00`. The planner reconstructed the complete goal from `goal_text`, and the
   executions succeeded, but route repair remains a reliability warning under
   lexical variation.
2. The recovered “sit, stand, pick MIDAS, return to ALEX” plan preserved all
   actions despite `normalized_intents=[pick_object]`. This proves that complete
   `goal_text` currently protects execution better than the normalized intent
   list. The planner handoff should continue to treat normalized intents as
   hints rather than the full objective.

### 🟣 Observability gaps

1. The original harness incorrectly required `target_selection` for singular
   named targets. Four successful cases were initially marked failed. The new
   case manifest now reserves this requirement for quantified selection.
2. The harness marked the quantified case passed because it checked selected
   members and terminal evidence but did not compare requested capabilities with
   emitted plan steps. This is a material false positive. Add a semantic plan
   coverage oracle before using this suite for a freeze decision.
3. Incremental artifacts can contain `finished_at_unix_sec` while the runner is
   still active. Process exit and final case count must remain the completion
   gate.

### 🔵 Runtime pressure

1. `hri_face_detect_yunet` repeatedly skipped approximately 100 frames every
   5-6 seconds. It did not invalidate the symbolic fixtures in this run, but it
   remains separate detector-profile pressure.

## Checks passed

- Grounding: injected ATLAS, MIDAS, TITAS, ALEX, work table, and storage shelf
  were present in grounded context. The dialogue inventory named all required
  entities and stated the table support relation on the first turn.
- Singular composition: navigate/pick/sit/report completed with coherent natural
  speech.
- Difficult posture/manipulation: kneel/pick-under-table/stand/report completed.
- Multi-turn composition: dialogue inventory followed by sit/stand/pick/return
  retained the prior entities and completed the physical chain.
- KB postcondition: pick ATLAS/place on storage shelf/kneel/report completed and
  `/kb/query` confirmed `codex_extreme_apple oro:isOn codex_extreme_shelf`.
- Failure and replan: `fail_once_pick` produced failure evidence, planner version
  2, successful retry, terminal completion, and one coherent chatbot-authored
  report.
- Speech: no duplicate semantic utterance and no raw planner text leakage was
  found in these cases.
- Reporting: all observed `report_result` outputs were natural summaries rather
  than concatenated internal values.

## E2E questionnaire

- Simple dialogue: **pass (carried evidence)**. The new inventory turn remained dialogue-only.
- KB query dialogue: **pass**. Four named entities and support relations were used immediately.
- Simple skill execution: **pass (carried evidence)**.
- Composite skill execution: **degraded**. Five tested chains completed, but quantified recovery dropped non-visit actions.
- Simple fake-skill scenarios: **pass for targeted pick failure/retry**.
- Composite fake-skill scenarios: **pass for fail-once recovery; incomplete for broad random policy coverage**.
- Capability-extreme set: **6/7 manually adjudicated trajectories passed**. Raw harness labels are not the final score because four singular-target labels were false negatives and one quantified label was a false positive.

## Seam hypothesis audit

| ID | Hypothesis | Discriminating evidence | Status | Target |
|---|---|---|---|---|
| H1 | Chatbot selected only two of three objects | `target_selection.member_ids` contained all three canonical ids | Rejected | none |
| H2 | KB projection omitted recent objects or relations | grounded context and first-turn inventory contained all required entities and support facts | Rejected for this run | grounding remains a holdout |
| H3 | Planner target-selection recovery is semantically lossy | six requested intents became three navigation steps plus report in `validated_target_selection_recovery` | Accepted | newer `planner_llm/planner_engine.py::_validated_target_selection_recovery` |
| H4 | `report_result` still concatenates or leaks internals | every observed completion was coherent and contextual | Rejected for this run | keep holdout |
| H5 | Failure feedback does not return to replanning | fail-once pick produced plan version 2 and successful closure | Rejected | keep deep-fake holdout |
| H6 | Harness labels accurately represent semantic success | four false negatives and one false positive found by trace adjudication | Accepted | questionnaire semantic oracle |
| H7 | Prompt drift is the primary cause | route repair occurred, but exact H3 loss is deterministic recovery code | Blocked as primary cause | reopen only after H3 is fixed and holdouts rerun |

## Evaluation design

The new `capability_extreme` set uses seed `20260716` and varies report,
return, and quantified-object wording while retaining deterministic fixtures.
Its scoring follows four useful precedents: final state and repeat consistency
from [tau-bench](https://arxiv.org/abs/2406.12045), progress-aware scoring from
[AgentBoard](https://arxiv.org/abs/2401.13178), clarification and recovery from
[TEACh](https://arxiv.org/abs/2110.00534), and long-horizon compositional tasks
from [BEHAVIOR-1K](https://arxiv.org/abs/2403.09227) and
[VLMbench](https://arxiv.org/abs/2206.08522).

## Freeze gates

1. Correct H3 in the newer planner branch and add a regression test proving
   quantified visit recovery cannot discard posture, look, gesture, or requested
   report semantics.
2. Add plan-step semantic coverage to the harness. Selected-member equality is
   necessary but insufficient for composite success.
3. Rerun `capability_extreme` under `all_success`, `fail_once_pick`, and one
   deterministic alternating/random profile with isolated dialogue ids.
4. Require two consecutive runs with no semantic objective loss, no duplicate
   speech, and truthful terminal reports before freezing.
