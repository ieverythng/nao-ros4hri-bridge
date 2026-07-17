# Runtime Performance Review: Grounding-Strong Final Candidate

Date: 17 July 2026  
Runtime: `nao_ros2`, response-first pipeline, lab OpenAI-compatible endpoint  
Score: **8.8/10**  
Standing: **strong candidate, not yet frozen**

The current stack is substantially stronger than the previous capability-extreme
baseline. KnowledgeCore insertion, relation revision, grounded retrieval,
person/object separation, skill target selection, and postcondition queries were
all reliable under long context. Two execution-admission regressions and one
lossy planner recovery path prevent a 9.0+ freeze.

## 🔴 Critical

1. **Quantified recovery can still discard requested capability families.** The
   maximal request to stand, visit every object, look at each one, wave, sit,
   and report reached planner execution with only three `navigate_to` steps and
   `report_result`. The terminal report omitted the dropped actions and used
   internal identifiers. A partial plan must not be accepted as full success.
2. **Acknowledgement and admitted route can disagree.** For “Can you move your
   head in all directions?”, the chatbot said, “Yes, I can move my head in all
   directions. How can I assist you?”, while route repair admitted execution and
   the robot moved its head. The physical execution and final report succeeded,
   but the acknowledgement was a capability answer rather than an execution
   commitment. Route repair must not publish stale wording.

## 🟠 Execution Admission And Recovery

1. The request to sit, stand, pick MIDAS, return to ALEX, and report was
   understood at the text and entity level, but chatbot handoff added
   `bring_object` and omitted a compatible `target_selection`. Planner correctly
   refused the incomplete contract and chatbot unnecessarily asked which object
   was meant. “Return to ALEX” described robot motion, not object delivery.
2. The all-success fake-deep missing-object case reached replan but did not
   produce a clear terminal closure in the captured trajectory. This remains a
   degraded recovery case.
3. `fail_once_navigation` was applied and direct logs show `step_failed` reaching
   planner feedback. The questionnaire artifact nevertheless marked the cases
   `not_scored` and claimed the failure was not exercised. Failure/replan scoring
   from this artifact is invalid; the runtime path is partially evidenced but
   not accepted as a complete recovery result.

## 🟢 Knowledge And Grounding

The stateful KB stress chain passed **7/7 semantic cases**:

- Newly inserted TITAS and MIDAS facts were available on the first relevant turn.
- Names, colors, support locations, and ALEX remained visible after long context.
- Revising TITAS from `work_table` to `storage_shelf` was reflected immediately.
- Delivery selected the exact grounded object and person, then wrote a verified
  postcondition.
- Moving MIDAS to the shelf and delivering all shelf objects to ALEX preserved
  both objects and produced a correct final query: TITAS is a gold cup, MIDAS is
  a blue book, and both are with ALEX.
- ALEX stayed `kind=person`; object counts and quantified object selections did
  not absorb the person.

The raw harness labels for `work_table` versus “work table” are lexical false
negatives. The underlying RDF values, grounded-context projection, model answer,
skill selection, and postconditions agree.

## 🟡 Dialogue And Reporting

- Simple dialogue, visible-object questions, names, colors, counts, immediate
  object hydration, and reflective follow-up all passed.
- Ordered three-object navigation produced per-object arrival messages and a
  coherent final summary.
- Person look-at and person navigation/wave cases selected ALEX correctly.
- Normal report paths were natural in the kitchen delivery, walk/pick/sit,
  under-table pick, pick/place, and gold-apple cases.
- The lossy maximal recovery report was not natural: “I completed destination
  navigation to codex_extreme_book...” This is local to incomplete recovery
  evidence, not evidence that the normal report-result path is globally broken.

## 🟣 Runtime And Observability

- Preflight status was `ready_for_semantic_scoring`; all required nodes were
  present, lifecycle nodes were active, KnowledgeCore was ready, and both LLM
  preflights passed.
- No LLM connectivity failure, duplicate active goal, invalid planner JSON,
  invalid executable plan, or execution-report fallback was observed.
- The final 75-minute snapshot counted **83 route-repair events** and five
  rules-intent fallbacks. Route repair is carrying too much semantic load even
  though most resulting trajectories succeed.
- The harness produced two important false negatives: ordered multi-object
  navigation completed despite missing correlated planner evidence, and the
  kitchen-to-operator case correctly resolved “you” rather than clarifying.
  Manual trace adjudication was required.

## Questionnaire Summary

| Suite | Semantic result | Qualification |
|---|---:|---|
| Main | 21/21 trajectories completed | One acknowledgement/route mismatch is degraded |
| Capability extreme | 5/7 pass | Admission mismatch and lossy recovery are real failures |
| KB stress | 7/7 pass | Thesis-grade state, grounding, skill, and postcondition chain |
| Fake deep, all success | 8/9 pass or useful | Missing-object recovery lacks terminal closure |
| Fake deep, fail once navigation | Not scored | Direct failure evidence contradicts harness phase extraction |

## Acceptance Before Freeze

1. Make route repair transactional: validate route, acknowledgement, goal text,
   intent families, and target selection before any acknowledgement is spoken.
   Retry or clarify on mismatch; do not add phrase-specific routing markers.
2. Reject deterministic planner recovery when it cannot preserve every admitted
   capability family. Structured failure is preferable to successful partial
   execution.
3. Correct the robot-return versus object-delivery role model and require a
   compatible target selection only when delivery is actually requested.
4. Repair failure-profile trace correlation, then rerun `fail_once_navigation`
   and `fail_once_pick` from a clean runtime.
5. Preserve the current KB/person behavior as mandatory holdouts for every fix.

## Artifacts

The complete machine-readable evidence is under
`docs/artifacts/runtime_review_2026-07-17/`:

- `preflight.json`
- `main.json`
- `capability_extreme.json`
- `kb_stress.json`
- `fake_deep_all_success.json`
- `fake_deep_fail_once_navigation.json`
- `final_snapshot.json`

**Finding count:** 🔴 2 critical, 🟠 3 execution/recovery, 🟢 6 confirmed
grounding strengths, 🟡 5 dialogue/report observations, 🟣 4 runtime and
observability findings.
