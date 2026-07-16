# Seam Hypothesis Audit: Capability-Extreme Objective Preservation

Date: 16 July 2026  
Decision state: **bounded handoff**  
Related runtime score: **8.4/10**

## 1. Target contract

### Problem

For a grounded quantified request containing several capability families, the
stack selected all requested objects but executed only the quantified navigation
operation and a final report. The acknowledgement promised stand, navigation,
look, wave, sit, and report. The accepted plan contained three `navigate_to`
steps and `report_result`.

### Required outcome

- Preserve the complete admitted execution objective across chatbot handoff,
  planner validation/recovery, orchestrator dispatch, execution feedback, and
  final speech.
- Quantified target selection must cover every intended member exactly once,
  subject to explicit failure/replan policy.
- A fallback or recovery path must never convert a richer objective into a
  successful partial plan without user-visible qualification.
- `report_result` must summarize only actions supported by execution evidence.

### Non-goals

- Do not add phrase-specific route markers or hardcoded user utterances.
- Do not move planning into `nao_orchestrator` or executable plans into
  `chatbot_llm`.
- Do not require `target_selection` for a singular named target.
- Do not change ROS topics, actions, services, AB levels, or speech ownership.
- Do not modify canonical prompts unless a later discriminating probe isolates
  prompt behavior and a bounded SkillOpt iteration accepts the change.

### Protected seams and owners

- `chatbot_llm`: route, complete `goal_text`, normalized intent hints, grounded
  target selection, and acknowledgement wording.
- `planner_llm`: executable plan generation, validation, retry, and replan.
- `nao_orchestrator`: deterministic validation, dispatch, lineage, and feedback.
- AB=1 skills: execution-time evidence and postconditions.
- `dialogue_manager`: sole speaking/Say-dispatch owner.

### Acceptance gate

1. The maximal quantified prompt produces plan steps covering stand, three
   visits, three looks, wave, sit, and final report, or returns a truthful
   structured failure/clarification. Partial success is prohibited.
2. `target_selection.member_ids` equals the three grounded object ids and keeps
   ALEX as the person recipient/gesture target without treating ALEX as an
   object member.
3. Every executed step retains one `goal_id`, `plan_id`, `plan_version`, and
   stable `step_id` lineage.
4. Final speech mentions only succeeded actions and occurs exactly once after
   the acknowledgement.
5. The existing singular composite, KB postcondition, and fail-once replan
   holdouts remain passing.
6. Two clean rebuild runs pass: one `all_success`, one deterministic failure
   profile.

### Evidence budget

- One source/test pass on the branch used to build the image.
- One clean full-container rebuild before semantic scoring.
- Two live maximal cases and the targeted fail-once holdout.
- No prompt mutation in this audit.

## 2. Baseline evidence

| Observation | Expected | Actual | Evidence |
|---|---|---|---|
| Grounded entities | ATLAS, MIDAS, TITAS, ALEX and support relations | All present and used on the first inventory turn | `nao_capability_extreme_all_success_2026-07-16.json` |
| Quantified selection | Three table objects | Exact three canonical object ids | same artifact, `extreme_all_objects_visit_look_wave_sit` |
| Complete chatbot handoff | Six capability families plus report | `posture_stand`, `navigate_to`, `look_at`, `wave_greet`, `posture_sit`, `report_result` | planner-request trace |
| Executable plan | Preserve every requested capability | `navigate_to` x3 plus `report_result` | plan mode `validated_target_selection_recovery` |
| Report truthfulness | Report only executed actions | Terminal path completed, but acknowledgement over-promised omitted actions | runtime trace |
| Failure/replan | Failed pick returns to planner and closes naturally | Version 2 succeeded with one natural completion | `nao_capability_extreme_fail_once_pick_2026-07-16.json` |
| Current mounted source | Match code loaded by live planner process | Mounted file no longer contains `_validated_target_selection_recovery`; running process had loaded it at launch | container source inspection after run |

The final row is a runtime provenance boundary. The observed failure is valid for
the launched process, but the currently mounted source may already contain an
unverified correction. No new source patch should be accepted until a clean
rebuild distinguishes loaded-runtime behavior from current staged code.

## 3. Approach registry

| ID | Family | Mechanism | Affected seams | Discriminating probe | Expected observation | Status | Exact gap or reopen condition |
|---|---|---|---|---|---|---|---|
| H-01 | Grounding/KB | “All objects” omitted one entity before planning | scene grounding, chatbot | Compare KB rows, grounded context, and selection ids | Missing id upstream | rejected | All three ids were present and selected |
| H-02 | Chatbot handoff | Chatbot reduced the request to navigation only | chatbot response/intent handoff | Compare acknowledgement, `goal_text`, intents, and selection | Missing actions in handoff | rejected for this trace | Handoff preserved all six capability families |
| H-03 | Prompt/model | Planner model omitted non-navigation actions | planner provider | Inspect raw model attempts and validation errors | Invalid/incomplete model plan before recovery | candidate | Preserve raw model attempts in focused rerun |
| H-04 | Planner recovery | Validated target-selection fallback compiles only selection operation and report | planner engine | Compare request intents with recovery plan | Plan loses posture/look/wave | accepted for launched runtime | Current mounted source changed; clean rebuild must reproduce or reject |
| H-05 | Orchestrator | Orchestrator filtered valid non-navigation steps | orchestrator validation/dispatch | Compare planner plan payload with accepted feedback | Full plan published, reduced dispatch | rejected | Published plan already contained only four steps |
| H-06 | Registry/AB | Requested skills unavailable or aliased incorrectly | skill registry projections | Verify allowed skills and registry support | Missing `look_at`, motion, or wave support | rejected | Other cases executed these skills; chatbot listed them as valid intents |
| H-07 | Harness | Semantic scorer treats member coverage as full objective coverage | review harness | Compare harness pass with emitted plan steps | False pass despite dropped capabilities | accepted | Add plan semantic coverage oracle |
| H-08 | Reporting | `report_result` fabricated omitted actions | chatbot system turn, report wrapper | Compare report text with succeeded-step evidence | Report claims look/wave/posture | blocked | Captured speech window did not preserve a complete final utterance for this case |
| H-09 | Source/runtime drift | Running process and mounted source represent different revisions | build/runtime provenance | Clean rebuild, record file digest before launch, rerun exact seed | Failure disappears or maps to current source | active | Rebuild required |

### Active-route assumptions and artifacts

**H-04 assumptions**

- The running planner imported the implementation present when the container
  launched.
- Trace mode `validated_target_selection_recovery` names the actual decision
  route rather than a stale label.
- Artifact: all-success JSON plus container log plan payload.
- Change class if reproduced: planner code and focused tests only.

**H-07 assumptions**

- Target membership and plan capability coverage are independent semantic
  dimensions.
- Artifact: harness `pass` for the exact case whose plan omitted four requested
  capability families.
- Change class: evaluation harness only.

**H-09 assumptions**

- Mounted source may have changed after planner startup because another agent or
  branch update modified the shared workspace.
- Artifact: initial source inspection found the recovery method at lines
  849-949; later inspection found no occurrence without restarting the process.
- Change class: none until clean rebuild evidence exists.

## 4. Discriminating probes and results

### Probe P-01: KB to grounded-context chain

Injected three objects, a person, table, shelf, names, colors, support facts,
visibility, and reachability. The chatbot named all required entities and the
support relation on the first dialogue turn.

**Result:** rejects H-01 for this run.

### Probe P-02: Handoff decomposition

Compared user text with acknowledgement, normalized intents, `goal_text`, scene
targets, and `target_selection`.

**Result:** rejects H-02. Chatbot represented the rich objective sufficiently
for the planner. Route repair remains a reliability warning, but it does not
explain the deterministic reduction in this trace.

### Probe P-03: Plan versus dispatch

Compared planner decision payload with execution feedback. Both contained three
navigation steps followed by report.

**Result:** rejects H-05 and accepts H-04 for the launched runtime.

### Probe P-04: Registry availability

The same runtime successfully executed `perform_motion`, `pick_object`,
`place_object`, navigation, and report in adjacent cases. The request also used
valid `look_at` and `wave_greet` intent labels.

**Result:** rejects H-06 as the cause of this reduction.

### Probe P-05: Failure/replan transport

Applied `fail_once_pick` to the difficult under-table case. Failure feedback
reached planner, version 2 was accepted, execution completed, and chatbot
produced one natural report.

**Result:** rejects a global replan failure. The artifact is valid evidence of
transport, though its dialogue id reused prior case history and should be
isolated in the freeze rerun.

### Probe P-06: Harness adversarial check

Manually compared raw verdicts with trace trajectories. Four singular-target
cases were false negatives because they lacked quantified `target_selection`.
The maximal quantified case was a false positive because member selection and
terminal evidence passed while requested capabilities were omitted.

**Result:** accepts H-07. Harness labels alone cannot authorize a freeze.

### Probe P-07: Runtime/source identity

After the run, inspected the mounted planner source. The recovery method that
named the live decision mode was absent, although the running process continued
to execute code loaded at launch.

**Result:** activates H-09. Current staged code must be rebuilt before deciding
whether H-04 still needs a patch.

## 5. Adversarial audit

- [x] Ownership remains unchanged. The proposed guard belongs in planner
  validation/recovery, not chatbot or orchestrator.
- [x] No new ROS interface is required.
- [x] Goal, plan, version, and step lineage were preserved in the observed run.
- [x] No duplicate speech authority or hidden executable plan is proposed.
- [x] No KB, perception, result, or completion fact should be fabricated.
- [x] Planning context remains evidence, not AB=1 execution proof.
- [x] People and objects remained distinct in quantified selection.
- [x] Registry and AB levels are outside the proposed change.
- [x] Success and fail-once recovery evidence exist.
- [ ] Cancellation and supersede behavior has not been rerun against a corrected
  maximal plan.
- [ ] The current staged planner source has not been cleanly rebuilt and scored.
- [ ] The harness lacks an accepted semantic plan-coverage oracle.

### Candidate-change challenge

The smallest safe planner behavior is a semantic eligibility guard around any
deterministic target-selection recovery. Recovery may compile the quantified
operation only when the admitted objective contains no additional capability
families beyond that operation and its reporting policy. If additional
capabilities exist, planner should use a valid model plan or return structured
failure/clarification after retries. It must not claim successful partial
execution.

Expanding the recovery compiler to synthesize arbitrary posture, look, gesture,
manipulation, and return sequences is rejected for the first patch. That would
turn a bounded recovery path into another deterministic planner and increase
the chance of future semantic loss.

The harness should independently compare requested capability families against
planned steps and relevant arguments. This is an evaluation correction, not a
runtime workaround, and must not influence planner behavior.

## 6. Decision: bounded handoff

**Chosen route:** H-09 first, then H-04 if reproduced; H-07 in parallel as an
evaluation-only correction.

The launched runtime provides strong evidence that the old target-selection
recovery path was semantically lossy. Current mounted source differs from the
loaded process, so an immediate code patch would risk duplicating or conflicting
with staged work. The bounded handoff is:

1. Identify the exact branch/commit/staged diff used by the mounted container
   source and record its planner file digest.
2. Clean rebuild the whole image and relaunch once.
3. Rerun the exact seeded maximal case.
4. If objective loss reproduces, add a failing planner-engine test first and
   implement the narrow recovery eligibility guard.
5. Add a harness regression test that fails when requested capability families
   disappear from the emitted plan.
6. Rerun singular composition, maximal quantified composition, KB postcondition,
   and fail-once replan holdouts.

## 7. Residual risk and next probe

### Residual risks

- The current source may already fix H-04, but this is unproven until rebuild.
- Route repair at confidence 0.00 occurred twice. Prompt policy remains a
  holdout, not the leading cause.
- The fail-once rerun reused an existing dialogue id, so long-context behavior
  and isolated recovery behavior are not fully separated.
- `report_result` remained natural in observed cases, but the maximal dropped
  action case needs a complete final-speech capture after the planner fix.
- Detector frame skips remain separate performance pressure.

### Exact next probe

Run after a clean rebuild from the intended planner branch:

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py \
  --container nao_ros2 \
  --case-set capability_extreme \
  --case-names extreme_all_objects_visit_look_wave_sit \
  --mode speech \
  --speech-voice-scope case \
  --fake-policy-profile all_success \
  --expected-turn-pipeline-mode response_first \
  --out /tmp/nao_capability_extreme_rebuild_holdout.json
```

Accept only if the plan covers every requested capability or returns a truthful
non-execution outcome. Selected-member equality and polished speech are not
sufficient.
