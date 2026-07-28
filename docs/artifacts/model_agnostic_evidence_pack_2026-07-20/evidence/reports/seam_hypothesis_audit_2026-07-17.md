# Seam Hypothesis Audit: Admission, Recovery, And Harness Correlation

Date: 17 July 2026  
Decision: **bounded handoff**  
Related runtime score: **8.8/10**

## 1. Target Contract

The stack must preserve one coherent user objective from chatbot route selection
through planner handoff, execution, and final speech. An acknowledgement must
describe the route that will actually run. Recovery may not silently remove
requested actions. Grounded entities and predicate relations must remain usable
under long context, with people and objects kept distinct. The review harness
must distinguish runtime failures from missing or incorrectly correlated
evidence.

Non-goals are phrase-specific routing hints, moving planning into the
orchestrator, introducing a second speech owner, or changing canonical prompts
without a bounded SkillOpt baseline and holdout gate.

Acceptance requires the two failing capability-extreme prompts to preserve all
semantic actions or return truthful clarification/failure, plus clean
`fail_once_navigation` and `fail_once_pick` evidence. The 7/7 KB stress chain and
person-grounding cases are protected holdouts.

## 2. Baseline Evidence

| Observation | Evidence | Classification |
|---|---|---|
| Fresh and revised KB facts were used immediately across seven stateful cases | `runtime_review_2026-07-17/kb_stress.json` | confirmed strength |
| ALEX remained a person and was selected correctly for look-at, wave, navigation, and delivery | main and KB artifacts | confirmed strength |
| Head-motion acknowledgement answered capability while repaired route executed motion | main artifact and live UI | real regression |
| Maximal all-object request was reduced to navigation plus report | capability-extreme artifact | real regression |
| Return-to-ALEX request was interpreted as object delivery, then lacked delivery selection | capability-extreme artifact | real regression |
| Injected navigation failure appears in direct logs but not questionnaire phase fields | fail-once artifact and container logs | harness contradiction |

## 3. Approach Registry

| ID | Family | Hypothesis | Probe | Result | Status |
|---|---|---|---|---|---|
| H-01 | KB/grounding | Missing or stale facts cause the regressions | Stateful insert, revise, query, execute, postcondition chain | 7/7 semantic pass | rejected |
| H-02 | Person/object roles | ALEX is lost or treated as an object | Inspect grounded kinds and skill selections | Correct in main and KB suites | rejected globally |
| H-03 | Chatbot route transaction | Route repair changes route after acknowledgement generation | Compare spoken ack, explicit/inferred route, repair marker, planner handoff | Capability ack followed by execution | accepted |
| H-04 | Chatbot semantic admission | “Return to ALEX” is mapped to `bring_object` and demands the wrong selection contract | Compare goal text, normalized intents, scene targets, and target selection | Object MIDAS explicit; delivery intent added; selection absent | accepted |
| H-05 | Planner recovery | Target-selection recovery compiles only quantified operation and report | Compare admitted intents with recovered steps | Stand/look/wave/sit dropped | accepted |
| H-06 | Orchestrator filtering | Orchestrator drops valid planner steps | Compare plan payload with dispatched feedback | Reduced plan already emitted by planner | rejected |
| H-07 | Report-result global failure | Chatbot cannot summarize multi-step evidence naturally | Compare normal reports with recovery report | Most normal paths natural; lossy path malformed | rejected globally, active locally |
| H-08 | Harness correlation | Phase extraction and lexical oracles misclassify valid trajectories | Compare raw logs, speech, RDF postconditions, and artifact flags | Multiple false negatives and one missed injected failure | accepted |
| H-09 | Provider instability | LLM connectivity explains the regressions | Preflight and fallback inspection | Zero connectivity failures | rejected |

## 4. Discriminating Probes And Results

### P-01: Stateful grounding chain

Inserted TITAS and MIDAS, revised support relations, queried exact predicates,
executed grounded deliveries, checked postconditions, and queried both objects
after long context. Every semantic state assertion passed.

**Result:** H-01 is rejected. The visible regressions are downstream of grounded
context creation.

### P-02: Acknowledgement versus route

The head-motion prompt produced a capability-style acknowledgement. Route repair
then admitted execution, and planner/skills completed the motion.

**Result:** H-03 is accepted. Route repair currently repairs routing without
repairing already generated user-facing wording. The safe boundary is a
pre-publication response transaction in chatbot, not a new orchestrator phrase
guard.

### P-03: Full objective versus recovered plan

The maximal prompt and acknowledgement contained stand, all-object navigation,
look-at, wave, sit, and report. The accepted recovery contained only three
navigations and report.

**Result:** H-05 is accepted and H-06 rejected. Planner recovery must prove
semantic coverage before returning `planned`.

### P-04: Return role semantics

MIDAS and ALEX were both grounded. The chatbot added `bring_object` for a request
where the robot should return while holding the object. Planner rejected the
missing delivery selection and chatbot asked which object despite MIDAS being
explicit.

**Result:** H-04 is accepted. Admission needs a structured distinction between
robot destination and object recipient. This must not be solved with utterance
markers.

### P-05: Failure injection correlation

The fake server changed to scenario mode with one override. Direct logs recorded
`step_failed` for `navigate_to` and planner feedback with retry budget one. The
artifact simultaneously recorded `failure_observed=false`,
`execution_feedback_observed=false`, and “configured fake failure was not
exercised.”

**Result:** H-08 is accepted. This run cannot score recovery completion. The
phase collector must correlate goal/plan lineage from the direct trace instead
of relying on its current case window alone.

## 5. Adversarial Audit

- Ownership remains intact: chatbot validates its response transaction, planner
  validates semantic plan coverage, orchestrator continues deterministic
  dispatch, and dialogue manager remains the speech owner.
- No new ROS interface or hardcoded route phrase is required.
- Proposed validation must preserve goal, plan, version, and step lineage.
- A partial plan may not be represented as successful completion.
- Planning-time grounded context remains selection evidence, not proof of an
  AB=1 action result.
- People and objects remained semantically distinct in the accepted KB chain.
- No prompt change is accepted from this audit. Any prompt mutation requires
  SkillOpt train and holdout evidence.
- Failure, clarification, and replan coverage remains incomplete until harness
  correlation is corrected and rerun.

## 6. Decision: Bounded Handoff

Accept H-03, H-04, H-05, and H-08 as independent contributors. Reject a KB
hydration explanation for these traces. The smallest safe implementation order
is:

1. Fix questionnaire goal/plan correlation so failure profiles can be trusted.
2. Make chatbot response publication atomic across acknowledgement, route,
   confidence, goal text, intents, and target selection. Retry once or clarify
   on internal mismatch.
3. Add planner semantic-coverage validation to any deterministic recovery. If
   recovery cannot preserve all requested capability families, fail or clarify
   rather than execute a subset.
4. Represent robot destination separately from object recipient in admission
   checks, using existing structured contracts where possible.
5. Rerun the exact failing prompts and failure profiles while holding the KB
   stress chain fixed as a regression gate.

## 7. Residual Risk And Next Probe

The final snapshot counted 83 route repairs. Most ended successfully, but that
volume means the repaired path is effectively a normal policy path and deserves
the same consistency validation as a direct model route. The next probe should
capture the raw chatbot JSON, repaired response object, published
acknowledgement, and planner request under one dialogue-turn id for ten route
repairs. A mismatch rate above zero blocks freeze.

After the harness fix, run one clean all-success maximal case, one
`fail_once_navigation`, and one `fail_once_pick`. Require full capability
coverage, one coherent acknowledgement, one natural terminal utterance, and
verified KB postconditions. Until those gates pass, the correct disposition is
bounded handoff rather than acceptance.

## 8. Compound-Plan Closure Addendum

The exact all-objects/look/wave/sit failure was isolated further after this
broader audit. A public `PlannerEngine.plan_request` test proved that the first
validation repair could add a selected target while retaining one coverage
error. The prior error-count heuristic treated that as no progress and denied
the final bounded retry. Removing that heuristic restored the existing
three-call budget while preserving the repeated-output stop and all semantic
validators.

The first clean replacement-backend run on image v27 still failed before
planning because the integrated launch did not expose the existing
`intent_request_timeout_sec` parameter. The intent client retained its
10-second default and collapsed the six-intent objective to `wave_greet` after
transport timeout. Image v28 exposed that parameter without changing its
default or chatbot policy. With the live value set to 180 seconds, the same
fixture preserved all six intents and the three authoritative member IDs.

The v28 maximal case passed. Planner version 1 admitted the complete ten-step
trajectory, but misplaced `report_policy=final` into the fake-only
`wave_greet.result_mode` argument. The fake skill rejected that value. Planner
version 2 then retained stand, all three navigate/look pairs, wave, sit, and
final reporting. The chatbot-owned closure named ATLAS, MIDAS, TITAS, and ALEX.
The explicit head-motion route/ack control also passed.

This closes H-05 for the exact compound trajectory. It does not close the
broader H-03 acknowledgement transaction, H-04 robot-return semantics, or H-08
harness-correlation work, so the overall audit remains a bounded handoff.
Planner admission currently checks required skill arguments but not unsupported
optional arguments. Rejecting model-generated fake-control arguments is a
separate registry-validation hypothesis and needs its own blast-radius audit.

Evidence:

- `runtime_v22_critic_hardening/extreme_all_objects_v27_codex_gpt54.json`
- `runtime_v22_critic_hardening/extreme_all_objects_v28_codex_gpt54_timeout180.json`
- `runtime_v22_critic_hardening/route_ack_explicit_control_v28_codex_gpt54.json`
- `compound_planner_recovery_design_review_2026-07-17.docx`

## 9. Real Ollama backend ablation

The canonical vLLM endpoint at `10.7.138.215:8004` was unreachable during
qualification (`HTTP 000`). The clean v28 image was then launched three times
against the host Ollama daemon, using the same maximal fixture and the same
180-second timeout policy. The temporary Codex-backed compatibility adapter was
not used for these runs.

| Backend | Evidence | Interpretation |
| --- | --- | --- |
| `nemotron-3-super:cloud` | Both preflights passed. Chatbot preserved all six intents and target selection. Planner returned no admissible plan after bounded retries. | Model capability limitation at the planner JSON/plan contract. No partial execution was admitted. |
| `gemma4:31b-cloud` | Both preflights passed. Intent JSON was malformed; retry exhaustion handed off only `wave_greet` without target selection. | Model capability limitation at chatbot structured intent, with the existing route/ack mismatch exposed. |
| `gemma4:cloud` | Both preflights passed. Invalid response and intent JSON triggered route repair; planner recovered navigation plus report only. | Reproduces the known lossy recovery defect under a real Ollama backend. The harness correlated target selection and all four executed steps. |
| `nemotron-3-ultra:cloud` | `/api/chat` returned HTTP 400 `unexpected EOF`; Ollama reported a missing cloud-stub manifest. | Not qualified. |

This ablation narrows the interpretation of the earlier Codex-backed pass. That
adapter used a stateless `codex exec --model gpt-5.4` process and supplied only
the stack request payload, so it had no prior conversation or task context. It
was useful as a deterministic compatibility probe, but it was not an Ollama
hosted Qwen model and cannot establish production-backend capability. The real
Ollama runs show that model choice changes which structured contract fails; they
do not support adding another unconditional fallback.

The CRITIC holdouts remain:

1. A route repair must not publish an execution acknowledgement unless the
   repaired intent and target-selection transaction is consistent.
2. Planner recovery must fail or clarify when it cannot preserve the complete
   requested capability set.
3. Robot destination and object recipient must remain distinct.
4. Harness correlation must continue to adjudicate logs, JSONL lineage, and
   terminal speech separately.

Evidence:

- `runtime_v22_critic_hardening/extreme_all_objects_ollama_nemotron_super.json`
- `runtime_v22_critic_hardening/extreme_all_objects_ollama_gemma4_31b.json`
- `runtime_v22_critic_hardening/extreme_all_objects_ollama_gemma4.json`
