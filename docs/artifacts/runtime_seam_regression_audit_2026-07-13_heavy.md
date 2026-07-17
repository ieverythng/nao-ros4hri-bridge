# Runtime Seam Regression Audit, 13-14 July 2026

## Executive finding

The apparent movement from an 8.2/10 runtime review to a 6/10 safety cap is
not evidence that the whole stack suddenly deteriorated. The two results used
different acceptance strength.

The 8.2 review established that the main ROS trajectory was operational: turns
entered through ROS4HRI, both LLMs responded, planner requests were published,
skills returned feedback, and user-facing speech usually closed the turn. The
latest pass retained that health. The canonical main questionnaire still
reported 20/20 trajectory passes, and focused tests passed 156/156 inside the
exact runtime image.

The later audit added adversarial wording and manually compared grounded entity
roles, requested target sets, executed targets, KB postconditions, and final
reports. That stronger review found cases where a trajectory completed but the
robot completed the wrong semantic task. The main example visited a kitchen,
a table, and people for a request to visit every object, then stated that the
request was complete. This requires a safety cap even though every transport
and lifecycle breadcrumb was present.

The defensible interpretation is therefore:

- **8.2/10 remains evidence of broad runtime availability under the previous
  rubric.**
- **The latest 6/10 cap is a correction to semantic confidence, not a measured
  collapse of every seam.**
- **Several recent changes worked, but their contracts are optional or do not
  reach the final dispatch/scoring boundary.**
- **KnowledgeCore fixture contamination amplified model variability and made
  some later cases incomparable with clean-world cases.**
- **Prompt mutation is not justified yet. Structural payload and validation
  gaps already explain the strongest counterexamples.**

The recommended decision is a bounded handoff. Preserve the recent safety and
recovery changes, then close the structured selection, postcondition, report,
fixture-isolation, and scorer gaps test-first.

## 1. Target contract

### Required runtime behavior

The scored runtime must use the pending deslop checkout and nested
`chatbot_llm` working tree. Dialogue remains owned by `chatbot_llm` and
`dialogue_manager`, executable plans by `planner_llm`, deterministic admission
and dispatch by `nao_orchestrator`, and execution evidence by AB=1 skills.

For a grounded multi-object request, the following facts must survive the full
chain:

1. the selected operation, such as visit or deliver;
2. the exact selected object identifiers;
3. the source location when selection is location-scoped;
4. the person identifier when that person is a recipient;
5. the requested ordering and report policy;
6. execution results for every selected member;
7. a final natural-language closure that covers the evidenced chain.

A transport-successful plan is not a semantic success unless the selected set
and requested postcondition are satisfied.

### Acceptance gates

- Fresh KB objects and predicates are available on the next user turn.
- A person is never described as an object on a table or visited as one of the
  requested objects.
- Supports and locations are not substituted for selected object members.
- Grouped delivery covers every selected object and preserves the named person
  as recipient.
- Ordered navigation visits exactly the selected objects.
- `report_result` or completion wording covers the full evidenced chain.
- Failed KB mutation or skill execution is never reported as success.
- Dialogue-only turns, especially explicit non-action turns, do not publish a
  planner request or KB mutation.
- Failure profiles prove the configured failure occurred and correlate the
  replan and closure with the same goal and plan lineage.
- Questionnaire status checks semantic targets, effects, and closure, not only
  route, feedback, terminal, and speech markers.

### Protected owners

- `chatbot_llm`: route selection, execution admission, structured target
  handoff, and user-facing wording.
- `planner_llm`: plan generation, structured fallback, replan policy, and
  planner dialogue acts.
- `nao_orchestrator`: deterministic validation, dispatch, lineage, and
  execution feedback.
- `planner_common`: normalized cross-node contracts and semantic validators.
- `kb_skills`: KnowledgeCore mutation transport.
- AB=1 skills: fresh action evidence and declared KB effects.
- `dialogue_manager`: lifecycle and speaking.

### Explicit non-goals

- No new lexical route markers or planner phrase recognizers.
- No prompt mutation without a bounded SkillOpt baseline and holdout gate.
- No planner policy moved into `nao_orchestrator`.
- No raw KB cache introduced into either LLM.
- No detector-quality scoring in this audit.
- No claim that fake execution proves real robot motion.

### Evidence budget

This audit uses the completed main, environment, deep-fake, adversarial, unit,
and source-diff artifacts. It does not rerun the live stack. The next thread
should implement only the P0 test batches below before another full scored run.

## 2. Baseline evidence and provenance

### Exact source and runtime state

| Item | Frozen value | Interpretation |
|---|---|---|
| Main checkout | `/home/juanbeck/nao-ros4hri-bridge` | Source corresponding to the scored image |
| Main branch | `refactor/deslop_repo` at `540dfd5` | 32 modified tracked files plus untracked runtime artifacts/helpers |
| Main index | No index-staged changes | The relevant work is working-tree state, despite being described conversationally as staged |
| Remote relation | Local branch behind `origin/refactor/deslop_repo` by two commits | The remote-only delta is thesis and completion-plan documentation, not runtime source |
| Nested checkout | `src/chatbot_llm`, branch `feat/planner_llm_hooks` at `96ecc34` | Dirty adapter, response fallback, and adapter tests |
| Runtime image | `iiia:nao` | `sha256:1a37fc2936af2e0d2b289c5698f780f7d01fe8ad6de1f977621db6a4f045e7e4` |
| Frozen image | `iiia:naofrozen` | Not used or modified |
| Chatbot/planner model | `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` | Both preflights passed on first attempt |
| Routing profile | `response_first` | Grounded-context digest disabled |

Runtime imports matched the pending host files for planner engine, planner
supervisor, planner report outcome, fake engine, fixture validation, chatbot
planner adapter, and chatbot response fallbacks. A stale image is therefore
rejected as the explanation for the observed semantic failures.

### Scored artifacts

- `/tmp/nao_runtime_snapshot_20260713_heavy_preflight.json`
- `/tmp/nao_main_20260713_heavy_changes_clean.json`
- `/tmp/nao_environment_20260713_heavy_changes.json`
- `/tmp/nao_fake_all_success_20260713_heavy_changes.json`
- `/tmp/nao_fake_fail_once_navigation_20260713_heavy_changes.json`
- `/tmp/nao_fake_fail_once_pick_20260713_heavy_changes.json`
- `/tmp/nao_fake_delivery_blocked_20260713_heavy_changes.json`
- `/tmp/nao_fake_recipient_missing_20260713_heavy_changes.json`
- `/tmp/nao_fake_every_other_20260713_heavy_changes.json`
- `/tmp/nao_fake_random_seeded_20260713_heavy_changes.json`
- `/tmp/nao_adversarial_language_20260713_heavy_changes.json`
- `/tmp/nao_runtime_snapshot_20260713_heavy_final.json`

Two earlier partial main artifacts were contaminated by interrupted runs and
are excluded from scoring.

### Aggregate baseline

| Suite | Harness result | Semantic audit result |
|---|---:|---|
| Canonical main | 20/20 pass | Basic dialogue, fresh KB, and simple execution healthy; ordered generic request was false success |
| Environment | 10 pass, 1 not scored | Role leakage, incomplete grouped reports, and failed fixture isolation found |
| Fake all-success | 8 pass, 1 fail | Gold-apple safety guard worked; planner did not recover to a valid handoff |
| Fail-once navigation | 2 degraded | Failure path lacked reliable post-terminal closure |
| Fail-once pick | 1 harness pass | Failure and version-2 replan occurred; expectation did not require full recovery closure |
| Delivery blocked | 2 degraded | Truthful terminal behavior improved, but no complete recovery closure |
| Recipient missing | 1 fail, 1 degraded | Safe clarification was misgraded by success-oriented expectations |
| Every-other and seeded random | 2 fail | Ambiguous stale ALEX entities blocked meaningful policy exercise |
| Focused runtime tests | 156/156 pass | Current tests cover ideal payloads, not the live bypass shapes |

The final snapshot contained 82 deduplicated fallback events: 70 route repairs
and 12 invalid executable plans. These are not all user-visible failures, but
their volume confirms that compatibility repair paths remain part of ordinary
operation rather than exceptional fallback.

## 3. Why 8.2 and the latest cap can both be evidence-backed

### Different theorem strength

The earlier score answered: "Does a complete turn usually cross the intended
ROS seams and close?" The later audit asked: "Did the same turn preserve the
correct entity roles, selected set, postcondition, and natural-language
closure?" A pass at the first level does not imply a pass at the second.

### Changed test pressure

The later run added paraphrases, explicit non-action statements, long-context
references, multiple fixtures with duplicate semantic names, per-member
delivery expectations, and manual inspection of executed target identifiers.
Those cases were not represented by the previous aggregate score.

### Hardening exposed old faults

The new person-as-support validator correctly converted an unsafe gold-apple
plan into a failure. The new cleanup guard correctly refused to score a
contaminated fixture. The new unchanged-replan guard stopped one repeated
blocked plan. Each change can reduce a naive score while increasing system
truthfulness.

### Real variability still exists

Not every discrepancy is a scoring artifact. Equivalent grounded requests
produced different model plans under different wording, and the explicit "do
not act yet" turn leaked to execution after first producing a correct dialogue
acknowledgement. These are current runtime defects. They do not support the
claim that KB transport or the entire planner loop is broken.

## 4. Recent change coverage map

| Recent working-tree change | Intended protection | Live evidence | Coverage gap | Verdict |
|---|---|---|---|---|
| `PlannerRequest.target_selection` normalization | Carry authoritative selected members across planner seam | Explicit unit fixtures produce deterministic deliver/visit plans | Field remains optional; no invariant requires it for grouped or quantified execution | Partial |
| Chatbot adapter derives grouped delivery selection | Recover selection without phrase matching | Ideal test with `user_intent.object`, `recipient`, `bring_object`, and `report_result` passes | Route-repair payloads often omit `object`, `recipient`, or `report_result`; live grouped requests had no selection | Partial |
| Planner `_target_selection_decision` | Build selected object-only fallback | Works when the complete selection exists | Returns `None` on absent/invalid selection and falls through to broad model or lexical fallback | Partial |
| `plan_semantic_errors` person placement check | Stop treating a person as support | Gold-apple `place_object(..., ALEX)` was blocked truthfully | It does not validate visit selections and selected-set coverage is inactive when selection is absent | Proven but narrow |
| Delivery member coverage in `plan_semantic_errors` | Require every selected member to be handed off | Focused tests pass with explicit selection | Orchestrator calls validator without `target_selection`; final dispatch does not re-establish selected-set coverage | Incomplete final gate |
| Supervisor unchanged-replan fingerprint | Stop identical blocked replans | Blocked delivery reached truthful help rather than looping indefinitely | Does not create a new strategy or guarantee natural closure; fingerprint is scoped only to repeated blocking feedback | Proven for bounded loop |
| Fake engine prefers `object_id` in signature | Make fail-once stable across argument aliases | Fail-once pick fired and produced plan version 2 | Does not verify KB post-effects or complete recovery closure | Proven for policy selection |
| Environment fixture validator | Reject fixtures without explicit person/robot location | Packaged fixture validation tests pass | Valid fixture insertion does not imply isolation after prior fake effects | Proven for setup only |
| Questionnaire fixture retraction | Separate environment groups | Guard detected remaining `placeOf` and inferred facts | Subject-outgoing cleanup cannot remove all incoming/cross-fixture causes or inferred closure | Detection proven, cleanup failed |
| Post-terminal speech scoring | Stop acknowledgements satisfying recovery closure | Fail-once navigation and blocked delivery became degraded | Applied mainly to execution expectations; some `observe` cases bypass it; timestamp correlation is not goal-lineage correlation | Partial |
| Semantic name projection in compact entities | Prefer `dbp:name` and stable `codex_` identifiers | Names remained visible in grounded context | Effect on duplicate-name disambiguation and role selection is untested | Unproven |
| Registry mapping regex hardening | Avoid malformed fake alias projection | No regression observed | Unrelated to the semantic counterexamples | Neutral |
| SVG and location fixture updates | Improve environment readability and valid placement | Fixtures load and expose explicit locations | Does not constrain selected task members | Orthogonal |

### High-impact final-gate omission

`planner_llm` calls:

```python
plan_semantic_errors(steps, request.grounded_context, request.target_selection)
```

The new orchestrator guard calls:

```python
plan_semantic_errors(validated_steps, data.get('grounded_context', {}))
```

The third argument is omitted. The orchestrator therefore enforces the person
placement rule but cannot enforce selected delivery member coverage. It also
has no selected-visit validation. This does not mean the orchestrator should
select targets. It means the authoritative selection contract must accompany
the accepted plan so the deterministic owner can verify that the planner did
not change its meaning.

## 5. Failure topology

```text
user turn
  -> chatbot response and route
     -> optional user_intent.object / recipient / intent_sequence
        -> planner_request_adapter
           -> target_selection present? -----------------------+
              | yes                                            | no
              v                                                v
        bounded member set                              broad grounded inventory
              |                                                |
              v                                                v
        planner selected fallback                    model or lexical fallback
              |                                                |
              +------------------- plan ------------------------+
                                      |
                                      v
                         planner semantic validation
                         (selection-aware only if present)
                                      |
                                      v
                       orchestrator semantic validation
                       (currently selection-unaware)
                                      |
                                      v
                               AB=1 execution
                                      |
                                      v
                         report_result or completion
                         (may cover only latest result)
                                      |
                                      v
                       trajectory questionnaire pass
                       (does not compare selected set)
```

Three optionality points align in the failing cases:

1. target selection may be absent;
2. final dispatch does not receive it;
3. scoring does not require it.

The result is a complete ROS trajectory with no component responsible for
proving that the executed set equals the requested set.

## 6. Approach registry

| ID | Hypothesis family | Discriminating probe | Evidence | Status | Exact gap or reopen condition |
|---|---|---|---|---|---|
| H1 | Stale runtime image | Compare runtime imports with dirty host files | Seven runtime-critical hashes matched | Rejected | Reopen only after a new image/source mismatch |
| H2 | General LLM connectivity failure | Dialogue and KB preflight plus ordinary turns | Both preflights passed; no backend-unreachable speech | Rejected | Reopen on provider timeout or fallback evidence |
| H3 | General KB ingestion delay | Insert fresh entities, then ask name, relation, and count | Main fresh KB block passed 5/5 on next turns | Rejected as general cause | Reopen with a clean-world missed predicate and timestamped query evidence |
| H4 | Missing structured target selection | Compare live planner requests with complete grounded groups | Grouped and ordered failures had full context but no `target_selection` | Supported, high confidence | Need end-to-end schema-required selection test |
| H5 | Planner ignores valid selection | Replay identical request with and without explicit selection | Unit tests prove explicit selection drives bounded fallback | Rejected for valid selection; supported for absent-selection fallback | Need paired live ROS probe |
| H6 | Final deterministic validator protects selection | Inspect orchestrator call and plan envelope | Orchestrator omits selection argument | Rejected | Close by propagating and testing selection at admission |
| H7 | Report loss begins in report-result skill | Compare plan steps with final wording | Failing grouped plan often omitted `report_result` and enabled completion | Not primary cause | Reopen if a complete report step still yields partial wording |
| H8 | Report intent is lost before planning | Compare complete `goal_text`, normalized intents, selection report policy, and plan | Goal requested reporting; normalized intents omitted `report_result`; selection absent or `none` | Supported | Add structured report policy and holdout |
| H9 | Fake server unavailable | Exercise all-success and fail-once aliases | Fake feedback and fail-once pick occurred | Rejected as general cause | Reopen on missing action/service evidence |
| H10 | Fake success effects are complete and atomic | Query every member after grouped delivery | Kitchen cup remained on table after apparent completion | Rejected | Need per-member effect journal and postcondition query |
| H11 | Fixture cleanup restores isolation | Run sequential fixture groups and absence guard | Guard found residual `placeOf` and inferred location facts | Rejected | Reopen after namespace/effect-aware cleanup passes twice |
| H12 | Long context alone loses KB facts | Compare main long dialogue KB sequence with contaminated fixture sequence | Main sequence passed; failures track duplicate identities and group semantics | Rejected as sole cause | Reopen with clean isolated long-context miss |
| H13 | Prompt drift is the root cause | Compare equivalent wording and structural payload completeness | Wording changes plans, but missing contracts independently explain unsafe acceptance | Blocked as first intervention | Requires SkillOpt baseline and structured holdouts |
| H14 | Existing deterministic route repair is sufficient | Explicit "do not act yet" adversarial turn | Correct dialogue ack was followed by `kb_add` execution and second failure speech | Rejected | Need response-contract retry/clarification without adding phrase markers |
| H15 | Questionnaire pass means semantic success | Compare passed cases with target roles, effects, and final text | Pass awarded to non-object navigation and incomplete grouped report | Rejected | Add semantic oracle and lineage correlation |
| H16 | Person/object projection is consistently safe | Inspect table descriptions and ordered targets | Person described on table; people visited as requested objects | Rejected | Need role assertions at projection, selection, plan, and score boundaries |
| H17 | Recent semantic-name change caused the regression | Paired old/new compact projection on same KB graph | No paired evidence available | Blocked | Run deterministic projection diff before editing labels |
| H18 | Replan transport is generally broken | Fail-once pick and blocked delivery | Plan version 2 and truthful help were observed | Rejected as general cause | Specific recovery strategies remain incomplete |

## 7. Discriminating probes and results

### 7.1 Main dialogue and KB sequence

Basic dialogue, fresh insertion, exact name, color/support relation, and count
all passed after evidence settled. This rejects a general KnowledgeCore
freshness or LLM backend failure. It does not prove isolation across fixture
groups or correct task selection from a crowded graph.

### 7.2 Generic ordered-object navigation

The request was:

> Walk to every object and let me know when you get to each one.

The robot reported arrival at `kitchen`, `table_1`, a recipient person, and an
anonymous person, then stated that all requested navigation was complete. The
harness marked the case pass because route, planner request, feedback,
terminal, and speech evidence were present.

This is the strongest safety counterexample. It proves all of the following:

- grounded candidate inventory was not reduced to object members;
- the planner or fallback accepted location and person roles as targets;
- final validation did not reject extra or wrong-kind targets;
- closure did not compare reported targets with requested targets;
- trajectory scoring can certify false success.

### 7.3 Deep-fake ordered table walk

The grounded context exposed two table location groups and five object members.
The planner request did not contain `target_selection`. The resulting plan
navigated to table identifiers rather than the contained object identifiers and
inserted report steps around those targets. Again, the harness passed the
trajectory.

This case narrows H4 further: the new selected-member fallback was not wrong;
it was never eligible.

### 7.4 Grouped work-table delivery

The planner request contained a complete goal and a grounded work-table group
with cup, manual, and phone members plus ALEX as a person. It contained no
`target_selection`. The planner used `grounded_location_group_fallback`,
executed three `bring_object` steps, omitted `report_result`, enabled completion
speech, and finally reported only the phone.

The execution chain may have run all three skills, but the user-facing contract
closed only the last result. This is not evidence that chatbot-authored report
generation failed. The report step and explicit final-report policy were absent
before report generation was invoked.

### 7.5 Kitchen delivery and post-effect query

The grouped kitchen turn reported only the final book. The immediate follow-up
stated that the cup was still on the table. This distinguishes two defects:

1. incomplete closure, because only one member was described;
2. incomplete or contradictory KB effect, because the earlier cup state
   remained queryable after apparent delivery success.

### 7.6 Gold-apple handoff

The model proposed `place_object` with ALEX as the placement support. The new
semantic validator blocked the plan and chatbot wording truthfully explained
that a person is not a placement surface. This is a successful recent change.

The remaining gap is recovery: the planner did not reformulate the task as a
handoff through `bring_object` or `deliver_object`. A safety rejection should
remain a rejection until a separately validated strategy exists.

### 7.7 Natural-chat KB mutation

"Add a red cup to your KB" correctly reached `route=execution` and
`intent=kb_add`. The planner emitted predicate-only statements, including
`dbp:color: red`, without an RDF subject. The orchestrator rejected the request
and the chatbot explained the failure truthfully.

Safety and wording were healthy. Mutation synthesis was not. This seam is
largely untouched by the recent target-selection and recovery changes.

### 7.8 Fixture cleanup and identity contamination

The cleanup guard found remaining facts on `codex_recipient_person`, including
`placeOf codex_skill_phone`, `placeOf codex_table_object`, and inferred location
types. Later state contained three entities named ALEX:
`codex_gold_recipient`, `codex_iiia_alex`, and `codex_lab_alex`.

The new cleanup code queries and retracts outgoing facts for subjects declared
by the previous fixture. It does not maintain a per-case effect journal, remove
all cross-subject causes, or prove that reasoning closure has settled after
retraction. Detection is correct; isolation is not restored.

### 7.9 Deep fake recovery policies

- `all_success`: 8/9 trajectory pass. Gold-apple placement was rejected safely.
- `fail_once_navigation`: both selected cases degraded because no reliable
  post-terminal recovery closure was observed.
- `fail_once_pick`: the failure fired and planner version 2 appeared. The case
  retained an `observe` expectation, so the harness pass did not prove full
  recovery closure.
- `delivery_blocked`: repeated execution stopped, but complete closure did not
  follow terminal evidence.
- `recipient_missing`: safe clarification was penalized by a success-oriented
  case expectation.
- `every_other` and seeded random: duplicate ALEX identities blocked the task
  before the configured fake policy could be meaningfully assessed.

### 7.10 Explicit non-action turn

The turn "Remember that ALEX is the recipient ... Do not act yet" first
produced an appropriate dialogue acknowledgement. It then leaked into a
`kb_add` execution path and produced a second RDF-format failure utterance.

This is not an orchestrator duplicate relay of one event. It is two semantic
decisions for one user turn: dialogue followed by unintended execution. The
relevant seam is chatbot response/route contract resolution before speech and
planner handoff. Adding another phrase marker for "do not act" would repeat the
reward-hacking pattern already present in `route_heuristics.py`.

## 8. Untouched or insufficiently touched seams

### 8.1 Chatbot execution-admission completeness

The new adapter can preserve or derive `target_selection`, but the chatbot
response schema does not make a bounded selection mandatory for grouped
execution. Route repair can still construct execution from a verbal
acknowledgement and rule-derived intent while leaving source, recipient,
ordering, and report policy empty.

**Required direction:** an execution response that needs grounded selection
must either contain a valid structured selection or be retried/returned to
chatbot clarification before its acknowledgement is spoken. Do not infer the
missing contract from new phrase lists.

### 8.2 Planner absent-selection policy

The planner treats missing `target_selection` as permission to continue into
the model and legacy fallbacks. For multi-member tasks this preserves the old
ambiguity. The planner cannot distinguish "selection intentionally broad" from
"selection contract silently lost."

**Required direction:** make selection-required operations explicit in the
request contract. Missing selection should produce structured clarification or
validation failure, not broad inventory planning.

### 8.3 Final orchestrator semantic validation

The orchestrator owns deterministic plan admission but currently receives no
selection argument in its semantic validator call. Selected-set equality and
role checks are therefore not final invariants.

**Required direction:** propagate the normalized selection alongside the plan
metadata and validate exact member coverage, recipient identity, operation, and
forbidden extra targets. The orchestrator verifies; it does not choose.

### 8.4 Report policy and whole-chain closure

`request_requests_report()` only checks whether normalized intents contain the
exact `report_result` label. The complete `goal_text` may ask for a report while
that label is missing. Some planner fallbacks add a goal-text compatibility
check, but grouped delivery does not consistently do so. When `report_result`
is absent, completion wording can reflect the latest result rather than the
selected chain.

**Required direction:** carry `report_policy` as structured request data and
validate it against the plan. The report outcome must compare completed events
with selected members before chatbot wording.

### 8.5 Fake skill effect verification

The fake engine now chooses stable policy signatures, but success payloads and
KB mutations are not scored as one atomic postcondition. A successful series
can leave earlier members at their source.

**Required direction:** journal intended and applied effects per goal/step and
query each selected member after completion. A fake success is still required
to satisfy its declared symbolic effect.

### 8.6 KnowledgeCore fixture isolation

Fixture validity and cleanup detection improved. Run isolation remains weak
because inferred relations and skill-created cross-fixture effects outlive the
fixture group.

**Required direction:** use run-unique fixture namespaces and an explicit
asserted-effect journal. Retract causal asserted facts, wait for rematerialized
closure, then prove both outgoing and incoming fixture relations are absent.

### 8.7 Questionnaire semantic oracle

The scorer knows whether a turn entered, planned, executed, terminated, and
spoke. It does not know what should have been selected, what was forbidden, or
which final KB relations must hold.

**Required direction:** extend cases with expected selected IDs or role-based
selectors, forbidden kinds, expected skill sequence shape, report coverage,
and postcondition queries. Correlate all evidence by `goal_id`, `plan_id`,
`plan_version`, and case start time.

### 8.8 Natural-chat KB mutation schema

The mutation path rejects malformed RDF but does not reliably construct a
subject-qualified statement set from natural language.

**Required direction:** make mutation subject, predicate, and object an
explicit structured contract before planner execution. Preserve `kb_skills` as
the mutation boundary.

### 8.9 Route heuristics and acknowledgement consistency

The recent dirty changes did not materially alter the large deterministic
route-repair layer. Existing markers include action verbs and KB mutation words
such as "remember." This can override incomplete LLM output and create an
execution route after a conversational acknowledgement.

**Required direction:** require valid explicit route and execution-admission
fields from the chatbot response. On inconsistency, retry once or return a
chatbot-authored clarification. Do not add more markers.

### 8.10 Compact entity naming

The latest contract prefers `dbp:name` and preserves `codex_` identifiers in
some display paths. This may improve readability, but duplicate semantic names
still require stable IDs and role fields. No current evidence proves this
change caused or solved selection ambiguity.

**Required direction:** run a paired projection fixture with duplicate ALEX
names, distinct IDs, locations, and roles. Compare the exact chatbot and
planner JSON before changing naming code.

## 9. Test coverage gap matrix

| Existing test proves | Missing counterexample |
|---|---|
| Explicit `target_selection` survives adapter normalization | Route-repaired execution with no object/recipient slots must not hand off an unbounded request |
| Ideal grouped delivery derives selection from exact structured fields | Equivalent grounded request with omitted optional fields must retry or clarify |
| Planner builds selected delivery/visit fallback | Extra location/person targets and missing selected members must fail validation |
| Person cannot be `place_object` support | Planner must recover to handoff only when registry and grounding support it |
| Delivery selection detects missing handoff members in planner | Orchestrator must receive and enforce the same selection |
| Unchanged blocking replan is stopped | Changed but semantically equivalent blocked plan and natural terminal closure |
| Fail-once uses `object_id` alias | Failure, replan, effect, terminal, and closure all correlate to one goal |
| Fixture has explicit robot/person location | Sequential fixtures leave no incoming, outgoing, or inferred cross-run effects |
| Speech appears after a terminal timestamp | Speech belongs to the same goal and covers the expected selected set |
| Fresh KB facts answer ordinary queries | Clean long-context grouped task preserves exact member and recipient roles |
| Canonical prompt fields exist | No prompt drift assessment is currently accepted; SkillOpt remains required |

## 10. Adversarial audit

### Ownership

The recommended changes preserve ownership. `chatbot_llm` provides a complete
execution-admission contract, `planner_llm` plans within it,
`nao_orchestrator` verifies and dispatches it, skills provide evidence, and
chatbot/dialogue management owns wording and speech.

### Lineage

Recent replan tests prove plan version changes can occur. The questionnaire
still uses broad log-window markers in some cases. Historical or unrelated
failure lines can therefore satisfy policy-exercise checks. Every new semantic
oracle must correlate goal, plan, version, step, and timestamp.

### Duplicate speech

No direct duplicate planner-act relay was established in the inspected
canonical cases. The explicit non-action counterexample produced duplicate
semantic handling through dialogue followed by unintended execution. The fix
belongs before planner handoff, not in another speech dedupe ledger.

### Grounded truth

The system fabricated completion when it claimed an object sequence was
complete after visiting non-object targets. It also exposed contradictory
post-effects after grouped delivery. These are acceptance-blocking regardless
of fake or real execution mode.

### People and objects

Person/object separation is enforced for `place_object` support, but not yet at
every target-selection and ordered-navigation boundary. This invariant must be
checked at projection, request, plan, dispatch, effect, report, and score.

### Prompt discipline

No prompt edit is accepted by this audit. The current strongest failures have
source-level contract explanations. Any later prompt change must use
SkillOpt with paired train/holdout cases for dialogue route safety, grounded KB
queries, grouped execution admission, report closure, and duplicate speech.

### Registry and AB levels

No evidence suggests registry ownership or AB-level projection caused the
wrong-target cases. The fake server was available and exercised. Registry
changes should remain out of the first repair batch.

## 11. Decision: bounded handoff

### Accepted findings

1. Runtime/source mismatch is rejected.
2. General LLM and KnowledgeCore transport failure is rejected.
3. Structured target selection is the correct direction but is not a total
   execution contract.
4. The new person-placement and unchanged-replan protections worked.
5. The final orchestrator guard does not receive target selection.
6. Report intent and selected-set closure are not reliably preserved.
7. Fixture cleanup detects contamination but does not restore isolation.
8. The questionnaire can pass semantically wrong execution.
9. Explicit non-action can still leak to execution through the existing route
   repair path.

### Rejected first interventions

- adding more action or non-action phrase markers;
- broad prompt rewriting;
- moving selection or wording into the orchestrator;
- suppressing failures to recover the previous numeric score;
- treating a clean route/terminal/speech trajectory as semantic proof;
- changing the fake registry before postcondition verification exists.

### Runtime readiness decision

The pending working tree is **not ready to freeze as the final validation
baseline**. It is materially safer than the earlier state in person-placement
and repeated-block handling, but complete grouped selection and truthful
closure are still optional. The 6/10 cap remains appropriate until the
wrong-kind ordered target case and explicit non-action execution leak are
closed.

## 12. Test-first repair sequence

### P0-A: Make execution selection total

**Files likely involved**

- `src/chatbot_llm/chatbot_llm/planner_request_adapter.py`
- chatbot response schema/validation modules identified by source tracing
- `src/planner_common/planner_common/contracts.py`
- `src/planner_llm/planner_llm/planner_engine.py`

**Red tests**

1. A grouped execution response missing source, recipient, or selected members
   does not publish an executable planner request.
2. A valid structured response preserves source, members, recipient, ordering,
   and report policy without goal-text phrase matching.
3. A visit selection accepts only object members from the selected group.
4. Missing selection for a selection-required operation yields structured
   clarification or validation failure.

**Acceptance**

No broad model or lexical fallback is eligible for a selection-required request
without the structured selection contract.

### P0-B: Re-establish semantics at final admission

**Files likely involved**

- `src/planner_common/planner_common/report_outcome.py` or a focused semantic
  validation module if this file becomes too broad
- `src/nao_orchestrator/nao_orchestrator/intent_rules.py`
- planner/orchestrator contract tests

**Red tests**

1. Selected visit plan with a person, location, or extra object fails.
2. Selected delivery missing one member fails.
3. Selected delivery using a different recipient fails.
4. The orchestrator receives the same normalized selection used by planner
   validation.

**Acceptance**

The final accepted plan's operation, targets, recipient, ordering dependencies,
and report policy are a valid refinement of the chatbot-owned selection.

### P0-C: Make report policy and closure explicit

**Files likely involved**

- `src/planner_common/planner_common/contracts.py`
- `src/planner_common/planner_common/report_outcome.py`
- planner request and report-result integration code
- chatbot system-turn execution-report code

**Red tests**

1. Final report policy requires one terminal `report_result` even when
   normalized intents omit the exact label.
2. Per-target report policy requires correctly dependent reports for every
   selected visit.
3. Whole-chain outcome contains every successful member and every failure.
4. Chatbot wording cannot claim completion when selected-member evidence is
   incomplete.

**Acceptance**

The final user-facing message is generated from full plan evidence and selected
postconditions. Last-step-only completion is not eligible for grouped tasks.

### P0-D: Isolate KB fixtures and verify fake effects

**Files likely involved**

- `.codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py`
- fixture helpers under `src/nao_chatbot`
- fake skill effect/mutation integration

**Red tests**

1. Two fixtures with the same semantic name but different IDs remain isolated.
2. Cleanup removes outgoing and incoming asserted causes, then waits for
   rematerialization.
3. Every delivered member satisfies its recipient/location postcondition.
4. Failed/replanned steps do not leave success effects.

**Acceptance**

Each scored environment group starts with a proven clean namespace and ends
with a per-member postcondition report.

### P0-E: Upgrade the questionnaire semantic oracle

**Files likely involved**

- `.codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py`
- its focused test module and case definitions

**Red tests**

1. The exact false-success ordered trace is graded fail.
2. A grouped delivery reporting only the final member is graded fail.
3. Safe clarification under `recipient_missing` is accepted for that profile.
4. A failure profile with no same-goal failure evidence is not scored.
5. Historical mirrored events cannot satisfy current-case recovery evidence.

**Acceptance**

No case receives pass unless requested targets, executed targets, postconditions,
and closure agree under the same lineage.

### P0-F: Close dialogue-to-execution leakage

**Files likely involved**

- chatbot response schema/parser and `turn_engine.py`
- `route_heuristics.py` only to remove superseded repair behavior after tests,
  not to add markers

**Red tests**

1. Explicit dialogue route plus conversational acknowledgement cannot publish a
   planner request in the same turn.
2. Missing or contradictory route triggers one schema retry, then safe chatbot
   clarification.
3. A valid immediate execution request still reaches the planner.
4. The acknowledgement is not spoken until route and execution-admission
   fields pass validation.

**Acceptance**

One user turn has one validated semantic decision before speech. No phrase list
is added.

### P1: Natural-chat KB mutation contract

Add a typed subject/predicate/object mutation payload and validate it before
planner execution. Test add, revise, remove, missing subject, and ordinary
non-mutating KB questions.

### P1: Compact naming and duplicate identities

Run a paired projection test before changing naming logic. The test must retain
stable IDs, semantic labels, kinds, and locations for two people with the same
name and prove deterministic recipient clarification.

## 13. Residual risk and next probes

### Next discriminating source probe

Construct two `PlannerRequest` payloads with identical goal and grounded
context:

1. one with complete `target_selection`;
2. one without it.

The complete case must produce only selected object steps and the requested
report. The absent case must stop before broad planning. Feed both resulting
plans through orchestrator validation to prove the same invariant exists at
both boundaries.

### Next clean live probe

Use a run-unique fixture namespace containing:

- one table with exactly three object members;
- one second table with unrelated objects;
- two people with distinct IDs and names;
- one recipient at a grounded handoff location.

Run:

1. ordered visit of every object on the first table with per-target reporting;
2. grouped delivery of those objects to the named recipient with final report;
3. immediate KB queries for every object's final location;
4. one fail-once member and one blocked member;
5. one dialogue-only future or memory turn followed by an explicit execution
   turn.

Score exact IDs, kinds, step lineage, effects, and report coverage. Do not reuse
the current accumulated KB graph.

### Reopen conditions

- Reopen prompt causality only after structural contracts pass and equivalent
  wording still changes a valid selected plan.
- Reopen general KB freshness only after a clean isolated next-turn fact miss.
- Reopen fake-server availability only after action/service evidence is absent.
- Reopen compact naming causality only after a paired projection diff.
- Raise the score cap only after the false-success ordered trace and explicit
  non-action leakage are represented by failing tests and pass in a clean run.

## 14. Handoff checklist

- [ ] Preserve the working person-placement guard.
- [ ] Preserve the unchanged-block replan guard.
- [ ] Preserve fake `object_id` policy normalization.
- [ ] Make target selection mandatory where operation semantics require it.
- [ ] Propagate selection to final orchestrator admission.
- [ ] Validate selected visit and delivery set equality.
- [ ] Carry structured report policy independent of normalized intent loss.
- [ ] Build whole-chain outcome before chatbot wording.
- [ ] Journal and verify every fake skill KB effect.
- [ ] Isolate fixture namespaces and remove cross-run causes.
- [ ] Add semantic and lineage-aware questionnaire assertions.
- [ ] Replace route inconsistency inference with schema retry/clarification.
- [ ] Run SkillOpt before any prompt wording mutation.
- [ ] Repeat full, environment, and deep-fake suites from a clean KB state.

## 15. Validation already completed

- Focused tests in the exact runtime image: 156 passed.
- Runtime-critical host/container source hashes: matched.
- `git diff --check`: passed before this documentation expansion.
- ROS4HRI working-tree audit: completed and identified the expected sensitive
  packages.
- Live runtime and KnowledgeCore evidence: available in the listed `/tmp`
  artifacts.

The report itself changes no runtime source, prompt text, registry, launch
profile, or fake policy.
