# TFM Completion Masterplan

**Status:** Active thesis-closure tracker  
**Created:** 2026-07-13  
**Working branch:** `refactor/deslop_repo`  
**Latest committed thesis pass:** `08f9129` (`feat(TFM): First ablation of Chapter 5 and 6`)  
**Runtime baseline:** the stack is implementation-complete enough for final evaluation; remaining runtime work is evidence closure and narrowly scoped stabilization, not architectural expansion.  
**Paired view:** `docs/plans/tfm_completion_masterplan_2026-07-13.html`

## 1. Purpose And Editing Rule

This is the canonical closure plan for the written TFM. The repository-wide
implementation plan remains `docs/plans/nao_ros4hri_masterplan.md`; the full and
fake runtime trackers remain the detailed evidence ledgers. This file turns
those engineering records into a finite manuscript, experiment, synthesis, and
submission programme.

Juan is currently making hand edits in the LaTeX project. Do not rewrite or
normalise those edits automatically. Before every thesis edit:

1. inspect the current working-tree diff;
2. distinguish hand-authored text from the last committed baseline;
3. propose or apply only the explicitly requested slice;
4. compile locally and visually inspect affected pages;
5. preserve generated evidence and source revision provenance.

## 2. Sources Of Truth

Use the following precedence when a thesis sentence, tracker, and source file
disagree:

1. current source, tests, launch files, package-local `AGENTS.md`, and the
   canonical skill registry;
2. `docs/contracts.md`, `docs/current_workflow.md`, `docs/launch_profiles.md`,
   and accepted architecture decisions;
3. `ISSUE_TRACKER_FULL_SUITE.html` and `ISSUE_TRACKER_FAKE_SUITE.html` for
   dated runtime evidence;
4. `nao_ros4hri_masterplan.md` for cross-track implementation status;
5. Chapter 7 and historical artifacts, which must be updated when superseded.

The IIIAV3 review is an editorial and methodological correction ledger. Its
recommendations must still be checked against the local branch before they are
turned into implementation claims or exact ROS types.

## 3. Thesis Definition Of Done

The TFM is closed only when all of the following are true:

- [ ] The evaluated source commit, container image, prompt packs, registry, model
  settings, launch profile, and fixtures are frozen and recorded.
- [ ] Every research question maps to at least one declared metric and one
  results subsection.
- [ ] The final case manifest fixes case IDs, fixtures, expected routes, allowed
  plans, forbidden observations, repetitions, seeds, timeouts, and pass rules.
- [ ] The canonical `response_first` speech-ingress run has a complete evidence
  bundle with no unexplained fallback inference.
- [ ] The deterministic fake multi-step score is reported separately from the
  full-stack score and physical-robot evidence.
- [ ] Detector-enabled evidence is reported separately from KB-authored or
  preloaded-environment grounding.
- [ ] Chapter 5 distinguishes design invariants from empirically demonstrated
  behaviour and provides implementation/API provenance.
- [ ] Chapter 6 contains units of analysis, variables, profiles, acceptance
  rules, metric formulae, reset policy, repetitions, statistics, and threats to
  validity.
- [ ] Chapter 7 is rebuilt from the final artifacts; obsolete May-only status
  language is removed or retained solely as historical comparison.
- [ ] Chapters 8--10 answer the research questions using the final results and
  do not promise unfinished experiments.
- [ ] All diagrams, tables, citations, terminology, cross-references, and PDF
  pages pass visual QA.
- [ ] The final PDF and an archival evidence manifest can be regenerated from a
  clean checkout.

## 4. Current Baseline

### 4.1 Evidence already available

| Evidence | Current standing | Closure decision |
|---|---|---|
| 17 June response-first targeted speech sweep | Accepted historical success-path baseline, `8.6/10` | Reuse as longitudinal evidence; do not present as the final canonical run. |
| Fresh response-first validation session | Tracker reports `8.7/10` with speech ingress, KB mutation, fake guards, grouped delivery, and KB post-effects | Reuse only when the artifact bundle and exact revision are recoverable; otherwise rerun the corresponding final manifest. |
| 8 July fake multi-step speech run | Qualified `8.6/10`; `all_success` and `fail_once_navigation` passed `8/8`, `fail_once_pick` passed `7/8`, `delivery_blocked` passed `6/8` | Use for the policy heatmap and failure analysis, clearly marked as qualified. Rerun unstable cells after final freeze. |
| Direct fake action guard matrix | Structured success/failure substrate proven | Use as implementation verification, not as planner-recovery or hardware evidence. |
| Environment fixtures and stale-world guards | Implemented and exercised in the runtime-review harness | Include in methodology; final scored cases must record fixture and guard results. |
| Intent-first probes | Useful ablation evidence with known dialogue and planning regressions | Keep separate from the primary score; run only the frozen ablation subset required by the final claim. |

### 4.2 Remaining evidence gaps

- A clean rebuilt canonical `response_first` run after the 9 July
  `report_outcome`, KB cleanup, and top-level assessment changes.
- Stable long-suite evidence for `fail_once_pick` and `delivery_blocked`, or an
  explicit qualified limitation if the extraction flake persists.
- A frozen detector-enabled profile for object-recognition evidence; the cool
  profile's disabled detector cannot be scored as a detector failure.
- Final per-case CSV/JSON summaries generated from saved traces.
- A declared policy for whether the direct-execution comparison promised in the
  abstract is implemented, replaced with a static/no-replan baseline, or removed.
- Exact repetitions, seeds, timeout policy, and statistical treatment.

## 5. Chapter Closure Matrix

| Chapter | Current state | Required closure work | Gate |
|---|---|---|---|
| Front matter and abstract | Formatted; abstract still uses future tense and promises direct comparison | Rewrite last, after the evidence and baseline decision are frozen | Claims match Chapters 6--10 exactly |
| 1 Introduction | Substantive draft present | Correct language, align objectives and RQs with final metrics and contributions | Every RQ has a result and conclusion answer |
| 2 Background | Substantive draft present | Citation audit; define/cite KnowledgeCore, vLLM, prompt optimization, and recent related work | No unsupported implementation claim in literature review |
| 3 Architecture | Strong diagrams and requirements | Terminology and typo pass; ensure future components are visually marked | Architecture agrees with current contracts and grayscale output |
| 4 Runtime Contracts | Detailed and largely complete | Repair formatting artifacts; verify examples and types against frozen source; add breakable long identifiers | Contract examples validate and no overfull critical text remains |
| 5 Implementation | Expanded; hand edits in progress | Apply IIIAV3 precision corrections, add provenance/API inventory, supervisor invariants, truthful reporting mechanism, observability, and final configuration table | Every major claim points to source/config/interface evidence |
| 6 Validation Methodology | Good scaffold but under-specified experiment | Freeze unit, variables, profiles, manifest, acceptance rules, resets, repetitions, formulae, statistics, missing-event policy, ablations, and threats | Another researcher could repeat and score the suite |
| 7 Results | Outdated preliminary May-centred pass | Replace with final/qualified July evidence, quantitative tables, figures, errors, latency, and RQ-organised synthesis | Every number has an artifact and denominator |
| 8 Discussion | Short preliminary interpretation | Expand by RQ, compare with literature, explain modularity/recovery/grounding trade-offs, separate findings from speculation | Claims do not exceed evidence |
| 9 Limitations and Future Work | Very short | Separate observed limitations, scope limits, and future extensions; include detector, hardware, model, fixture, and evaluation threats | No thesis-blocking task is disguised as future work |
| 10 Conclusion | Preliminary | Rewrite after Results/Discussion; answer each RQ and state contributions in past tense | Contains no new result or future-tense promise |
| Appendices | Minimal | Add frozen manifest, runtime/config table, interface inventory or generated registry table, commands, and artifact index | Main text stays readable while reproduction details remain available |

## 6. IIIAV3 Correction Register

### P0: Claims and methodology

- [x] **TFM-C01:** Replace runtime “guarantees” with enforced constraints or
  design invariants unless the final evidence demonstrates the behaviour.
- [x] **TFM-C02:** Keep internal registry decomposition terminology out of the
  TFM-facing vocabulary. Describe the reviewed skill registry, runtime
  availability, executor mappings, and skill contracts directly.
- [ ] **TFM-C03:** Decide the direct-execution baseline. Implement a fair atomic
  baseline, substitute an explicitly defined static/no-replan baseline, or
  remove the comparison promise from the abstract.
- [ ] **TFM-C04:** Freeze the experimental unit as one execution under a fixed
  tuple of utterance, profile, fixture, model, prompt revision, skill policy,
  seed, and source revision.
- [ ] **TFM-C05:** Add acceptance rules and denominators for every reported
  metric. A timeout or log-derived inference cannot silently count as success.
- [x] **TFM-C06:** Rename “Deep Fake-Skill” to “Multi-Step Deterministic
  Fake-Skill” throughout manuscript-facing text.

### P1: Chapter 5 implementation precision

- [ ] **TFM-I01:** Add implementation scope and frozen source revision.
- [ ] **TFM-I02:** Add a source-to-responsibility provenance map.
- [ ] **TFM-I03:** Add a consolidated ROS interface inventory with interface,
  message/action type, owner, QoS, lifecycle creation stage, profile, timeout,
  and real/fake status. Verify exact types from source.
- [x] **TFM-I04:** Define compact grounded context operationally. Use the exact
  schema version only if it is consistently versioned in source and artifacts.
- [x] **TFM-I05:** Define `response_first` and `intent_first`: stage order,
  route authority, shared prompt context, and baseline/ablation role.
- [ ] **TFM-I06:** Add planner output admission pseudocode: prompt, extraction,
  normalization, deterministic validation, bounded repair, publish/fail.
- [ ] **TFM-I07:** Expand supervisor invariants and state transitions for
  `goal_id`, `plan_id`, `plan_version`, stable `step_id`, waiting, cancellation,
  supersede, stale feedback, and replan join.
- [ ] **TFM-I08:** Describe planner-time summary stripping/rejection for
  `report_result` and executor-side construction from live result payloads.
- [ ] **TFM-I09:** Describe planner-gate admission, dialogue-act deduplication,
  execution report accumulation, and the orchestrator's non-planning boundary.
- [ ] **TFM-I10:** Replace “stabilises object identities” with the implemented
  matching/recency behaviour and define `visible`/freshness semantics.
- [ ] **TFM-I11:** Document fake-policy precedence and metadata. Retain KB guards
  and post-effects only where final source and tests prove them.
- [ ] **TFM-I12:** Add final runtime configuration and SkillOpt definition.
- [x] **TFM-I13:** Ground the `dialogue_manager` subsection in its ROS4HRI
  multi-person/group dialogue lifecycle, history, chatbot, and chat/ask/say
  interfaces without moving planning ownership into the package.
- [x] **TFM-I14:** Cross-link implementation statements to functional
  requirements and Contracts 1--3 and 7 where the corresponding runtime
  constraint is enforced.

### P1: Chapter 6 experimental completeness

- [ ] **TFM-M01:** Add independent and dependent variables.
- [ ] **TFM-M02:** Add profile table: E2E main, planner-isolated, fake
  multi-step, detector-enabled, and intent-first ablation.
- [ ] **TFM-M03:** Freeze case manifest with route, required/permitted/forbidden
  skills, order constraints, dialogue stages, terminal state, timeout, fixture,
  and replan policy.
- [ ] **TFM-M04:** Define exactly-once speech per semantic stage, allowing one
  acknowledgement and one terminal report only when the declared policy expects
  both.
- [ ] **TFM-M05:** Define reset sequence: active goal, fake counters/policy, KB
  fixture, scene stabilization, node readiness, trace start, then injection.
- [ ] **TFM-M06:** Add metric formulae for route accuracy, valid-plan rate,
  skill-set coverage, order correctness, safe-failure rate, phase completion,
  clarification precision/recall, speech multiplicity, and stale-state rate.
- [ ] **TFM-M07:** Report latency using median, IQR, 95th percentile, maximum,
  and timeout count; retain mean only as a supplementary statistic.
- [ ] **TFM-M08:** Define repetitions, seeds, exclusions, infrastructure retry
  policy, missing events, `degraded`, `fail`, and `not run` denominators.
- [ ] **TFM-M09:** Add holdout prompts and threats to validity.

### P2: Presentation and references

- [ ] **TFM-P01:** Check architecture crop, captions, margins, landscape
  rotation, and grayscale future-component distinction.
- [ ] **TFM-P02:** Consider combining chatbot/planner figures as labelled panels
  only if text remains readable; otherwise retain separate full-width figures.
- [ ] **TFM-P03:** Add a supervisor state diagram and results graphics.
- [ ] **TFM-P04:** Normalize `navigate_to`/`walk_to`/`/skill/move_to`, grounded
  context naming, planner dialogue topics, adapter spelling, KnowledgeCore, and
  fake-profile terminology.
- [ ] **TFM-P05:** Audit duplicate bibliography entries, software URLs,
  versions/access dates, repository citation, KnowledgeCore, vLLM, and SkillOpt.
- [ ] **TFM-P06:** Complete spelling, grammar, cross-reference, orphan heading,
  table continuation, and overfull-box review.

## 7. Final Experiment Programme

### 7.1 Freeze gate

- [ ] Select and record the final source SHA and nested repository SHAs.
- [ ] Build the container once from that revision; record image tag/digest.
- [ ] Record chatbot/planner model IDs, provider type, decoding settings,
  timeouts, prompt hashes, registry hash, response mode, digest setting,
  detector profile, and fake policy configuration.
- [ ] Freeze case manifest before the scored run. Later defects are reported or
  repaired under an explicit amendment; cases are not silently changed.

### 7.2 Minimum profile matrix

| Profile ID | Entry point | Required evidence | Thesis use |
|---|---|---|---|
| `E2E-RF` | ROS4HRI tracked voice + `LiveSpeech`, `response_first` | Turn trace, admitted request when eligible, plan, feedback, terminal state, speech | Primary full-stack score |
| `PLAN-ISO` | Orchestrator planner gate | Request, plan validation, dispatch, feedback | Diagnostic plan/schema score |
| `FAKE-MULTI` | Speech or declared planner gate with fake actions | Policy, seed, fake events, plan lineage, recovery, speech | Controlled recovery score |
| `DET-E2E` | Detector-enabled launch + speech | Detector provenance, scene summary, compact projection, answer/plan | Perception-grounding result |
| `ABL-IF` | Frozen `intent_first` subset | Same phase evidence as comparable cases | Ablation only |

### 7.3 Required final runtime gates

- [ ] Run canonical `response_first` questionnaire after lifecycle, speech QoS,
  model readiness, KB, and action-server preflight.
- [ ] Run environment fixtures with stale-world guard and post-action KB query.
- [ ] Run fake policy ladder: `all_success`, `fail_once_navigation`,
  `fail_once_pick`, `delivery_blocked`, and `recipient_missing`; add seeded stress
  only after deterministic policies.
- [ ] Rerun unstable ordered-walk/report cells in-suite and in isolation to
  distinguish context/order effects from deterministic functional failure.
- [ ] Run the bounded detector profile and keep its score separate.
- [ ] Run only the ablation cases needed to support the final comparison.
- [ ] Generate summary CSV/JSON/HTML from traces and preserve raw artifacts.

### 7.4 Stop rule

Do not continue broad runtime development to chase a perfect score. After the
frozen rerun, only fix a defect when it blocks a central research question,
invalidates evidence provenance, creates false success, duplicates speech, or
breaks deterministic plan admission. Otherwise record the defect as a measured
limitation and close the manuscript around it.

## 8. Results Synthesis And Graphics

### 8.1 Required result tables

- [ ] Final configuration and provenance table.
- [ ] Case manifest and acceptance table.
- [ ] Per-profile aggregate metrics with denominators and confidence/dispersion.
- [ ] Per-case status table with pass, degraded, fail, and not-run reasons.
- [ ] Fake policy by case matrix.
- [ ] Error taxonomy: routing, invalid plan, grounding, admission, execution,
  report/speech, timeout, observability.
- [ ] Research-question evidence matrix.

### 8.2 High-value figures

1. **Profile score overview:** grouped bars or dot plot for E2E, planner-isolated,
   fake multi-step, detector, and ablation; never collapse them into one score.
2. **Fake policy heatmap:** cases by outcome policy, using pass/degraded/fail and
   annotated counts.
3. **Phase-completion funnel:** injected, routed, admitted, planned, dispatched,
   terminal, and spoken observations.
4. **Latency distributions:** box/violin or median-IQR plot per phase/profile,
   with timeout counts adjacent.
5. **Recovery lineage trace:** one representative fail-once case showing
   `goal_id`, plan versions, stable/completed steps, failure, and terminal act.
6. **Grounding freshness comparison:** expected fixture entities versus admitted
   compact context and stale-world violations.

Every figure must be generated from a checked-in script or preserved data table,
carry `N`, profile, date/revision, and denominator in its caption, and remain
legible in grayscale.

## 9. Research-Question Claim Ledger

Before rewriting Results, create one row per RQ with:

| Field | Required content |
|---|---|
| Claim | The narrow statement the evidence supports |
| Metric | Formula and denominator |
| Profile | Which portion of the architecture was exercised |
| Artifact | Exact file/run identifier |
| Result | Value and uncertainty/dispersion |
| Limitation | What the run does not demonstrate |
| Chapter destination | Results, Discussion, Limitations, Conclusion |

No headline number should appear without this ledger. Historical scores may be
shown as development progression, but the final answer to an RQ must use the
frozen experiment or be explicitly qualified.

## 10. Architectural Guardrails During Closure

- `dialogue_manager` remains dialogue lifecycle and speaking owner.
- `chatbot_llm` owns user-facing LLM dialogue, route selection, grounded
  projection, and planner ingress, but not executable plan generation.
- `planner_llm` owns planning, supervision, retry/replan policy, and planner
  dialogue acts, but not skill execution or direct speech.
- `nao_orchestrator` owns admission, deterministic validation, dispatch,
  lineage, feedback, and dialogue-act relay, but not LLM planning policy.
- `kb_skills` remains the KnowledgeCore transport boundary.
- `nao_scene_grounding` remains the detector normalization and scene-summary
  owner.
- Runtime skills own fresh execution-time evidence for their effects.
- Prompt changes require a prospective SkillOpt baseline and holdout ledger.
- Nested/upstream packages receive seam-focused changes only.

## 11. Execution Order

### Phase A: Freeze the evaluation contract

- [ ] Resolve baseline decision and abstract promise.
- [ ] Freeze RQs, units, variables, profiles, case manifest, metrics, and resets.
- [ ] Create artifact naming convention and final configuration record.

### Phase B: Collect final evidence

- [ ] Freeze source/container/model configuration.
- [ ] Run preflight, E2E main, environment, fake multi-step, detector, and bounded
  ablation profiles.
- [ ] Preserve raw traces and generated summaries.

### Phase C: Write the evidence chapters

- [ ] Finalize Chapter 6 first.
- [ ] Rebuild Chapter 7 from the frozen artifacts and graphics.
- [ ] Expand Chapter 8 by RQ and literature comparison.
- [ ] Separate Chapter 9 limitations from future implementations.
- [ ] Rewrite Chapter 10 and abstract in past tense.

### Phase D: Reconcile implementation chapters

- [ ] Apply Chapter 5 provenance/API/supervisor/reporting corrections against the
  frozen revision.
- [ ] Recheck Chapters 3 and 4 for terminology and contract drift.
- [ ] Generate registry/interface/config appendices from frozen source where
  practical.

### Phase E: Submission QA

- [ ] Compile from clean build directory.
- [ ] Inspect every page at desktop and print-equivalent scale.
- [ ] Resolve undefined references, missing citations, duplicate keys, bad table
  breaks, clipped diagrams, captions, and material overfull boxes.
- [ ] Verify UAB cover/front matter, page numbering, lists, bibliography, and
  appendices.
- [ ] Archive PDF, source ZIP, source SHAs, container digest, and evidence index.

## 12. Immediate Next Session Checklist

1. Preserve the current Chapter 5 hand-edit diff and review its ownership
   wording against the architectural guardrails.
2. Decide whether the direct-execution comparison remains in scope.
3. Create the frozen case-manifest table and RQ-to-metric map.
4. Locate and inventory the June/July run artifacts that can support final
   figures; mark each `reuse`, `rerun`, or `exclude`.
5. Amend Chapter 6 with acceptance criteria and reset/repetition policy before
   collecting the final run.
6. Run the clean canonical evidence pass, then replace Chapter 7's obsolete
   May-centred preliminary narrative.

## 13. Progress Log

| Date | Change | Evidence |
|---|---|---|
| 2026-07-13 | Created closure masterplan from local thesis, IIIAV3 correction ledger, current runtime trackers, and recent commits | `08f9129`, `ISSUE_TRACKER_FULL_SUITE.html`, `ISSUE_TRACKER_FAKE_SUITE.html`, current chapter inventory |
| 2026-07-13 | Completed the first surgical Chapter 5 correction slice: TFM-facing skill terminology, KB first-use definition, goal-supervisor clarification, grounded-context and contract links, route bullets, response/intent ordering, dialogue-manager provenance, and the revised simulator/WME stack figure | `TFM-C01`, `TFM-C02`, `TFM-C06`, `TFM-I04`, `TFM-I05`, `TFM-I13`, `TFM-I14` |
