# TFM Final Results Prism Handoff

**Date:** 17 July 2026
**Purpose:** Complete the final Results, Discussion, limitations, conclusion, and appendix after the frozen runtime evidence is available.

## 1. Starting Point

The repository thesis has been synchronized with the supplied 16 July PDF. The synchronized source includes the revised title, ROS 2 and HRI framing, reorganized early chapters, expanded validation boundaries, a final-run reporting template, and the extended Discussion.

Edit only:

- `docs/thesis/TFM_reformatted_UAB_local/sections/06_validation_methodology.tex`, if the final manifest changes a declared denominator;
- `docs/thesis/TFM_reformatted_UAB_local/sections/07_results.tex`;
- `docs/thesis/TFM_reformatted_UAB_local/sections/08_discussion.tex`;
- `docs/thesis/TFM_reformatted_UAB_local/sections/09_limitations_future_work.tex`;
- `docs/thesis/TFM_reformatted_UAB_local/sections/10_conclusion.tex`;
- `docs/thesis/TFM_reformatted_UAB_local/sections/11_appendix.tex`;
- generated figures under the thesis figure directory.

Read the thesis-local `AGENTS.md` before editing. Do not modify runtime packages from the writing pass.

## 2. Evidence Gate

Do not write an observed 100 percent result until one immutable bundle proves it. A complete final tuple records:

- root and nested repository revisions and clean/dirty state;
- container image tag, image ID, and digest;
- imported source revisions from inside the image;
- chatbot and planner model configuration;
- prompt-pack, registry, questionnaire, harness, and oracle hashes;
- launch profile and arguments;
- fixtures, fake policy, seeds, detector state, timestamps, and node-uniqueness preflight.

The current 17 July review is an 8.8/10 strong candidate, not the final thesis tuple. Do not copy its provisional score or unresolved findings into the canonical result.

Required final products:

```text
manifest.json
case_results.csv
policy_matrix.csv
robustness_repetitions.csv
latency_summary.json
operational_index.json
error_taxonomy.csv
representative_replan_trace.json
rq_evidence_ledger.csv
figures/
raw/
```

Every number in Chapter 7 must be derivable from these files. An isolated rerun may explain a non-pass but cannot replace its full-sweep result.

## 3. Final Chapter 7 Structure

Chapter 7 must be led by the canonical run, not by dates or F-series identifiers.

### 7.1 Canonical Validation Tuple and Headline Result

State the complete tuple and present explicit denominators before the secondary 1--10 index.

| Evidence family | Required denominator | Claim |
| --- | ---: | --- |
| Full-runtime questionnaire | 21 cases | Public ingress, routing, semantic contract, execution, and closure |
| Environment and KB suite | Final declared case count | Query, mutation, freshness, role integrity, and postconditions |
| Deterministic fake-skill matrix | Nine cases under each applicable policy | Dispatch, feedback, recovery, and truthful failure |
| Compositional stress profile | Seven `capability_extreme` cases | Preservation of multiple requested capability families |
| Frozen robustness set | Five cases times three repetitions | Bounded repeatability under paraphrase and multi-turn reference |
| Real-adapter profile | Exact executed trial count | TTS, posture, head motion, and other implemented NAO adapters only |
| Detector profile | Exact detector trial count | Detector-to-grounding path only |

Use `N/N` rather than an unqualified percentage. Report pass, degraded, fail, and not scored counts. Present the operational index and all eight components after these denominators.

### 7.2 Full-Runtime Semantic Contract Results

Group the 21 cases by dialogue, knowledge, simple execution, composite execution, and route-safety holdout. Required columns:

```text
family | cases | route correct | plan valid | target set exact |
role/order correct | execution complete | report coverage |
postcondition correct | exactly-once speech | overall pass
```

A terminal trajectory is insufficient. Compare requested, selected, planned, executed, and reported targets, including people, support, location, recipient, and movable-object roles.

### 7.3 Stateful Environment and KB Validation

Report state and wording independently:

```text
case | pre-query | expected mutation/no-mutation | post-query |
state freshness | response correctness | verdict | reason
```

The stable TITAS/MIDAS chain is the representative positive example if it passes under the frozen tuple. A fluent answer does not prove a KB postcondition, and a correct direct query does not prove adequate user-facing wording.

### 7.4 Deterministic Failure and Recovery Matrix

Use a case-by-policy heatmap for:

- `all_success`;
- `fail_once_navigation`;
- applicable `fail_once_pick` cases;
- applicable `delivery_blocked` cases;
- applicable `recipient_missing` cases.

Cells are pass, degraded, fail, or not applicable. Include a short reason code. Report separate denominators for injected failure observed, correct feedback, valid plan-version progression, safe retry/clarification, truthful terminal outcome, and exactly-once speech.

Include one lineage trace with `goal_id`, `plan_id`, `plan_version`, relevant `step_id`, failed result, revised work, terminal result, and final dialogue act.

### 7.5 Compositional Stress and Robustness

Report the seven capability-extreme cases separately from the 21-case questionnaire. A pass requires preservation of every admitted capability family. A partial plan followed by successful execution is a semantic failure.

Show all three repetitions of each frozen robustness case. Do not retain only the best repetition. Use descriptive counts; fifteen observations do not support population-level inference about arbitrary language.

### 7.6 Controlled Ablations

Keep only comparisons that change one declared factor while the rest of the tuple is fixed:

- `response_first` versus `intent_first`;
- canonical model versus alternate model.

If provenance or controls are incomplete, label the comparison diagnostic and move it to the developmental appendix. Do not infer causality from an F-series comparison that changed several factors.

### 7.7 Bounded Embodiment Evidence

Split this into two profiles:

1. **Real-adapter validation:** TTS, head motion, posture, and any other genuinely executed NAO skills. Report exact trials and observed evidence. Do not imply physical manipulation or navigation.
2. **Detector path:** sufficiently stable detections entering scene grounding and supporting later reasoning. Detector precision, identity settling, and fact lifetime remain lightly evaluated limitations.

LocateAnything is future work. Do not present it as evaluated detector performance.

### 7.8 Canonical Error Taxonomy, Operational Index, and RQ Synthesis

Retain an error taxonomy even when every canonical count is zero:

```text
route/acknowledgement mismatch
grounding or target-selection loss
role violation
plan-schema or registry rejection
capability-family omission
skill execution failure
recovery or lineage failure
report-coverage failure
KB postcondition failure
speech multiplicity or closure failure
harness or evidence-correlation gap
```

Then report the eight operational-index components and update the existing RQ table from canonical evidence first. The index remains secondary to case and semantic denominators.

## 4. Chapter 8 Structure

Chapter 8 explains mechanisms and implications. It must not repeat the canonical tables.

Keep the existing architectural, prompt/skill-contract, planning, grounding, execution-evidence, ROS 2/HRI, and validation sections. Add four compact diagnostic narratives:

1. **Target and role preservation:** the earlier wrong-target trajectory and gold-apple selection failure.
2. **Execution versus symbolic post-state:** stale or contradictory KB location relations after apparent delivery.
3. **Objective preservation:** acknowledgement/route mismatch and recovery that dropped requested capability families.
4. **Harness validity:** late speech, lineage correlation, and false-negative phase extraction.

Each narrative states:

```text
observed symptom -> responsible seam -> introduced guard/oracle ->
canonical rerun outcome -> remaining limitation
```

Keep each narrative short. Its purpose is to support a methodological or architectural conclusion, not to reproduce the development log.

Detector wording must remain bounded: the detector-backend interface, grounding path, and KB projection are implemented; reasoning over stable KB entities is validated; detector quality, identity churn, and temporal fact stability remain separate limitations.

## 5. Appendix Structure

Add two appendix components.

### Canonical Case Ledger

Expose every canonical question, policy, repetition, and verdict in compact tables. Include concise failure reasons and artifact references. Raw payloads and logs remain in the evidence bundle.

### Developmental and Diagnostic Evidence

Remove F identifiers from reader-facing Chapter 7 prose. Preserve them only in a narrow appendix provenance column:

```text
semantic label | internal evidence ID | tuple difference | diagnostic finding |
introduced guard/oracle | canonical status | artifact path/hash
```

Do not graph historical scores as a performance trend because their tuples and oracle strength differ. If error evolution is shown, use a milestone table:

```text
diagnostic milestone | observed failure class | owning seam |
introduced guard/oracle | canonical-run status
```

## 6. Conditional Result Language

Prepare the successful form, but replace tokens only from the frozen bundle:

> Under the frozen canonical tuple, `[MAIN_PASS]/21` full-runtime cases satisfied their declared route, semantic-contract, execution, reporting, and speech requirements. Target-set exactness was `[TARGET_EXACT]/[TARGET_DENOM]`, verified postconditions were `[POST_PASS]/[POST_DENOM]`, and `[SPEECH_PASS]/[SPEECH_DENOM]` expected semantic stages were spoken exactly once.

> The deterministic policy matrix contained `[POLICY_PASS]/[POLICY_DENOM]` applicable cases that ended in the required success, recovery, clarification, or truthful failure. All accepted replans preserved goal and plan-version lineage, and no failed case was reported as completed.

If any value is not complete, use the qualified form:

> The canonical run satisfied `[PASS]/[DENOM]` declared cases. The remaining `[COUNT]` cases were limited by `[SEAM]`; their results are reported as `[degraded/fail/not scored]` and are not replaced by isolated diagnostic reruns.

Never write “the stack achieved 100 percent” without the denominator and declared scope. Prefer “all `N` frozen cases satisfied the declared contracts.”

## 7. Claim Boundaries

- This is scenario-based software and integration validation, not unrestricted task-success estimation.
- Fake skills validate routing, planning, deterministic dispatch, feedback, recovery, reporting, and symbolic effects. They do not validate physical navigation, grasping, or manipulation.
- Real-adapter evidence supports only the implemented and observed NAO skills.
- Detector evidence is separate from stable KB-authored grounding.
- Direct atomic execution is a safety control, not a causal planner baseline.
- Historical or focused runs are diagnostic and are not pooled with the canonical tuple.
- A final utterance is not proof of a physical or KB effect.

## 8. Required References

- Thesis source: `docs/thesis/TFM_reformatted_UAB_local/`
- Supplied PDF used for synchronization: `/home/juanbeck/Downloads/main.pdf`
- Reporting precedents: `docs/artifacts/llm_embodied_agent_results_reporting_research_2026-07-17.md`
- Current validation method: Chapter 6
- Evidence storage contract: `docs/evaluation/README.md`
- Current runtime assessment: `docs/artifacts/runtime_review_2026-07-17.md` (diagnostic only)
- Current hypothesis audit: `docs/artifacts/seam_hypothesis_audit_2026-07-17.md`

## 9. Completion Gate

Before returning the final manuscript:

1. Verify every numeric claim against the frozen bundle.
2. Check that every RQ cites at least one canonical evidence family and states its boundary.
3. Confirm no F identifier leads a Chapter 7 subsection.
4. Confirm every historical example is diagnostic, not part of the final denominator.
5. Compile with `latexmk -pdf main.tex`.
6. Render and inspect the title, contents, all Chapter 7 tables/figures, Chapter 8 transitions, appendices, and bibliography.
7. Run `git diff --check` and the repository change audit.
