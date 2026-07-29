# Runtime Review: Qwen3VL failure-position evidence addendum

Run ID: `qwen3vl-failure-position-addendum-2026-07-22`

Date: 22 July 2026

## Executive result

This addendum expands the Qwen3VL evidence base with 61 case records from the F13 semantic audit and F14 targeted hardening runs. It provides direct traces for failure injection at the beginning and middle of composite plans, plus end-stage recipient and recovery-closure observations. The evidence supports a case-level separation between stack coherence and model-sensitive behavior.

The package is evidence reconciliation, not a new live stress run. No new runtime score is claimed. The earlier v34 review remains the frozen Qwen3VL qualification source. F13 and F14 are diagnostic runs across successive runtime contexts, so their combined status counts describe the available evidence but do not form one controlled model score.

## Runtime-review standing

### Runtime Review — `qwen3vl-failure-position-addendum-2026-07-22`

**Score:** not scored. This artifact packages and reconciles prior runtime evidence.

**Overall sentiment:** expanded failure-position coverage with bounded provenance.

**Critical finding:** the package must not be used to claim that every configured failure occurred at the requested plan step. The two low-confidence rows and the medium-confidence navigation row retain their evidence gaps explicitly.

## Source runs and provenance

| Run | Date | Cases | Pass | Degraded | Fail | Not scored | Provenance |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| F13 semantic audit | 13 July 2026 | 54 | 39 | 5 | 4 | 6 | qualified diagnostic run, mutable-image and dirty-worktree caveat |
| F14 targeted hardening | 14 July 2026 | 7 | 6 | 0 | 1 | 0 | diagnostic run spanning v4, v5, and v6 image contexts |
| Combined addendum | 13–14 July 2026 | 61 | 45 | 5 | 5 | 6 | descriptive aggregation only |

The declared model is `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` on the `iiia:nao` runtime image family. F13 uses image identifier `sha256:1a37fc...45e7e4` in its metadata. F14 retains three image identifiers for v4, v5, and v6. The original manifests and metadata are included under `evidence/`.

## Failure-position coverage

| Position | Records | Representative evidence | Confidence |
| --- | ---: | --- | --- |
| Beginning | 5 | missing-object recovery, KB compiler-boundary rejection, planner clarification before an executable plan | high, except the navigation row at medium confidence |
| Middle | 2 | fail-once pick after successful finding; delivery-blocked bring after scan and find preparation | high |
| End | 2 | recipient-bound delivery failure; missing recovery closure after a completed or terminal path | high for recipient boundary, medium for closure observation |
| Middle to end | 2 | `every_other` and `random_seeded` policies on grouped delivery | medium |
| Unknown | 2 | configured navigation or delivery policy with successful kitchen trace | low |

The classification is based on the emitted plan and stack event trace. It is not a claim about metric robot position or physical distance.

## Highest-value traces

### Beginning: missing-object recovery

The F13 all-success profile includes `fake_deep_missing_object_recovery`. The first executable `find_object` step fails, and the trace shows a recovery/replan path. This is useful because it exercises the planner and executor feedback seam before any successful manipulation. The case was assessed as pass under the source harness expectation.

### Beginning: planner admission before execution

The F14 v6 gold-apple multiturn case had complete fixture context but the planner published `ask_clarification` before producing an executable plan. The retained `gold_v6.log.gz` contains the planner request and the clarification act, while no executable plan events are present for the failed turn. The later follow-up query, “Where is the gold apple now?”, received a grounded location answer. This pair separates an admission or target-selection failure from the later grounded query path.

### Beginning: KnowledgeCore compiler boundary

The F13 main run includes `kb_mutation_add_red_cup`. The first `kb_add` step is rejected because the submitted statements do not satisfy the expected RDF-style form, and the case remains assessed as pass for its intended contract test. This trace belongs to the KB mutation boundary, not to perception or physical execution.

### Middle: pick recovery

The F13 `fail_once_pick` case shows `find_object` succeeding, `pick_object` failing, then a retry plan with `scan`, `find_object`, `pick_object`, and `report_result` succeeding. This is the clearest middle-position recovery trace in the package. It demonstrates plan-version progression and a completed user-facing path after an intermediate skill failure.

### Middle: delivery-blocked recovery

The F13 grouped-work-table case under `delivery_blocked` records successful finding and scan preparation, followed by `bring_object` failure and a replan. The case was degraded because recovery closure was not spoken after terminal evidence. The execution failure and the speech closure defect are therefore separate findings in the same row.

### End: recipient boundary and closure

The F13 grouped-work-table case under `recipient_missing` reaches navigation and finding before recipient-bound delivery fails. The case reports clarification despite complete fixture context. A separate F13 gold-apple row under the same profile completes the execution trace but lacks recovery closure speech. These two rows show why end-stage analysis should include both the final executable boundary and the post-failure dialogue contract.

## Controls and stack-coherent paths

The addendum retains nine successful controls. They include ordered walking, grouped work-table delivery, IIIA kitchen delivery, and the gold-apple follow-up across F13 and F14. F14 v4 ordered walking, v5 grouped delivery, v6 grouped delivery, v6 kitchen delivery, and the v6 gold-apple follow-up all passed their source assessments except the initial gold-apple multiturn turn.

These controls show that the integrated path can preserve grounded context, planner admission, execution feedback, AB=1 skill dispatch, and speech output under the recorded run conditions. They do not remove the model-sensitive failures in target selection, clarification, composite coverage, or terminal closure.

## Thesis interpretation

The evidence supports four bounded claims:

1. Stack ownership and execution lineage are observable in successful and recovered cases. The traces contain planner requests, plan IDs, step events, replans, and speech evidence rather than only final text.
2. Beginning failures tend to prevent execution from entering the intended plan. The F14 gold-apple failure is an admission or target-selection failure, while F13 missing-object recovery and the KB case exercise early executable or contract boundaries.
3. Middle failures can be recovered when the planner receives a typed failure event. The fail-once pick case is the strongest example. Delivery-blocked recovery reached a replan but exposed a missing closure obligation.
4. End-stage failures are often supervision or recipient-boundary failures after earlier execution has been observed. They should not be reported as evidence that the whole stack failed to launch or that the model fabricated a robot effect.

These findings are consistent with the previous Qwen3VL v34 review, which found high stack readiness and residual variance in high-composition planning, target selection, and closure. They do not establish that Qwen3VL is the sole cause. The runs changed fixture, image, policy, and prompt/runtime context over time, so causal attribution requires a new frozen same-image comparison.

## Evidence quality and open gaps

### Passed packaging checks

- F13 and F14 manifests, metadata, raw JSON, and compressed logs are retained.
- The prior model-agnostic ZIP was checked and did not contain F13 or F14 artifacts.
- Every derived row retains source file, case name, profile, status, event trace, reason, and classification basis.
- Missing source status is represented as `not_scored`.
- The archive is accompanied by SHA-256 checksums and an extraction test.

### Remaining probes

For a final controlled thesis table, run one frozen image and one fixture snapshot through three explicit cells:

1. beginning: planner admission or first `find_object` failure;
2. middle: `pick_object` or `bring_object` failure after a successful prelude;
3. end: recipient-bound delivery or post-failure closure failure.

Capture the full interaction trace for each cell, retain the exact fake-policy seed, and repeat the same utterance under the same model. This is the reopen condition for replacing the present diagnostic labels with controlled replicate measurements.

## Machine-readable outputs

- `analysis/failure_position_dataset.csv`
- `analysis/failure_position_dataset.jsonl`
- `analysis/failure_position_summary.json`
- `analysis/failure_position_trace_excerpts.md`

The raw artifacts remain authoritative. The derived files are an analysis projection designed for filtering, table generation, and later statistical aggregation in the TFM workflow.
