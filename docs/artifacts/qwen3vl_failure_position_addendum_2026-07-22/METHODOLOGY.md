# Methodology

## Objective

The purpose of this addendum is to separate stack-coherent behavior from failures that appear at different positions in a multi-step interaction. Qwen3VL is the primary model because it has the largest historical evidence base in this workspace. The analysis does not assume that a failure belongs to the model or to the stack before checking the emitted route, planner request, execution feedback, recovery plan, and speech closure.

## Source material

The projection reads the original `raw/*.json` case files from F13 and F14. It does not rewrite those files. The source runs contain:

- run manifests and aggregate metrics;
- per-case assessments and phase observations;
- planner and executor event excerpts;
- grounded-context and route breadcrumbs;
- successive stack logs, including compressed logs and the large final F13 snapshot;
- image and runtime provenance declarations.

The source model declared by both metadata files is `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` on the `iiia:nao` runtime image family. The runtime image identifiers are retained in the source metadata where available. F13 is qualified diagnostic evidence. F14 is diagnostic evidence across v4, v5, and v6 image contexts.

## Unit of analysis

One dataset row represents one case from one raw JSON file. A repeated case name in different profiles is intentionally kept as a separate row. The stable `record_id` combines the run, source file, and case index. This prevents the aggregation from treating the same natural-language task as an independent replicate when it was run under a different failure policy.

The original case status is preserved. Missing status in adversarial records is represented as `not_scored`, rather than being interpreted as a pass or a fail.

## Failure-position taxonomy

The stage label is a classification of the emitted plan and observed runtime evidence.

| Stage | Operational definition | Example in this package |
| --- | --- | --- |
| Beginning | Failure before the first executable step, during target selection or planner admission, or at the first executable operation | F14 gold-apple case published `ask_clarification` before an executable plan; F13 missing-object recovery failed on the first `find_object` |
| Middle | A prelude step succeeds and an intermediate action fails, with a recovery or replan trace | F13 fail-once pick and delivery-blocked grouped-work-table cases |
| End | A delivery-boundary failure or post-failure terminal/closure condition occurs after earlier steps have completed | F13 recipient-missing grouped delivery and missing recovery closure |
| Middle to end | Alternating or seeded policies affect a composite sequence, but no single failed step can be isolated | F13 `every_other` and `random_seeded` grouped delivery |
| Unknown | The profile is configured but the retained case trace does not prove that the selected failure happened in this case | F13 kitchen-delivery cases under navigation and delivery-blocked profiles |
| Control | A corresponding success trajectory used to distinguish stack wiring from an injected failure | F13 and F14 all-success grouped delivery, ordered walk, kitchen delivery, and gold-apple follow-up |

The labels are not physical robot positions. In particular, an end-stage closure observation can follow a failure injected earlier. The fields `failure_site`, `failure_step_names`, `observed_failure_stage`, and `position_basis` are retained together so that this distinction remains visible.

## Evidence extraction

The projection parses `execution_feedback` lines in each case's retained `log_excerpt`. It records plan IDs, event types, and step names. A failed step is accepted as directly observed only when the trace contains `step_failed` for that step. The classifier uses explicit profile and case names only for the targeted probes already defined by the source run. Cases outside that selection remain in the dataset as `other_qwen3vl_evidence` so the package is complete without overstating coverage.

Confidence is assigned as follows:

- `high`: the case trace identifies the failed step or clearly records pre-execution planner admission failure;
- `medium`: the profile and case objective identify the intended position, but the raw excerpt does not isolate the failed step, or the case mixes multiple possible failure sites;
- `low`: the policy was configured, but the case trace shows a successful or otherwise non-diagnostic trajectory.

## Interpretation rules

The analysis uses the following separation:

1. A pass under an injected failure profile means that the harness observed the expected recovery or control trajectory. It does not mean that every possible failure path is correct.
2. A degraded or failed case with `terminal_observed=true` can still have a closure defect. Terminal execution and user-facing recovery speech are separate observations.
3. A clarification before an executable plan is a planner-admission or target-selection observation. It is not an executor failure.
4. A `kb_add` rejection is treated as a KnowledgeCore contract or compiler-boundary event. It is not classified as an LLM hallucination without inspecting the submitted statement syntax and the subsequent case expectation.
5. A success control is evidence that the corresponding launch, grounding, planner, executor, and speech seams can form a coherent path under that run. It does not prove invariance across models or images.

## Limitations and reopen condition

The F13 and F14 runs do not hold one image, one fixture snapshot, and one worktree constant across all cases. The aggregate percentages are therefore descriptive run summaries, not a controlled causal estimate of Qwen3VL quality. The F13 fail-once-navigation ordered-walk case reports an injected failure and missing closure, but its retained excerpt shows successful navigation events rather than an isolated `navigate_to` failure. It remains a medium-confidence target. Two kitchen-delivery rows are low-confidence unknowns for the same reason.

For a stronger final thesis table, rerun the three canonical profiles against one frozen image and one fixture snapshot, using the same utterance and failure seed for each position. Capture the complete interaction trace, not only the case excerpt. The current package should be reopened for that controlled run rather than silently upgraded to a qualification result.
