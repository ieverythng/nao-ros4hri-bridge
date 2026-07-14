# Runtime Seam Deslop And Stabilization Plan

**Date:** 2026-07-14
**Status:** Main implementation accepted; final deep-fake runtime gate pending
**Paired view:** `docs/plans/runtime_seam_deslop_2026-07-14.html`
**Authority:** This plan executes the current stabilization tranche under
`docs/plans/nao_ros4hri_masterplan.md`. The masterplan remains the canonical
repository-wide integration tracker.

## Objective

Reduce the current cross-package churn without weakening grounded execution.
The normal path should allow each LLM-owned seam to perform its declared job:
`chatbot_llm` owns route, target selection, and user-facing wording;
`planner_llm` owns executable planning and supervision; `nao_orchestrator`
validates and executes; AB=1 skills provide fresh effect evidence; `kb_skills`
owns KnowledgeCore transport.

Deterministic code validates contracts and preserves evidence. It must not
silently replace valid model output with a second policy implementation.

## Frozen Problems

1. `report_result` is natural in some runs but can omit targets, describe the
   wrong action, or fall through several independent renderers.
2. Route, scope, recipient, and reporting policy can be inferred more than once
   across chatbot response, chatbot intent, planner handoff, and planner
   fallback logic.
3. Planner-invalid output can enter broad deterministic repair before the model
   receives a bounded validation retry with the exact contract errors.
4. KB placement effects can retain mutually exclusive `isIn`, `isAt`, or
   `isOn` facts, or accept incomplete post-effect evidence.
5. Runtime traces identify some fallback pressure but do not consistently state
   which owner changed which semantic field.
6. Fixture, fake-skill, and runtime-harness changes are mixed into the same
   working batch, making regressions difficult to attribute.

## Protected Invariants

- One speaking authority and one user-facing utterance per turn.
- No executable plan inside chatbot intent output.
- No planner wording policy or direct robot execution.
- No direct KnowledgeCore writes from LLM nodes.
- People, objects, supports, and navigation locations remain distinct.
- `goal_id`, `plan_id`, `plan_version`, and stable `step_id` lineage survive
  retries and replans.
- Fake and real AB=1 skills emit the same effect-evidence contract.
- `perform_motion` remains real by default, with explicit honest open-loop
  behavior when robot feedback is unavailable.
- No prompt wording change without a recorded SkillOpt baseline, train set,
  holdout set, and accept or reject decision.

## Normal Authority Flow

```text
response stage
  proposes natural acknowledgement and advisory route
intent stage
  owns route + intent + target_selection + request kind + report policy
planner_common
  normalizes and validates the declared contract
planner_llm
  plans from the authoritative selection, then validates its output
nao_orchestrator
  admits, dispatches, correlates evidence, and applies verified KB effects
chatbot_llm
  verbalizes structured report_outcome after execution
```

`response_first` remains the production call order. If response wording
contradicts the authoritative intent result, the response stage receives one
retry with the route and target decision locked. `intent_first` remains an
ablation mode.

## Fallback Classes And Budget

| Tier | Meaning | Examples | Healthy-path budget |
| --- | --- | --- | --- |
| A | Changes turn semantics | route repair, inferred target selection, deterministic plan synthesis, report wording replacement | Zero for ordinary dialogue, KB queries, and grounded success paths |
| B | Recovers execution transport or a declared failure | planner retry, replan, action timeout, KB mutation retry, fake fail-once policy | Only in the corresponding failure or recovery case |
| C | Static compatibility or configuration default | import shim, benign coercion, absent optional parameter | Inventory only |

Every Tier A or B activation must expose the existing trace channel with owner,
fallback identifier, trigger, input source, output effect, lineage, and whether
semantics changed. No new ROS topic is required.

## Retry Policy

- Chatbot response or intent validation: at most one retry for the failed stage.
- Planner validation: at most two retries.
- Planner retry two is permitted only when retry one made measurable progress,
  such as becoming parseable, reducing validation errors, or changing the
  invalid-output fingerprint.
- Stop on repeated output, repeated error sets, missing user information, or an
  unavailable backend.
- Heuristic salvage is permitted only after retries are exhausted, must be
  traced as Tier A, and must never broaden grounded target scope.
- Primitive motion may use an explicit deterministic execution mode because it
  is a declared primitive path, not a planner repair.

## Report Result Contract

`chatbot_llm` remains the primary wording authority. The execution seam provides
`report_outcome`, step summaries, recipients, failures, exclusions, and ordered
events as structured evidence. Deterministic rendering is an emergency path
only when chatbot output is empty, unsafe, or factually inconsistent with the
validated outcome.

The validator must reject false completion, omitted required targets, recipient
as completed object, stale intermediate arrival, and duplicate speech. It
should not rewrite a valid natural response merely because it differs from a
preferred template.

Prompt or system-addendum changes use a bounded SkillOpt round. Training cases
cover ordered multi-object work, grouped and person delivery, intermediate
reports, blocked or partial delivery, and head motion plus wave. Holdouts cover
simple wave, table placement, missing target, ordinary dialogue, KB questions,
future action, and duplicate-speech prevention.

## Strict KB Effect Contract

1. AB=1 skill results provide fresh post-action effect evidence.
2. `nao_orchestrator` validates the effect shape and delegates query or mutation
   through `kb_skills`.
3. A spatial move removes stale mutually exclusive `oro:isIn`, `oro:isAt`, and
   `oro:isOn` values for the moved object before confirming the new relation.
4. Destination identifiers and predicates come from structured skill evidence,
   not identifier-word inference.
5. Missing, contradictory, or unverifiable post-effect evidence fails the
   postcondition and enters planner recovery or truthful failure.
6. People cannot be placement supports or deliverable objects. Physical
   supports remain objects but are excluded from grouped member expansion.

## Implementation Tranches

### T0: Baseline And Inventory

- Freeze root and nested-repository diffs and active image provenance.
- Record every semantic fallback and its owner.
- Run existing focused tests without changing behavior.
- Separate environment or harness failures from semantic failures.

**Exit:** each live hypothesis has a discriminating test or runtime probe.

### T1: Contract And Chatbot Authority

- Keep `target_selection` normalization in `planner_common`.
- Make chatbot intent output authoritative for route and target selection.
- Reduce `planner_request_adapter` to contract assembly, canonical ID
  resolution, membership checks, and bounded continuation rehydration.
- Remove healthy-path goal-text reconstruction of scope, recipient, operation,
  or report policy after equivalent structured fields are present.
- Add normalized trace records for route repair, rules backfill, inferred target
  selection, and report replacement.
- Consolidate report validation and emergency rendering so the prompt pack,
  structural addendum, and fallback module do not each own wording policy.

**Red gates:** contradictory response and intent; valid target selection being
overridden; named-person absence; grouped location expansion; report omission;
duplicate speech.

### T2: Planner Retry And Salvage

- Validate model output and return exact schema or semantic errors to the model.
- Implement the bounded one-plus-conditional-second retry policy.
- Detect repeated fingerprints and stop retrying.
- Keep deterministic target-selection compilation as traced post-retry salvage
  only; it cannot expand selection or invent destinations.
- Keep clarification for genuinely missing information and truthful failure for
  exhausted invalid output.

**Red gates:** malformed JSON progress, unchanged invalid plan, invalid
placement support, report-result omission, known-good simple motion.

### T3: Orchestrator And KB Effects

- Keep report outcome construction in the shared structured contract.
- Remove generic completion wording from the orchestrator normal path.
- Strictly validate fresh effect evidence and stale spatial-fact removal.
- Preserve execution and replan lineage through every feedback event.
- Keep action transport fallbacks explicit and Tier B traced.

**Red gates:** object moved between supports, grouped objects placed at one
destination, person delivery, contradictory effect, failed postcondition, and
blocked delivery recovery.

### T4: Fake Skills, Fixtures, And Motion

- Keep fake skills scenario-driven and contract-equivalent to real AB=1 skills.
- Centralize fixture definitions and cleanup verification.
- Ensure fake by default except the declared real `perform_motion` path.
- Verify head and posture requests use honest open-loop dispatch when feedback
  is absent and never fabricate convergence.

### T5: Harness And Documentation

- Normalize runtime fallback counters and source/image provenance.
- Keep semantic score separate from observability and `not_scored` cases.
- Update full-suite, fake-suite, masterplan, and dated evidence only after a
  clean rebuilt run.
- Archive this plan after accepted conclusions are folded into the masterplan.

## Validation Ladder

For each tranche:

1. Add or identify one failing test at the narrowest truthful layer.
2. Patch only the owning seam.
3. Run focused tests and protected controls.
4. Run affected package suites, `py_compile`, registry consistency when
   relevant, `git diff --check`, and the ROS4HRI change audit.
5. Apply behavior-preserving deslop only after tests are green, then rerun them.
6. Build one clean overlay from the intended base image. Stop and replace the
   full container once, verify source hashes and node uniqueness, then score.
7. Run the exact former failure, a known-good control, focused success and
   failure cases, adversarial variants, and only then the full plus deep fake
   suites.

The current live container may be observed before replacement, but it cannot
prove source changes that are not in its image.

## 14 July Closure

T1 through T4 are implemented and pass focused source gates. Live evidence
accepts chatbot-owned reporting, strict KB mutations, grounded grouped delivery,
human/object separation, real-by-default motion dispatch, and open-loop head
motion without joint feedback. The runtime harness now isolates tracked voices
and treats live-perception additions as valid only for `visible_objects` scope.

One final T5 gate remains. The invalid-intent retry-exhaustion provenance change
passes the full chatbot seam (`156` tests), but vLLM became unreachable during
the clean v7 planner preflight. Do not mark the complete deep-fake profile
accepted until the endpoint-backed ordered-walk holdout and the full
`all_success` plus `fail_once_navigation` profiles finish.

## Runtime Acceptance

- Zero healthy-path Tier A fallbacks in dialogue, KB query, grounded single
  delivery, grouped delivery, ordered execution, and normal report wording.
- Correct clarification for missing recipient or genuinely ambiguous target.
- No implicit replan when clarification is required.
- No false completion, missing required report targets, or duplicate speech.
- Successful KB mutation removes stale spatial predicates and verifies the new
  object location; unsupported evidence fails truthfully.
- Humans and objects remain role-correct through planning, execution, KB
  effects, and report wording.
- Full suite and deep fake suite artifacts contain correlated turn, goal, plan,
  execution, report, KB, fallback, and latency evidence.

## Current Baseline

- Nested `chatbot_llm` focused suite: 180 tests passed on 2026-07-14.
- Root focused modules compile successfully.
- The first root pytest invocation is not a semantic result because local
  collection lacks generated ROS message packages (`chatbot_msgs`). It must be
  rerun in the sourced ROS environment or container.
- The running container is `nao_ros2` from `iiia:nao`; its installed source
  provenance must be verified before any score is attributed to current work.
- No prompt mutation is accepted by this plan without a SkillOpt ledger.

## Implementation Status (2026-07-14)

- T0 complete: fallback ownership, dirty-tree scope, image provenance, and
  source/runtime distinction are recorded.
- T1 source-green: intent schema owns `target_selection`; chatbot retries one
  invalid selection; adapter derivation is limited to retry exhaustion, rules
  fallback, and structured continuation.
- T2 source-green: planner performs one retry and a conditional second retry
  only after measurable validation progress; repeated output stops.
- T3 source-green: generic orchestrator delivery completion is removed; strict
  typed spatial effects reject object or untyped destinations; stale spatial
  aliases and reciprocals are removed before a verified add; emergency reports
  name completed objects and recipients.
- T4 source-green: fake posture state, fail-once continuity, fixture location
  validation, and real-motion/open-loop ownership remain covered.
- Source gates: 262 cross-package tests, 189 focused chatbot tests, 4 preloader
  tests, registry consistency, compilation, diff check, and ROS4HRI audit pass.
- Clean image: `iiia:nao-deslop-20260714-v1` built successfully and host/container
  hashes match.
- Runtime gate: `preflight_not_scored`. `10.7.138.215:8004` refused connections,
  leaving chatbot and dialogue unconfigured and planner absent. See
  `docs/artifacts/runtime_seam_deslop_preflight_2026-07-14.md`.

## Stop And Rollback Rules

- Stop a tranche when its acceptance gate passes, its bounded iteration budget
  is exhausted, or external evidence is unavailable.
- Mark missing runtime evidence `not_scored`; do not count it as model failure.
- Revert a hypothesis-specific semantic edit that fails its protected holdouts.
- Do not keep a broad fallback because it rescues one test while changing valid
  model output elsewhere.
- Do not stage, commit, or push until the user requests the final repository
  operation.
