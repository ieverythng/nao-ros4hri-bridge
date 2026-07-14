# Seam Hypothesis Audit, 2026-07-14

## Target contract

The live response-first stack must admit a grounded single-object delivery when
the object and its current location are already present in `grounded_context`.
It must continue to require an explicit structured target selection for grouped
or ambiguous delivery. Grounded locations must never be classified as
deliverable objects.

Protected owners are `chatbot_llm` for routing and handoff projection,
`planner_common` for contract normalization, `planner_llm` for planning and
supervision, `nao_orchestrator` for admission and execution, and `kb_skills`
for KnowledgeCore transport. No prompt wording, ROS interface, or launch
profile change is part of this round.

Acceptance requires focused source tests to pass, no location/object type
collision in the contract projection, and clean rebuilt live runs to prove the
former pear, phone, and cup turns without a complete-context clarification.

## Baseline evidence

The source was validated through three clean overlays built from `iiia:nao`:
`iiia:nao-sos-20260714-v25`, `v26`, and `v27`. Each runtime used one full
response-first launch and the same simulator profile. The integrated
`interaction_trace_viewer` is operator-owned, and an additional operator viewer
may be present during inspection. Viewer multiplicity is excluded from core
stack uniqueness and semantic scoring.

The exact former failure was recorded as turn
`__default__:cbdf92bc:8`:

- `chatbot_llm` produced `route=execution`, `intent=bring_object`, and a
  grounded pear target.
- The grounded context contained `pear_xiuwe` with `oro:isAt Lab`, plus a Lab
  location group containing that pear.
- The published planner request had `scene_targets: ["pear xiuwe"]`, but no
  `target_selection` object.
- `planner_llm` returned `ask_clarification` before calling the LLM planner,
  with reason `I need a complete grounded target selection before I can plan
  that task.`

The same live payload classified `Kitchen`, `Lab`, and `Table` as
`kind: "object"` inside `grounded_context.entities`, although the separate
`locations` view classified them correctly. Domain objects such as the pear and
phones also carried spatial RDF materialization types, so a class-only fix would
have been unsafe.

## Approach registry

| ID | Family | Mechanism | Discriminating probe | Status | Exact gap or reopen condition |
| --- | --- | --- | --- | --- | --- |
| H-01 | Contract | Planner gate rejects any `bring_object` without `target_selection` | Trace the planner request and confirm whether the planner provider is called | accepted | Reopen if a single grounded target is still clarified after clean rebuild |
| H-02 | Grounding | Object/location projection loses or mislabels location semantics | Normalize a spatial container with a domain object carrying spatial RDF types | accepted | Reopen if location records enter object selection or counts after rebuild |
| H-03 | Prompt/model | The response or planner model independently asks for location | Compare route, handoff payload, planner provider invocation, and planner dialogue reason | rejected for this trace | Reopen only after H-01 is removed and a complete payload still yields a clarification |
| H-04 | Runtime wiring | Duplicate launch, stale DDS graph, or KB transport caused the clarification | Check process count, node list, KnowledgeCore port, and live query evidence | rejected for this trace | Reopen if a clean single-stack run reproduces the same reason |
| H-05 | Perception/KB freshness | The object relation was absent at the decision boundary | Inspect `chatbot_turn_result.grounded_context` and KnowledgeCore query results | rejected for this trace | Reopen if the rebuilt run shows stale or missing `oro:isAt` before handoff |
| H-06 | Harness | Trace correlation attached a clarification to the wrong turn | Match turn id, goal id, planner request, and planner dialogue act | rejected for this trace | Reopen if future artifacts lack stable lineage or post-terminal timing |
| H-07 | Startup initialization | Readiness, lifecycle, or KB startup order differs between repeated demonstrations | Capture one launch pid, core-node uniqueness, lifecycle state, baseline KB query, and bind/timeout logs before the first turn | active | Reopen on every clean rebuild until two consecutive preflight gates pass |

## Discriminating probes and results

1. **Planner admission probe.** The trace proved a complete object/location
   payload and an immediate planner clarification. Source inspection located the
   exact early gate in `planner_engine.py`. The gate was narrowed to preserve
   grouped/ambiguous selection while allowing one uniquely matched grounded
   object to reach the planner.
2. **Location classification probe.** Contract tests showed that a materialized
   `cyc:SpatialThing-Localized` book with an RDF `Book` type must remain an
   object. The normalizer now gives domain object types precedence and only
   classifies spatial containers/supports as `location`.
3. **Source controls.** `planner_common` contract tests: 55 passed.
   `planner_llm` engine tests: 53 passed. Nested chatbot planner adapter and
   knowledge snapshot tests: 72 passed. `ruff`, `py_compile`, and `git diff
   --check` passed.
4. **Runtime boundary.** The v25 clean launch had singular core nodes, active
   lifecycle owners, KnowledgeCore readiness, and the expected simulator camera.
   The former work-table delivery then completed three `bring_object` steps and
   a `report_result` step without clarification. The chatbot emitted invalid JSON
   in that turn, so route repair and the rules intent fallback admitted the
   request. This was recorded as a fallback-rate finding, not a semantic pass
   without qualification.
5. **Recovery probe.** The v26 `fail_once_navigation` run initially failed a
   navigation step with `on_failure: replan`, but its skill payload suggested
   asking the user. The planner supervisor patch gave recoverable plan policy
   precedence over that suggestion. After the v26 clean rebuild, planner version
   2 was published, all remaining ordered targets completed, and post-failure
   speech reported the arrivals. The v26 `fail_once_pick` composite case also
   replanned and completed. `delivery_blocked` correctly produced an
   `ask_for_help` act and truthful blocked-path speech without retrying.
6. **Location/count probe.** The v27 live trace reported
   `entities=12`, `objects=6`, `people=2`, and `locations=4`. The location
   entities were the desk, handoff area, lab room, and work table. The populated
   relation view contained the desk and work-table groups, with object members
   kept separate from the location records.
7. **Initialization probe.** Two consecutive fresh startup windows passed the
   stack-owned preflight: core nodes were singular, dialogue and orchestrator
   lifecycle states were active, the four KB services were present, and the
   response-first/digest-disabled parameters matched the review profile. The
   NAOqi connection to `172.26.112.130:9559` remained refused, so real-robot
   actuation is not validated by this simulator run. Camera and simulator
   perception remained available.
8. **Support-object semantic probe.** The v28 source was rebuilt from
   `iiia:nao` and the fresh response-first stack passed the lab inventory and
   grouped work-table delivery cases. The live `chatbot_turn_result` payload
   classified `codex_lab_desk` (`Desk`) and `codex_lab_table_section` (`Table`)
   as `kind: "object"`, while `codex_lab_room` remained `kind: "location"`.
   The derived support groups still contained only the supported computer,
   keyboard, cup, manual, and phone. This preserves physical object identity
   without losing location-aware grouped queries.
9. **Context assembly probe.** The main v28 launch had
   `grounded_context_digest_enabled=false`, so the optional scene digest was
   not fed to the chatbot. The live turn trace exposed only the compact
   grounded-context JSON. The source path bounds the live KB query to 120 rows
   and 8,000 characters, projects at most 30 entities to the chatbot, and
   trims dialogue history to the configured 20-message window. The retained
   `recent_scene_memory` is tracked for observability but is not appended to
   the active prompt in this path. This rules out an unbounded context flood as
   the primary explanation for the v28 run. It does not rule out semantic
   contamination from contradictory KB relations, route repair, or stale
   dialogue history inside the bounded window.
10. **Fresh cleanup probe.** Both requested v28 cases passed, but final fixture
    cleanup left eight inferred KnowledgeCore facts (`owl:Thing` and pose
    materializations) after retracting the fixture statements. The questionnaire
    correctly records this as cleanup contamination rather than a semantic
    runtime failure. It remains a harness/KB lifecycle issue to fix before
    treating long multi-case runs as isolated.

## Adversarial audit

- [x] Changes remain in planner contract and planner admission owners.
- [x] No ROS topic, service, action, launch, or prompt contract changed.
- [x] Domain object types outrank generic spatial RDF materialization.
- [x] People remain distinct from objects and locations.
- [x] Grouped delivery still requires structured target selection.
- [x] No executable plan is embedded in chatbot intent payloads.
- [x] No speech fallback or output sanitizer was added.
- [x] Focused success and clarification controls are covered by tests.
- [x] Clean rebuilt live proof covers grouped delivery, ordered recovery, pick
      recovery, blocked delivery, and live location counts.
- [x] Physical support entities remain objects in both grounded JSON and the
      optional digest, while support anchors stay out of group member lists.
- [x] Two consecutive clean startup gates passed for stack-owned nodes. The
      operator-owned trace viewer is excluded from this gate.
- [ ] Fallback-free chatbot JSON remains a quality holdout. One grouped case
      used route repair and a rules intent fallback even though execution and
      reporting were correct.
- [ ] Fixture cleanup needs a separate harness pass. One gold-apple case was
      correctly marked `not_scored` after stale KnowledgeCore facts remained
      during preflight; it was not counted as a semantic failure.
- [ ] Context poisoning remains a bounded hypothesis, not a confirmed root
      cause. The next probe should compare the same request with empty history,
      bounded history, and the full active history while logging route repair,
      target admission, and exact grounded-context relations.

## Decision: bounded handoff

The source diagnosis and runtime fixes are accepted for this bounded handoff.
The former clarification was a contract/admission regression, not evidence of
an unreachable LLM endpoint or missing KB fact. Location classification and
top-level count normalization are live-proven, and the replan supervisor now
distinguishes autonomous recoverable failures from delivery failures that need
user choice. The stack is not a fallback-free or real-robot acceptance yet.

## Residual risk and next probe

The next bounded probe is a fallback-free response/intent holdout over the
already passing grouped and single-object cases. Track whether the LLM returns
valid JSON without route repair or rules intent fallback, while preserving the
same semantic acceptance criteria. In parallel, repair the questionnaire's
fixture cleanup verification so stale facts yield `not_scored` before the case,
without weakening the guard. A separate real-robot run is required for NAOqi
actuation because the current endpoint is unreachable.

For the context-poisoning question, do not increase the context window or add
more prompt instructions as a first response. Capture the assembled request
shape for one passing and one failing repeated-KB turn, then ablate only the
history window and the optional digest. A failure that persists with empty
history and the same grounded JSON points to route or contract handling; a
failure that disappears only when history is removed points to bounded-history
contamination. The operator-owned `interaction_trace_viewer` remains outside
this diagnosis and must not be modified or scored as a core failure.

Do not change prompt text until that structural holdout is complete. If a
complete grounded single-object request still clarifies after the rebuild,
start a bounded SkillOpt baseline and holdout for the prompt/model route rather
than adding another deterministic fallback.
