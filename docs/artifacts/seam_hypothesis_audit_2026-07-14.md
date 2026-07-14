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
11. **Context-boundary probe.** The v28 trace reproduced the unsupported scene
    claim. The current turn contained five objects, only `apple_jbdym` had a
    `Table` relation, and the other visible phone, pear, and apple had no table
    relation. The response nevertheless named stale `ATLAS`, `MIDAS`, `TITAS`,
    and `VEGA` entities from an earlier lab turn and said all objects were on
    the table. Source tests now enforce a scene boundary for current inventory
    and attribute queries: the live grounded snapshot is sent without the
    previous dialogue window, and the returned history starts a fresh scene
    window. Reflective scene-change questions retain history.
12. **Context architecture probe.** The MCP comparison found no missing
    transport primitive that explains the regression. The existing
    `kb_skills -> grounded_context_v3` projection is already the specialized
    resource seam. An additional stateful context server would add session
    complexity without defining freshness. The research note is recorded in
    `docs/artifacts/context_projection_research_2026-07-14.md`.
13. **Clean v29 runtime probe.** Image `iiia:nao-sos-20260714-v29` was built
    from `iiia:nao`, the full stack was stopped and relaunched, and the active
    profile reported `response_first` with the digest disabled. Core nodes and
    KnowledgeCore services were singular. Two current-scene speech queries
    emitted `CONTEXT_BOUNDARY` in the chatbot log, and their replies stayed
    within the live fixture evidence. The environment questionnaire produced
    one semantic pass and one `not_scored` case because targeted cleanup left
    inferred base-table and robot-station facts. The latter is an explicit
    harness/KB lifecycle limitation, not a lowered semantic score.
14. **Live grouped-delivery follow-up diagnosis.** Direct log inspection of
    the `sim_person_lznze` case separated two failures. The first response
    correctly rejected the misspelled `iznze` reference. After the user said
    `I meant person lznze!`, the grounded context contained both phone IDs and
    the person ID, but the planner request contained only that correction as
    `goal_text`, with empty `normalized_intents` and `scene_targets`. The
    planner consequently returned no JSON object. This was a handoff-contract
    loss, not stale grounded context. The source patch now carries the
    rejected execution task across a named-person correction, resolves
    `sim_person_*` suffixes, expands `both phones` to concrete object IDs, and
    reopens planner admission only for a uniquely grounded correction, then
    clears the continuation after publication.
15. **v30 rebuild gate.** The source patch rebuilt successfully from
    `iiia:nao` as `iiia:nao-sos-20260714-v30`. The full launch reached the
    simulator, KnowledgeCore, HRI nodes, fake skills, and operator viewer, but
    the required chatbot and planner preflights both failed with connection
    refused at `10.7.138.215:8004`. The alternate historical address
    `192.168.50.86:8004` also did not respond from the host. No semantic score
    was assigned to this launch. The failure is external model-endpoint
    reachability, and the `/chatbot_llm` lifecycle remained unconfigured while
    `/planner_llm` exited, so this is not valid runtime evidence for the
    continuation patch.
16. **Fresh normal-base v31 preflight.** The overlay
    `iiia:nao-sos-20260714-v31` was built from the unchanged `iiia:nao` image
    after the handoff-admission test was added. The old v30 container was
    stopped before launching one fresh `nao_ros2` container. The startup graph
    contained one each of `chatbot_llm`, `dialogue_manager`,
    `nao_orchestrator`, `kb/knowledge_core`, `fake_skill_server`, and the HRI
    nodes; KnowledgeCore reached `ready`, and the active parameters were
    `response_first` with the grounded-context digest disabled. The
    operator-owned RQT/viewer process is excluded from semantic uniqueness.
    Chatbot and planner preflight again failed with connection refused at
    `10.7.138.215:8004`, so the run is `preflight_not_scored`, not a semantic
    regression. The snapshot now records this explicitly under
    `derived.preflight`, including the missing planner node and inactive
    lifecycle states, instead of relying on a zero fallback count.
    Snapshot: `/tmp/nao_runtime_v31_preflight.json`.
- [ ] Context poisoning from retained dialogue history is source-fixed and
      unit-tested, but a longer live repeated-KB sequence remains pending.
- [x] Current-scene queries reset stale dialogue scene context while reflective
      scene-change questions retain history. The operator-owned
      `interaction_trace_viewer` remains outside this change.
- [x] v29 clean rebuild verified the context-boundary log marker and did not
      reproduce the stale lab-name claim in the new current-scene replies.

## Decision: bounded handoff

The source diagnosis and runtime fix are accepted for this bounded handoff.
The former clarification was a contract/admission regression, not evidence of
an unreachable LLM endpoint or missing KB fact. Location classification and
top-level count normalization are live-proven, and the replan supervisor now
distinguishes autonomous recoverable failures from delivery failures that need
user choice. The stack is not a fallback-free or real-robot acceptance yet,
and the questionnaire still needs a KnowledgeCore reset or stronger
inferred-fact cleanup strategy between isolated fixture cases.

## Residual risk and next probe

The next bounded probe is a fallback-free response/intent holdout over the
already passing grouped and single-object cases. Track whether the LLM returns
valid JSON without route repair or rules intent fallback, while preserving the
same semantic acceptance criteria. In parallel, repair the questionnaire's
fixture cleanup verification so stale facts yield `not_scored` before the case,
without weakening the guard. A separate real-robot run is required for NAOqi
actuation because the current endpoint is unreachable.

For the context-poisoning question, the source ablation now removes the old
dialogue window for current-scene inventory and attribute queries. The next
runtime probe must repeat the failing sequence on a clean rebuilt stack and
verify that the live `chatbot_turn_trace` contains only the current scene
claim. A failure that persists with the boundary points to route or contract
handling; a failure that disappears confirms bounded-history contamination.
The operator-owned `interaction_trace_viewer` remains outside this diagnosis
and must not be modified or scored as a core failure.

Do not change prompt text while the model endpoint is unreachable. The current
canonical chatbot pack already requires a concise semantic `goal_text` rather
than a transcript; the new structural continuation test enforces that
contract without changing wording. Once the endpoint is reachable, run the
bounded SkillOpt baseline and holdout for short correction turns before any
prompt mutation. The holdout must include ordinary dialogue, current-scene KB
queries, direct execution, grouped delivery, and report-result wording.
