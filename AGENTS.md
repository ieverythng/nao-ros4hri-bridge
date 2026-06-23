# Agent Guide

This repository is a ROS 2 Jazzy workspace for a NAO-centered ROS4HRI thesis
stack. Its main design constraint is separation of dialogue, planning,
deterministic execution, perception grounding, and symbolic knowledge.

## Start Here

Before a meaningful change:

1. Read the nearest package-local `AGENTS.md` and README.
2. Read `docs/current_workflow.md` for ownership and runtime flow.
3. Read `docs/contracts.md` for cross-node payloads and
   `docs/launch_profiles.md` for launch/runtime wiring.
4. For architectural work, read the relevant file under `docs/architecture/`
   and the active plan under `docs/plans/`.
5. Run `python3 scripts/ros4hri_change_audit.py --mode working`.
6. Inspect the working tree, including nested repositories, before editing.

Use `docs/README.md` to navigate documentation. Treat `docs/artifacts/`,
`docs/handoffs/`, generated GitNexus wiki pages, and dated reports as evidence
and provenance, not as the current contract unless an active doc points to them.

When documentation disagrees, use this precedence:

1. current source, tests, launch files, and package-local `AGENTS.md`
2. accepted ADRs and active architecture contracts
3. `docs/current_workflow.md`, `docs/contracts.md`, and `docs/launch_profiles.md`
4. `docs/plans/nao_ros4hri_masterplan.md`
5. generated knowledge pages and historical artifacts

Call out meaningful drift instead of silently choosing a convenient version.

## Architectural Invariants

- `dialogue_manager` owns dialogue lifecycle and speaking.
- `chatbot_llm` owns user-facing LLM dialogue, route selection, grounding
  projection, and planner handoff. It does not create executable plans.
- `planner_common` owns planner request, plan, feedback, and dialogue-act
  normalization contracts.
- `planner_llm` owns task planning, supervision, retry/replan policy, and
  structured planner dialogue acts. It does not execute robot skills or speak.
- `nao_orchestrator` owns planner admission/gating, deterministic validation and
  dispatch, execution lineage, and feedback. It does not become an LLM policy,
  dialogue, or global world-model node.
- `kb_skills` is the KnowledgeCore query/mutation transport boundary. LLM nodes
  do not write directly to KnowledgeCore.
- `nao_scene_grounding` owns detector normalization, detector-derived scene
  facts, optional spatial overlay merging, and scene-summary publication.
- AB=1 skills own fresh execution-time evidence for their effects. Planning-time
  context is evidence, not proof that an action succeeded.
- `nao_chatbot` owns the integrated launch profiles and operator-facing wiring.
- Keep NAO-specific behavior behind adapter/execution packages and preserve
  ROS4HRI-compatible public interfaces.

Preserve the planner loop:

```text
dialogue_manager -> chatbot_llm -> nao_orchestrator planner gate -> planner_llm
planner_llm -> executable plan -> nao_orchestrator -> AB=1 skill
nao_orchestrator -> execution feedback -> planner_llm
planner_llm -> structured dialogue act -> dialogue/speech owner
```

The exact planner-dialogue relay and wording mode is launch-profile-sensitive.
Verify current launch and source wiring before changing it; preserve one
user-facing utterance authority per turn.

## Contract Guardrails

- Reuse existing ROS interfaces. Use topics for streamed state, services for
  short request/response operations, and actions for long-running,
  feedback-bearing, or cancellable work.
- Keep planner metadata canonical under nested `plan`.
- Preserve goal lineage through `goal_id`, `plan_id`, and `plan_version`, with
  stable `step_id` values where replans may mid-join.
- Do not reintroduce removed planner seams: `goal_token`, `world_model_*`,
  planner `ack_mode`/`ack_text`, duplicate top-level plan metadata, or hidden
  executable plans inside chatbot intent payloads.
- Treat grounded context as a compact, bounded planner projection. Keep raw
  detector, KB, spatial, and result evidence at their owning execution seams.
- Do not fabricate perception, proximity, KB facts, action results, or success.
  Image-plane centers are not metric distance evidence.
- Keep people and objects semantically distinct.
- Ordinary questions must not mutate the KB. Explicit planner-visible mutations
  go through `nao_orchestrator` and `kb_skills`.
- The canonical skill/AB registry is
  `src/Neural-Wokbench/src/skill_common/skill_common/defaults/ab_registry.json`.
  Runtime/planner/docs registry views are projections; update and check them
  together. AB=0 entries are interfaces/primitives, AB=1 entries are runtime
  skills, and AB>=2 entries are non-runtime proposals unless explicitly
  promoted.

## Prompt And LLM Contract Changes

- Treat prompt text as runtime-critical code. Do not add prompt wording,
  examples, or policy rules as a quick patch before checking whether the same
  contract already exists in the canonical prompt pack, system-turn addendum,
  planner prompt, or structured runtime payload.
- Canonical chatbot response and intent policy belongs in
  `src/chatbot_llm/config/chat_prompt_pack.yaml`. Canonical planner policy
  belongs in `src/planner_llm/config/planner_prompt_pack.yaml`. Python prompt
  builders and system-turn addenda should provide structural framing only unless
  a bounded SkillOpt iteration proves a local runtime wording contract is needed.
- Every prompt or LLM-facing addendum change must have a SkillOpt ledger entry:
  baseline, mutation batch, train result, holdout result, and accept/reject
  decision. Use the active tracker or a dated artifact; do not leave prompt
  behavior changes as unlogged source edits.
- Do not patch prompt text first and create the ledger afterwards. If a prompt
  wording change is made without the bounded SkillOpt baseline and holdout gate,
  revert it unless the user explicitly accepts the process violation.
- Prompt edits must preserve known-good seams. Before accepting a wording
  mutation, run focused holdouts for dialogue route safety, KB-query behavior,
  execution admission, report-result wording, and duplicate-speech prevention
  when the edited prompt can affect those seams.

## Package Sensitivity

Prefer local first-party integration packages for stack-specific behavior:

- `planner_common`, `planner_llm`, `nao_orchestrator`
- `nao_chatbot`, `nao_scene_grounding`, `nao_*` adapters

Use narrow, compatibility-conscious changes in nested or upstream-aligned
packages, including `chatbot_llm`, `dialogue_manager`, `Neural-Wokbench`,
`interaction_skills`, `communication_skills`, `motions_skills`, and
`std_skills`. Do not broadly normalize, deslop, or redesign them as collateral
work.

Preserve an existing package's lifecycle and `rpk` patterns. When runtime wiring
changes, update launch files, package metadata, docs, and tests together.

Package-local guidance currently exists at:

- `src/planner_llm/AGENTS.md`
- `src/chatbot_llm/AGENTS.md`
- `src/nao_orchestrator/AGENTS.md`
- `src/fake_skills/AGENTS.md`

## GitNexus Knowledge Layer

GitNexus under `tools/knowledge/` is a narrowing tool, not the final authority.

- Check freshness with `tools/knowledge/status.sh` before architecture-sensitive
  work.
- Before the first serious graph pass, run
  `scripts/bootstrap_socialminds_sources.sh` so ignored ROS4HRI/reference sources
  are visible. If network/bootstrap is unavailable, state the coverage gap.
- Explore with `query -> context -> source`.
- Debug with `query symptom -> context suspect -> source`.
- Refactor with `context + impact` before edits.
- Use raw `rg` and direct source reads to confirm graph findings.
- Refresh with `tools/knowledge/index_repo.sh` after major changes or when stale.
- If the shared HTTP backend is running, use
  `GITNEXUS_USE_HTTP=1 tools/knowledge/codex_with_gitnexus.sh` and avoid local
  graph commands that contend for its database.

See `docs/knowledge/WORKFLOWS.md`.

## Validation

Run the narrowest relevant checks first, then widen based on affected seams:

1. focused package/unit tests
2. `python3 -m py_compile` for touched Python entrypoints
3. `python3 scripts/check_skill_registry_consistency.py` for registry changes
4. launch argument/profile tests for runtime wiring changes
5. containerized or live ROS checks for cross-node behavior
6. `python3 scripts/ros4hri_change_audit.py --mode working` after changes

For planner/executor/runtime changes, verify the affected success path and at
least one failure, cancellation, supersede, or clarification path. For
user-facing flows, use structured traces and verify there is no duplicate speech
or fabricated completion. State clearly when live ROS, KnowledgeCore, simulator,
or robot validation remains pending.

Do not relaunch or disturb an existing live stack unless the user requests it.

## Documentation Hygiene

- Keep durable operational truth in the active docs listed by `docs/README.md`.
- Put dated investigations, generated reports, and temporary handoffs under
  `docs/artifacts/` or `docs/handoffs/`.
- Keep the masterplan as the canonical cross-track integration plan.
- Update contract/architecture docs when behavior changes; do not preserve stale
  examples merely for compatibility.
- Keep required Markdown/HTML plan pairs synchronized where the docs contract
  calls for them.
