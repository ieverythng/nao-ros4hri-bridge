# Repo Boundaries

Use this reference when a change touches multiple packages or the ownership is
unclear.

## First-party packages that are usually safe to evolve directly

These are the main local integration and robot-adapter packages for this repo.
Prefer working here first when you can solve the problem without changing
upstream-sensitive packages.

- `planner_common`
- `planner_llm`
- `nao_orchestrator`
- `nao_chatbot`
- `nao_scene_grounding`
- `nao_say_skill`
- `nao_look_at`
- `nao_skills`
- other `nao_*` robot-adapter packages introduced specifically for this stack

## Nested repo or fork-tracked packages: use narrower changes

These packages may live inside the monorepo, but they follow upstream or
separate-repo expectations closely enough that broad refactors are riskier.
Prefer seam-focused changes and keep compatibility in mind.

- `chatbot_llm`
- `dialogue_manager`
- `interaction_skills`
- `communication_skills`
- `motions_skills`
- `std_skills`
- other imported or bootstrapped ROS4HRI packages under `src/`

## Ownership reminders

### Dialogue side

- `dialogue_manager` owns dialogue lifecycle and speaking ownership.
- `chatbot_llm` owns user-facing language generation, grounded dialogue, and
  planner ingress publication.

### Planning side

- `planner_llm` owns supervision, planning, replanning, cancellation policy,
  and planner dialogue acts.
- `planner_common` owns planner request, plan, feedback, and dialogue-act
  contracts.
- `nao_orchestrator` owns planner-request gating and planner-dialogue relay in
  the live seam profile; preserve this bridge instead of direct dialogue-manager
  wiring to planner topics.

### Execution side

- `nao_orchestrator` owns deterministic validation and execution.
- NAO-specific skill execution stays in robot-adapter packages.

### Grounding and KB side

- `nao_scene_grounding` owns detector normalization and detector-derived
  grounding.
- `kb_skills` owns local access to KnowledgeCore.

## Package-pattern checks

When a package already documents its own type, preserve it.

Look for these cues in package READMEs and manifests:

- `package type:`
- `scaffold basis:`
- `lifecycle` imports or launch usage
- upstream ROS4HRI ownership statements

Example from this repo:

- `nao_orchestrator` explicitly describes itself as a lifecycle orchestration
  node with an `rpk`-based scaffold.
- `nao_say_skill` explicitly documents itself as a local NAO-specific lifecycle
  skill and not a replacement for dialogue ownership.

## Practical rule

If the fix can live in a first-party adapter or seam package, prefer that over a
change to a nested repo or upstream-aligned package.
