# ROS4HRI Guardrails

Use this reference as the actual review checklist.

## 1. ROS core checks

### Topics vs services vs actions

- Use topics for continuous state and observations.
- Use services for synchronous request/response behavior.
- Use actions for long-running operations that need feedback or cancellation.
- Reuse existing ROS interfaces before inventing new ones.

Official ROS references:

- ROS 2 lifecycle design: <https://design.ros2.org/articles/node_lifecycle.html>
- ROS 2 concepts/docs index: <https://docs.ros.org/en/rolling/Concepts.html>

### Lifecycle usage

- Preserve lifecycle nodes where the package already follows that pattern.
- Prefer lifecycle usage for local robot-facing nodes that need explicit
  configure/activate transitions and predictable launch control.
- Do not force lifecycle onto upstream packages unless there is a strong,
  repo-wide reason.

### Validation expectations

For Python ROS packages, prefer this order:

1. targeted unit tests
2. `python3 -m py_compile` for touched entrypoints
3. launch argument checks
4. containerized or live ROS checks when runtime wiring changed

## 2. ROS4HRI and SocialMinds checks

### Preserve the public contract

- Favor ROS4HRI-compatible interfaces for public behavior.
- Keep robot-specific details behind adapter packages.
- Reuse SocialMinds or ROS4HRI package boundaries rather than recreating them in
  planner or chatbot code.

### Preserve ownership

- `dialogue_manager`: dialogue flow and speaking lifecycle
- `chatbot_llm`: user-facing language generation and planner ingress
- `planner_llm`: supervision and planning
- `nao_orchestrator`: deterministic execution
- `kb_skills`: KB transport boundary
- `nao_scene_grounding`: detector-to-KB bridge

### Avoid known bad shortcuts

- Do not bypass exposed ROS driver topics with direct robot SDK calls when the
  ROS seam already exists.
- Do not let planner code perform direct execution.
- Do not let executor code absorb dialogue or planning policy.
- Do not duplicate KnowledgeCore transport outside `kb_skills`.

Authoritative ROS4HRI and SocialMinds entry points:

- ROS4HRI docs: <https://ros4hri.github.io/index.html>
- SocialMinds GitLab group: <https://gitlab.iiia.csic.es/socialminds/ros4hri>

## 3. Repo-specific checks

### Planner stack

- `planner_common` should be the contract source of truth.
- `planner_llm` should reason over abstract skill contracts, not robot-specific
  topics or SDK calls.
- `nao_orchestrator` should accept validated plans and publish execution
  feedback, but not become an LLM policy node.

### Dialogue stack

- Keep `chatbot_llm` and `dialogue_manager` changes minimal and seam-focused.
- Prefer adding planner-facing hooks over rewriting the dialogue loop.

### LLM readiness and failure visibility

- Treat chatbot and planner LLM calls as ROS runtime dependencies, not hidden
  implementation details, when a launch profile depends on them for demo or
  operator use.
- Demo-critical profiles should fail fast or visibly warn during preflight
  before accepting the first user turn. Startup logs should name the selected
  chatbot and planner models, required/optional preflight policy, and the node
  that has become ready.
- Chatbot LLM failures on execution-looking turns may publish a planner request
  from the original goal text, but must not synthesize executable plans or speak
  raw backend errors.
- Planner backend failures should surface as planner failure dialogue acts, not
  user clarifications. Clarification should remain reserved for ambiguous or
  underspecified human requests.
- Do not move speaking ownership into planner or executor packages; planner
  dialogue acts remain hints consumed by `dialogue_manager`.

### Grounding and perception

- Keep detector normalization and scene grounding in `nao_scene_grounding`.
- Keep person-manager or future world-model work additive, not duplicative.

## 4. Repo docs to consult before large changes

Read these when the change is architectural or spans multiple packages:

- `docs/launch_profiles.md`
- `docs/planner_architecture_current.md`
- `docs/artifacts/demo_status_and_contracts.md`
- `docs/artifacts/thesis_planning_handoff.md`
- `docs/artifacts/planner_supervisor_phase_c_handoff.md`
- `docs/artifacts/ros4hri_fork_delta_ledgers.md`

## 5. What I found on the internet

I did not find an established Codex-style ROS4HRI guardrail skill that matches
this exact use case.

Closest related artifacts:

- official ROS and ROS4HRI documentation
- ROS4HRI runtime "skills" packages such as `interaction_skills`
- general community automation skills around ROS 2, such as the public
  `ros2-skill` entry on skills.rest: <https://skills.rest/skill/ros2-skill>

That means a repo-bound skill is justified here because the important knowledge
is not generic ROS knowledge alone; it is the combination of ROS, ROS4HRI,
SocialMinds conventions, and the specific ownership boundaries of this stack.
