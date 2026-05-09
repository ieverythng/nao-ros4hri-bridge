# Demo Planner Repo Sweep Handoff

Target context:

- Main repo: `ieverythng/nao-ros4hri-bridge`
- Active demo branch: `fix/head_motion_skill`
- Chatbot fork: `ieverythng/nao_chatbot_llm`
- Active chatbot branch: `feat/planner_llm_hooks`
- Immediate goal: make the planner loop clear, stable, and demoable tomorrow.

## 1. Current architecture to preserve

Do **not** collapse the stack into one all-purpose LLM node. Preserve the layered ROS4HRI architecture:

```text
/humans/voices/*/speech
  -> dialogue_manager
  -> chatbot_llm
  -> direct /intents OR /planner/request
  -> planner_llm
  -> /intents
  -> nao_orchestrator
  -> robot/mock skills
  -> /planner/execution_feedback
  -> planner_llm
  -> /planner/dialogue_act when speech/clarification is needed
  -> dialogue_manager
```

Canonical responsibilities:

| Node/package | Responsibility |
| --- | --- |
| `dialogue_manager` | Dialogue lifecycle and speaking ownership |
| `chatbot_llm` | Response generation, intent declaration, KB snapshot injection, planner routing |
| `planner_llm` | Goal supervision, plan generation, replanning, planner dialogue acts |
| `planner_common` | JSON contract normalization helpers |
| `nao_orchestrator` | Deterministic plan validation, ordered execution, feedback publication |
| `kb_skills` | KnowledgeCore query/revise boundary |
| `nao_scene_grounding` | Detector-to-KB object facts and `/scene/summary` |

Important design rule:

> The planner decides **what needs to be communicated**. The dialogue layer decides **how it is communicated**.

So `planner_llm` should publish `/planner/dialogue_act` for clarification, progress, failure explanation, completion, etc. It should not become the primary speech owner. `say` steps can exist only when speech order is part of an executable plan.

## 2. Demo success criteria

Tomorrow’s demo should prove the loop, not a full robot intelligence claim:

1. `/planner/request` visible from `chatbot_llm` or fixture.
2. `planner_llm` receives goal/intents/context and emits `/intents` with `Intent.data.plan`.
3. `nao_orchestrator` validates the plan and publishes `/planner/execution_feedback`.
4. `planner_llm` observes feedback.
5. Completion/failure/clarification is visible via feedback and/or `/planner/dialogue_act`.

Recommended topic echoes:

```bash
ros2 topic echo /planner/request
ros2 topic echo /intents
ros2 topic echo /planner/execution_feedback
ros2 topic echo /planner/dialogue_act
```

## 3. Highest-priority code checks before demo

### A. Stop silently hiding unsupported planner steps

Current risk: the planner skill registry can filter unsupported steps. This may turn an invalid multi-step plan into a partial valid plan without making the truncation obvious.

Action:

- Add a strict diagnostic path around `SkillRegistry.filter_supported_steps()`.
- If model output contains steps but some are rejected, emit either:
  - a `plan_invalid`-style decision, or
  - a planner dialogue act explaining unsupported steps, depending on the path.
- At minimum, log rejected steps with step id/name/type/args.

Suggested minimal implementation direction:

```python
normalized_steps = normalize_plan_steps(steps)
filtered_steps = self._skill_registry.filter_supported_steps(normalized_steps)
if len(filtered_steps) != len(normalized_steps):
    # avoid silent partial-plan success during demo diagnostics
    return []
return filtered_steps
```

Better follow-up:

- Add `filter_supported_steps_with_rejections()` returning `(supported, rejected)`.
- Include `rejected_steps` in `failure_reason` or `raw_model_output` diagnostics.

### B. Strengthen skill registry prompt manifest

Current risk: the model previously generated `look_at` args like `{"target": "blueberry"}` instead of the orchestrator contract requiring `target_frame`/`frame_id` or `policy=reset`.

Reason: `PlannerSkill.prompt_summary()` currently omits several useful schema fields. The planner prompt needs more explicit parameter guidance.

Patch direction in `src/planner_llm/planner_llm/skill_registry.py`:

```python
def prompt_summary(self) -> dict:
    return {
        'name': self.name,
        'aliases': list(self.aliases),
        'category': self.category,
        'params': list(self.params),
        'required_params': list(self.required_params),
        'preconditions': list(self.preconditions),
        'expected_effects': list(self.expected_effects),
        'observable_success': list(self.observable_success),
        'failure_modes': list(self.failure_modes),
        'retryable': self.retryable,
        'can_request_user_help': self.can_request_user_help,
        'can_request_clarification': self.can_request_clarification,
        'robot_adapter_mapping': self.robot_adapter_mapping,
    }
```

Also make the derived `look_at` contract more explicit:

```python
'look_at': {
    'default_payload': {
        'name': 'look_at',
        'category': 'attention',
        'params': ['target_frame', 'frame_id', 'policy', 'x', 'y', 'z'],
        'required_params': ['target_frame OR frame_id OR policy=reset'],
        'failure_modes': [
            'look_at step missing target_frame or reset policy',
            'look_at target dispatch failed',
        ],
        ...
    }
}
```

### C. Make `step_succeeded` feedback semantically clearer

Current risk: in the orchestrator, `step_succeeded` may be published with `status='running'` and `event_type='step_succeeded'`. The supervisor uses `event_type`, so this works, but topic echo during demo is confusing.

Patch direction in `src/nao_orchestrator/nao_orchestrator/orchestrator.py`:

```python
self._publish_plan_feedback(
    intent_name=intent_name,
    source=source,
    plan_context=plan_context,
    status='succeeded',
    event_type='step_succeeded',
    step=step,
)
```

Patch direction in `src/planner_common/planner_common/contracts.py`:

```python
def _default_feedback_event_type(status: str) -> str:
    mapping = {
        'accepted': 'plan_accepted',
        'running': 'step_started',
        'succeeded': 'step_succeeded',
        'completed': 'plan_completed',
        'invalid': 'plan_invalid',
        'failed': 'step_failed',
    }
    return mapping.get(str(status or '').strip().lower(), '')
```

Optional: keep backward compatibility in tests by allowing either `running + step_succeeded` or `succeeded + step_succeeded`, but prefer the latter.

### D. Keep planner acknowledgements out of orchestrator unless ordered

Current code has the right direction: `_maybe_dispatch_acknowledgement()` no-ops because planner acknowledgement speech should be realized through planner dialogue acts.

Do not revert this.

Recommended policy:

- `acknowledge`, `progress_update`, `ask_clarification`, `ask_for_help`, `explain_failure`, `notify_completion`, `notify_cancellation` go through `/planner/dialogue_act`.
- `say` steps stay in `Intent.data.plan` only when speech ordering is part of the plan itself.

### E. Refresh GitNexus runtime artifacts

The current `docs/knowledge/ROS_RUNTIME_GRAPH.md` appears stale relative to the active planner branch. It shows parameter-name placeholders such as `/planner_request_topic` and misses newer runtime wiring visible in active docs.

Before demo or after final code changes, run:

```bash
tools/knowledge/index_repo.sh --force
# or
tools/knowledge/post_commit_refresh.sh
```

If the GitNexus UI/backend is already running, restart it after the refresh.

## 4. Chatbot branch checks

Repo: `ieverythng/nao_chatbot_llm`, branch `feat/planner_llm_hooks`.

What looks good:

- `chatbot_llm` remains aligned with the upstream dialogue backend contract.
- `chatbot_llm` publishes planner requests only when planner mode is enabled and a turn is execution-oriented.
- Planner requests include `goal_text`, `normalized_intents`, `scene_targets`, `requested_plan`, and `grounded_context`.
- Normal planner requests intentionally omit raw `user_text`; `goal_text` is the clean planner-facing objective.
- Generation caps are set low (`response_max_tokens: 64`, `intent_max_tokens: 64`) to avoid Qwen runaway outputs during speech-facing turns.

Potential follow-up, not required for tomorrow:

- `DialogueSession.active_planner_goal_id` is set when a planner request is published, but there is no visible reset on planner completion/cancellation unless a future feedback/dialogue-act listener updates it.
- For true multi-turn clarification handling, `chatbot_llm` eventually needs a way to classify a user reply as `clarification_answer` for the active goal.
- This is Phase B/C work; do not block the demo on it.

## 5. Demo slide talking points

Use this phrasing:

> The system is now split into a dialogue layer, a planning/supervision layer, and a deterministic execution layer. `chatbot_llm` declares the user goal and routes execution turns. `planner_llm` turns the goal into a structured plan and supervises progress over time. `nao_orchestrator` validates and executes the plan without owning the LLM reasoning. Feedback closes the loop and allows replanning or dialogue acts.

For the JSON slide, show `/planner/request` from `chatbot_llm`:

```json
{
  "request_id": "role:1",
  "goal_id": "goal_role_1",
  "request_kind": "new_goal",
  "goal_text": "look left",
  "normalized_intents": ["head_look_left"],
  "ack_text": "Okay, I will look left.",
  "ack_mode": "say",
  "scene_targets": [],
  "requested_plan": [],
  "grounded_context": {
    "knowledge_snapshot": {"summary_text": "..."},
    "scene_summary": {},
    "world_model_snapshot": {},
    "world_model_text": ""
  },
  "planner_mode": "default",
  "interaction_mode": "speech"
}
```

For the planner slide, show `Intent.data.plan`:

```json
{
  "goal_id": "goal_role_1",
  "plan": {
    "plan_id": "plan_123",
    "plan_version": 1,
    "status": "planning",
    "retry_budget": 1,
    "steps": [{
      "id": "step_1",
      "type": "skill",
      "name": "perform_motion",
      "args": {"object": "head_look_left"},
      "requires": [],
      "on_failure": "replan",
      "retry_budget": 0
    }]
  }
}
```

## 6. Suggested validation commands

Main workspace:

```bash
PYTHONPATH=src/planner_common:src/planner_llm:src/nao_orchestrator:src/kb_skills \
python3 -m pytest -q \
  src/planner_common/test/test_contracts.py \
  src/planner_llm/test/test_planner_engine.py \
  src/planner_llm/test/test_supervisor.py \
  src/nao_orchestrator/test/test_nao_orchestrator_intent_rules.py
```

Chatbot fork inside workspace:

```bash
cd src/chatbot_llm
PYTHONPATH="$PWD:$OLDPWD/src/planner_common:$OLDPWD/src/kb_skills" \
python3 -m pytest -q \
  test/test_planner_request_adapter.py \
  test/test_turn_engine.py
```

ROS topic smoke test:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py
ros2 run planner_llm publish_fixture request
ros2 topic echo /planner/request
ros2 topic echo /intents
ros2 topic echo /planner/execution_feedback
ros2 topic echo /planner/dialogue_act
```

## 7. Codex prompt seed

Use this prompt:

```text
We are preparing the `fix/head_motion_skill` branch of `ieverythng/nao-ros4hri-bridge` for a planner demo. Preserve the ROS4HRI architecture: dialogue_manager owns dialogue/speech, chatbot_llm owns response + intent declaration + planner routing, planner_llm owns goal supervision/planning/replanning/dialogue acts, nao_orchestrator owns deterministic validation/execution/feedback.

Please make a focused demo-hardening pass only. Priorities:
1) Prevent silent partial plans when SkillRegistry filters unsupported model steps. Add logging or strict invalidation if generated steps are dropped.
2) Expand PlannerSkill.prompt_summary() so the model sees params, aliases, observable_success, and robot_adapter_mapping. Make the look_at registry contract explicit: use target_frame/frame_id or policy=reset, not args.target.
3) Change orchestrator step_succeeded feedback to status='succeeded' with event_type='step_succeeded', and update planner_common default event mapping accordingly.
4) Keep planner acknowledgement/progress/completion as /planner/dialogue_act, not executor-owned speech, except for explicit say steps inside ordered plans.
5) Run focused tests for planner_common, planner_llm, nao_orchestrator, and chatbot_llm planner_request_adapter.
6) Do not make broad refactors or change upstream-sensitive dialogue_manager contracts.
```
