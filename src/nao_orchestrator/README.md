# nao_orchestrator

`nao_orchestrator` is the lifecycle orchestration package for NAO in the
migrated ROS4HRI stack.

Steady-state target:

- consume `/intents`
- dispatch canonical and NAO-specific skills
- replace the local mission-controller role previously hosted in `nao_chatbot`

What moved out of the old `mission_controller` on purpose:

- user-text ingestion now belongs to `dialogue_manager`
- chatbot turn execution now belongs to `dialogue_manager` + `chatbot_llm`
- assistant-text generation/history no longer lives in the orchestrator
- `nao_orchestrator` only owns downstream intent normalization and robot-skill dispatch

Transition support:

- optional subscription to the older string topic `/chatbot/intent`
- direct dispatch to `/nao/say`, `/skill/replay_motion`, `/skill/do_head_motion`,
  and `/skill/look_at`
- topic fallbacks matching the old mission-controller flow:
  `/chatbot/posture_command` and `/joint_angles`
- duplicate-intent suppression to avoid double-dispatch while legacy and new paths coexist

Current migration boundary:

- `nao_orchestrator` already covers the old mission-controller execution side:
  say dispatch, posture/replay-motion dispatch, retained head motion, and look-at reset
- conversational speech intents are ignored by default because spoken chatbot
  replies are already owned by `dialogue_manager -> /tts_engine/tts -> nao_say_skill`
- `kb_query_visible_people`, `kb_query_visible_objects`, and
  `kb_query_scene_change` are preserved as distinct intent labels from
  `chatbot_llm`, but the orchestrator currently only logs and ignores them so
  dialogue ownership stays upstream
- `chatbot_llm` does not connect directly to `nao_orchestrator` in steady state
  because the canonical flow is `dialogue_manager -> /intents -> nao_orchestrator`
- the older `/chatbot/intent` adapter is available but disabled by default

Structured intent metadata now also passes through `Intent.data` when present:

- `ack_text`: preferred acknowledgement text for the turn
- `ack_mode`: acknowledgement mode hint, currently informational
- `scene_targets`: grounded entities or labels relevant to the request
- `plan`: optional ordered execution steps for the orchestrator

Supported `plan` step types:

- `say`
- `skill`
- `look_at`
- `noop`

This keeps the top-level ROS contract stable while allowing richer downstream
execution plans to arrive from `chatbot_llm`.

High-level downstream flow:

1. `chatbot_llm` emits canonical HRI intents plus optional `ack_text`,
   `ack_mode`, `scene_targets`, and `plan`
2. `nao_orchestrator` parses that metadata from `Intent.data`
3. if a valid `plan` is present, the orchestrator tries to execute it first
4. if there is no valid plan, the package falls back to the migrated legacy
   routing for speech, motion, and look-at resets

The orchestrator still does not own KB prompting or detector subscriptions; it
only consumes the enriched downstream contract.

Manual smoke examples:

```bash
ros2 topic pub --once /chatbot/intent std_msgs/msg/String "{data: 'posture_stand'}"
ros2 topic pub --once /chatbot/intent std_msgs/msg/String "{data: 'head_look_left'}"
ros2 topic pub --once /chatbot/intent std_msgs/msg/String "{data: '{\"intent\":\"__intent_say__\",\"object\":\"Testing migrated say dispatch.\"}'}"
```

```bash
ros2 topic pub --once /intents hri_actions_msgs/msg/Intent "{intent: 'perform_motion', data: '{\"object\":\"stand\"}'}"
```

## Provenance

- package type: local lifecycle orchestration node, not a forked upstream repo
- scaffold basis: `rpk` mission-controller/lifecycle template
- architecture style: hybrid replacement
  - replaces the old local `mission_controller` execution role
  - aligns steady-state I/O to the migrated ROS4HRI flow: `/intents` in,
    skill actions out
  - retains a small compatibility layer for legacy intent/topic bridges while
    migration cleanup completes
- design constraint: this package must stay downstream-only and must not grow
  back into a chatbot or dialogue runtime

## Parameters

The package defaults are in `config/00-defaults.yml`. The most important knobs
are:

- `intent_topic`
- `enable_legacy_intent_bridge`
- `legacy_intent_topic`
- `nao_say_action`
- `dispatch_speech_intents`
- `replay_motion_action`
- `head_motion_action`
- `look_at_action`
- `posture_command_topic`
- `dedupe_window_sec`

Effective defaults from `config/00-defaults.yml`:

- `/intents` is the primary subscribed topic
- `/chatbot/intent` stays available as an optional legacy bridge
- `/nao/say` stays disabled for conversational replies unless
  `dispatch_speech_intents:=true`
- `/skill/replay_motion`, `/skill/do_head_motion`, and `/skill/look_at` remain
  the canonical downstream skill routes
- `/chatbot/posture_command` and `/joint_angles` remain temporary topic
  fallbacks during migration cleanup

## Planned Intent Handling

When a structured `plan` is present in `Intent.data`, `nao_orchestrator` tries
to execute those steps in order before falling back to the older intent routing
rules.

Current planned-step behavior:

- `say`: dispatch speech through `/nao/say`
- `skill`: currently supports motion-oriented routes such as replay motion and
  look-at reset
- `look_at`: currently scaffolded to reset-oriented look-at behavior
- `noop`: explicit no-op placeholder

If no valid `plan` exists, the package keeps the legacy migrated behavior for
speech, posture, head motion, look-at reset, and KB query intent observation.

## Design Rule

`nao_orchestrator` should stay downstream-only. It must not grow back into a
dialogue or chatbot node. User-turn ingestion belongs to `dialogue_manager`,
and model interaction belongs to `chatbot_llm`.
