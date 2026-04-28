# Slide-Ready Runtime Contracts

Branch basis: `nao-ros4hri-bridge@fix/head_motion_skill` and `nao_chatbot_llm@feat/planner_llm_hooks`.

---

## 1. chatbot_llm JSON Output

**Owner:** `chatbot_llm`  
**Purpose:** response generation, route decision, intent declaration, planner routing.  
**Important:** chatbot does not own executable plan steps.

```json
{
  "verbal_ack": "Okay, I will do that.",
  "route": "execution",
  "confidence": 0.82,
  "user_intent": {
    "type": "head_look_left",
    "goal": "look left",
    "goal_text": "look left",
    "ack_text": "Okay, I will look left.",
    "ack_mode": "say",
    "scene_targets": [],
    "request_kind": "new_goal",
    "interaction_mode": "speech"
  }
}
```

**Allowed route values**

```text
dialogue | knowledge_query | execution
```

**Policy**

```text
chatbot_llm declares the user-facing intent and routes execution turns.
planner_llm owns executable planning and supervision.
nao_orchestrator owns deterministic execution.
```

---

## 2. Planner Request

**Topic:** `/planner/request`  
**ROS type:** `hri_actions_msgs/msg/Intent`  
**Publisher:** `chatbot_llm`  
**Subscriber:** `planner_llm`

```json
{
  "request_id": "role:1",
  "goal_id": "goal_role_1",
  "parent_goal_id": "",
  "supersedes_goal_id": "",
  "request_kind": "new_goal",
  "goal_text": "look left",
  "normalized_intents": ["head_look_left"],
  "ack_text": "Okay, I will look left.",
  "ack_mode": "say",
  "scene_targets": [],
  "dialogue_context": [],
  "requested_plan": [],
  "grounded_context": {
    "knowledge_snapshot": {
      "summary_text": "Entities currently seen by the robot: cup_1 (Cup)"
    },
    "scene_summary": {},
    "world_model_snapshot": {},
    "world_model_text": ""
  },
  "planner_mode": "default",
  "interaction_mode": "speech",
  "dialogue_turn_id": "role:1"
}
```

**Field policy**

```text
goal_text is the primary planner objective.
normalized_intents are strict labels from chatbot_llm.
scene_targets are grounded target labels/entities.
requested_plan is optional only; planner_llm should not depend on it.
user_text is legacy-only and should not appear in normal planner requests.
```

**ROS envelope note**

```text
priority and confidence appear in ros2 topic echo because they belong to hri_actions_msgs/Intent.
Planner semantics live inside Intent.data.
```

---

## 3. Planner Output

**Topic:** `/intents`  
**ROS type:** `hri_actions_msgs/msg/Intent`  
**Publisher:** `planner_llm`  
**Subscriber:** `nao_orchestrator`

```json
{
  "goal_id": "goal_role_1",
  "ack_text": "",
  "ack_mode": "",
  "user_facing_reason": "",
  "scene_targets": [],
  "grounded_context": {
    "knowledge_snapshot": {},
    "scene_summary": {},
    "world_model_snapshot": {},
    "world_model_text": ""
  },
  "plan": {
    "goal_id": "goal_role_1",
    "plan_id": "plan_123",
    "plan_version": 1,
    "status": "planning",
    "validation_status": "draft",
    "failure_reason": "",
    "user_facing_reason": "",
    "replan_hint": "",
    "retry_budget": 1,
    "scene_targets": [],
    "communication_policy": {
      "emit_acknowledge": false,
      "emit_progress": false,
      "emit_completion": true,
      "emit_failure": true
    },
    "steps": [
      {
        "id": "step_1",
        "type": "skill",
        "name": "perform_motion",
        "args": {
          "object": "head_look_left"
        },
        "requires": [],
        "on_failure": "replan",
        "retry_budget": 0
      }
    ]
  }
}
```

**Allowed step types**

```text
noop | say | skill | look_at
```

**Allowed failure policies**

```text
fail | continue | replan | clarify | ask_user | ignore
```

**Policy**

```text
planner_llm plans over abstract skills.
It must not call robot-specific topics, NAOqi APIs, or hardware directly.
```

---

## 4. Execution Feedback

**Topic:** `/planner/execution_feedback`  
**ROS type:** `std_msgs/msg/String`  
**Publisher:** `nao_orchestrator`  
**Subscriber:** `planner_llm`

```json
{
  "goal_id": "goal_role_1",
  "plan_id": "plan_123",
  "plan_version": 1,
  "intent": "raw_user_input",
  "source": "planner_llm",
  "event_type": "step_succeeded",
  "status": "running",
  "reason": "",
  "validation_status": "draft",
  "replan_hint": "",
  "retry_budget": 1,
  "blocking": false,
  "unmet_preconditions": [],
  "needs_user_input": false,
  "scene_targets": [],
  "validation_errors": [],
  "timestamp_sec": 1777040000.0,
  "step": {
    "id": "step_1",
    "type": "skill",
    "name": "perform_motion",
    "retry_budget": 0,
    "on_failure": "replan",
    "requires": []
  }
}
```

**Important event types**

```text
plan_accepted
step_started
step_succeeded
step_failed
plan_invalid
plan_completed
```

**Policy**

```text
nao_orchestrator validates and executes.
planner_llm interprets feedback and decides whether to continue, replan, ask the user, fail, or cancel.
```

---

## 5. Planner Dialogue Act

**Topic:** `/planner/dialogue_act`  
**ROS type:** `std_msgs/msg/String`  
**Publisher:** `planner_llm`  
**Subscriber:** `dialogue_manager`

```json
{
  "goal_id": "goal_role_1",
  "plan_id": "plan_123",
  "plan_version": 1,
  "act": "notify_completion",
  "priority": "normal",
  "await_user_response": false,
  "reason": "goal completed",
  "text_hint": "I am looking to the left now.",
  "slots_needed": [],
  "context": {
    "scene_targets": [],
    "plan_step_count": 1,
    "requested_intents": ["head_look_left"],
    "goal_text": "look left",
    "status": "completed"
  }
}
```

**Allowed dialogue acts**

```text
acknowledge
progress_update
ask_clarification
ask_for_help
explain_failure
notify_completion
notify_cancellation
```

**Policy**

```text
planner_llm decides what must be communicated.
dialogue_manager/chatbot_llm realize how it is communicated.
planner_llm should not directly own speech output.
```

---

## 6. Knowledge Snapshot

**Owner:** `chatbot_llm`  
**Source:** `/kb/query` through `kb_skills`  
**Purpose:** compact prompt-ready symbolic context from KnowledgeCore.

```json
{
  "summary_text": "Entities currently seen by the robot: cup_1 (Cup)"
}
```

**Default query group**

```text
myself sees ?entity && ?entity rdf:type ?type
```

**Policy**

```text
knowledge_snapshot is not a native KnowledgeCore object.
It is chatbot_llm's bounded LLM-facing view of KB facts.
```

---

## 7. Scene Summary

**Topic:** `/scene/summary`  
**ROS type:** `std_msgs/msg/String`  
**Publisher:** `nao_scene_grounding`  
**Consumer:** `chatbot_llm`, operator/debug tools, future WME consumers.

```json
{
  "observer": "myself",
  "backend": "emorobcare_cv",
  "objects": [
    {
      "entity_id": "detected_cup_320_240",
      "label": "cup",
      "kb_class": "Cup",
      "score": 0.91,
      "tracker_id": "",
      "source": "emorobcare_cv",
      "center_x": 320.0,
      "center_y": 240.0,
      "last_seen_sec": 1777040000.0
    }
  ]
}
```

**Policy**

```text
/scene/summary is detector-grounded observation metadata.
knowledge_snapshot is prompt-ready KB query output.
They are related but not the same contract.
```

---

## 8. World Model Context

**Planned topics**

```text
/world_model/enriched_snapshot
/world_model/enriched_text
```

**Purpose**

```text
Future WME input for planner_llm.
Not required for the current Monday planner-loop demo.
```

**Where it lands**

```json
{
  "grounded_context": {
    "world_model_snapshot": {},
    "world_model_text": ""
  }
}
```

---

## 9. Demo-Friendly Contract Flow

```text
User speech
  -> dialogue_manager
  -> chatbot_llm JSON route decision
  -> /planner/request
  -> planner_llm plan envelope
  -> /intents
  -> nao_orchestrator validation + execution
  -> /planner/execution_feedback
  -> planner_llm supervisor
  -> /planner/dialogue_act if user-facing speech is needed
```

**One-line explanation**

```text
The demo proves a closed-loop planner architecture: intent routing, structured planning, deterministic execution, feedback, and dialogue-aware supervision.
```
