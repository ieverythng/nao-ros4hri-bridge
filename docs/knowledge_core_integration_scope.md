# KnowledgeCore Integration Scope

This document records exactly what needs to be present in the workspace for the
ROS4HRI migration to use the upstream symbolic knowledge stack, and which parts
of the current integration are upstream versus local glue.

## Official Runtime Packages

The supported runtime path for this workspace is the official SocialMinds Jazzy
package set, not ad hoc source overlays in `src/`.

For the migrated NAO stack, the required official packages are:

- `socialminds-ros-jazzy-kb-msgs`
- `socialminds-ros-jazzy-knowledge-core`
- `socialminds-ros-jazzy-oro`
- `socialminds-ros-jazzy-interaction-sim`
- `socialminds-ros-jazzy-expressive-face`
- `socialminds-ros-jazzy-hri-face-detect-yunet`
- `socialminds-ros-jazzy-hri-person-manager`
- `socialminds-ros-jazzy-hri-emotion-models`
- `socialminds-ros-jazzy-hri-emotion-recognizer`
- `socialminds-ros-jazzy-hri-visualization`
- `socialminds-ros-jazzy-rqt-chat`
- `socialminds-ros-jazzy-rqt-human-radar`
- `socialminds-ros-jazzy-ui-server`
- `ros-jazzy-gscam`
- `ros-jazzy-image-transport-plugins`
- `ros-jazzy-rosbridge-server`
- `ros-jazzy-rqt-image-view`
- `ros-jazzy-rqt-reconfigure`

The active packages that remain in `src/` for the migration + KB path are:

- `src/chatbot_llm`
- `src/dialogue_manager`

Canonical clone sources for the forked migration packages:

- `https://github.com/ieverythng/nao_chatbot_llm.git`
  branch: `feat/juan_nao_chatbot`
- `https://github.com/ieverythng/dialogue_manager.git`
  branch: `juan-feat-1`

Canonical source repositories for the official knowledge/simulator packages:

- `https://github.com/pal-robotics/kb_msgs.git`
- `https://gitlab.iiia.csic.es/socialminds/neurosymbolic-ai/knowledge_core.git`
- `https://gitlab.iiia.csic.es/socialminds/ros4hri/interaction_sim.git`
- `https://github.com/severin-lemaignan/openrobots-ontology.git`

The tracked helper for source inspection is:

- `./scripts/bootstrap_socialminds_sources.sh`

That helper clones into `ref_src/knowledge_sources/` for reference only. Those
repos are not part of the active build graph unless we explicitly decide to do
source-level upstream work.

These packages should stay upstream and unmodified unless a specific bug forces
an explicit local patch.

The forked overlays are part of the migration itself:

- `chatbot_llm` is the NAO-side backend where the read-only KB-to-LLM adapter
  lives
- `dialogue_manager` is the canonical ROS4HRI dialogue runtime used both by the
  migrated NAO stack and by the official simulator stack

## Why `kb_msgs` Is Required

`kb_msgs` is part of the official ROS interface for `knowledge_core`.

In the upstream `knowledge_core` repo:

- `package.xml` declares `kb_msgs` as an `exec_depend`
- `knowledge_core_ros.py` imports ROS services and messages from `kb_msgs`
- the upstream README says `kb_msgs` is required when using the ROS interface

`oro` is also required in practice for the current packaged
`knowledge_core.launch.py`, because its default configuration loads
`ontology://oro/oro.owl`.

`reasonable` is separate and optional. It provides OWL RL reasoning support,
but it is not the ROS transport layer.

## Upstream Behavior

What stays upstream in the current test path:

- `kb_msgs` defines the ROS message/service types
- `knowledge_core` stores symbolic facts, performs optional reasoning, and
  exposes `/kb/query`, `/kb/revise`, `/kb/events`, and related APIs
- `interaction_sim` provides the upstream simulator launch and perspective files
- `oro` provides the ontology resource loaded by `knowledge_core`

No source changes are currently required inside the official knowledge stack.
For the current Docker-based smoke tests, `knowledge_core`, `oro`,
`interaction_sim`, and the simulator-side HRI/UI packages all come from the
official SocialMinds Jazzy apt feed. The only local code in this path remains
the `chatbot_llm` and `dialogue_manager` fork overlays plus the `nao_chatbot`
launch composition around them.

The current Docker image also installs `pyasyncore` and `pyasynchat` as
runtime compatibility dependencies for the official `knowledge_core` package on
Python 3.12.

The migrated stack also assumes these fork overlays are present:

- `src/chatbot_llm`
- `src/dialogue_manager`

## Local Changes In The Migration Workspace

The current local integration changes are limited to the existing forked
packages and NAO launch surface.

### `chatbot_llm`

Local change:

- add a read-only `/kb/query` client
- fetch KB state once per response turn
- convert the structured `/kb/query` result into prompt text for the LLM

This is the only vital local seam in the current test path. It exists because
our `chatbot_llm` backend is the NAO-side LLM implementation and upstream
`knowledge_core` does not natively know how to feed an LLM prompt.

### `dialogue_manager`

Current test-path change:

- no custom default role configuration is required

For the current test path, `dialogue_manager` stays on its upstream-style
default chat flow and does not inject a custom KB role block.

### `nao_chatbot`

Local changes:

- add launch support for starting `knowledge_core` in the migrated NAO stack
- document the official simulator-based smoke-test path
- compose the official simulator perception/UI nodes without duplicating the
  migrated `chatbot_llm`, `dialogue_manager`, or `knowledge_core` nodes

## What The Previous `knowledge_snapshot` Seam Was

`knowledge_snapshot` is a local `chatbot_llm` concept, not an upstream
`knowledge_core` concept.

It was a role-level JSON block used to tell `chatbot_llm` how to query
`/kb/query` before a turn:

- `enabled`
- `patterns`
- `vars`
- `models`
- `max_results`
- `max_chars`

That mechanism still exists in the local `chatbot_llm` code, but it is not the
default integration path anymore.

## Current Default Test Path

The current default test path is deliberately narrower:

- `interaction_sim` is available as the upstream simulator package
- the simulator perception/UI path is launched from `nao_chatbot` while still
  using the official `interaction_sim` perspective and package set
- `chatbot_llm` has KB querying enabled through node parameters
- `dialogue_manager` uses its normal default chat behavior
- `knowledge_core` remains the only symbolic store

This keeps the test closer to upstream behavior while preserving the minimum
LLM-side glue needed for the NAO chatbot.

## Current Verified State

The current branch has been validated on the official simulator-side path with:

- `start_interaction_sim:=true`
- `gscam` publishing laptop webcam frames to `/camera/image_raw`
- `hri_face_detect_yunet`, `hri_person_manager`, `hri_emotion_recognizer`,
  `hri_visualization`, `rqt_human_radar`, and `rqt_chat` running through the
  official `interaction_sim` perspective
- `knowledge_core` exposing `/kb/query`
- `chatbot_llm` querying `/kb/query` once per response turn through the local
  read-only `knowledge_snapshot` seam

What has already been observed working:

- the `interaction_sim` RQT perspective shows live camera input plus face
  detection
- the person appears in `rqt_human_radar`
- `chatbot_llm` logs show that the KB snapshot path is active during live turns
- the chatbot, `dialogue_manager`, and `nao_say_skill` stay on the migrated
  stack while the simulator-side perception/UI layer remains upstream

This means the symbolic path is alive, but it does not yet mean the robot can
answer grounded visual questions reliably.

## Current Limits

The current behavior gap is not that `knowledge_core` is absent. The main
issue is that the LLM-side grounding is still too generic.

At the moment:

- `chatbot_llm` queries `/kb/query` using the default triple pattern
  `?s ?p ?o`
- the result is formatted as a flat text block and injected into the prompt as
  `Knowledge base snapshot:`
- the prompt does not yet explicitly tell the LLM that this snapshot is live,
  authoritative scene state
- the query is not specialized for:
  - visible persons
  - detected objects
  - current user identity / reference resolution
  - spatial relations that matter for robot behavior

So the current integration is good enough to prove that KB data reaches the
LLM, but not yet good enough to guarantee sensible answers to questions such
as:

- "Can you see me?"
- "Who is in front of you?"
- "What is in the knowledge base right now?"
- "What objects are near you?"

There is also a perception-side limitation:

- the current official simulator path gives us face/person/emotion grounding
  from the laptop webcam
- object extraction is not yet formalized in the chatbot prompt or query shape
- the face detector can fall behind on CPU-only laptop runs, which was already
  visible in the warning `Face_detect's processing too slow`

## Why The KB Answers Still Feel Weak

The current stack is behaving as implemented:

1. `knowledge_core` stores symbolic facts and exposes them over `/kb/query`
2. `chatbot_llm` fetches a snapshot
3. the snapshot is appended to the prompt
4. the LLM decides how much to use it

That is enough for "KB is connected", but not enough for robust grounded
dialogue.

The next improvement needs to happen in `chatbot_llm`, not in
`dialogue_manager`:

- add prompt language that explicitly says the robot is ingesting live symbolic
  scene state
- state that the snapshot should be treated as the robot's current grounded
  world model when answering perception/scene questions
- query the KB in a more structured way than raw `?s ?p ?o`
- introduce role or prompt-pack text that explains how to talk about people,
  objects, and spatial relations in a user-facing way

## Prompt / KB Work Still Needed

The next LLM-facing improvements should be treated as first-class migration
work:

1. Formalize live KB grounding in prompts

- update the response-stage prompt so it explicitly says the knowledge snapshot
  is live symbolic data from the current scene
- update the intent-stage prompt only if needed for grounded action selection
- ensure the model prefers KB-backed answers over generic fallback statements

2. Make the snapshot more understandable

- stop relying only on the default `?s ?p ?o` dump
- add narrower default patterns for people / detected entities / spatial
  relations when those are available
- format KB rows into readable phrases rather than raw triples when possible
- decide how the active speaker should map to detected persons

3. Add person/object scene understanding

- keep face/person/emotion from the official simulator path
- extend the KB-facing extraction so the prompt can explain "who is visible"
  and "what is near the robot" in plain language
- add object-grounding once the official detection side used for the demo is
  chosen

The main local files for this next phase are:

- `src/chatbot_llm/config/00-defaults.yml`
- `src/chatbot_llm/chatbot_llm/backend_config.py`
- `src/chatbot_llm/chatbot_llm/knowledge_snapshot.py`
- `src/chatbot_llm/chatbot_llm/knowledge_snapshot_client.py`
- `src/chatbot_llm/chatbot_llm/prompt_builders.py`
- `src/chatbot_llm/config/chat_prompt_pack.yaml`

## Robot Camera And RViz2 Path

The current simulator path is webcam-based. For the demo, we also need the real
robot camera/TF path.

What is still needed:

- switch the perception stack from the laptop webcam topic to the NAO camera
  topic
- visualize the robot model, TF tree, and camera feed in `rviz2`
- confirm the robot-side frames and distances are coherent before relying on
  scene grounding for the demo

The current local launch uses:

- `/camera/image_raw`
- `/camera/camera_info`
- frame `camera`
- static transforms from `sellion_link -> camera` and `base_link -> sellion_link`

That is acceptable for laptop simulator tests, but not the final robot demo
path.

Per Severin's note, the intended official robot-side path is to use the
`nao_robot` package from the SocialMinds apt repository:

- `sudo apt install socialminds-ros-jazzy-nao-robot`
- `ros2 launch nao_robot nao_robot.launch.py nao_ip:=x.x.x.x`

That path should become the reference for the real-robot demo because it gives
us a cleaner RViz / TF / robot-camera bring-up.

For the next pass, the launch surface should support both modes explicitly:

- simulator webcam mode
- real robot camera / TF / RViz mode

Recommended launch-level additions for that phase:

- a parameter or launch arg for image topic
- a parameter or launch arg for camera info topic
- a parameter or launch arg for camera frame
- a `start_rviz` flag
- an RViz config preloaded for robot camera + TF + HRI overlays

The expected robot-facing topics will likely be namespaced under `nao_robot`,
for example:

- `/nao_robot/camera/front/image_raw`
- `/nao_robot/camera/front/camera_info`

Those exact topics should be confirmed against the actual `nao_robot` runtime
when we switch the launch.

## Interaction Sim Vs Real Robot

The intended split is:

- `interaction_sim` for home testing and KB/UI iteration
- `nao_robot` + RViz for robot-camera validation and demo readiness

This means:

- we should keep the official simulator perspective for quick webcam tests
- we should not treat the simulator static transforms as the final robot TF
  solution
- real distance reasoning for the demo should be validated on the robot-side
  TF tree and camera topics

## Cursor Handoff

The next implementation pass should focus on these concrete outcomes:

1. Keep the current simulator path working

- laptop webcam -> `gscam`
- face/person/emotion -> `knowledge_core`
- `rqt` perspective + browser KB viewer for rapid iteration

2. Improve KB-grounded answers in `chatbot_llm`

- formalize prompt instructions for live symbolic context
- improve query patterns / formatting for people and objects
- ensure questions about current perception use the KB snapshot instead of
  generic LLM priors

3. Add the robot-camera demo path

- bring up `nao_robot`
- remap the perception stack to the robot camera topics
- add `rviz2` launch support
- verify TF and camera geometry before relying on scene-grounded answers

4. Keep ownership boundaries intact

- `knowledge_core`, `interaction_sim`, and `nao_robot` remain upstream/official
  packages unless a real upstream bug forces a fork
- prompt/query shaping belongs in the local `chatbot_llm` fork
- dialogue ownership stays in `dialogue_manager`
- robot speech ownership stays in `nao_say_skill`

## Expected Smoke Test

After the official packages are installed and the workspace is built, the
intended local test is:

1. launch `knowledge_core` and confirm `/kb/query` is available
2. optionally launch the official simulator perception/UI layer with
   `start_interaction_sim:=true`
3. confirm the browser KB view is available from `knowledge_core`
4. add or move objects/persons through `rqt_human_radar` or `/kb/revise`
5. verify the symbolic state through `/kb/query`
6. ask the chatbot about those objects/persons through `rqt_chat`
7. for the robot-demo phase, switch to the robot camera / TF / RViz path and
   verify the same symbolic updates there

If that works, the migration is complete enough for the first integration test.
