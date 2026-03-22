# NAO ROS4HRI Bridge

ROS 2 Jazzy workspace for the NAO ROS4HRI migration. The active tree mixes
local robot-side runtime packages, fork-tracked upstream dialogue packages,
local interface packages, and optional external KnowledgeCore/simulator
dependencies.

## Workspace Layout

Robot-side runtime packages in this repo:

- `nao_chatbot`: launch surfaces and operator utilities
- `nao_orchestrator`: downstream `/intents` consumer and NAO skill dispatcher
- `kb_skills`: dedicated KnowledgeCore client boundary and KB skill metadata
- `nao_say_skill`: NAO-specific `/nao/say` execution bridge
- `nao_replay_motion`: replay-motion, posture compatibility, and head motion
- `nao_look_at`: scaffolded `/skill/look_at` implementation
- `asr_vosk`: local lifecycle ASR node
- `simple_audio_capture`: local microphone source for the ASR path

Fork-tracked upstream runtime repos carried locally:

- `dialogue_manager`: canonical owner of `/skill/chat`, `/skill/ask`, and
  `/skill/say`
- `chatbot_llm`: backend dialogue contract using the local Ollama-based
  response and intent pipeline

Interface-only packages shipped in the workspace:

- `communication_skills`
- `nao_skills`
- `motions_skills`
- `std_skills`

Optional external integrations used for KB/simulator testing:

- `knowledge_core`
- `kb_msgs`
- `interaction_sim`
- `oro`

Those KnowledgeCore and simulator packages are not vendored into the active
`src/` tree here. Runtime normally uses the official SocialMinds Jazzy
packages, while `./scripts/bootstrap_socialminds_sources.sh` clones
reference-only copies into `ref_src/knowledge_sources/`.

## Dialogue And KB Grounding

Current migrated flow:

```text
speech input -> dialogue_manager -> chatbot_llm
    -> /intents -> nao_orchestrator
    -> /nao/say | /skill/replay_motion | /skill/do_head_motion | /skill/look_at
```

This split is deliberate:

- `dialogue_manager` owns dialogue state and canonical communication skills
- `chatbot_llm` owns model interaction and prompt construction
- `nao_orchestrator` stays downstream-only and dispatches robot-side intents
- `knowledge_core` remains an upstream symbolic store accessed through public
  ROS APIs

Knowledge grounding is local to `chatbot_llm`, not to `knowledge_core`
itself:

- `knowledge_core` exposes `/kb/query` through `kb_msgs/srv/Query`
- the local `kb_skills` package is the dedicated ROS-facing client boundary for
  KnowledgeCore interactions
- `chatbot_llm` uses `kb_skills` to call `/kb/query` once per response turn
- the returned JSON bindings are formatted into a bounded text snapshot
- that snapshot is appended to the response and intent prompts as grounded
  scene context

The default query shipped by `chatbot_llm` is visibility-focused:

- `myself sees ?entity && ?entity rdf:type ?type`

If a dialogue role passes a JSON `knowledge_snapshot` block,
`chatbot_llm` merges those role-level overrides with the node defaults for
patterns, variables, models, and output limits.

## Launch Profiles

Primary launch files live in `src/nao_chatbot/launch/`:

- `nao_chatbot_ros4hri_migration.launch.py`: primary migrated runtime
- `nao_chatbot_ros4hri_with_asr.launch.py`: migrated runtime plus local ASR
- `nao_chatbot_asr_only.launch.py`: isolated local ASR pipeline

Useful launch combinations:

Primary migrated stack:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py
```

Real robot TF/camera validation:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_nao_robot:=true \
  start_rviz:=true \
  nao_ip:=172.26.112.62
```

Real robot camera plus simulator-side tools only:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_nao_robot:=true \
  start_rviz:=true \
  start_interaction_sim:=true \
  start_interaction_sim_perception:=false \
  start_interaction_sim_tools:=true \
  nao_ip:=172.26.112.62
```

Quick reference:

- [docs/launch_profiles.md](docs/launch_profiles.md)
- [docs/current_workflow.md](docs/current_workflow.md)
- [docs/node_interactions_map.md](docs/node_interactions_map.md)
- [docs/ollama_chatbot_architecture.md](docs/ollama_chatbot_architecture.md)
- [docs/knowledge_core_integration_scope.md](docs/knowledge_core_integration_scope.md)
- [docs/asr_vosk_setup.md](docs/asr_vosk_setup.md)
- [docs/nao_camera_vlm_research.md](docs/nao_camera_vlm_research.md)

## Build And Test

Build the local packages shipped in this repo:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  std_skills communication_skills motions_skills kb_skills nao_skills \
  chatbot_llm dialogue_manager nao_orchestrator nao_say_skill \
  nao_replay_motion nao_look_at nao_chatbot asr_vosk simple_audio_capture
```

Run the local validation suite:

```bash
./scripts/run_tests.sh
./.venv/bin/pre-commit run --all-files
```

## KnowledgeCore References

Bootstrap the upstream reference clones if you need to inspect the official
sources locally:

```bash
./scripts/bootstrap_socialminds_sources.sh
```

Those clones are reference-only. The current supported runtime path for
KnowledgeCore and `interaction_sim` is the official SocialMinds Jazzy package
feed, not ad hoc local overlays under `src/`.

## Notes

- `src/dialogue_manager/` and `src/chatbot_llm/` are nested git repos; review
  and PR them in their own histories, not through the monorepo diff.
- `knowledge_core` does not natively inject prompt text into the LLM. The
  read-only KB-to-prompt seam lives in the local `chatbot_llm` fork.
- old `mission_controller`, `ollama_chatbot`, and `nao_skill_servers`
  runtime surfaces have been removed from the active workspace.
- `naoqi_driver` already exposes camera topics and joint interfaces, so future
  vision work should build on those ROS interfaces instead of adding a parallel
  capture stack.
