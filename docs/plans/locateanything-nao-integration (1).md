# LocateAnything-3B → NAO ROS2 HRI Bridge: Integration Blueprint

> **Status:** Architecture analysis — ready for implementation planning
> **Model:** NVIDIA LocateAnything-3B via `locate-anything.cpp` (mudler/LocalAI)
> **Quantization:** q8_0 (6.3 GB, box-identical to f32) / q6_k (5.5 GB, box-identical)
> **Target hardware:** RTX 5070 Ti (16 GB VRAM), Ryzen 7 5800X
> **Models stored:** `/mnt/c/Users/Admin/PROJECTS/Models/`

---

## Table of Contents

1. [What LocateAnything-3B Is](#1-what-locateanything-3b-is)
2. [Current NAO Architecture Overview](#2-current-nao-architecture-overview)
3. [Where LocateAnything Fits](#3-where-locateanything-fits)
4. [Integration Architecture](#4-integration-architecture)
5. [Message & Service Contracts](#5-message--service-contracts)
6. [End-to-End Flow: "Bring Me the Coffee Mug"](#6-end-to-end-flow-bring-me-the-coffee-mug)
7. [Friction Points & Risks](#7-friction-points--risks)
8. [Implementation Phases](#8-implementation-phases)
9. [Key Design Decisions](#9-key-design-decisions)

---

## 1. What LocateAnything-3B Is

NVIDIA's open-vocabulary object detection VLM, ported to pure C++/ggml by the LocalAI team.

| Component | Detail |
|-----------|--------|
| Language model | Qwen2.5-3B |
| Vision encoder | MoonViT |
| Projector | 2-layer MLP |
| Detection mechanism | Token-space — emits coordinate tokens `<0>`..`<1000>` that decode to bounding boxes |
| Decoding | Greedy only (sampling degrades boxes) |
| Input | Single image + open-vocabulary text prompt |
| Output | JSON array of `{label, box: [x1,y1,x2,y2]}` detections |

### CLI Interface

```bash
locate-anything-cli detect --model models/locate-anything-q8_0.gguf \
    --input street.jpg \
    --prompt "Locate all the instances that matches the following description: person</c>car" \
    --annotated out.png
# Returns JSON: {"detections":[{"label":"person","box":[...]}, ...]}
```

### Performance Profile

| Mode | CPU (Ryzen 9 9950X) | GPU (GB10) vs PyTorch bf16 |
|------|---------------------|----------------------------|
| slow (pure AR) | 14.26s | — |
| hybrid (default) | 22.32s | ~1.7-2.1× faster |
| fast (MTP-only) | 19.45s | — |
| **q8_0 slow** | **4.89s** | **~4.8× over f32 PyTorch** |

On your RTX 5070 Ti with CUDA build, expect sub-second inference for q8_0.

### Available GGUF Quantizations

| File | Size | Fidelity | Notes |
|------|------|----------|-------|
| `locate-anything-q8_0.gguf` | 6.3 GB | **Box-identical to f32** | ✅ Recommended, downloaded |
| `locate-anything-q6_k.gguf` | 5.5 GB | **Box-identical to f32** | ✅ Downloaded |
| `locate-anything-f16.gguf` | 9.2 GB | Full precision LM | Fits in 16GB VRAM |
| `locate-anything-q5_k.gguf` | 5.1 GB | Sub-pixel drift | Acceptable for most use |
| `locate-anything-q4_k.gguf` | 4.7 GB | Sub-pixel drift | Smallest, least precise |

---

## 2. Current NAO Architecture Overview

### Package Map

```
src/
├── asr_vosk/                  # Speech-to-text (Vosk)
├── communication_skills/      # Social gesture skills
├── fake_skills/               # Simulated skill servers (find_object, navigate_to, etc.)
├── interaction_trace_viewer/  # Debug/trace visualization
├── kb_skills/                 # KnowledgeCore query + mutation clients
├── nao_chatbot/              # Stack launch, intent routing
├── nao_look_at/              # Head gaze control
├── nao_orchestrator/         # Plan executor (deterministic)
├── nao_replay_motion/        # Motion playback
├── nao_say_skill/            # Text-to-speech
├── nao_scene_grounding/      # Detector normalization + symbolic grounding
├── nao_skills/               # Real skill action servers (ScanScene, etc.)
├── planner_common/           # Shared contracts, skill registry bridge
├── planner_llm/              # LLM-based task planner
└── simple_audio_capture/     # Audio capture node
```

### Current Perception Pipeline

```
Camera → [detector backend] → /detected_objects or /yolo/tracking
  → nao_scene_grounding (normalize + identity matching)
    → /scene/summary (JSON String, lossy: centers only, no bbox corners)
    → /kb/revise (transient KB facts: sees, rdf:type)
      → nao_orchestrator scan_skill_server (consumes summary for scan results)
```

### Current Limitations

1. **`/scene/summary` is lossy** — only keeps label, confidence, image-plane center. Bounding box corners are discarded.
2. **No real `find_object` skill** — current implementation is fake/simulated.
3. **No 2D→3D bridge** — detections stay in image coordinates; no actionable geometry for navigation or manipulation.
4. **Continuous detection model** — current backends (YOLO, emorobcare) run always-on class-agnostic detection. Not suited for open-vocabulary on-demand queries.
5. **No world-model publisher identified** — `planner_llm` subscribes to `/world_model/enriched_snapshot` and `/enriched_text`, but no publisher exists in the reviewed packages.

---

## 3. Where LocateAnything Fits

### The Right Place: Detector Backend, Not Core Logic

LocateAnything integrates cleanly as **a new detector backend** feeding into `nao_scene_grounding`, NOT embedded inside the orchestrator or planner.

```
Camera image → [LocateAnything detector node] → normalized detections
  → nao_scene_grounding (existing normalization + identity logic)
    → /scene/summary + /kb/revise
      → orchestrator scan skill / planner context
```

### Why This Architecture

- `nao_scene_grounding` already handles detector-agnostic normalization, identity matching, and KB writes
- The orchestrator is deliberately deterministic — it executes plans, doesn't run inference
- The planner consumes world-model context, not raw detections
- Adding LA as a backend preserves all existing contracts while adding open-vocabulary capability

### Operating Model: On-Demand, Not Always-On

LocateAnything is **much more expensive** than YOLO-class detectors (~5s CPU, sub-second GPU). Use it for:

- ✅ **On-demand target grounding** — "find the red cup", "locate the person"
- ✅ **Low-rate scene refresh** — periodic broad scans during planning
- ❌ **High-frequency continuous detection** — use lightweight detectors (YOLO) for ambient awareness

---

## 4. Integration Architecture

### Phase 1: CLI Wrapper Node (POC)

```
┌─────────────────────────────────────────────────────────────┐
│  nao_locate_anything (new package)                          │
│                                                              │
│  locate_anything_node.py                                     │
│  ├── Subscribes to: /camera/image (sensor_msgs/Image)       │
│  ├── Service: /perception/ground_text                       │
│  │   Request: image + query_text + candidate_labels          │
│  │   Response: normalized detections JSON                    │
│  ├── Spawns: locate-anything-cli detect --model ...         │
│  └── Publishes: /scene/grounding_result (std_msgs/String)   │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│  nao_scene_grounding (existing, extended)                    │
│                                                              │
│  ├── Adds: locate_anything backend adapter                  │
│  ├── Normalizes detections → ObjectObservation              │
│  ├── Identity matching (center-distance fallback)           │
│  └── Publishes: /scene/summary + /kb/revise                  │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│  nao_orchestrator (existing)                                │
│                                                              │
│  ├── scan_skill_server: consumes /scene/summary             │
│  ├── find_object skill: calls /perception/ground_text       │
│  └── Executes plan steps, chains result payloads            │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│  planner_llm (existing)                                     │
│                                                              │
│  ├── Receives: /planner/request with scene_targets          │
│  ├── Context: grounded_context.visual_grounding             │
│  ├── Emits: multi-step plan (find_object → look_at → ...)  │
│  └── Replans on execution feedback                          │
└─────────────────────────────────────────────────────────────┘
```

### Phase 2: C++ ROS2 Node (Production)

Replace CLI subprocess with a dedicated C++ node using `la_capi.h` or shared library (`-DLA_SHARED=ON`). Benefits:
- No per-call process spawn overhead
- Direct GPU memory management
- Proper threading and cancellation
- Real-time performance suitable for interactive robot operation

---

## 5. Message & Service Contracts

### New Service: `/perception/ground_text`

**Request:**
```yaml
string request_id              # unique grounding request ID
sensor_msgs/Image image       # the image to analyze (or URI)
sensor_msgs/CameraInfo camera_info  # intrinsics for 3D projection
string query_text             # e.g. "coffee mug" or "red cup on table"
string[] candidate_labels     # optional: ["mug", "cup"]
string[] attributes           # optional: ["red"]
string decode_mode            # fast | hybrid | slow (default: hybrid)
bool project_to_3d            # if depth/TF available
```

**Response:**
```yaml
bool success
string message
string backend                # "locateanything-3b"
string model_name             # "LocateAnything-3B-q8_0"
string image_frame
builtin_interfaces/Time stamp
string result_json            # normalized detections (see below)
```

**Normalized Detection JSON:**
```json
{
  "request_id": "ground_123",
  "query_text": "coffee mug",
  "backend": "locateanything-3b",
  "model": "LocateAnything-3B-q8_0",
  "image_frame": "camera_color_optical_frame",
  "timestamp_sec": 1712345678.25,
  "detections": [
    {
      "detection_id": "la_det_1",
      "label": "mug",
      "canonical_label": "cup",
      "score": 0.86,
      "bbox": {"x1": 412, "y1": 201, "x2": 566, "y2": 411},
      "center_px": {"x": 489.0, "y": 306.0},
      "attributes": {"color": "red"},
      "source": "locateanything-3b",
      "pose_hint": {
        "frame_id": "camera_color_optical_frame",
        "x": 0.0, "y": 0.0, "z": 0.0,
        "valid": false
      }
    }
  ]
}
```

### New Topic: `/scene/grounding_result`

Query-specific grounding events (separate from global `/scene/summary`):

```json
{
  "request_id": "ground_123",
  "goal_id": "goal_77",
  "query_text": "bring me the coffee mug",
  "target_text": "coffee mug",
  "backend": "locateanything-3b",
  "status": "succeeded",
  "best_match": {
    "entity_id": "detected_mug_la_det_1",
    "label": "mug",
    "kb_class": "Cup",
    "score": 0.86,
    "bbox": {"x1": 412, "y1": 201, "x2": 566, "y2": 411},
    "pose_hint": {"frame_id": "map", "x": 1.2, "y": -0.4, "z": 0.85, "valid": true}
  },
  "candidates": [...],
  "summary_text": "I found a coffee mug on the table.",
  "actionability": {
    "look_at_ready": true,
    "navigation_ready": false,
    "grasp_ready": false
  }
}
```

### Extended `/scene/summary` (Backward Compatible)

Add optional fields to existing object entries:

```json
{
  "entity_id": "detected_mug_la_det_1",
  "label": "mug",
  "kb_class": "Cup",
  "score": 0.86,
  "tracker_id": "",
  "source": "locateanything-3b",
  "center_x": 489.0,
  "center_y": 306.0,
  "last_seen_sec": 1712345678.25,
  "bbox": {"x1": 412, "y1": 201, "x2": 566, "y2": 411},
  "attributes": {"color": "red"},
  "grounding_query": "coffee mug",
  "pose_hint": {
    "frame_id": "camera_color_optical_frame",
    "x": 0.31, "y": -0.08, "z": 0.94,
    "valid": true
  }
}
```

### Extended KB Facts

Current: `myself sees detected_cup_X`, `detected_cup_X rdf:type Cup`

Add transient grounding facts:
- `detected_mug_la_det_1 hasLabel mug`
- `detected_mug_la_det_1 hasColor red`
- `detected_mug_la_det_1 groundedBy LocateAnything3B`
- `detected_mug_la_det_1 hasImageBBox "412,201,566,411"`
- `detected_mug_la_det_1 inFrame camera_color_optical_frame`

### Extended Planner `grounded_context`

```json
{
  "knowledge_snapshot": {...},
  "scene_summary": {...},
  "world_model_snapshot": {...},
  "world_model_text": "...",
  "visual_grounding": {
    "query_text": "coffee mug",
    "backend": "locateanything-3b",
    "status": "succeeded",
    "best_match": {...},
    "candidates": [...],
    "needs_disambiguation": false
  }
}
```

---

## 6. End-to-End Flow: "Bring Me the Coffee Mug"

### Phase 1: ASR → Request Formation

```
User speaks → asr_vosk → "bring me the coffee mug"
  → chatbot forms PlannerRequest:
    goal_text: "bring me the coffee mug"
    normalized_intents: ["bring_object"]
    scene_targets: ["coffee mug", "mug", "cup"]
```

### Phase 2: LLM Planning

```
planner_llm receives request + world model context
  → emits plan:
    Step 1: find_object(target="coffee mug", candidate_labels=["mug","cup"])
    Step 2: look_at(policy="track")  [resolves from Step 1 result]
    Step 3: navigate_to(target="coffee mug")  [if pose_hint valid]
    Step 4: report_result
```

### Phase 3: Perception / Grounding

```
Orchestrator executes find_object:
  → captures camera frame
  → calls /perception/ground_text with prompt "coffee mug</c>mug</c>cup"
  → LocateAnything returns detections with bboxes
  → filters/ranks, publishes /scene/grounding_result
  → updates /scene/summary + KB facts
  → returns action result with best_match entity
```

### Phase 4: Skill Chaining

```
Orchestrator stores last_result_payload
  → Step 2 (look_at) resolves target frame from Step 1's pose_hint
  → Step 3 (navigate_to) gated on actionability.navigation_ready
  → If not ready: replan or ask_user clarification
```

### Phase 5: Completion / Replanning

```
report_result speaks grounded summary
PlannerSupervisor uses execution feedback + world model for replanning
On ambiguity: ask_clarification ("I found two mugs — which one?")
On not_found: retry scan → find_object, or explain failure
```

---

## 7. Friction Points & Risks

### 7.1 Performance & Execution Model

- LA is ~5s on CPU per image; sub-second on GPU with CUDA build
- Running synchronously in a Python callback blocks the executor
- **Solution:** Worker thread/process, bounded input queue, stale-frame dropping, "latest only" behavior

### 7.2 Continuous vs On-Demand Model Mismatch

- Current `nao_scene_grounding` assumes continuous detector output
- LA is prompt-driven and single-image
- **Solution:** Use LA as on-demand grounding skill, not always-on detector. Keep lightweight detectors (YOLO) for ambient awareness.

### 7.3 Prompt Engineering & Label Normalization

- LA prompts can be rich ("red coffee mug on the left")
- Current stack expects stable labels ("cup", "bottle", "person")
- **Solution:** Canonical label aliases before identity matching; normalize prompt outputs back to stack vocabulary

### 7.4 Identity Stability

- LA has no tracker IDs — current fallback uses center-distance-only matching
- Prompt-driven detections cause box jitter and label synonym churn
- **Solution:** Switch to IoU + center distance matching; canonicalize labels before matching

### 7.5 Image Pipeline Gap

- `nao_scene_grounding` currently has no `sensor_msgs`/`cv_bridge` dependency
- **Solution:** Put image subscription in the new detector node, not in scene_grounding

### 7.6 Lossy Downstream Contract

- `/scene/summary` discards bbox corners — only keeps centers
- Future "look at detected object" or "point to object" needs full boxes
- **Solution:** Extend summary with optional `bbox` field (backward compatible)

### 7.7 Scan-Skill Timing

- `scan_skill_server` performs head sweeps then reads cached scene summary
- If LA inference is slow, head sweep positions may complete before evidence arrives
- **Solution:** Explicit sync: move head → capture frame → run LA → collect detections → continue

### 7.8 No 2D→3D Bridge

- Current stack has no built-in transformer from image bbox to 3D pose
- Navigation and manipulation require actionable geometry
- **Solution:** Future `resolve_target_pose` skill using depth camera + TF tree

---

## 8. Implementation Phases

### Phase A: Low-Risk Integration (Weeks 1-2)

1. Build `locate-anything.cpp` with `-DLA_GGML_CUDA=ON` on Windows host
2. Create `nao_locate_anything` ROS2 package with CLI wrapper node
3. Implement `/perception/ground_text` service
4. Add `locate_anything` backend adapter to `nao_scene_grounding`
5. Return detections in `find_object.result_payload_json` (reuse ScanScene action)
6. Update planner skill registry to prefer `find_object` for object-specific requests

**Outcome:** Planner becomes open-vocabulary object-aware without changing core orchestrator.

### Phase B: World-Model Visibility (Weeks 3-4)

7. Publish grounding results to `/scene/grounding_result`
8. Extend `/scene/summary` with optional bbox/attributes/pose fields
9. Revise KB with richer transient facts (label, color, bbox, frame)
10. Feed grounding evidence into `/world_model/enriched_snapshot` and `/enriched_text`

**Outcome:** Planner uses recent grounding evidence for replanning and dialogue.

### Phase C: Actionable Execution (Weeks 5-8)

11. Replace CLI subprocess with C++ ROS2 node using LA C API
12. Add 2D→3D projection using depth camera + TF resolution
13. Add stable target frames for grounded entities
14. Enhance orchestrator `look_at` step to resolve target from prior result payload
15. Gate `navigate_to` / manipulation skills on actionability flags

**Outcome:** LocateAnything drives gaze, navigation, and eventually manipulation.

---

## 9. Key Design Decisions

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | **On-demand, not continuous** | LA is expensive; use for named-object queries, lightweight detectors for ambient awareness |
| 2 | **Detector backend, not core logic** | Preserves orchestrator determinism and planner separation of concerns |
| 3 | **Service/action boundaries** | Planner asks for skills; skills call perception services. No direct VLM calls from planner |
| 4 | **Symbolic ≠ geometric actionability** | Bbox match is enough for `find_object` and planner context, NOT for `navigate_to` or manipulation |
| 5 | **Explicit provenance** | Every grounding result carries request/goal linkage, query text, backend/model, timestamp/frame, confidence |
| 6 | **Backward-compatible extensions** | New fields in `/scene/summary` are optional; existing consumers ignore unknown keys |
| 7 | **CLI wrapper → C++ node** | Fast POC with CLI subprocess; production hardening with shared library integration |

---

## Appendix A: Build Commands

```bash
# On Windows host (where CUDA toolkit lives):
git clone --recursive https://github.com/mudler/locate-anything.cpp
cd locate-anything.cpp
cmake -B build -DLA_BUILD_CLI=ON -DLA_GGML_CUDA=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# Models already downloaded:
# C:\Users\Admin\PROJECTS\Models\locate-anything-q8_0.gguf (5.9 GB)
# C:\Users\Admin\PROJECTS\Models\locate-anything-q6_k.gguf (5.2 GB)
```

## Appendix B: Quick Smoke Test

```bash
./build/locate-anything-cli detect \
  --model C:/Users/Admin/PROJECTS/Models/locate-anything-q8_0.gguf \
  --input test_image.jpg \
  --prompt "Locate all the instances that matches the following description: person</c>cup" \
  --annotated output.png
```

## Appendix C: RQT Integration

For debugging and operator tooling:
- `/scene/summary` → existing `interaction_trace_viewer` node
- `/scene/grounding_result` → new rqt plugin showing detection overlays
- `/perception/ground_text` service → callable from rqt_service_caller for manual testing
- Annotated output images (`--annotated`) → publish as `sensor_msgs/Image` for rqt_image_view

---

*This document synthesizes analysis from two parallel deep-dive reviews of the full NAO ROS2 HRI bridge codebase, covering 16 packages and their interconnections.*
