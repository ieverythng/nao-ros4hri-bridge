# v2 Embodied Skills, Speech, and Autonomy Integration

Date: 28 July 2026

Status: roadmap only. None of the capabilities in this document extend the
frozen v1 result unless they pass the acceptance gates below.

## Source-truth baseline

The active v1 source exposes three robot-facing motion surfaces:

| Surface | Current implementation | v1 claim |
| --- | --- | --- |
| `/skill/replay_motion` and `/skill/do_posture` | `nao_replay_motion` uses `ALRobotPosture.goToPosture` for named postures | Robot adapter present |
| `/skill/do_head_motion` | Publishes bounded head yaw and pitch commands | Open-loop command evidence |
| `/skill/look_at` | `nao_look_at` implements the ROS4HRI `LookAt` action over TF and head joints | Real adapter present, fake orchestration remains the default |

There is no active `MoveTo` action, physical `walk_to` implementation, or
qualified `wave` motion in v1. `point_at` is not implemented. These are v2
promotion candidates.

## NAOqi capability map

The NAOqi 2.8 motion API provides joint stiffness, joint interpolation,
locomotion, Cartesian control, task management, and safety reflexes. ALMotion
runs at 50 Hz and creates a motion task for each request. Safety reflexes are
enabled by default and may alter requested movement.

Primary references:

- <https://doc.aldebaran.com/2-8/naoqi/motion/almotion.html>
- <https://doc.aldebaran.com/2-5/naoqi/motion/control-joint.html>
- <https://doc.aldebaran.com/2-8/dev/python/intro_python.html>

The stack should use these APIs only behind ROS action servers. Direct planner
or chatbot calls to NAOqi would bypass cancellation, lineage, and deterministic
admission.

### Candidate API use

| v2 capability | NAOqi primitive | Required ROS boundary |
| --- | --- | --- |
| `wave` | `angleInterpolation` or a verified stored motion | Named, cancellable replay-motion action |
| `look_at` refinement | Head `angleInterpolationWithSpeed` or the existing joint command topic | Existing ROS4HRI `LookAt` action |
| body-relative movement | `moveTo(x, y, theta)` or equivalent locomotion task | New AB=0 `MoveTo` action |
| velocity-limited walking | `moveToward` with bounded configuration | Same `MoveTo` server, never planner-direct |
| `point_at` prototype | Arm and hand joint interpolation, later Cartesian control | New action with target frame and explicit evidence policy |

Collision protection, fall management, stiffness policy, cancellation, posture
preconditions, and operator clearance remain mandatory. No acceptance test may
disable a critical safety reflex.

## Promotion order

### 1. Real `look_at`

Promote the existing adapter before adding a new skill. The physical test set
must cover reset, one static TF target, one tracked person frame, target loss,
cancellation, missing joint subscribers, and repeated goals. Completion must
mean either measured convergence or an explicitly labelled open-loop command,
not successful publication alone.

### 2. Named `wave`

Add one allowlisted `wave` trajectory to `nao_replay_motion`. Keep the motion
independent of a person target. `wave_at(person)` remains a later composition
because it combines target selection, gaze or body orientation, social policy,
and gesture execution.

### 3. Body-relative `MoveTo`

Introduce a local primitive with bounded metres and radians, conservative speed
profiles, posture checks, timeout, cancellation, and explicit task cleanup.
Start with dry-run payload validation. Physical trials should progress through
zero motion, rotation in place, short forward movement, cancellation, obstacle
response, and recovery to a stable posture.

Semantic `navigate_to(location)` remains a higher-level composition. It needs
localization, a frame-qualified destination, obstacle evidence, segment or path
selection, and post-motion verification.

### 4. `point_at`

A camera bounding-box centre is not a spatial target. A prototype may use a
flat image-plane mapping for a clearly labelled demonstration, but the supported
skill requires a target frame or calibrated 3D ray, reachability checks, joint
limits, self-collision protection, hand pose, cancellation, and visible result
evidence.

## Speech service recommendation

Keep speech transport outside the dialogue and planner nodes.

### Preferred main-PC service

Use Speaches behind ZeroTier as the first integration target. It provides an
OpenAI-compatible API, Docker deployment, streaming transcription, GPU or CPU
execution, dynamic model loading, faster-whisper ASR, and Kokoro or Piper TTS:

- <https://github.com/speaches-ai/speaches>
- <https://github.com/SYSTRAN/faster-whisper>
- <https://github.com/OHF-Voice/piper1-gpl>

Recommended initial configuration:

- ASR: faster-whisper `turbo` on CUDA with VAD enabled. Use `distil-large-v3`
  when English-only latency and accuracy measurements justify it.
- TTS: Kokoro for the presentation-quality path, with Piper as the low-resource
  offline reserve.
- CPU-only ASR reserve: `whisper.cpp`, which provides a local server and VAD
  support (<https://github.com/ggml-org/whisper.cpp>).

The existing Vosk profile remains the no-network fallback. Native
`ALSpeechRecognition` is designed around predefined phrases, so it is useful
for small command vocabularies but not as the primary open-domain thesis ASR.

### ROS ownership

1. A remote ASR adapter publishes ROS4HRI speech input with language, timing,
   confidence, endpoint identity, and request correlation.
2. `dialogue_manager` continues to own dialogue lifecycle.
3. A remote TTS adapter implements the existing Say action and supports cancel.
4. Startup preflight reports endpoint health, selected models, and latency.
5. The robot-local Say adapter remains available when remote TTS is unhealthy.

Remote failure must be decided before a turn starts. Do not switch ASR, TTS, or
LLM models halfway through a turn solely because latency is high.

## Autonomous behaviour arbitration

Autonomous animation should be a bounded idle client, not another planner.

```text
operator stop / safety fault
          > admitted user or planner goal
          > explicit robot command
          > allowlisted idle behaviour
```

The arbiter admits idle actions only when no user or planner goal is active. A
new admitted goal cancels the idle action through the same ROS action boundary.
Allowed v2 starter behaviours are gaze reset, breathing or low-amplitude idle
motion, and a posture-safe wave. Walking and target-aware gestures are excluded
until their physical gates pass.

Every autonomous action must carry an origin, action ID, timestamps, priority,
cancellation reason, and result. This keeps live presentation behaviour visible
without creating hidden execution authority.

## Model selection policy

The CLI model is authoritative. At startup, the resolver inventories vLLM and
then Ollama, probes the requested model, and selects the highest-priority healthy
candidate. The selected model is pinned for the launch. A fallback emits a
visible event naming the unavailable preferred model and the replacement.

Timeout and readiness thresholds must be separate. One slow request does not
prove endpoint failure. Runtime model switching is allowed only after a
terminal provider failure and a new turn boundary, and it must invalidate any
unfinished planner proposal from the previous model.

## Acceptance gates

Each promoted capability requires:

1. one canonical ROS action or service contract;
2. registry and AB-level consistency;
3. unit tests for bounds, normalization, cancellation, and failure mapping;
4. a clean image rebuild from `iiia:nao`;
5. one physical success and one physical failure or cancellation record;
6. no duplicate speech and no fabricated completion;
7. an evidence bundle that separates command publication, measured physical
   effect, semantic result, and observability quality.

The current v1 image and thesis result remain unchanged until a v2 slice meets
all applicable gates.
