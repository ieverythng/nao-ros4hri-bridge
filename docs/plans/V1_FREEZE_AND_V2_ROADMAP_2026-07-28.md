# NAO ROS4HRI v1 Freeze and v2 Roadmap

Date: 28 July 2026

## Freeze decision

The v1 implementation is frozen for thesis and presentation use. The runtime
image is `iiia:nao-final`, digest
`sha256:bb82c158092a69870dae48272b3b20fd8cd4ecfe97a0e61a2253a2cca00c663e`.
It is an alias of the validated image previously called
`iiia:nao-runtime-v34-final-frozen-review`.

The final thesis qualifies a bounded architecture, not unrestricted robot
autonomy. The v1 contribution is the integration and validation of dialogue,
semantic routing, symbolic grounding, structured planning, deterministic
execution admission, typed feedback, controlled skill adapters, replanning,
and evidence-based reporting. The primary Qwen3-VL qualification produced 64
passes and eight non-passes across 72 records. The separate model comparison
shows interface portability together with semantic variance.

## Frozen ownership

| Layer | v1 owner | Frozen responsibility |
| --- | --- | --- |
| Dialogue and speaking | `dialogue_manager` | Dialogue lifecycle and the single Say dispatch |
| User-facing LLM | `chatbot_llm` | Response generation, route selection, grounded projection, planner ingress |
| Planning and supervision | `planner_llm` | Plan generation, retry/replan policy, structured dialogue acts |
| Contracts | `planner_common` | Planner request, target selection, plan, result, and feedback normalization |
| Deterministic execution | `nao_orchestrator` | Admission, validation, ordered dispatch, lineage, feedback, KB effects |
| Knowledge transport | `kb_skills` | KnowledgeCore query and mutation boundary |
| Scene grounding | `nao_scene_grounding` | Detector normalization and symbolic scene projection |
| Launch and operator wiring | `nao_chatbot` | Integrated simulator, robot, demo, and ASR profiles |

No v2 feature may move these responsibilities without an explicit architecture
decision and a replacement qualification.

## v1 capability boundary

| Capability | v1 state | Evidence boundary |
| --- | --- | --- |
| Dialogue, KB query, KB mutation | Qualified within the frozen questionnaire | Software and symbolic-state evidence |
| Planner and orchestrator | Qualified for the tested task families | Typed plan, lineage, admission, feedback, and safe rejection evidence |
| Fake skills and replanning | Qualified within retained deterministic profiles | Software execution and recovery evidence |
| Posture and named replay | Robot-facing adapter present | Integration checks, not repeated physical-performance trials |
| Head motion | Robot-facing adapter with honest open-loop mode | Command publication evidence; convergence may be unavailable |
| `look_at` | Real ROS4HRI adapter present, fake dispatch remains the default | Source and adapter tests pass; physical target-frame validation is a v2 gate |
| Text-to-speech | Robot-facing Say adapter present | Integration evidence, not speech-quality benchmarking |
| Vosk ASR | Opt-in local profile present | Utility path only; not part of the primary qualification |
| Navigation and manipulation | Fake skills | No physical accuracy or safety claim |
| `walk_to` and `wave_greet` | Fake or proposal-level in the active source | The older promotion plan overstates implementation and is superseded here |
| Object detection | Available as a separate profile | Disabled in the primary semantic run; detector accuracy is not scored |

## Known v1 limitations

1. Model-generated target selection, role assignment, plan coverage,
   clarification, and closure remain variable.
2. The generic `openai_compatible` planner adapter is qualified against the
   tested vLLM behavior. Its optional sampling and chat-template fields are not
   guaranteed to work with every nominally compatible provider.
3. Failure-position evidence is stronger at the beginning and middle of plans
   than at a dedicated final-skill injection.
4. Physical navigation, grasping, placement, delivery, perception accuracy,
   unrestricted autonomy, and human safety remain outside the result.
5. The detector and live-person paths can experience frame pressure and identity
   churn. Deterministic KnowledgeCore fixtures remain the stable qualification
   surface.

## v2 workstreams

### V2-A: embodied skill promotion

1. Validate real `look_at` first. Prove reset, one TF target, one person target,
   cancellation, missing TF, and no-joint-controller failure on the physical
   robot.
2. Add a guarded named `wave` motion through the existing replay-motion action.
   Keep `wave_at(person)` separate because target-aware social gesture policy is
   an AB>=2 composition.
3. Introduce an AB=0 local `MoveTo` action over bounded body-relative `x`, `y`,
   and yaw. Start with dry-run, then operator-cleared physical trials.
4. Keep semantic `navigate_to` as AB>=2. It requires localization, target pose,
   path or segment selection, obstacle/safety checks, local movement, and
   post-motion verification.
5. Treat `point_at` as AB>=2. It requires a frame-qualified target pose, arm and
   hand kinematics, workspace and collision limits, and visible completion
   evidence. A 2D image-plane approximation may support a research prototype,
   but it cannot be reported as spatially accurate pointing.

### V2-B: perception, WME, and Neural Workbench

- Route LocateAnything through `nao_scene_grounding`; do not create a second
  world-state owner.
- Introduce WME as an additive evidence and hypothesis layer over current KB and
  scene contracts.
- Use Neural Workbench for trace-derived capability comparison, AB promotion,
  and bounded adaptive experiments. It must not bypass the canonical skill
  registry or orchestrator.
- Preserve separate people, objects, locations, supports, and frame-qualified
  evidence throughout the projection.

### V2-C: speech services

- Keep Vosk as the offline fallback.
- Add a remote ASR adapter served from the main PC over ZeroTier. The preferred
  implementation is a Whisper-compatible service with streaming or short-chunk
  transcription, explicit language, confidence metadata, and a health endpoint.
- Add a remote TTS adapter only behind the existing Say action. The dialogue
  manager remains the speaking owner and the adapter must support cancellation.
- Record endpoint identity and latency in startup preflight. An unavailable
  remote speech service must fail visibly or fall back before the first turn.

### V2-D: autonomous behavior arbitration

Autonomous idle behavior should not become a second planner. Add one
deterministic arbitration seam above robot skills:

1. user and planner goals have priority over idle behavior;
2. autonomous actions may start only when no admitted goal is active;
3. a new user goal cancels or supersedes the autonomous action through the
   existing action boundary;
4. posture, gaze, and low-amplitude idle motions use explicit allowlists and
   safety parameters;
5. every autonomous action emits the same lineage and result evidence as an
   inbound request.

### V2-E: model qualification

The CLI-selected model remains the first choice. Backend fallback occurs only
at startup after inventory and readiness probes, not in the middle of a turn.
Each selected model is pinned for the launch and produces a visible selection
event. Every replacement model repeats the sensitive route, role, universal-set,
composition, clarification, and closure cases.

## Acceptance gates for v2

Every v2 slice must pass:

1. a source-level red/green test at the owning seam;
2. ROS4HRI ownership and interface audit;
3. registry consistency when capability metadata changes;
4. one clean image rebuild from `iiia:nao`;
5. one success and one failure or cancellation path;
6. a physical trial record when the claim concerns real robot behavior;
7. a dated report that keeps physical, semantic, and observability results
   separate.

## Documentation and evidence policy

The active contract surfaces are `docs/current_workflow.md`,
`docs/contracts.md`, `docs/launch_profiles.md`, and this masterplan. Dated runs
remain under `docs/artifacts/`. The issue trackers embedded in evidence ZIPs are
immutable provenance snapshots and are not edited. Their canonical successors
under `docs/plans/` carry the v1 freeze status.
