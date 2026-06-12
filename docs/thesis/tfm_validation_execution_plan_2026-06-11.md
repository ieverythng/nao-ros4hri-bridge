# TFM Validation Execution Plan

Date: 2026-06-11  
Scope: Repeatable fake-skill, simulator, and robot validation runs for the
semi-symbolic planner architecture.

## Evidence Pipeline

Each formal run must produce:

1. interaction-trace JSONL as the authoritative event record;
2. the narrative fake-skill probe report for qualitative trace inspection;
3. JSON and CSV metrics for analysis;
4. the generated HTML metrics dashboard for rapid comparison;
5. an experiment-log row recording expected and observed behavior.

Run the automated fake-skill sweep with:

```bash
python3 scripts/run_live_fake_skill_scenario_probe.py
```

The probe writes JSONL and Markdown under `docs/artifacts/` and invokes
`scripts/summarize_validation_traces.py` to generate matching JSON, CSV, and HTML
metrics artifacts.

A dashboard generated from the existing 2026-06-03 trace is available at
`docs/artifacts/fake_skill_validation_metrics_20260603_sample.html`. It is a
format/example artifact, not a replacement for a new formal experiment run.

## Scenario Set

| ID | Requirement exercised | Expected evidence | Acceptance gate |
| --- | --- | --- | --- |
| `success_default` | normal grounded execution | valid plan, successful `find_object`, terminal feedback | completed with one dialogue owner |
| `deterministic_ambiguous` | clarification/recovery | ambiguous failure payload and clarification/replan decision | failure is attributable and recoverable |
| `deterministic_path_blocked` | execution failure | path-blocked result and feedback | no fabricated completion |
| `spatial_near_object` | metric spatial evidence | frame-qualified position and `distance_m` in result evidence | distance claim uses metric evidence |
| `spatial_far_object` | proximity contrast | larger frame-qualified distance than near case | near/far distinction matches evidence |
| `random_seeded_stress` | deterministic stochastic policy | repeatable seeded outcome sequence | repeated runs with same seed agree |
| `all_fail_always` | terminal failure | terminal failure dialogue and no completion claim | failure remains explicit |
| `area_person_found` | human-inclusive perception | person appears under `people`, not `objects` | spoken/trace report includes person |
| explicit KB add/revise/remove | chat-authorized mutation | planner selects one KB mutation skill; `/kb/revise` succeeds | concrete requested predicate changes |
| ordinary KB question | mutation safety | knowledge-query/dialogue route; no mutation dispatch | zero KB writes |

## Spatial Validation Protocol

The simulator currently supplies a robot/camera TF chain but no object TF frames.
For spatial tests, publish frame-qualified poses on the configured
`scene_grounding_spatial_overlay_topic`:

```json
{
  "objects": [
    {
      "entity_id": "detected_cup_320_240",
      "frame_id": "base_link",
      "position": {"x": 1.0, "y": 0.2, "z": 0.7}
    }
  ]
}
```

The expected chain is:

`spatial overlay -> nao_scene_grounding -> /scene/summary + /kb/revise -> compact grounded_context -> chatbot/planner`

Image-plane `center_x` and `center_y` must never count as metric evidence.

## Metrics and Representation

The HTML dashboard is intended for fast run comparison. CSV is the thesis-table
source, JSON preserves machine-readable detail, and JSONL remains the
authoritative trace.

Primary quantitative metrics:

- completion rate over traces containing execution feedback;
- median correlated-trace duration;
- plan output, feedback, and dialogue-act counts;
- unsupported-step and invalid-plan rate;
- recovery and clarification success;
- duplicate speech rate;
- grounded-target correctness;
- unauthorized KB mutation count;
- spatial-answer correctness for frame-qualified cases.

Uncorrelated trace groups are reported separately and must not contribute to
completion-rate calculations.

## Experiment Phases

1. **Contract gate:** unit tests, registry consistency, Python compilation, and
   ROS4HRI change audit.
2. **Fake-skill gate:** deterministic scenario sweep and dashboard review.
3. **Simulator gate:** object detection plus spatial overlays, human-inclusive
   visibility questions, and KB mutation requests.
4. **Robot gate:** repeat only safe scenarios; replace simulator overlays with
   real frame-qualified sources when available.
5. **Ablation runs:** planner disabled, grounding removed, feedback removed, and
   reduced registry metadata.

## Supervisor Decision Required

Select the canonical frame and source for real metric object positions. Until a
source publishes trustworthy 3D poses, proximity answers must remain unknown or
request clarification. This is a data-source decision, not an LLM prompt issue.
