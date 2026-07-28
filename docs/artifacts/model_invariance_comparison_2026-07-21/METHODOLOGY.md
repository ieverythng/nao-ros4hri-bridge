# Model invariance E2E comparison

Run identifier: `model-invariance-e2e-2026-07-21`

Date: 21 July 2026

## Question

Does the frozen NAO ROS4HRI stack preserve its owned contracts when the language model is changed, and which observed failures are shared stack-sensitive behavior versus model-specific variance?

The comparison keeps the ROS graph, frozen container image, prompt and generation parameters, KnowledgeCore fixtures, fake-skill policies, turn pipeline, questionnaire definitions, and speech evidence policy fixed. The model and endpoint are the experimental factors.

## Endpoint inventory

The requested vLLM endpoint was probed before the comparison:

```text
http://10.7.138.215:8004/v1/models
HTTP 000, connection unavailable
```

The endpoint therefore has no scoreable vLLM semantic cell in this run. The local Ollama inventory returned 11 advertised models. The three cells selected for the frozen comparison were:

1. `gemma4:31b-cloud`, baseline
2. `nemotron-3-super:cloud`, reserve candidate
3. `gemma4:cloud`, reserve candidate

The complete inventory is recorded in `analysis/comparison_summary.json`.

## Frozen runtime tuple

- Image: `iiia:nao-runtime-v34-final-frozen-review`
- Image ID: `sha256:bb82c158092a69870dae48272b3b20fd8cd4ecfe97a0e61a2253a2cca00c663e`
- ROS profile: `nao_chatbot_sim.launch.py`
- Turn pipeline: `response_first`
- Chatbot and planner thinking: disabled
- Chatbot and planner token, temperature, timeout, and provider settings: held constant
- Fake skills: enabled
- Object detection: disabled
- Scene grounding: enabled
- KnowledgeCore: enabled
- vLLM managed service: disabled
- Endpoint probes and launch preflight: required before semantic scoring

The historical Gemma4 31B baseline was initially recorded with `start_naoqi_driver=true`. The NAOqi endpoint was unavailable and timed out. The Nemotron and Gemma4 cloud semantic cells used `start_naoqi_driver=false` to remove that external dependency from model comparison. This is a controlled semantic profile, not a claim about physical robot connectivity. The baseline artifact is retained with its original profile so the profile change remains auditable.

## E2E case families

Each scoreable model cell ran the runtime-review harness against the live ROS graph:

| Family | Purpose | Cases in the standard cell |
| --- | --- | ---: |
| `environment` | preloaded scene and grounded interaction validation | 11 |
| `main` | dialogue, grounding, KB, simple skills, and composites | 21, with global timeout rows excluded from rates |
| `kb_stress` | insertion, relation revision, grounded delivery, postcondition, and follow-up | 7 |
| `robustness` | grouped delivery, role separation, missing recipient, deferred intent, and multi-turn carry-over | 5 |
| `fake_deep` all-success | deep plan and executor lineage under deterministic success | 9 |
| `capability_extreme` | posture, navigation, selection, placement, reporting, and recovery composition | 7 |
| targeted fake failure | fail-once navigation, replan, and post-failure reporting | 1 |

The baseline also retains an earlier full fail-once-navigation artifact. It contains non-applicable cases and a harness timeout, so it is diagnostic evidence and is not merged into the comparable targeted-cell rate.

## Status semantics

- `pass`: expected evidence and terminal behavior observed.
- `degraded`: the case was scoreable, but a required terminal, speech, or recovery property was incomplete.
- `fail`: the case was scoreable and a required property was not met.
- `not_scored`: excluded because correlation or a required model-dependent precondition was unavailable.
- blank status: unresolved global timeout, retained as harness evidence and excluded from the denominator.

The live questionnaire is a runtime stress evaluation over a fixed case set. It is not a deterministic unit-test count. Deterministic source and harness tests are separate evidence and must not be added to the live case denominator.

## Attribution method

The seam audit uses shared and model-specific outcomes as discriminating evidence:

- A failure reproduced by both Gemma variants under the same frozen application contract is a candidate stack or contract seam. It is not automatically blamed on the model.
- A failure isolated to Nemotron is a model or backend variance candidate unless trace inconsistency shows an observability problem.
- A failure with missing or contradictory trace evidence is marked as an observability or correlation issue.
- A passed preflight proves launch readiness and endpoint reachability. It does not prove planner quality.

Raw per-case JSON remains authoritative. The CSV and JSONL datasets are derived indexes with source-file provenance.

Container launch logs were inspected live during each startup and are represented in the startup snapshots and per-case log excerpts. The reusable container was switched between model cells, so not every transient launch-log file was copied into the final pack. This is recorded as a provenance limitation rather than treated as missing semantic evidence.

## Reproduction

The main questionnaire command was:

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py \
  --container nao_ros2 \
  --case-set main \
  --speech-voice-scope group \
  --out <model>/main.json
```

The same command pattern was used for `environment`, `kb_stress`, `robustness`, `fake_deep`, and `capability_extreme`. The targeted recovery cell used `--case-names fake_deep_ordered_walk_report --fake-policy-profile fail_once_navigation`.

## Limitations

The comparison has one run per model, not a statistical sample over repeated independent model generations. It supports a bounded engineering attribution and a thesis results table, not a universal model ranking. The Gemma4 baseline profile difference and the unavailable vLLM endpoint are explicit limitations. The next run should repeat the sensitive cases at least three times per model after a stable vLLM endpoint is available.
