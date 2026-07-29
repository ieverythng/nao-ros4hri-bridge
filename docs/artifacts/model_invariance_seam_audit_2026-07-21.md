# Model invariance seam hypothesis audit

Date: 21 July 2026

## Target contract

- Problem: determine which observed runtime failures remain invariant stack
  failures and which vary with the selected language model or backend.
- Required outcome: run the same frozen `iiia:nao-runtime-v34-final-frozen-review`
  image and the same ROS4HRI questionnaire families against three available
  models, preserving raw JSON, startup logs, snapshots, and model parameters.
- Non-goals: changing prompts, contracts, launch defaults, registries, fake-skill
  policies, or planner code during the comparison; qualifying vLLM when its
  endpoint does not advertise a reachable model; treating HTTP availability as
  semantic quality.
- Protected seams and owners: `dialogue_manager` owns speech lifecycle and
  speaking; `chatbot_llm` owns dialogue and route selection; `planner_llm` owns
  planning and supervision; `nao_orchestrator` owns admission, dispatch,
  lineage, and feedback; `kb_skills` owns KnowledgeCore transport;
  `nao_scene_grounding` owns scene projection; AB=1 skills own fresh effects.
- Acceptance gate: each model receives the same launch tuple and all applicable
  questionnaire families. Every case is retained even when the model or
  backend fails. A model-specific result is not promoted to a stack conclusion
  without owner-level evidence and an adversarial comparison.
- Evidence and time budget: one clean startup and one full pass per model,
  covering main, environment, KB stress, robustness, fake-deep success and
  failure, capability-extreme, and targeted fail-once recovery. No source or
  prompt mutation during the run.

## Baseline evidence

| Observation | Source or command | Expected | Actual | Evidence reference |
| --- | --- | --- | --- | --- |
| vLLM inventory | `curl http://10.7.138.215:8004/v1/models` | HTTP 200 with model ids | Connection failure, `HTTP_STATUS=000` | comparative run probe |
| Ollama inventory | `curl http://127.0.0.1:11434/api/tags` | HTTP 200 with candidates | 11 advertised models | comparative run probe |
| Current chatbot | `ros2 param get /chatbot_llm model` | Explicit baseline model | `gemma4:31b-cloud` | baseline startup snapshot |
| Current planner | `ros2 param get /planner_llm model` | Same baseline model | `gemma4:31b-cloud` | baseline startup snapshot |
| Dialogue lifecycle | `ros2 lifecycle get /dialogue_manager` | `active [3]` | `active [3]` | baseline startup snapshot |
| Runtime image | `docker inspect nao_ros2` | Frozen v34 image | `iiia:nao-runtime-v34-final-frozen-review` | per-model startup snapshot |

## Approach registry

| ID | Family | Mechanism | Affected seams | Discriminating probe | Expected observation | Status | Exact gap or reopen condition |
| --- | --- | --- | --- | --- | --- | --- | --- |
| H-01 | Runtime wiring | A provider or endpoint mismatch causes failures independently of model quality | launch, chatbot transport, planner provider | inventory, named preflight, startup params | failures cluster at preflight or transport and repeat across models | active | endpoint unavailable prevents vLLM semantic comparison |
| H-02 | Model policy | Different model outputs cause route, intent, target, or plan coverage variance while stack owners remain coherent | chatbot, planner, prompt contract | same main, robustness, and capability cases across models | failure rates and selection/coverage fields vary by model | active | requires comparable JSON and trace evidence |
| H-03 | Grounding and KB | A shared grounding or KnowledgeCore issue affects all models | KB, scene grounding, chatbot context | environment and KB stress with grounded entity checks | same object/relation absence or freshness issue across candidates | active | interaction simulator object evidence must be present |
| H-04 | Executor and recovery | Orchestrator, fake-skill, or report-result behavior creates model-independent failures | planner, orchestrator, fake skills, speech | fake-deep all-success, fail-once, delivery-blocked, replan | same lineage, dispatch, or speech failure across models | candidate | requires post-effect and terminal speech evidence |
| H-05 | Observability | Questionnaire timing or trace correlation makes valid behavior look failed | speech ingress, trace viewer, harness | startup gate, structured trace counts, speech evidence per case | failure appears only in measurement fields or log timing | candidate | resolve against raw trace and topic samples |
| H-06 | External service variance | Ollama cloud quotas, auth, or backend latency create model-specific unavailable runs | Ollama endpoint, transport, preflight | per-model liveness and first-error timestamps | explicit HTTP/quota/auth errors precede later failures | active | mark dependent cases not scored, do not infer stack failure |

## Round log

| Round | Routes selected | New evidence | Redirect or rejection | Next probe |
| --- | --- | --- | --- | --- |
| 0 | H-01 through H-06 | vLLM is unreachable; Ollama has three viable candidates for comparison | vLLM semantic route blocked | clean baseline snapshot and Gemma full suite |
| 1 | H-01, H-02, H-03, H-06 | Three startup gates passed for Gemma4 31B, Nemotron 3 Super, and Gemma4 cloud. vLLM remained unavailable. | vLLM semantic qualification was not scored. | compare the fixed-case matrices and retain the endpoint gap |
| 2 | H-02, H-03, H-04, H-05 | Gemma variants reproduced the same main and extreme failures. Nemotron preserved environment execution but added KB wording, trace-correlation, and planner coverage variance. | Shared failures are not assigned to a model without a discriminating owner probe. | complete the owner-level attribution and report |
| 3 | H-01 through H-06 | Evidence pack, JSONL/CSV dataset, comparison summary, and raw per-model artifacts completed. | No source, prompt, launch, registry, or fake-policy mutation was made during the cells. | rerun sensitive cases three times per model when vLLM is reachable |

## Adversarial audit

- [x] Ownership boundaries remain unchanged in the frozen runtime cells.
- [x] ROS interfaces and topic/service/action choices remained valid at startup.
- [x] Goal, plan, version, and step lineage were retained in the structured traces.
- [x] No duplicate speech or hidden executable plan was observed in the scored evidence.
- [x] No unsupported perception, KB, proximity, result, or completion claim was accepted as a passing result.
- [x] AB level and canonical registry semantics were not changed during the comparison.
- [x] Success and relevant failure/recovery paths were exercised for each model.
- [x] No prompt, source, launch, registry, or fake-policy mutation occurred between model cells.

## Decision

- Decision: `bounded handoff` with three completed Ollama model cells and one blocked vLLM cell.
- Chosen route: controlled model/backend comparison with fixed stack image.
- Evidence that satisfies the gate: live endpoint inventory, per-model startup
  snapshots, full questionnaire JSON, structured traces, logs, a thesis-facing
  dataset, and a comparison report with explicit model-versus-stack attribution.
- Residual risk: vLLM cannot be compared semantically until its endpoint returns
  an advertised model and a named completion. Ollama cloud availability may
  change during the run.
- Exact next probe if unresolved: repeat the affected model cell after the
  first explicit backend error, without changing any other runtime setting.

## Completed comparative findings

The standard comparable families produced the following strict pass rates. A
`degraded` case remains in the denominator because it was scoreable, while
`not_scored` and global-timeout rows are excluded.

| Model | Main | Environment | KB stress | Robustness | Fake deep success | Capability extreme | Comparable interpretation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `gemma4:31b-cloud` | 17/20 | 11/11 | 7/7 | 3/5 | 9/9 | 3/7 | Baseline remained coherent on grounding, KB, and deterministic fake execution. |
| `gemma4:cloud` | 18/21 | 11/11 | 7/7 | 3/5 | 9/9 | 3/7 | Reproduced the Gemma baseline failure shape, including maximal planning and missing-recipient closure. |
| `nemotron-3-super:cloud` | 17/19 plus one degraded | 11/11 | 4/6 | 3/5 | 8/9 plus one degraded | 0/6 | Environment and targeted replan passed, while planner coverage and KB wording were less stable. |

The most important shared failures were `skill_pick_phone_generic`,
`skill_pick_object_on_table` (with a degraded rather than failed Nemotron
outcome), `maximal_kitchen_cup_to_operator`,
`extreme_kneel_under_table_pick_report`,
`extreme_all_objects_visit_look_wave_sit`,
`extreme_pick_place_kneel_report`,
`extreme_unreachable_object_recovery`, and
`robust_missing_recipient`. The two Gemma variants reproduced the same
failure names and reasons in the comparable cells. This supports a stack or
contract investigation for those seams before assigning the failures to
Gemma quality.

Nemotron-only variance included omitted grounded terms in two KB speech cases,
missing execution feedback in one trace-correlated case, incomplete report or
skill coverage in extreme plans, and one fake-deep recovery degradation. Its
targeted fail-once navigation cell passed with an observed failure and replan,
which is evidence that the executor recovery seam can remain coherent when the
model emits an admissible plan.

## Evidence index

- Comparative report: `docs/artifacts/model_invariance_comparison_2026-07-21/REPORT.md`
- HTML report: `docs/artifacts/model_invariance_comparison_2026-07-21/REPORT.html`
- Templated design report: `docs/artifacts/model_invariance_comparison_2026-07-21/report/Model_Invariance_Runtime_Qualification_Report.docx`
- Dataset: `docs/artifacts/model_invariance_comparison_2026-07-21/analysis/dataset.csv` and `dataset.jsonl`
- Summary: `docs/artifacts/model_invariance_comparison_2026-07-21/analysis/comparison_summary.json`
- Raw model artifacts: `docs/artifacts/model_invariance_comparison_2026-07-21/raw/`
