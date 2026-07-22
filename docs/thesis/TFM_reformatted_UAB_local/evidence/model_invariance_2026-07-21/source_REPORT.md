# Runtime Review: model invariance across Ollama candidates

Run ID: `model-invariance-e2e-2026-07-21`

Date: 21 July 2026

## Executive result

The frozen stack reached the required startup gate with three Ollama models. The vLLM endpoint was unavailable and therefore has no semantic score in this run. The strongest evidence is not a universal model ranking. It is the separation between invariant stack behavior and model-sensitive planning behavior.

Both Gemma variants reproduced the same main and extreme-capability failure shape. They passed the full environment matrix, the stateful KB chain, and the all-success fake-skill matrix. Nemotron preserved the environment matrix and the targeted fail-once navigation replan, but showed additional variance in KB wording, trace correlation, target selection, report closure, and extreme multi-step coverage.

The results support the following bounded conclusion:

> The NAO ROS4HRI stack preserved its ownership, grounding, execution-lineage, and fake-skill recovery boundaries across the tested Ollama models. Model-generated target selection, plan coverage, clarification wording, and terminal closure remained variable. The repeated Gemma failure shape requires a stack-seam investigation before those failures are attributed to Gemma quality.

This conclusion is suitable for the TFM Results and Discussion sections when accompanied by the raw JSON and the controlled-profile caveat below.

## Runtime review format

### Overall sentiment

`stable with model-dependent degradation in high-composition planning`

### Startup gate

All three scoreable model cells had:

- chatbot, planner, orchestrator, dialogue manager, KnowledgeCore, and fake-skill nodes present;
- active lifecycle state for the required core nodes;
- chatbot and planner preflight success on the selected model;
- zero fallback events during the scored cell;
- structured interaction traces available for case-level attribution.

The optional `knowledge_viewer` configuration warning and missing downstream TTS subscriber were retained as runtime observability conditions. They did not prevent the required semantic graph from becoming ready. The NAOqi connection timed out because `172.26.112.143:9559` was unavailable. The alternative semantic cells disabled the external driver to prevent this dependency from changing the model comparison.

### Model cells

| Cell | Backend and model | Startup | Profile | Role |
| --- | --- | --- | --- | --- |
| Baseline | Ollama `gemma4:31b-cloud` | ready for semantic scoring | historical baseline with `start_naoqi_driver=true` | baseline |
| Reserve 1 | Ollama `nemotron-3-super:cloud` | ready for semantic scoring | controlled semantic profile with `start_naoqi_driver=false` | second candidate |
| Reserve 2 | Ollama `gemma4:cloud` | ready for semantic scoring | controlled semantic profile with `start_naoqi_driver=false` | third candidate |

The frozen image was `iiia:nao-runtime-v34-final-frozen-review` with image ID `sha256:bb82c158092a69870dae48272b3b20fd8cd4ecfe97a0e61a2253a2cca00c663e`.

## Endpoint inventory and selection

The requested vLLM probe was:

```text
http://10.7.138.215:8004/v1/models
```

The final probe failed with `HTTP_STATUS=000`, so the served vLLM model could not be identified and no vLLM semantic result was invented. The local Ollama inventory returned 11 names:

`deepseek-v4-flash:cloud`, `gemini-3-flash-preview:latest`, `gemma4:31b-cloud`, `gemma4:cloud`, `glm-5.2:cloud`, `kimi-k2.5:cloud`, `kimi-k2.6:cloud`, `minimax-m2.7:cloud`, `nemotron-3-super:cloud`, `qwen3-coder:480b-cloud`, and `qwen3.5:cloud`.

The three tested names were selected because they were visible in the live inventory and could pass the required chatbot and planner preflight. Availability was treated as a launch condition, not as evidence of semantic quality.

## Frozen evaluation contract

The following factors were held constant across the comparison:

- the v34 frozen container image;
- ROS 2 Jazzy launch profile and ROS domain;
- response-first turn pipeline;
- chatbot and planner provider settings, model parameters, timeouts, token budgets, and thinking policy;
- KnowledgeCore fixtures and grounded context projection;
- fake-skill server and all-success/failure policy definitions;
- questionnaire source, case names, speech voice scope, and trace collection;
- scene grounding enabled and object detection disabled.

The historical Gemma4 31B baseline was captured before the NAOqi dependency was isolated. It is kept as the baseline artifact. The Nemotron and Gemma4 cloud cells use the controlled no-driver profile because the physical NAOqi endpoint was unreachable. This difference is a declared runtime precondition and prevents a physical robot connection timeout from being misclassified as model behavior.

## E2E score matrix

The table reports strict scoreable outcomes. A pass is counted as a pass. A degraded case is retained as scoreable but is not counted as a pass. `not_scored` and blank global-timeout rows are excluded from the denominator.

| Case family | Gemma4 31B | Gemma4 cloud | Nemotron 3 Super |
| --- | ---: | ---: | ---: |
| Environment | 11/11 | 11/11 | 11/11 |
| Main | 17/20 | 18/21 | 17/19 plus 1 degraded |
| KB stress | 7/7 | 7/7 | 4/6 |
| Robustness | 3/5 | 3/5 | 3/5 |
| Fake deep, all success | 9/9 | 9/9 | 8/9 plus 1 degraded |
| Capability extreme | 3/7 | 3/7 | 0/6, 1 not scored |
| Targeted fail-once navigation | diagnostic full run retained | 1/1 | 1/1 |

The comparable standard-family pass rates were:

| Model | Passes | Scoreable cases | Strict pass rate | Reading |
| --- | ---: | ---: | ---: | --- |
| `gemma4:31b-cloud` | 50 | 59 | 84.7% | Strong shared-stack behavior with high-composition planning failures. |
| `gemma4:cloud` | 51 | 60 | 85.0% | Same failure shape as the Gemma4 31B baseline. |
| `nemotron-3-super:cloud` | 43 | 56 | 76.8% | Environment and targeted recovery remained reliable; high-composition and KB wording varied more. |

The rates are descriptive one-run engineering evidence. They are not a statistical confidence interval and should not be presented as a universal ranking.

## What remained invariant

### Grounded environment behavior

All three models passed the 11-case environment matrix. The frozen scene fixtures, KnowledgeCore access, grounded context projection, and basic response path were therefore available to each model. This is strong evidence against a broad stack-wide grounding outage in this run.

### Stateful KnowledgeCore behavior

Both Gemma variants passed all seven KB stress cases. Nemotron passed four of six scoreable cases and failed two wording checks:

- `kb_stress_revise_support` omitted `storage shelf` and `work table` from speech;
- `kb_stress_revised_relation_query` omitted `TITAS`, `storage shelf`, `MIDAS`, and `work table` from speech.

The failures were speech-content omissions, not evidence that the KB mutation transport itself was unavailable. This separates KnowledgeCore state transport from model-mediated verbal projection.

### Basic fake-skill execution

The all-success fake-deep matrix passed 9/9 for both Gemma variants. Nemotron passed 8/9 and degraded on `fake_deep_missing_object_recovery` because recovery evidence lacked terminal or speech closure. The targeted fail-once navigation probe passed for Nemotron and Gemma4 cloud with an observed injected failure and replan. The fake-skill server, orchestrator admission, and execution lineage can therefore remain coherent when the emitted plan is admissible and the recovery condition is applicable.

### Failure honesty

No model cell was accepted as a passing result when the expected planner request, execution feedback, target selection, postcondition, clarification speech, or terminal evidence was absent. The questionnaire retained failures instead of converting them to successful conversational responses. This protects the thesis claim that stack coherence is measured through execution evidence rather than fluency alone.

## Shared failures and seam attribution

The most important comparison is the failure intersection. Both Gemma variants failed the same cases with the same broad reasons:

| Case | Observed pattern | Initial attribution |
| --- | --- | --- |
| `skill_pick_phone_generic` | no expected planner request and execution feedback | stack or contract seam candidate |
| `skill_pick_object_on_table` | no expected planner request and execution feedback; Nemotron dispatched a partial recovery and was degraded | shared admission or target normalization seam, with model-sensitive variation |
| `maximal_kitchen_cup_to_operator` | clarification was not expressed in correlated speech | shared response or dialogue-act closure seam candidate |
| `extreme_kneel_under_table_pick_report` | no expected ordered motion, pick, motion, report sequence | shared planner coverage or capability-contract seam candidate |
| `extreme_all_objects_visit_look_wave_sit` | target selection was not carried in planner evidence | shared universal-selection or context-size seam candidate |
| `extreme_pick_place_kneel_report` | clarification or plan coverage failed before expected execution | shared high-composition planning seam candidate |
| `extreme_unreachable_object_recovery` | expected motion, pick, and report sequence was absent | shared recovery admission or target-selection seam candidate |
| `robust_missing_recipient` | clarification text was not correlated even though the absent recipient was recognized | shared clarification speech seam candidate |

These cases are not sufficient to prove that the stack implementation is faulty. They are sufficient to reject the claim that Gemma quality alone explains the failures. The next discriminating probes should inspect planner requests, route intents, target selections, and structured dialogue acts at the first failing seam.

## Model-specific variance

Nemotron introduced additional failures not reproduced by either Gemma cell:

- two KB speech cases omitted grounded terms;
- `robust_object_role_exclusion` failed at planner request and execution feedback;
- `extreme_walk_pick_sit_report` and `extreme_dialogue_sit_stand_grab_return` failed plan coverage;
- `extreme_dialogue_inventory` was not scored because trace correlation was internally inconsistent;
- `extreme_pick_place_kneel_report` and `extreme_unreachable_object_recovery` executed partial skill sequences but omitted expected final reporting;
- `fake_deep_missing_object_recovery` degraded on terminal or speech closure;
- `main` included one degraded partial pick recovery and one not-scored KB mutation trace.

These outcomes are model or backend variance candidates. The trace inconsistency is a measurement seam and must not be presented as a semantic planner failure without checking the raw trace.

## Seam hypothesis adjudication

| Hypothesis | Evidence | Status | Next discriminating probe |
| --- | --- | --- | --- |
| H-01 runtime wiring or provider mismatch | all three required preflights passed; vLLM was unavailable before scoring | ruled out for Ollama cells, blocked for vLLM | repeat only after vLLM `/v1/models` and completion probe succeed |
| H-02 model policy variance | Gemma variants share a stable failure shape; Nemotron adds coverage and wording variance | supported | capture first chatbot route, planner request, target selection, and dialogue act for shared failures |
| H-03 grounding and KB | environment 11/11 for all; Gemma KB 7/7; Nemotron wording omissions only | broad outage rejected; projection variance supported | compare grounded context digest with spoken terms and planner payload for the same KB facts |
| H-04 executor and recovery | targeted fail-once navigation passed for Nemotron and Gemma cloud; partial-report failures remain | executor core supported, closure seam open | repeat fail-once pick, delivery-blocked, and recipient-missing with stable terminal speech capture |
| H-05 observability | Nemotron contains internally inconsistent trace rows; global timeout rows exist | active measurement limitation | reconcile trace JSONL, speech topics, and harness timestamps for each not-scored row |
| H-06 external service variance | vLLM unavailable; Ollama cloud models passed preflight with zero fallback events | vLLM blocked, Ollama cell usable | record endpoint identity, first error, and quota/auth status during a stable window |

## Deterministic tests versus runtime stress

The earlier count of runtime-review tests should not be described as 69 randomized stress trials. The formal runtime review combines different evidence classes:

1. deterministic source and harness checks, which validate parser, manifest, registry, contract, and owner behavior;
2. live questionnaire cases, which exercise the running ROS graph and a fixed sequence of interactions;
3. model-sensitive runtime cases, where the same input can produce different route, target, plan, wording, and timing outcomes.

This comparison belongs to the third class. It is a controlled E2E stress evaluation over fixed case families, not a deterministic proof that every possible natural-language request will succeed. The raw JSON preserves the model response path, structured trace, speech observations, and timeout evidence needed for a thesis discussion of non-determinism and model variance.

## Thesis interpretation

The evidence supports three separate claims:

1. The stack has a stable architectural core. Environment grounding, KnowledgeCore transport, fake-skill execution, execution lineage, and failure honesty were repeatedly exercised across changing models.
2. The stack is not semantically invariant at the model boundary. Target selection, multi-step coverage, clarification wording, report closure, and model-dependent recovery remain variable.
3. A model replacement cannot be qualified from endpoint availability or one successful dialogue. It requires the same frozen E2E matrix, the same attribution rules, and repeated sensitive cases.

A conservative TFM formulation is:

> The evaluation shows that the ROS4HRI stack preserves its principal ownership and execution contracts across the tested language-model backends. At the same time, generated target selection, multi-step plan coverage, clarification wording, and terminal report closure vary with the selected model. Shared failures across the two Gemma variants were retained as stack-seam hypotheses rather than attributed to model quality alone.

## Artifacts and provenance

- `analysis/comparison_summary.json`: model inventory, startup gates, per-suite rates, case matrix, and shared-failure intersection.
- `analysis/dataset.csv`: tabular per-case index with status, observations, fallback markers, skills, speech count, and provenance path.
- `analysis/dataset.jsonl`: JSONL form for TFM processing.
- `raw/gemma4_31b/`: baseline JSON outputs.
- `raw/nemotron_3_super/`: Nemotron JSON outputs, including controlled startup snapshots.
- `raw/gemma4_cloud/`: Gemma cloud JSON outputs.
- `METHODOLOGY.md`: frozen tuple, status semantics, and reproduction contract.
- `inventory_probe.md`: vLLM and Ollama endpoint evidence.
- `PROVENANCE_GAPS.md`: explicit note about transient launch logs inspected live but not all copied after container reuse.
- `../model_invariance_seam_audit_2026-07-21.md`: hypothesis registry and adversarial audit.

The HTML version is `REPORT.html`. The retained artifact-template design report is in `report/Model_Invariance_Runtime_Qualification_Report.docx`.
