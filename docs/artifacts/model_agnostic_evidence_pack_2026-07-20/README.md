# NAO ROS4HRI model-agnostic runtime evidence pack

This package consolidates the frozen v34 runtime review, earlier model
ablations, historical vLLM and Ollama runs, the current Ollama fallback switch,
the machine-readable dataset, and the Design Report. It is intended for thesis
review and for a later TFM writing pass.

## What the evidence supports

The v34 stack was qualified against
`QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` with a score of 8.4/10. That score is
an evidence-weighted runtime qualification, not a claim that every model has
the same behavior. Deterministic tests and live runtime cases are reported
separately. The runtime cases include ROS ingress, dialogue routing, grounded
context, KnowledgeCore effects, planner admission, fake-skill execution,
failure recovery, speech, and structured trace correlation.

The current demonstration uses `gemma4:31b-cloud` because vLLM was unreachable
at the time of the switch and this Ollama model passed a live completion probe.
It has only a preflight and one smoke-turn qualification in this session. The
pack therefore preserves the distinction between an operational demo fallback
and a fully qualified replacement.

## Dataset interpretation

`analysis/dataset.csv` and `analysis/dataset.jsonl` contain one record per
questionnaire case. Each row retains its source artifact and run group. The
`attribution_class` column is intentionally conservative:

- `stack_coherent_success` means the observed case matched its recorded
  trajectory and evidence was internally coherent.
- `model_or_backend_variance` means a degraded or failed case is associated
  with route, target-selection, JSON, coverage, or planner behavior that can
  vary with model output.
- `runtime_dependency` covers backend reachability and transport failure.
- `harness_observability` covers an evidence or correlation inconsistency.
- `stack_contract_or_runtime` is retained where the available artifact does
  not support a narrower attribution.

These labels are an analysis aid, not new experimental measurements. The raw
JSON artifacts remain the authority.

## Experimental method

The frozen campaign held the runtime image, launch profile, prompts, contracts,
KnowledgeCore fixtures, fake-skill policy, token limits, timeout settings, and
questionnaire cases constant. Model changes were treated as an independent
factor. Application-level sampling ablations were kept separate from model
ablations. A result was not promoted from a model-specific observation to a
stack claim unless the owning runtime evidence supported that conclusion.

The final live model check adds an operational selector. It queries vLLM
`/v1/models` first, probes an advertised model, then checks Ollama `/api/tags`
and a small `think=false` completion. An explicit CLI model is tried first.
Fallback is pinned at launch and emits a JSONL event instead of switching in
the middle of a turn because of latency.

## Contents

- `report/Model_Agnostic_Runtime_Qualification_Report.docx`: formal Design
  Report based on the retained artifact-template-design-report visual system.
- `analysis/dataset.csv`: tabular case-level dataset.
- `analysis/dataset.jsonl`: lossless line-oriented case records.
- `analysis/dataset_summary.json`: counts and run inventory.
- `analysis/model_transition_summary.json`: model and attribution summary.
- `evidence/current_v34/`: frozen v34 primary qualification artifacts.
- `evidence/ablations/`: application sampling and token-budget ablations.
- `evidence/historical_vllm/` and `evidence/historical_ollama/`: prior model
  runs used for reserve ordering and variance analysis.
- `evidence/current_ollama_switch/`: current endpoint probes, launch log, and
  smoke result.
- `evidence/reports/`: prior Markdown and HTML runtime reports.

## Limits

The package does not claim that the current Ollama fallback is equivalent to
the qualified Qwen3-VL run. It also does not infer semantic quality from HTTP
availability alone. A replacement qualification requires the same frozen
runtime review, including repeated robustness and capability-extreme cases.
