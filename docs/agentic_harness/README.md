# Universal Agentic Harness

This subsection initializes the Universal Agentic Harness research and
implementation track. The project studies how to turn an LLM into a bounded,
observable subsystem agent by compiling task-specific interaction modules from
the Neural Workbench AB capability graph.

## Canonical Document

- `universal_agentic_harness_foundation.md` (+ HTML) - project thesis,
  primary-source harness survey, current-stack extraction map, AB-aware
  architecture, model-serving design, hypothesis registry, evaluation plan,
  and implementation phases.
- `neural_workbench_adaptive_ab_harness.md` (+ HTML) - deeper Workbench
  extension covering relative AB frames, task control bands, AB3 agents inside
  AB4 systems, pulse-graph search, capability posteriors, maintained
  interaction skills, and reviewed trace-to-crystallization.

## Relationship To Existing Work

The project builds on, but does not replace:

- the canonical AB registry in
  `src/Neural-Wokbench/src/skill_common/skill_common/defaults/ab_registry.json`;
- Neural Workbench candidate generation, verification, energy scoring,
  selection, and trace memory;
- `planner_common` runtime contracts;
- `chatbot_llm` dialogue routing, context projection, and structured response
  enforcement;
- `planner_llm` planning, provider, validation, retry, and supervision seams;
- `nao_orchestrator` deterministic execution and evidence ownership.

The proposed kernel is model-independent and ROS-independent. NAO is the first
reference subsystem and validation environment, not the universal package's
permanent execution model.

An initial compatibility implementation may live in this repository to reduce
migration risk. That placement is a proving arrangement, not an architectural
dependency: core contracts must import neither ROS nor NAO packages and must be
portable without changing their meaning.

## Current Decision

The preferred package boundary is a future pure-Python `ab_harness` package in
the Neural Workbench research repository. It should depend on `skill_common`
instead of copying the AB registry. ROS nodes, Codex/Pi/OpenHands workers, and
served-model backends should integrate through adapters.

No runtime package has been extracted yet. The foundation document defines the
acceptance gates that must pass before moving code out of `chatbot_llm` or
`planner_llm`.

## First Implementation Slice

1. Stabilize `HarnessSpec`, `TaskSpec`, `InteractionModuleSpec`, `ModelProfile`,
   and `TraceEvent` schemas.
2. Add an AB task-projection compiler over `skill_common.ABRegistry`.
3. Extract provider-neutral structured-output and provider-capability helpers
   behind compatibility adapters.
4. Keep prompt text and NAO policy in their current owning packages.
5. Prove behavior parity with existing chatbot and planner tests.
6. Run same-model harness ablations before claiming harness uplift.
7. Transform `chatbot_llm` and `planner_llm` incrementally into harness clients,
   with their existing suites acting as behavioral parity gates.

The adaptive extension deliberately starts with release H0: one AB-grounded,
role-scoped, traceable agent path using existing model calls and runtime owners.
Candidate search, trace-derived priors, entropy, and crystallization are H1-H3
capabilities and are not required to prove the first useful harness.

## Rendering

Markdown remains canonical. Regenerate both polished HTML companions with:

```bash
python3 scripts/render_agentic_harness_docs.py
```

The shared theme lives under `docs/agentic_harness/assets/` and intentionally
matches the Neural Workbench extended formal masterplan visual language.
