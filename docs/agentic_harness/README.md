# Universal Agentic Harness

This subsection initializes the Universal Agentic Harness research and
implementation track. The project studies how to turn an LLM into a bounded,
observable subsystem agent by compiling task-specific interaction modules from
the Neural Workbench AB capability graph.

## Canonical Document

- `universal_agentic_harness_masterplan.md` (+ HTML) - canonical implementation
  plan, H0-H5 release spine, AB4/AB5 boundary, subsystem map, external-harness
  adoption decisions, seam-hypothesis audit, ablations, and ordered work queue.
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

The parent repository now contains a pure-Python `src/ab_harness` H0 contract
proof. It implements frame-relative role bands, a hashed read-only canonical
registry snapshot, task projection with decomposition closure, deterministic
output/reachability checks, append-only JSONL traces, and temporary NAO payload
adapters. Seven focused tests pass. It is not yet wired into live nodes, and the
full H0 lifecycle grammar remains incomplete. The masterplan separates that
implemented proof from H1 runtime and H2 cooperative integration work.

## H0 Completion Slice

1. Preserve the implemented frame, control-band, registry snapshot, projection,
   gate, trace, and NAO compatibility proof.
2. Add the missing `HarnessSpec`, `TaskSpec`, `ModelProfile`, environment,
   permission, effect-evidence, and lifecycle-event schemas.
3. Compile AB closure from required task effects rather than only requested IDs.
4. Keep prompt text and NAO policy in their current owning packages.
5. Complete a synthetic lifecycle replay before integrating any live node.
6. Prove behavior parity with existing chatbot and planner tests in H2 shadow
   and cooperative modes.
7. Run same-model harness ablations before claiming harness uplift.

The adaptive extension deliberately starts with release H0: one AB-grounded,
role-scoped, traceable agent path using existing model calls and runtime owners.
The canonical release spine is now defined in the masterplan: H1 adds the
executable lifecycle, H2 proves cooperative NAO parity, H3 adds trace adaptation,
H4 quarantines crystallization, and H5 proves cross-runtime/domain universality.

## Rendering

Markdown remains canonical. Regenerate both polished HTML companions with:

```bash
python3 scripts/render_agentic_harness_docs.py
```

The shared theme lives under `docs/agentic_harness/assets/` and intentionally
matches the Neural Workbench extended formal masterplan visual language.
