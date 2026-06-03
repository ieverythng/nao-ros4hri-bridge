# Thesis Artifact Pack

Date: 2026-06-03

This folder contains thesis-facing documentation artifacts generated from the
active NAO ROS4HRI planner stack. They are written to match the restrained
technical-memory style used by the supervisor walkthrough documents: A4 report
layout, formal title page, compact tables, and explicit runtime contracts.

## Artifacts

| Artifact | Purpose |
|---|---|
| `tfm_architecture_and_implementation_reference_2026-06-03` | Architecture, package ownership, runtime flow, and chapter integration material. |
| `tfm_runtime_contracts_and_semantics_2026-06-03` | Planner request/output, grounding, feedback, and dialogue-act contract reference. |
| `tfm_validation_protocol_2026-06-03` | Scenario matrix, metrics, ablations, validation gates, and experiment log template. |

Each artifact is generated as `.md`, `.html`, and `.pdf`.

## Regeneration

```bash
python3 scripts/generate_thesis_artifacts.py
```

The generator is intentionally local and conservative so future thesis passes can
refresh the artifact set without changing the visual language.
