# AB Harness H0

Pure-Python proof of the first Universal Agentic Harness release. The package is
staged in the parent NAO repository for fast parity testing but its core
contracts contain no ROS or NAO imports.

## Implemented

- frame-relative `ABControlBand` and role contracts;
- read-only canonical AB registry snapshot with content hash;
- task-scoped projection with inspectable decomposition closure;
- deterministic output type, object reachability, AB-level, and effect-claim
  gate;
- append-only JSONL trace records;
- temporary NAO chatbot/planner role and payload adapters.

## Boundary

`chatbot_llm`, `planner_llm`, Neural Workbench, and runtime packages are not
modified or imported. `nao_h0.py` describes compatibility only. It does not
publish ROS messages, call skills, change prompts, or replace existing
validation.

## Test

```bash
PYTHONPATH=src/ab_harness .venv/bin/python -m pytest -q src/ab_harness/test
```
