# planner_llm Agent Notes

- Keep this package as planning/supervision owner only. Do not move execution logic here.
- Planner outputs must remain executable-step-only; user-facing completion wording belongs to `chatbot_llm` post-execution.
- Use canonical AB registry as source of truth for skill semantics and aliases.
- `communication_policy` is normalized in `planner_common`; keep resolution deterministic and documented.
- Any schema changes touching planner payloads must update `planner_common` contracts and tests in lockstep.
