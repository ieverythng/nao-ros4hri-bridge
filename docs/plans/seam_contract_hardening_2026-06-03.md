# Seam Contract Hardening Pass

**Date:** 2026-06-03  
**Branch context:** `feat/TFM-LLM_planner` with selective intake from `refactor/deslop_repo` and `refactor/promptpack_llms`  
**Status:** Implemented; targeted tests passed, GitNexus refresh still needs follow-up

## Summary

This pass hardens the robot-facing LLM seams so planner, chatbot, orchestrator,
and skill adapters reason over one compact contract instead of overlapping legacy
fields. The main behavior target is predictability: fewer duplicated grounding
facts, clearer look-at dispatch policy, and contract rules owned by shared
planner code rather than repeated inside runtime packages.

## Grounded Context Contract

- `grounded_context.entities` is the LLM-facing visible world.
- `class` is the canonical entity type field.
- `relations` carries extra KB facts only; it must not repeat `rdf:type` when
  that value is already present as `class`.
- `counts` and `entity_counts` are removed from prompt/planner-facing grounded
  context. Consumers should inspect the bounded `entities` or `references`
  arrays directly.
- `state_t0` remains optional and disabled by default for prompts unless the
  existing runtime flag enables it.

Example:

```json
{
  "grounded_context": {
    "entities": [
      {
        "id": "cup_jrjic",
        "label": "cup",
        "kind": "object",
        "class": "Cup",
        "visible": true,
        "relations": [
          {"predicate": "dbp:color", "object": "blue"},
          {"predicate": "oro:isOn", "object": "table_1"}
        ]
      }
    ]
  }
}
```

## Implementation Tracks

- **Shared contracts:** keep grounding normalization, report-result validation,
  and numeric coercion in `planner_common` so planner/orchestrator packages do
  not carry duplicated private contract logic.
- **Chatbot handoff:** emit grounded context without `counts` or duplicate type
  relations; prompt wording must reference `entities[].class` and extra
  `relations` only.
- **Planner prompt pack:** update YAML and fallback prompt text using a bounded
  SkillOpt-style mutation so the LLM expects compact KB format, no count fields,
  and supported `look_at` policy arguments.
- **Look-at seam:** accept targetless `look_at` policies `auto`, `social`,
  `random`, and `reset` through the existing `/skill/look_at` action seam.
  Target tracking still requires `target_frame`.
- **Docs:** keep this plan and the masterplan as active documentation surfaces;
  update grounding examples in related plan walkthroughs when the contract
  changes.

## SkillOpt Prompt Gate

Target artifacts:

- `src/planner_llm/config/planner_prompt_pack.yaml`
- `src/planner_llm/planner_llm/prompt_pack.py`
- prompt/turn tests in `src/planner_llm/test/` and `src/chatbot_llm/test/`

Objective:

- Reduce prompt ambiguity around compact KB grounding, `look_at` policy dispatch,
  and speech/execution ownership.

Train scenarios:

- visibility-only scene question
- explicit scan/report request
- look_at person/object target
- targetless look_at policy
- ambiguous target

Holdout scenarios:

- greeting-only social turn
- motion plus report
- report_result after live scan
- planner backend failure
- unsupported target

Acceptance gate:

- No prompt mutation is accepted if tests reintroduce `counts`, duplicate
  same-class `rdf:type`, `say` inside executable plans, or duplicate
  planner/chatbot speech ownership.

## Validation

- `python3 scripts/ros4hri_change_audit.py`
- `python3 -m pytest` for touched planner_common, planner_llm, chatbot_llm,
  interaction_trace_viewer, nao_orchestrator, and nao_look_at tests.
- `python3 -m py_compile` for touched Python modules.
- Re-run `tools/knowledge/index_repo.sh` after the pass. The first refresh
  attempt exited non-zero after analyzer parser output, and `tools/knowledge/status.sh`
  still reported stale index metadata.

## Notes

- `refactor/deslop_repo` supplied the helper-consolidation direction and tracking
  rationale.
- `refactor/promptpack_llms` supplied the look-at policy direction; the large
  launch TUI changes remain out of this pass.
- ROS4HRI ownership remains unchanged: chatbot routes, planner plans,
  orchestrator dispatches, and NAO skill packages execute robot-adapter behavior.
