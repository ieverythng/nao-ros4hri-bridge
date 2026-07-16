# Runtime Seam Deslop Preflight

**Date:** 2026-07-14
**Final candidate base:** `iiia:nao` (`bb62ec1a4589`)
**Last accepted live overlay:** `iiia:nao-deslop-20260714-v6`
**Result:** Main runtime accepted; final deep-fake retry holdout blocked by endpoint outage

## Source Provenance

The overlay built 22 packages from `iiia:nao`. The inspected host and container
source hashes were identical:

| Seam | SHA-256 |
| --- | --- |
| `planner_common/contracts.py` | `6f138f3da64a7de5a104684bd90c55357965ccc1d843ad04082cb082c3fd716b` |
| `planner_llm/planner_engine.py` | `32a76800486021df877285e8ce22d38193c9d79cffe61fad5d79dc2a292d3289` |
| `nao_orchestrator/kb_effects.py` | `ec760b6df5c0a2cf767708c25f1437df32a2b90c29a7ecb72da59663326eac4b` |
| `nao_orchestrator/orchestrator.py` | `03e61ee1a9de6ab5f0d22705402541a4d842e7ca1e4518c131105d12b8ea4fd7` |
| `chatbot_llm/turn_engine.py` | `06ab855b619a1ec43f2b289a7c3c6fb74127faf786394521699790e4935c3142` |
| `chatbot_llm/planner_request_adapter.py` | `ae392952c1344acfd6ef1a348e5da5e9323d13d227669fb8f77745c8c028ebe0` |
| `chatbot_llm/response_fallbacks.py` | `00465bd2201b9284061f0dab88ea485667ac74bf95717c97125aea54be58fa63` |
| `chatbot_llm/config/chat_prompt_pack.yaml` | `efab8ea5e4074e4764238043081049beba104d0e4d510aa8fb951e1d4c19c43b` |

## Preflight Evidence

- KnowledgeCore reported ready and exposed `/kb/query` and `/kb/revise`.
- `nao_orchestrator` was active.
- `fake_skill_server` was present once with the expected fake skill set.
- `chatbot_llm` remained `unconfigured` because its required model preflight
  failed.
- `dialogue_manager` remained `unconfigured` because chatbot configuration did
  not complete.
- `planner_llm` exited after three failed provider probes and was absent from
  the settled node graph.
- Direct host probe to `http://10.7.138.215:8004/v1/models` returned connection
  refused (`HTTP 000`).
- The snapshot recorded zero semantic fallback events because no semantic turns
  were injected. This is not evidence of fallback-free behavior.

Generated snapshot:
`/tmp/nao_deslop_preflight_20260714.json`

## Subsequent Runtime Evidence

The endpoint recovered after the initial v1 preflight. Clean v4 and canonical
`iiia:nao` launches then proved the main seams:

- all 14 cases completed before the default 420-second main-suite ceiling passed;
- the seven-case main tail produced five direct passes, and its two apparent
  failures were resolved by a targeted rerun and an oracle correction;
- explicit natural-language `kb_add`, add/revise/query/remove isolation, grouped
  delivery, head motion, person lookup, and ordered object visits passed;
- grouped delivery produced natural chatbot-owned closure with no
  `report_result` fallback;
- fixture cleanup ended with `contaminated=false` and no remaining facts;
- the rebuilt `iiia:nao` image matched all 56 changed runtime files and every
  inspected built Python module.

The speech harness initially retained several transient-local voice publishers.
That caused subscription churn and false injection failures. The harness now
keeps one active tracked-voice publisher, and the crossing probe from KB dialogue
to head motion passed both cases with zero fallback markers.

The final clean v7 launch was not scored. The vLLM endpoint returned connection
refused for all three planner preflight attempts and six direct host probes.
`planner_llm` failed closed as designed.

## Decision

The main runtime, KB mutation, grouped delivery, motion, and report-result seams
are accepted for the current source batch. The complete deep-fake success and
forced-navigation-failure suites remain pending. Reopen when
`http://10.7.138.215:8004/v1/models` responds, rebuild `iiia:nao` with the final
`chatbot_llm/turn_engine.py`, perform one clean full-container launch, and rerun
`fake_deep_ordered_walk_report` before the two complete deep-fake profiles.
