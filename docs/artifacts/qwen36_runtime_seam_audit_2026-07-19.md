# Qwen3.6 Runtime Seam Audit, 19 July 2026

## Target contract

- Problem: qualify `QuantTrio/Qwen3.6-35B-A3B-AWQ` as the dialogue, intent, and planner model, then identify stable generation and timeout parameters using a complete scored runtime suite.
- Required outcome: one model-provenance-safe run with active lifecycle nodes, valid KnowledgeCore state, correlated speech/trace/log evidence, no false completion, and recorded latency/fallback metrics.
- Non-goals: changing prompt policy without SkillOpt, moving ROS ownership, scoring detector recognition in the cool profile, or treating external endpoint replacement as model behavior.
- Protected seams and owners: `dialogue_manager` speaks, `chatbot_llm` builds dialogue and routing requests, `planner_llm` plans, `nao_orchestrator` validates and executes, and `kb_skills` owns KnowledgeCore transport.
- Acceptance gate: `/v1/models` and completion response both identify Qwen3.6; startup and keepalive preflights pass; the main suite completes without a harness timeout; targeted composite, KB stress, fake-skill failure/recovery, and three robustness repetitions are manually reconciled.
- Evidence budget: one baseline, a bounded parameter matrix, one locked full run, and at most two source-fix rebuild rounds before bounded handoff.

## Baseline evidence

| Observation | Source or command | Expected | Actual | Evidence reference |
| --- | --- | --- | --- | --- |
| Qwen3.6 provenance at startup | chatbot/planner startup logs, 08:00 CEST | Requested model is served | Both realistic preflights passed; chatbot inventory listed only Qwen3.6 | `/tmp/nao_qwen36_valid_window.log` |
| Endpoint provenance after 08:30 | `GET /v1/models` and direct completion | Qwen3.6 remains served | Endpoint advertises Qwen3-VL; Qwen3.6 returns HTTP 404 | `/tmp/nao_qwen36_baseline_snapshot.json` and direct probe output |
| Third blocker audit, 11:14 CEST | `GET /v1/models`, container and image inventory | Qwen3.6 is restored and v31 can launch | Only Qwen3-VL is advertised; no stack container is running; v31 remains available at `sha256:3859d1414627` | terminal probe, 19 July 2026 |
| Long-history transport | live turn 6 and later | One leading system role | HTTP 400: `System message must be at the beginning.` | container logs at 08:06:07, 08:19:44, 08:20:26 |
| Preflight semantics | `node_impl.py` | Warm backend without mutating dialogue | Stateless readiness probes; no session history or grounded-context seeding | `src/chatbot_llm/chatbot_llm/node_impl.py` |
| Runtime pressure | snapshot and rosout | Stable perception identities | face detector skips 100 images every 5 to 7 seconds and person IDs churn | `/tmp/nao_qwen36_baseline_snapshot.json` |

## Approach registry

| ID | Family | Mechanism | Affected seams | Discriminating probe | Expected observation | Status | Exact gap or reopen condition |
| --- | --- | --- | --- | --- | --- | --- | --- |
| H-01 | Transport contract | A periodic identity reminder creates a mid-history `system` message rejected by the Qwen3.6 chat template | chatbot history to OpenAI transport | Capture sixth-turn outbound roles and reproduce with a unit test | Two system roles before fix; one leading role after fix | accepted | Live proof requires v31 plus restored Qwen3.6 endpoint |
| H-02 | External runtime | vLLM endpoint was replaced while ROS nodes retained Qwen3.6 parameters | endpoint and preflight/keepalive | Compare startup model inventory, current `/v1/models`, and named completion | Startup Qwen3.6 succeeds; current named request returns 404 | accepted | Reopen scoring when `/v1/models` identifies Qwen3.6 |
| H-03 | Model parameters | Current token budgets or sampling values cause truncation, invalid JSON, or latency | dialogue, intent, planner | Bounded matrix over temperature and output budgets with frozen cases | A setting improves validity/latency without semantic loss | blocked | Qwen3.6 must be served continuously during all cells |
| H-04 | Context initialization | Preflight should seed grounded context into each dialogue | chatbot lifecycle and session history | Inspect preflight call path and first-turn trace | Preflight is stateless and must not own dialogue state | rejected | Reopen only if first-turn traces omit the current grounded projection |
| H-05 | Grounding freshness | face-processing overload and person churn destabilize recipient grounding | perception, person manager, KB | compare fixture-only and live-person suites while recording churn | fixture cases stable while live IDs change | active | Requires target-model run with separate deterministic and live-person windows |
| H-06 | Harness/observability | questionnaire timeout or duplicated trace mirrors create false negatives/count inflation | review harness | compare lineage-scoped JSONL to raw logs and speech topics | semantic status survives mirror duplication; missing evidence is not scored | active | Complete manual reconciliation on the locked full run |
| H-07 | Structured decoding | prompt-only JSON may be less reliable than vLLM schema-constrained decoding | chatbot intent and planner provider | compare the locked sampling cell with and without schema-constrained decoding | lower invalid-JSON rate without semantic field loss | candidate | Run only after sampling and token budgets are locked so effects are separable |

## Discriminating probes and results

1. Startup logs prove Qwen3.6 was served and passed realistic chatbot/planner probes at 08:00. Current inventory and direct completions prove the server later changed models. This rejects a static ROS model-name typo.
2. Live failures begin when history contains the periodic identity reminder. A public-seam regression test reproduced roles `system,user,assistant,system,user`.
3. `chat_history.trim_messages` now folds all system instruction content into exactly one leading system message. The red test failed with two system roles, then passed after the change. The focused chatbot suite reports 121 passing tests, and Python compilation succeeds.
4. Preflight invokes isolated transport probes and never accesses or mutates `_DialogueSession.history`. It warms backend execution and validates readiness but does not seed a user's conversational context. Each real turn independently builds a fresh KnowledgeCore projection before its LLM request.
5. Image `iiia:nao-runtime-v31-qwen36-sampling-ablation` (`3859d1414627`) was rebuilt cleanly from `iiia:nao`; 25 packages built successfully. It includes the system-role fix and launch-configurable temperature, top-p, top-k, min-p, presence penalty, and repetition penalty for chatbot and planner. It has not yet been scored because the target model is absent from the endpoint.
6. The active questionnaire now captures typed runtime values for the chatbot and planner model, provider, sampling tuple, token budgets, timeouts, and thinking flag in every artifact's `runtime_metadata`. This closes the provenance gap between an ablation label and the configuration actually loaded by ROS. The full runtime-review script suite reports 69 passing tests.

## Parameter ablation contract

Use deterministic fixtures and independent group-scoped voices. Do not adapt prompts between cells.

| Phase | Chat temperature | Top-p | Response / intent tokens | Planner temperature / tokens | Timeouts | Purpose |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| A0 baseline | 0.2 | 0.9 | 192 / 256 | 0.1 / 800 | response 60, first 75, intent 10, planner 60 | reproduce current behavior |
| A1 Qwen baseline | 0.7 | 0.8 | 192 / 256 | 0.7 / 800 | unchanged | test the published non-thinking sampling profile with top-k 20, min-p 0, presence penalty 1.5, repetition penalty 1 |
| A2 deterministic | 0.0 | 1.0 | 192 / 256 | 0.0 / 800 | unchanged | isolate sampling variance and JSON validity |
| A3 bounded output | selected A0/A1/A2 | selected | 256 / 256 | selected / 1024 | unchanged | test truncation on maximal reports/plans |
| A4 timeout floor | selected | selected | selected | selected | response 30, first 60, intent 20, planner 45 | distinguish realistic latency from oversized waits |

Each cell runs fixed dialogue, KB query, grounded execution, maximal composite, and planner JSON cases. Record completion tokens, finish reason, response/intent/planner latency, invalid JSON, transport fallback, route repair, and semantic status. Advance only if the cell has zero transport/provenance failures. After sampling, budget, and timeout are locked, run one schema-constrained decoding comparison for intent/planner JSON. The locked configuration then runs the complete review ladder.

## Adversarial audit

- [x] Ownership boundaries remain unchanged.
- [x] No ROS interface or topic/service/action choice changed.
- [x] Goal, plan, version, and step lineage are unaffected by the history normalization.
- [x] No speech authority or executable plan was added.
- [x] No perception, KB, result, or completion claim was introduced.
- [x] Registry and AB levels are outside this change.
- [ ] Success and failure/recovery paths require live v31 evidence.
- [x] The nested `chatbot_llm` change is narrow and transport-focused.

## Decision: bounded handoff

- Chosen route: accept H-01 at source level and H-02 as the current external blocker; keep H-05 and H-06 active for the full run.
- Evidence satisfying the source gate: failing then passing transport tests, 290 chatbot behavioral tests, 101 planner tests, 7 launch-profile tests, 69 runtime-review harness tests, compilation, ROS4HRI ownership audit, and clean v31 image build.
- Residual risk: no live proof that Qwen3.6 accepts the folded message, no parameter comparison, and no complete scored suite while the server advertises Qwen3-VL.
- Reopen condition and next probe: `/v1/models` must again list `QuantTrio/Qwen3.6-35B-A3B-AWQ` and a named completion must succeed. Then launch v31, verify uniqueness/lifecycle/preflight, run A0 through A4, lock parameters, and execute the full runtime ladder.
