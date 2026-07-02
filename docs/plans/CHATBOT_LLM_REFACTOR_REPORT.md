# chatbot_llm Refactor Report — Tighten & Re-own (Two-Phase Deslop)

*30 June 2026. Supervisor-facing summary of the `chatbot_llm` turn-engine
deslop: a behaviour-preserving structural split (Phase 1) plus a diagnose-first,
SkillOpt-gated reduction of deterministic over-reach (Phase 2).*

## At a glance

- **Primary objective:** make the self-authored `chatbot_llm` turn logic small,
  honestly-owned, and reactive — so the LLM leads each turn and deterministic
  fallbacks only *nudge*, instead of silently overriding the model
  ("reward-hacking").
- **Phase 1 status — DONE (behaviour-preserving).** The 2772-line
  `turn_engine.py` was split into four focused modules; dead code removed;
  **137 focused tests pass** as an equivalence oracle.
- **Phase 2 status — LIVE-VALIDATED.** Every override was classified
  keep/sharpen/delete, then the classification was tested against a **running
  NAO stack**. Outcome: the over-restrictive open-domain wording mutation is
  **rejected** (live answers are already warm), only the duplicated-label deslop
  is **applied** under a passed unit + live regression gate, and the route/fallback
  safety guards are **kept** because live `ROUTE_DECISION` evidence shows the
  overrides are beneficial nudges (16.7 % rate, all `knowledge_query→dialogue`).
- **Score posture:** the demo baseline (`response_first`, **8.2 qualified /
  8.7 success-path**) is held. Live smoke + `intent_ablation` pass on all
  scored seams with exactly-once speech and no false completion.

## Why this was safe to do here (ownership map)

`chatbot_llm` is a nested repo with an upstream (IIIA SocialMinds) remote, so
"narrow, compatibility-conscious changes" is the default. The files in scope are
**purely self-authored** (no upstream lineage), which licenses a full deslop.

| Tier | Files | Treatment |
| ---- | ----- | --------- |
| Self-authored (deslop freely) | `turn_engine.py`, `knowledge_snapshot.py`, `planner_request_adapter.py`, plus the new `route_heuristics.py` / `response_fallbacks.py` / `system_turn.py` | split, dedupe, delete dead code |
| Upstream-tracked but ours (seam-narrow only) | `node_impl.py` | imports kept byte-stable; untouched |
| Upstream-owned (do NOT deslop) | `llm_client.py`, `messages.py`, `response_parser.py`, `role_handlers.py`, `start_node.py`, `__init__.py` | untouched |

## The problem: a deterministic override cascade fighting the model

`turn_engine.py` had grown to **2772 lines** holding, in one module: ~20 text
classifiers, a ~210-line `_resolve_planner_mode_turn` override cascade that
re-decided the route after the LLM, the intent-first `_lock_intent_first_route`
mirror, and ~12 acknowledgement/report fallback builders.

The risk this created — the reason for this work — is that deterministic guards
were re-classifying turns the model may already have routed correctly, so turns
could **fail or converge onto canned fallbacks** rather than the LLM's own
answer. Fallbacks that *replace* the model instead of *nudging* it are a form of
reward-hacking: they make the harness look healthy while masking what the LLM
would do.

### Before

```
user_text
   -> _query_intent / _query_response (LLM)
   -> _resolve_intent
   -> _resolve_planner_mode_turn  (~10 stacked keyword/regex overrides
                                    rewriting route + user_intent)
   -> _lock_intent_first_route    (more overrides)
   -> _sanitize_* / _fallback_* / _postprocess_* ack rewriters
   -> TurnExecutionResult
```

All of the above lived in **one 2772-line file**.

### After

```
turn_engine.py        (1601)  orchestration: execute_turn, DialogueTurnEngine,
                              TurnExecutionResult, the named ordered route policy,
                              the Phase-0 ROUTE_DECISION trace
route_heuristics.py    (458)  pure text/route classifiers (_is_*, _looks_like_*,
                              _repair_response_route, route constants)
response_fallbacks.py  (694)  model-output parsing + bounded route-safe spoken
                              text (acks, execution-report wording)
system_turn.py         (369)  __system__ payload extractors + prompt-addendum
                              framing (planner completion/dialogue/report)
```

Dependency direction is a clean one-way DAG:
`route_heuristics  <-  response_fallbacks  <-  system_turn  <-  turn_engine`.
The largest single file dropped **~42%** (2772 → 1601, net of the relocated
helpers plus the added Phase-0 trace and route-policy docstrings), and every
module is now under 700 lines except the orchestrator, each with a single
responsibility.

## Seams built — what, why, how

| Seam | What it owns | Why it exists | How it stays safe |
| ---- | ------------ | ------------- | ----------------- |
| `route_heuristics` | pure functions that classify `user_text`/`verbal_ack` into a route or intent nudge | isolate the "nudge" vocabulary from orchestration so it is testable and obviously side-effect-free | no transport/state/speech; imports only `intent_rules` + `kb_skills` |
| `response_fallbacks` | parse model JSON into an ack; produce bounded, route-safe text only when the model output is empty/unsafe | keep anti-fabrication / anti-duplicate-speech wording in one place | never invents perception or success; only words what payloads assert |
| `system_turn` | normalise internal `__system__` payloads; structural prompt-addendum framing | keep internal wording turns from re-running user-facing route policy | provides framing only; canonical wording stays in the prompt pack |
| named route policy | `_resolve_planner_mode_turn` now documents its fixed ordered phases | make the override order explicit and reviewable | byte-identical; provably-identical pure dialogue guards collapsed into one |

## How it was implemented

- **Phase 0 — instrument (trace-only, no behaviour change).** Added a
  `ROUTE_DECISION` turn-trace record (`llm_route` vs `final_route` vs
  `overridden` vs `reason`) on both the response-first and intent-first paths,
  so the override rate between the model's own route and the resolved route is
  measurable. **Live capture done:** 24 decisions across smoke + `intent_ablation`,
  **4 overrides (16.7 %), all `knowledge_query→dialogue`** on conversational or
  reflective turns — i.e. the guard nudges over-eager KB routing back to chat
  rather than fighting a correct model decision.
- **Phase 1 — structural deslop (behaviour-preserving).** Moved the classifiers,
  fallbacks, and system-turn helpers into the three new modules; re-imported the
  five externally-referenced names (`DialogueTurnEngine`, `TurnExecutionResult`,
  `_system_task_response_addendum`, `_extract_ack_text`,
  `_looks_like_json_payload`) so `node_impl.py` and the tests resolve unchanged;
  removed two dead helpers (`knowledge_snapshot._first_non_empty_value`,
  `planner_request_adapter._normalized_intents`); made the route policy a named,
  documented, ordered sequence and collapsed the three provably-identical
  dialogue-first guards. The **135-case focused suite is the equivalence oracle**.
- **Phase 2 — gated reduction (classified, then live-validated).** Each override
  was classified keep/sharpen/delete with the bias "LLM leads, fallbacks only
  nudge", and a SkillOpt ledger (baseline → mutation batch → holdout gate →
  decision) was written. The live `intent_ablation` + smoke gate then decided
  each mutation:
  - **Applied:** collapse the duplicated grounded-context prompt label to a
    single `Grounded context for this turn:` header (+ updated the 3 test
    assertions that locked the duplicate). Gate: 137 unit tests pass and a
    clean-stack live smoke re-run reproduced every answer — including the
    `codex_probe_cup` gold-cup grounding — with no duplicate speech.
  - **Rejected:** softening the open-domain refusal wording. Live
    `favorite_movie` ("…I love learning about movies. What's your favorite?") and
    `wave_particle` are already warm/correct, so the mutation would be
    unjustified prompt churn.
  - **Kept:** all anti-fabrication / route-safety guards, because the live
    override evidence shows they only nudge over-eager `knowledge_query` routing.

## Evidence and validation

| Check | Result |
| ----- | ------ |
| `py_compile` of all touched + new modules | Pass |
| Focused chatbot_llm suite (intent_adapter, knowledge_snapshot, skill_catalog, turn_engine, planner_request_adapter, planner_handoff) | **137 passed** |
| External import surface (`node_impl` + tests resolve all names) | Pass |
| `flake8` (project settings) on touched files | No new E/F/W (only pre-existing, tolerated `Q000`/`D1xx`) |
| `scripts/ros4hri_change_audit.py --mode working` | Pass |
| Live preflight: refactor seams byte-identical host↔container; `dialogue_manager active [3]`; Phase-0 trace present | Pass |
| Live `intent_ablation` (response_first) — 7 named holdouts | **All pass** (favorite_movie warm, wave_particle no `wave_greet`, future_navigation defers, kb_visible projects `oro:isOn`, look_report truthful, head_wave whole-chain) |
| Live smoke — dialogue / KB-grounding P0 / simple+composite execution | **Pass** (gold-cup `rdf:type`+`dbp:name`+`dbp:color` grounded and answered) |
| Live exactly-once speech | **Pass** (8 distinct utterances, each emitted once) |
| Live Phase-2 regression (clean stack, dedup'd prompt) | **Pass** — answers equivalent to pre-fix, no duplicate speech |

The split is behaviour-preserving by construction: no method body in the
orchestration path was rewritten, only relocated; the oracle confirms identical
decisions, and the live run confirms the deployed stack behaves as scored.

Live evidence is in the SkillOpt ledger
[`docs/artifacts/chatbot_llm_phase2_skillopt_2026-06-30.md`](../artifacts/chatbot_llm_phase2_skillopt_2026-06-30.md#live-holdout-result-2026-06-30)
(`/robot-runtime-performance-review`, container `nao_ros2`, `iiia:nao`,
`response_first`, robot offline / object-detection off by design).

## Prompt / LLM contract discipline

Exactly one LLM-facing change was made, and it went through the gate first: the
SkillOpt ledger entry pre-existed the edit (honouring the root `AGENTS.md` rule
"do not patch prompt text first and create the ledger afterwards"). Only after a
live baseline existed was the **duplicated grounded-context label** collapsed to
a single `Grounded context for this turn:` header, with the three locking test
assertions (`test_turn_engine.py`) updated in the same change. The
over-restrictive open-domain wording mutation was **rejected** because the live
holdout proved the current wording is already warm. Decisions and evidence are
recorded in
[`docs/artifacts/chatbot_llm_phase2_skillopt_2026-06-30.md`](../artifacts/chatbot_llm_phase2_skillopt_2026-06-30.md#live-holdout-result-2026-06-30).

## Seams that could still be improved

These are the live-evidenced follow-ups, ordered by value. None block the
current pass; they are the next reactivity gains.

1. **Push KB-vs-dialogue disambiguation into the prompt, shrink the guard.** The
   single recurring override is `knowledge_query → dialogue` (16.7 % of turns).
   The model keeps over-tagging reflective/preference/future questions as KB
   lookups, and the guard repairs it. That is the *healthy* direction (nudge,
   don't fabricate), but the cleaner fix is to teach the prompt pack when a
   question is conversational vs a live world-state query, so the model stops
   emitting `knowledge_query` there and the guard becomes a rare safety net
   instead of a frequent corrector. Gate any such wording change with the same
   live `intent_ablation` holdout.
2. **Sharpen the Phase-0 trace.** `ROUTE_DECISION.reason` currently reports the
   `intent_source` (`llm_response_route`), not the *name of the guard* that
   caused the downgrade. Emitting the firing rule id would turn the override rate
   into a per-rule override rate and make the next reduction pass data-driven.
3. **Prefer `dbp:name` over the entity id in grounded answers.** For the probe
   cup the robot says the name is `codex_probe_cup` (the entity id) rather than
   `TITAS` (`dbp:name`). The fact is grounded correctly; only the surface choice
   of identifier is off. This is a `grounded_context` projection nuance, not a
   chatbot routing bug, and predates this refactor.
4. **`turn_engine.py` is still the largest module (1601 lines).** The remaining
   structural target is the behaviour-changing merge of `_resolve_planner_mode_turn`
   and `_lock_intent_first_route` that Phase 1 deliberately left apart; it needs
   its own gated pass because it can change routing.
5. **#6/#7 `not explicit_route` gating — deferred, not rejected.** The live run
   showed 0 wrongly-overridden execution turns (8/8 clean), so there was no harm
   to fix. Revisit only if a future run shows an explicit LLM execution route
   being downgraded.
6. **Operational: KB service resilience across restarts.** Aggressive process
   kills can leave stale FastRTPS shared-memory locks and duplicate
   `knowledge_core` nodes, which make `/kb/revise` time out (it produced one
   transient "I cannot confirm" cup answer mid-session). Prefer
   `docker restart nao_ros2` (clears `/dev/shm`) over `kill -9` loops when
   relaunching. Not a chatbot defect.
7. **Bake the dedup into the image.** The applied label change is live in
   `nao_ros2` and present in the host working tree; run `build_docker.sh` so the
   next `iiia:nao` image carries it permanently.
8. **Intent-first stays unpromoted.** The `_lock_intent_first_route` mirror is an
   unscored ablation; do not make it the default.
9. Update [`ISSUE_TRACKER_FULL_SUITE.html`](ISSUE_TRACKER_FULL_SUITE.html) with
   the live Phase-2 results captured here.
