# chatbot_llm Phase 2 — Override Classification & SkillOpt Ledger (2026-06-30)

This artifact backs the **behaviour-changing** Phase 2 of the `chatbot_llm`
deslop. Phase 1 (structural split + dead-code removal) is already merged and is
behaviour-preserving. Phase 2 reduces deterministic route/fallback over-reach so
that **the LLM leads and fallbacks only nudge** — and every LLM-facing wording or
route-policy change here is gated by a SkillOpt entry per the root `AGENTS.md`
"Prompt And LLM Contract Changes" rule.

> Environment note (authoring): at authoring time no live NAO container / LLM
> endpoint was running, so the mutations were prepared and classified but NOT
> applied, decision `DEFERRED — pending live holdout`.
>
> **Live update (2026-06-30, later same day): the container is up and the live
> holdout has been run.** See [Live holdout result](#live-holdout-result-2026-06-30).
> Net outcome: the open-domain wording mutation is **REJECTED** (live evidence
> shows no refusal problem), and only the duplicated-label deslop (#15) is
> **ACCEPTED** under a passed unit + live regression gate. The route/fallback
> safety guards stay **KEPT** because the live `ROUTE_DECISION` evidence shows
> the overrides are beneficial nudges, not reward-hacking.

## Demo/thesis baseline to protect

- `response_first` is the demo default at **8.2 qualified / 8.7 success-path**.
- `intent_first` is an unscored ablation that regressed open-domain wording — it
  stays unpromoted.
- Named regression holdouts that MUST keep their current outcome:
  `intent_ablation_favorite_movie`, `intent_ablation_wave_particle`,
  `intent_ablation_future_navigation` ("Could we navigate to the probe cup
  later?"), `intent_ablation_look_report` ("How many directions did you move your
  head?"), `intent_ablation_kb_visible` (non-mutating KB question), and the
  multi-bring delivery (`composite_walk_every_object_reports_main`).

## Override classification (keep / sharpen / delete)

Bias: keep guards that prevent **fabricated execution / fabricated perception /
false completion / duplicate speech** (these encode real safety, not reward
hacking). Sharpen or delete guards that *re-decide a route the LLM already got
right*, moving that policy into the prompt pack where it belongs.

| # | Override (module) | Purpose | Verdict | Rationale / gate |
| - | ----------------- | ------- | ------- | ---------------- |
| 1 | `_is_reflective_execution_question` → dialogue (`turn_engine`/`route_heuristics`) | "what did you do?" must not re-execute | **KEEP** | Prevents false re-execution; protects `intent_ablation_look_report`. |
| 2 | greeting / short-social / capability → dialogue (collapsed guard) | keep social openers conversational | **SHARPEN** | Route is correct, but this is exactly what the prompt pack already states; keep Python as a *nudge only when route is missing*, let the LLM own it otherwise. |
| 3 | `_is_personal_preference_question` → dialogue + reset intent | "what's your favourite movie?" | **SHARPEN** | Route guard is fine; the **prompt wording** around refusing/avoiding open-domain chat is the over-restrictive part flagged in the tracker. Soften prompt, keep route nudge. Gate: `intent_ablation_favorite_movie`. |
| 4 | `_is_information_only_action_word_question` → dialogue | "explain wave–particle duality" must not fire `wave_greet` | **KEEP** | Genuine safety guard against action-verb collisions; protects `intent_ablation_wave_particle`. |
| 5 | repeat-action → execution | "do it again" | **KEEP** | Small, concrete, evidence-justified. |
| 6 | ack-implies-execution backfill → execution (no explicit route) | model spoke an action promise but routed dialogue | **SHARPEN** | Prime "fighting the model" surface; restrict to the *route-missing* case and let an explicit LLM route win. |
| 7 | rules-fallback execution detection → execution | LLM under-routed an action | **SHARPEN** | Keep only when LLM gave no usable route/intent. |
| 8 | `_repair_response_route` / `_route_is_contradictory` | route contradicts the spoken ack | **KEEP** | Route-safety + duplicate/false-completion prevention. |
| 9 | fallback intent backfill (`detect_intent`) | fill empty intent | **KEEP** | Bounded, only when intent empty. |
| 10 | knowledge-query guard (`_infer_kb_query_intent_from_text`) | route KB questions to `knowledge_query` | **KEEP** | Protects `intent_ablation_kb_visible`; non-mutating. |
| 11 | non-immediate action discussion → dialogue | "could we navigate … later?" | **KEEP** | Protects `intent_ablation_future_navigation`. |
| 12 | `_lock_intent_first_route` mirror guards | intent-first projection | **KEEP (unpromoted)** | Ablation only; do not promote `intent_first`. |
| 13 | `_sanitize_execution_ack`, `_postprocess_execution_report_ack` | strip premature/false completion + duplicate report wording | **KEEP** | Core anti-fabrication / anti-duplicate-speech seam. |
| 14 | `_fallback_*` ack builders (`response_fallbacks`) | bounded wording when LLM output empty/unsafe | **KEEP (nudge-only)** | They already fire only when the model returns nothing usable — this is the desired "nudge, don't replace" posture. |
| 15 | duplicated grounded-context label (`prompt_builders._knowledge_snapshot_block`) | prompt emits `Grounded context for this turn:` then `Grounded context:` | **DELETE (gated)** | LLM-facing text; **test-locked** at `test_turn_engine.py:180-183,352`. Proposed mutation below. |

## SkillOpt iteration (proposed, gated)

- Date: 2026-06-30
- Target artifacts:
  - `src/chatbot_llm/config/chat_prompt_pack.yaml` (soften over-restrictive
    open-domain refusal wording; absorb the "social/greeting → dialogue" and
    "uncertain no-action → dialogue" route policy as the canonical owner)
  - `src/chatbot_llm/chatbot_llm/prompt_builders.py` (remove the duplicated
    `Grounded context:` label)
  - `src/chatbot_llm/chatbot_llm/turn_engine.py` (restrict overrides #2/#6/#7 to
    the route-missing case once the prompt pack owns the policy)

### Baseline (current, before any mutation)

| Case | Expected | Actual (current) | Pass |
| ---- | -------- | ---------------- | ---- |
| `intent_ablation_favorite_movie` | warm open-domain answer, dialogue route | dialogue route enforced by guard #3; wording can sound refusal-like | Partial |
| `intent_ablation_wave_particle` | explain physics, no `wave_greet` | dialogue route (guard #4) | Pass |
| `intent_ablation_future_navigation` | acknowledge, no immediate execution | dialogue (guard #11) | Pass |
| `intent_ablation_kb_visible` | `knowledge_query`, no KB mutation | `knowledge_query` (guard #10) | Pass |
| grounded-context prompt label | single clear label | duplicated label emitted | Fail (slop) |

### Mutation batch (NOT yet applied — pending live gate)

1. `prompt_builders`: collapse the grounded-context block to a single
   `Grounded context for this turn:` label and update the three test assertions
   that encode the duplicate.
2. `chat_prompt_pack.yaml`: replace refusal-leaning open-domain wording with
   "answer warmly and stay in dialogue" wording; add the social/uncertain →
   dialogue route rule as canonical prompt policy.
3. `turn_engine`: gate overrides #2/#6/#7 on `not explicit_route` only, so an
   explicit LLM route is never overridden once the prompt owns the policy.

### Holdout gate (definition)

- Unit holdouts (runnable here): `test_turn_engine.py` route-safety,
  KB-query, greeting, execution-admission, and report-result cases;
  `test_prompt_pack.py`; `test_knowledge_snapshot.py`.
- Live holdouts (pending container): `/robot-runtime-performance-review`
  `--case-set main` and `intent_ablation`; score must stay ≥ 8.2/8.7 with no
  duplicate speech, raw planner leakage, or false completion.

### Unit-holdout result (current tree, Phase 1 applied, Phase 2 NOT applied)

| Suite | Result |
| ----- | ------ |
| `test_turn_engine.py` (79) | Pass |
| `test_intent_adapter.py` + `test_knowledge_snapshot.py` + `test_skill_catalog.py` + `test_planner_request_adapter.py` + `test_planner_handoff.py` | Pass (135 total with turn_engine) |

### Decision (superseded by the live holdout below)

- *Authoring-time:* **DEFERRED — pending live holdout.** Staged, not applied.
- *Final:* see [Live holdout result](#live-holdout-result-2026-06-30). Only #15 is
  applied; #2/#3 wording mutation is rejected; safety guards kept.

## Live holdout result (2026-06-30)

Stack: `nao_ros2`, `iiia:nao`, `chatbot_turn_pipeline_mode=response_first`,
`start_naoqi_driver:=false` (robot offline), `start_object_detection:=false`
(intentional), LLM endpoint `http://10.7.138.215:8004` (HTTP 200). Refactor
seams byte-identical host↔container before the run.

### Phase 0 override-rate evidence (live `ROUTE_DECISION`)

Aggregated across `smoke` + `intent_ablation` (`response_first`):

| llm_route → final_route | overridden | count |
| ----------------------- | ---------- | ----- |
| dialogue → dialogue | no | 6 |
| execution → execution | no | 8 |
| knowledge_query → knowledge_query | no | 6 |
| knowledge_query → **dialogue** | **yes** | 4 |

- **Override rate ≈ 16.7 % (4 / 24), every override is the same shape:**
  `knowledge_query → dialogue` on conversational/reflective/preference turns the
  model over-tagged as a KB lookup (e.g. "How many directions did you move your
  head?", "What is your favourite movie?", "Could we navigate … later?").
- These overrides are **beneficial nudges, not reward-hacking**: they keep
  history/preference questions in dialogue instead of firing an empty KB query.
  This is direct evidence for the **KEEP** verdicts on guards #1/#3/#10/#11.
- Observability seam: the Phase 0 `reason` field currently reports
  `intent_source` (`llm_response_route`), not the *name of the rule* that caused
  the downgrade. Sharpening it to emit the firing guard id is a cheap follow-up.

### Named-holdout outcomes (live `intent_ablation`, response_first)

| Holdout | Live answer / behaviour | Verdict |
| ------- | ----------------------- | ------- |
| `favorite_movie` | "That's a fun question! As an AI, I don't have personal preferences, but I love learning about movies. What's your favorite?" — warm, invites continuation | **Pass — warm, NOT refusal** |
| `wave_particle` | correct physics explanation, **no `wave_greet` fired**, no planner | Pass |
| `future_navigation` | "Yes, we can plan that later when you ask me to do it." — defers, no execution | Pass |
| `kb_visible` | "I can see a person and a cup on the codex_probe_table." — relation `oro:isOn` projected, no KB mutation | Pass |
| `look_report` | planner dispatch + truthful "looked at the probe cup, which is gold and located on the probe table" | Pass |
| `head_wave` (composite) | one ack + whole-chain completion "moved my head up … then performed a friendly wave" | Pass |
| KB-grounding P0 (clean stack) | injected `codex_probe_cup` (`rdf:type Cup`, `dbp:name TITAS`, `dbp:color gold`) confirmed via `/kb/query` → "The probe cup is named codex_probe_cup and its color is gold." | Pass |
| duplicate-speech | smoke: 8 distinct utterances, **each emitted exactly once** | Pass |

The `favorite_movie` baseline was previously scored `Partial` ("wording can
sound refusal-like"). **Live evidence contradicts that concern** — the answer is
warm and conversational. The over-restrictive-wording premise behind mutation
#2/#3 does not reproduce on the live stack.

### Mutation accept/reject (final)

| Mutation | Gate | Decision |
| -------- | ---- | -------- |
| #15 collapse duplicated `Grounded context:` label → single `Grounded context for this turn:` (+ 3 test assertions) | 137 focused unit tests pass; clean-stack live smoke re-run: all answers incl. gold-cup grounding equivalent to pre-fix, no duplicate speech | **ACCEPTED & APPLIED** |
| #2/#3 soften open-domain refusal wording in `chat_prompt_pack.yaml`; migrate social/uncertain→dialogue policy into the pack | live `favorite_movie`/`wave_particle`/greeting all warm & correct already | **REJECTED — not needed** (avoids unjustified prompt churn) |
| #6/#7 gate ack-implies-exec / rules-fallback overrides on `not explicit_route` | live override evidence shows execution routes are never wrongly overridden (0 execution overrides in 8) | **DEFERRED — no observed harm**; revisit only if a future run shows an execution turn wrongly downgraded |
| safety guards #1,#4,#5,#8,#9,#10,#11,#13,#14 | live ROUTE_DECISION + holdouts | **KEEP** |

Applied-change provenance: `prompt_builders.py` line 164 now emits a single
label; verified live in `nao_ros2` (`docker cp` into the running container, then
a full stack restart + clean smoke). The host working-tree source carries the
same change, so the next `build_docker.sh` bakes it into the image.

> Operational note: the live session required a stack restart. Aggressive
> `kill -9` of the prior launch left stale FastRTPS shared-memory port locks and
> duplicate `knowledge_core` nodes, which made `/kb/revise` time out and produced
> a transient "I cannot confirm the name or color" cup answer. A `docker restart
> nao_ros2` (clears `/dev/shm`) + single clean relaunch resolved it; the cup then
> grounded correctly. This was a restart artifact, **not** a prompt regression.

## Validation commands

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
PYTHONPATH="src/planner_common:src/kb_skills:src/chatbot_llm" \
  python3 -m pytest -q -p no:cacheprovider \
  src/chatbot_llm/test/test_turn_engine.py \
  src/chatbot_llm/test/test_intent_adapter.py \
  src/chatbot_llm/test/test_knowledge_snapshot.py \
  src/chatbot_llm/test/test_skill_catalog.py \
  src/chatbot_llm/test/test_planner_request_adapter.py \
  src/chatbot_llm/test/test_planner_handoff.py

# Live gate (run when the NAO container is up):
# python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py --case-set main
# python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py --case-set intent_ablation
```
