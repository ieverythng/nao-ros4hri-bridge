# Chatbot Target Selection SkillOpt Ledger

## Iteration 1

- **Date:** 2026-07-14
- **Target artifacts:**
  - `src/chatbot_llm/config/chat_prompt_pack.yaml`
  - `src/chatbot_llm/chatbot_llm/prompt_pack.py`
- **Objective:** Make the chatbot intent stage the declared author of bounded
  target selection for delivery and ordered visit requests. The planner handoff
  adapter should not need to reconstruct healthy-path scope from `goal_text`.
- **Train set:** grouped delivery from a support, explicit multi-object delivery
  to one person, ordered navigation with final report, and named-person
  delivery.
- **Holdout set:** simple wave, head motion plus report, current-scene KB query,
  missing recipient, future navigation discussion, reflective action question,
  and ordinary dialogue.
- **Acceptance gate:** the canonical intent schema and prompt define the bounded
  contract; focused prompt and chatbot tests pass; live train cases emit valid
  grounded selections without adapter derivation; holdout route and speech
  behavior remains stable after a clean rebuild.

## Baseline

| Case | Expected | Actual before mutation | Pass |
| --- | --- | --- | --- |
| Intent response schema | Optional bounded `target_selection` object | Field absent from `DEFAULT_INTENT_SCHEMA` | No |
| Canonical intent prompt | Intent owns grounded scope, operation, recipient, order, and report policy | Prompt defines only intent, goal text, scene targets, request kind, and sequence | No |
| Grouped delivery handoff | Model-authored selection passes through unchanged | Adapter reconstructs selection from goal text and grounded records | No |
| Explicit multi-object delivery | Model-authored member and recipient ids | Adapter reconstructs ids from scene targets and grounded records | No |
| Existing nested focused suite | Preserve current behavior | 180 tests pass | Yes |
| Current route/report holdouts | Preserve accepted runtime behavior | Accepted historical holdouts in `chatbot_llm_phase2_skillopt_2026-06-30.md`; clean rerun pending | Pending |

## Mutation Batch

1. **Add:** a reusable bounded target-selection JSON schema to the canonical
   intent response format.
2. **Add:** the same shape to the intent prompt example.
3. **Add:** one focused rule requiring grounded canonical ids for delivery and
   ordered visit selection, while preserving clarification when scope is not
   uniquely grounded.

No response-stage route rule, report wording, fallback renderer, or planner
prompt is changed in this iteration.

## Train Results

| Gate | Result |
| --- | --- |
| Canonical prompt/schema tests | Pass (5) |
| Missing-selection retry and primitive-motion control | Pass |
| Focused chatbot/adapter/grounding suite | Pass (189) |
| Clean overlay build and source-hash verification | Pass |
| Live grouped-delivery model output | Pass: explicit members, grounded ALEX recipient, final report policy |
| Ordered object visit | Pass: three canonical probe ids, sequential ordering, per-target reporting |
| Invalid intent JSON retry | Source pass; final retry-exhaustion salvage runtime holdout pending endpoint recovery |

## Holdout Results

Live grouped delivery, person lookup, head motion, KB dialogue, ordinary
dialogue, and ordered object visits passed. Missing synthetic operator identity
correctly clarified before execution. The final invalid-JSON retry-exhaustion
path is source-tested but was not rescored because the endpoint failed during
the clean v7 preflight.

## Decision

- **Status:** Core target-selection contract accepted; retry-exhaustion runtime
  holdout pending.
- **Rollback needed:** Yes if focused tests regress, live target selections are
  not grounded, or any protected live holdout changes route or speaking
  behavior. Do not commit the prompt mutation as runtime-accepted until the
  endpoint-backed holdout runs.

## Next Hypothesis

The turn engine now performs one validation retry with exact errors. When both
intent attempts fail for a quantified object request, it preserves
`retry_exhausted` provenance so the existing adapter derivation is eligible as
a traced Tier A salvage path. Ordinary dialogue and pure `llm` mode do not gain
that salvage behavior.
