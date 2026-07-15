# Report Result Fallback SkillOpt Ledger

## Iteration 1

- **Date:** 2026-07-14
- **Target artifact:**
  `src/chatbot_llm/chatbot_llm/response_fallbacks.py`
- **Objective:** Preserve chatbot-first natural reports while making the
  emergency delivery renderer complete and evidence-specific.
- **Train set:** grouped three-object delivery, recipient excluded from object
  scope, clarification continuation with two delivered objects, and malformed
  chatbot output that omits completed targets.
- **Holdout set:** ordered navigation, simple wave, head-motion chain,
  intermediate report, partial delivery, surface placement, and accepted
  natural collective delivery wording.
- **Acceptance gate:** valid chatbot reports pass unchanged; invalid or missing
  chatbot reports render all completed delivery objects and the grounded
  recipient; no generic success is emitted without object evidence; focused
  report tests and live holdouts pass after a clean rebuild.

## Baseline

| Case | Expected | Actual before mutation | Pass |
| --- | --- | --- | --- |
| Three-object delivery fallback | Name cup, manual, phone, and ALEX | `I completed the delivery.` | No |
| Recipient excluded from object scope | Name only delivered objects and recipient | `I completed the delivery.` | No |
| Clarification continuation | Recover delivered objects from successful steps | `I completed the delivery.` | No |
| Partial delivery | Do not claim full completion | `I could not complete the full delivery.` | Yes |
| Valid collective model report | Preserve natural model wording | Preserved | Yes |
| Ordered navigation fallback | Name visited targets and arrivals | Preserved | Yes |

## Mutation Batch

1. **Add:** one evidence-based delivery renderer over `report_outcome`, with a
   successful-step fallback when the structured outcome is absent.
2. **Replace:** generic successful delivery fallback with the evidence renderer.
3. **Delete:** goal-text-only generic completion from execution-report fallback.

No canonical prompt, system-turn addendum, validator rule, or normal chatbot
report path changes in this iteration.

## Train Results

| Gate | Result |
| --- | --- |
| Grouped three-object fallback | Pass, names cup, manual, phone, and ALEX |
| Recipient exclusion | Pass, recipient is not reported as an object |
| Clarification continuation | Pass, recovers both completed objects from steps |
| Focused chatbot suite | Pass (189) |

## Holdout Results

Ordered navigation, intermediate report, partial delivery, surface placement,
motion-chain, and accepted collective wording source holdouts pass. Live
holdouts are `not_scored` because model preflight failed before activation.

## Decision

- **Status:** Source-accepted, runtime acceptance pending.
- **Rollback needed:** Yes if valid chatbot wording is replaced, partial failure
  becomes success, ordered navigation regresses, or any object/recipient is
  fabricated.

## Next Hypothesis

If emergency rendering becomes rare and complete, simplify remaining lexical
validators only after runtime traces identify rules that never prevent a
factual error.
