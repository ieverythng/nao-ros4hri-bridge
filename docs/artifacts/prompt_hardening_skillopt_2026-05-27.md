# Prompt Hardening SkillOpt Log (2026-05-27)

## Iteration 1

- Date: 2026-05-27
- Target artifact(s):
  - `src/chatbot_llm/config/chat_prompt_pack.yaml`
  - `src/chatbot_llm/chatbot_llm/prompt_pack.py`
  - `src/planner_llm/config/planner_prompt_pack.yaml`
  - `src/planner_llm/planner_llm/prompt_pack.py`
- Objective:
  - Reduce planner handoff on greeting-only turns.
  - Prevent planner from inferring `wave_greet` on social-only input.
  - Keep execution acknowledgements from sounding like completion utterances.
- Train set:
  - `Hey Pop!` should stay dialogue route.
  - `Hello` should not infer `wave_greet`.
  - `Please wave at me` should still allow execution route.
  - `Do you see anyone right now?` should prefer `knowledge_query`.
- Holdout set:
  - `Can you scan the scene and tell me if you see anyone?` should remain execution-capable.
  - `Move your head right and then sit down` should remain execution-capable.
  - Planner invalid/mixed-step retry guardrails should remain intact.
- Acceptance gate:
  - Targeted prompt/route/planner tests pass.
  - No regressions in greeting + scan guardrail tests.

## Baseline

| Case | Expected | Actual (before) | Pass |
| --- | --- | --- | --- |
| Greeting-only social turn | dialogue-first | covered by route logic, but prompt text still permissive | Partial |
| Social text in planner | clarify/no inferred wave | no explicit prompt ban on wave inference from greeting | Fail |
| Execution ack wording | intent-to-act, no completion claim | not explicitly constrained in prompt packs | Partial |

## Mutation Batch

1. `add` chatbot route constraints:
   - greeting/social default to `dialogue` unless explicit action verb.
   - uncertain no-action turns default to `dialogue`.
   - execution `verbal_ack` should acknowledge intent-to-act, not completion.
2. `add` chatbot intent preference:
   - prefer `greet` for social turns over execution-like fallback labels.
3. `add` planner social guardrail:
   - forbid inferring `wave_greet` for greeting-only text.
   - require `decision=clarify` for social-only requests without explicit action.
   - mirror guardrails in fallback defaults.

## Train Results

| Case | Before | After | Delta |
| --- | --- | --- | --- |
| Greeting route guardrails in chatbot prompt packs | Partial | Explicit | Improved |
| Planner social spillover guardrails | Fail | Explicit | Improved |
| Execution ack wording boundary | Partial | Explicit | Improved |

## Holdout Results

| Case | Before | After | Delta |
| --- | --- | --- | --- |
| Planner scan/retry behavior tests | Pass | Pass | Stable |
| Chatbot planner-mode/knowledge-query/greeting tests | Pass | Pass | Stable |
| Prompt-pack parsing defaults/overrides | Pass | Pass | Stable |

## Validation Commands

```bash
python3 -m py_compile \
  src/chatbot_llm/chatbot_llm/prompt_pack.py \
  src/planner_llm/planner_llm/prompt_pack.py

PYTHONPATH=src/chatbot_llm:src/planner_llm:src/planner_common:src/kb_skills \
  pytest -q src/chatbot_llm/test/test_prompt_pack.py

PYTHONPATH=src/chatbot_llm:src/planner_llm:src/planner_common:src/kb_skills \
  pytest -q src/planner_llm/test/test_prompt_pack.py

PYTHONPATH=src/chatbot_llm:src/planner_llm:src/planner_common:src/kb_skills \
  pytest -q src/chatbot_llm/test/test_turn_engine.py -k "greet or greeting or planner_mode or knowledge_query"

PYTHONPATH=src/chatbot_llm:src/planner_llm:src/planner_common:src/kb_skills \
  pytest -q src/planner_llm/test/test_planner_engine.py -k "scan or invalid or retry or clarify"
```

## Decision

- Accept.
- Reason: objective improved with no targeted-regression signals.
- Rollback needed: no.

## Next Hypothesis

- Add one explicit anti-duplication wording rule for planner completion rendering:
  when execution already acknowledged in-turn, completion wording should avoid
  repeating “I will …” and focus on outcome state.
