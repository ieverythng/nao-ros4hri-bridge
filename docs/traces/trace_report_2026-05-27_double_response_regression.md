# Double Response Regression Trace Report

Date: 2026-05-27
Container: `nao_ros2`
Launch profile: `nao_chatbot_sim.launch.py` with planner mode enabled, fake skills enabled, and planner dialogue/completion wording set to `direct`.

## Acceptance Gate

| Case | Expected gate |
| --- | --- |
| Greeting-only | `route=dialogue`, no planner handoff, one chatbot-owned utterance |
| Execution request | chatbot says only a short future-tense acknowledgement |
| Execution completion | planner/orchestrator emits one completion or failure utterance |
| Report result | no parenthetical simulated action, no premature observation in ack, no unresolved `[evidence.*]` placeholders |

## Results

| Case | Input | Runtime result | Decision |
| --- | --- | --- | --- |
| `greeting` | `Hey Pop!` | `route=dialogue`, `intent=greet`, `planner_handoff_published=false`, speech: `Hello! How can I help you today?` | PASS |
| `scan` | `can you scan for people?` | chatbot ack: `Sure, I will look around and report what I can see.`; planner steps: `scan -> report_result`; completion: `I looked around and can report the current scene summary.` | PASS |
| `head_report_before_scan_tightening` | `Move your head to the right and tell me what you see!` | chatbot ack was clean, but planner selected `perform_motion -> scan -> report_result`; accepted as no double speech, then prompt was tightened to avoid scan keyword overuse | OBSERVED |
| `nav_blocked` | `Navigate to the kitchen.` with `path_blocked` | chatbot ack: `Sure, I will navigate to the kitchen.`; planner step: `navigate_to`; failure/clarification: `I could not navigate to kitchen.` | PASS |
| `final_head_report` | `Move your head to the right and tell me what you see!` after prompt tightening | chatbot ack: `Sure, I will move my head to the right and report what I see.`; planner step: `perform_motion`; completion: `I am looking to the right now.` | PASS |

## Accepted Mutation Batch

1. `chatbot_llm` prompt: execution `verbal_ack` must be future-tense acknowledgement only; no parenthetical action narration and no observations/results.
2. `chatbot_llm` turn engine: sanitize execution acknowledgements before they enter history, planner payloads, or TTS.
3. `nao_orchestrator` report_result: ignore unresolved planner template strings such as `[evidence.objects]` and fall back to the previous real skill result.
4. `planner_llm` prompt: choose scan from intent/context rather than keyword-matching "what do you see"; never emit unresolved report placeholders.

## Raw Captures

- `docs/traces/raw/2026-05-27/double_response_regression/greeting`
- `docs/traces/raw/2026-05-27/double_response_regression/scan`
- `docs/traces/raw/2026-05-27/double_response_regression/head_report_before_scan_tightening`
- `docs/traces/raw/2026-05-27/double_response_regression/nav_blocked`
- `docs/traces/raw/2026-05-27/double_response_regression/final_head_report`
