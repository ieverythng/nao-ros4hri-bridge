# Backend availability and switch evidence

Timestamp: 20 July 2026, Europe/Madrid

## vLLM

The last successful `/v1/models` query observed the served model
`cyankiwi/Qwen3.5-35B-A3B-AWQ-4bit`. A direct OpenAI-compatible completion
passed when `chat_template_kwargs.enable_thinking` was set to `false`.

The authoritative check immediately before the model switch returned:

```text
curl: Failed to connect to 10.7.138.215 port 8004: Couldn't connect to server
HTTP_STATUS=000
```

Therefore the vLLM endpoint was unavailable for the current demo session. The
older Qwen3-VL id was not advertised by the final successful inventory check.

## Ollama

`http://127.0.0.1:11434/api/tags` returned HTTP 200 and advertised:

```text
nemotron-3-super:cloud
kimi-k2.6:cloud
gemma4:31b-cloud
gemma4:cloud
minimax-m2.7:cloud
gemini-3-flash-preview:latest
glm-5.2:cloud
deepseek-v4-flash:cloud
qwen3-coder:480b-cloud
kimi-k2.5:cloud
qwen3.5:cloud
```

Small `think=false` completion probes passed for `nemotron-3-super:cloud`,
`gemma4:31b-cloud`, and `gemma4:cloud`. `qwen3.5:cloud` returned HTTP 403
because the account requires a subscription. A prior inspection reported the
`qwen3-coder:480b-cloud` endpoint as retired with HTTP 410.

The selected model was `gemma4:31b-cloud` because it is the documented demo
default and passed the current liveness probe. `nemotron-3-super:cloud` remains
the first alternate for a subsequent qualification run.

## Live ROS result

The restarted `nao_ros2` container used `gemma4:31b-cloud` for chatbot response,
intent classification, and planning. The planner and chatbot preflights passed
on attempt 1 of 3. The single `simple_dialogue_hey` speech smoke case passed,
with the expected `response_first` pipeline and clean fixture retraction.

The NAOqi connection timed out because the robot endpoint was not reachable in
this session. This is a separate robot-network condition, not an LLM selection
failure.
