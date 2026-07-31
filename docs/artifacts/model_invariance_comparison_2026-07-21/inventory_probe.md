# Backend inventory probe

Timestamp: 21 July 2026, final comparison run

## vLLM

Endpoint: `http://10.7.138.215:8004/v1/models`

Observed result: connection unavailable, `HTTP_STATUS=000`. No vLLM model identity was available, so no vLLM semantic result was fabricated or scored.

## Ollama

Endpoint: `http://127.0.0.1:11434/api/tags`

Advertised models:

- `deepseek-v4-flash:cloud`
- `gemini-3-flash-preview:latest`
- `gemma4:31b-cloud`
- `gemma4:cloud`
- `glm-5.2:cloud`
- `kimi-k2.5:cloud`
- `kimi-k2.6:cloud`
- `minimax-m2.7:cloud`
- `nemotron-3-super:cloud`
- `qwen3-coder:480b-cloud`
- `qwen3.5:cloud`

The three models exercised in this report were `gemma4:31b-cloud`, `nemotron-3-super:cloud`, and `gemma4:cloud`. Availability was established through preflight, then semantic behavior was evaluated through the ROS runtime questionnaire.
