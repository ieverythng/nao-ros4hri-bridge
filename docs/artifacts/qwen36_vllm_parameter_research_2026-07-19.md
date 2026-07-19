# Qwen3.6-35B-A3B-AWQ with vLLM: runtime and sampling configuration

**Date:** 2026-07-19  
**Scope:** `QuantTrio/Qwen3.6-35B-A3B-AWQ`, served through vLLM's OpenAI-compatible Chat Completions endpoint. Sources are restricted to the model repository, official Qwen material reproduced in that repository, and official vLLM documentation/source.

## Executive conclusion

For ordinary deterministic application turns, use Qwen3.6's non-thinking mode explicitly:

```json
{
  "temperature": 0.7,
  "top_p": 0.8,
  "max_tokens": 4096,
  "extra_body": {
    "top_k": 20,
    "min_p": 0.0,
    "repetition_penalty": 1.0,
    "presence_penalty": 1.5,
    "chat_template_kwargs": {"enable_thinking": false}
  }
}
```

The sampling values above are Qwen's non-thinking general-task settings. The `4096` output cap is an application recommendation, not a Qwen default. Raise it when the task needs longer output, while keeping rendered input plus output within the server's effective `max_model_len`. For machine-consumed JSON, additionally use vLLM structured outputs with a JSON Schema. Prompt-only requests for JSON do not provide the same syntactic guarantee.

## Sourced facts

### Model and serving limits

- The quantized repository declares AWQ, 4-bit weights, group size 128, GEMM format, and `text_config.max_position_embeddings = 262144`. Its tokenizer also declares `model_max_length = 262144` ([model `config.json`](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ/blob/main/config.json), [tokenizer configuration](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ/blob/main/tokenizer_config.json)).
- The model card states a native context length of 262,144 tokens and shows vLLM serving with `--max-model-len 262144`. It recommends vLLM 0.19.0 or later for Qwen3.6. The QuantTrio example is deliberately more conservative at `--max-model-len 32768`, so that example is a deployment choice rather than the model's architectural limit ([model card: overview and vLLM serving](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ#qwen36-35b-a3b)).
- Qwen documents extension to about 1,010,000 total tokens through explicit static YaRN configuration and `VLLM_ALLOW_LONG_MAX_MODEL_LEN=1`. Qwen warns that static YaRN can impair shorter-context performance and advises enabling it only when long contexts are required ([model card: processing ultra-long texts](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ#processing-ultra-long-texts)).
- vLLM validates the rendered prompt and requested output against the served model's `max_model_len`. In current vLLM source, `max_completion_tokens` takes precedence over the deprecated `max_tokens`; either value is interpreted as an output-token ceiling, not a total-context setting ([vLLM Chat Completions protocol](https://github.com/vllm-project/vllm/blob/main/vllm/entrypoints/openai/chat_completion/protocol.py)).

### Thinking mode and chat-template arguments

- Qwen3.6 thinks by default. The shipped template starts a new assistant turn with `<think>\n`; when `enable_thinking` is explicitly false, it instead emits an empty thinking block before the answer. The model card's OpenAI-compatible example passes this as `extra_body.chat_template_kwargs.enable_thinking = false` ([shipped `chat_template.jinja`](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ/blob/main/chat_template.jinja), [non-thinking example](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ#instruct-or-non-thinking-mode)).
- Qwen3.6 does **not** officially support Qwen3's `/think` and `/nothink` soft switches. Runtime code should therefore use the template argument rather than embedding those strings in prompts ([non-thinking example and note](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ#instruct-or-non-thinking-mode)).
- The template accepts `preserve_thinking`. Historical assistant reasoning is normally removed outside the latest interaction; setting `preserve_thinking=true` retains historical reasoning traces in the rendered prompt. This is separate from whether the next turn generates thinking ([model card: Preserve Thinking](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ#preserve-thinking), [shipped template](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ/blob/main/chat_template.jinja)).
- vLLM exposes `chat_template_kwargs` as additional values passed to the template renderer. It can also set deployment-wide defaults through `--default-chat-template-kwargs`; request values are part of Chat Completions' vLLM-specific extensions ([vLLM OpenAI-compatible server](https://docs.vllm.ai/en/latest/serving/online_serving/openai_compatible_server/), [protocol source](https://github.com/vllm-project/vllm/blob/main/vllm/entrypoints/openai/chat_completion/protocol.py)).

### Sampling

Qwen3.6 publishes task-specific settings, rather than one universal configuration ([Qwen3.6 best practices](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ#best-practices)):

| Mode | Task | `temperature` | `top_p` | Other Qwen values |
|---|---|---:|---:|---|
| Thinking | General | 1.0 | 0.95 | `top_k=20`, `min_p=0`, `presence_penalty=1.5`, `repetition_penalty=1` |
| Thinking | Precise coding | 0.6 | 0.95 | `top_k=20`, `min_p=0`, `presence_penalty=0`, `repetition_penalty=1` |
| Non-thinking | General | 0.7 | 0.8 | `top_k=20`, `min_p=0`, `presence_penalty=1.5`, `repetition_penalty=1` |
| Non-thinking | Reasoning | 1.0 | 1.0 | `top_k=40`, `min_p=0`, `presence_penalty=2`, `repetition_penalty=1` |

Qwen notes that high presence penalties can reduce repetition but may cause language mixing and a small performance loss. Its recommended output allowance is 32,768 tokens for most queries and 81,920 for unusually complex benchmark problems. These are capacity recommendations, not requirements that every production request reserve that many tokens ([Qwen3.6 best practices](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ#best-practices)).

vLLM uses request values when supplied. Otherwise, its OpenAI server can apply a repository `generation_config.json`; absent such model defaults, protocol fallbacks include `temperature=1`, `top_p=1`, `top_k=0`, and `min_p=0` ([vLLM server documentation](https://docs.vllm.ai/en/latest/serving/online_serving/openai_compatible_server/), [protocol source](https://github.com/vllm-project/vllm/blob/main/vllm/entrypoints/openai/chat_completion/protocol.py)). The QuantTrio file listing does not currently include `generation_config.json`, so callers should send the intended values explicitly ([repository file tree](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ/tree/main)).

### Structured JSON

- vLLM's OpenAI-compatible server supports constrained structured output by choice, regular expression, JSON Schema, grammar, and structural tags. Current vLLM uses `extra_body={"structured_outputs":{"json": schema}}`; legacy `guided_json` was removed in vLLM 0.12.0 ([vLLM structured outputs](https://docs.vllm.ai/en/latest/features/structured_outputs/)).
- A JSON Schema constrains syntax and schema-conforming structure during decoding. Semantic correctness still depends on the prompt, schema, and model. Qwen's own benchmark advice to request a JSON-shaped answer in the prompt is output standardization guidance, not constrained decoding ([Qwen3.6 best practices](https://huggingface.co/QuantTrio/Qwen3.6-35B-A3B-AWQ#best-practices), [vLLM structured outputs](https://docs.vllm.ai/en/latest/features/structured_outputs/)).

### Timeout interpretation

- `timeout` is not a Chat Completions generation field in vLLM's request protocol. Generation length is governed by `max_tokens` or `max_completion_tokens`; a client or reverse-proxy timeout is an HTTP waiting/deadline policy and does not mean "allow this many seconds of model reasoning" ([vLLM Chat Completions protocol](https://github.com/vllm-project/vllm/blob/main/vllm/entrypoints/openai/chat_completion/protocol.py)).
- vLLM's server-side HTTP keep-alive timeout is also distinct from generation duration. Current server source passes `VLLM_HTTP_TIMEOUT_KEEP_ALIVE` to Uvicorn as `timeout_keep_alive` ([vLLM API-server source documentation](https://docs.vllm.ai/en/latest/api/vllm/entrypoints/openai/api_server/)).

### Preflight calls, cache warming, and conversation state

- vLLM Automatic Prefix Caching (APC) can reuse KV-cache blocks from an earlier processed request only when a later request shares the same token prefix. APC avoids repeated **prefill** computation; it does not accelerate generation of new tokens. Prefix reuse must be enabled and remains subject to cache capacity and eviction ([vLLM APC overview](https://docs.vllm.ai/en/v0.13.0/features/automatic_prefix_caching/), [vLLM prefix-cache design](https://docs.vllm.ai/en/stable/design/prefix_caching/)).
- Chat Completions requests carry their complete `messages` list and vLLM renders that list for the current request. There is no conversation/session identifier in the Chat Completions request that causes an earlier answer to be appended automatically ([vLLM protocol source](https://github.com/vllm-project/vllm/blob/main/vllm/entrypoints/openai/chat_completion/protocol.py), [vLLM chat-template documentation](https://docs.vllm.ai/en/stable/serving/openai_compatible_server/#chat-template)).

Therefore, a preflight generation can exercise an already served model and, with APC enabled, populate reusable KV blocks for a later request with an identical token prefix. It cannot seed application conversation history unless the application explicitly includes the preflight messages/output in the later `messages`. A generic health prompt will not warm the KV prefix for an unrelated production prompt.

## Recommendations and inferences

The following are operational recommendations derived from the sourced behavior, not vendor guarantees.

1. **Set thinking mode on every request or at server startup.** Use `chat_template_kwargs: {"enable_thinking": false}` for low-latency dialogue, routing, extraction, and strict JSON. Use thinking mode selectively for tasks where additional reasoning justifies latency and token cost. Do not rely on the model's default or Qwen3 soft-switch strings.
2. **Keep `preserve_thinking` false unless historical reasoning is intentionally required.** Retaining it increases prompt tokens, exposes prior reasoning to future turns, and reduces the usable context budget. Preserve final assistant answers in application history as usual.
3. **Send the full Qwen sampling tuple explicitly.** At minimum set `temperature`, `top_p`, `top_k`, `min_p`, `presence_penalty`, and `repetition_penalty`; this avoids behavior changing with server defaults or model-repository metadata. Start from the matching row in the table, then validate against application-specific holdouts.
4. **Choose `max_tokens` from the output contract, not the architectural maximum.** Suggested starting caps are 512 to 2,048 for classification/extraction, 2,048 to 8,192 for ordinary dialogue or planning, and higher only for demonstrated long-form needs. Reserve enough total room for the fully rendered prompt, tool/schema text, optional thinking, and output.
5. **Treat 262,144 as the supported native total context ceiling, subject to the actual server launch value and available KV memory.** Do not enable million-token YaRN as a default. Its static scaling tradeoff and much larger KV requirement need workload-specific evaluation.
6. **For machine-consumed JSON, combine non-thinking mode, a narrow JSON Schema, and application-side parsing/schema validation.** Constrained decoding improves syntactic reliability. Keep retries bounded and report validation failure rather than accepting repaired or fabricated fields. Lower temperature may improve repeatability, but Qwen's published non-thinking baseline is 0.7, so deviations such as 0 or 0.1 should be validated rather than assumed superior.
7. **Interpret timeout as an end-to-end service deadline.** Size it for queueing, prompt prefill, optional reasoning, decoding, and network overhead. Align client, proxy, and orchestration deadlines, and handle timeout as an unknown/failed call unless the application has explicit cancellation and idempotency semantics.
8. **Use a preflight for readiness, not hidden state.** If latency warming matters, enable APC and issue a bounded preflight whose rendered prefix exactly matches the stable production prefix. Do not add its user/assistant exchange to application history unless it is semantically part of that conversation. Measure whether the cache survives realistic concurrency before depending on the latency benefit.

## Recommended request shapes

Non-thinking, schema-constrained extraction:

```python
completion = client.chat.completions.create(
    model="MY_MODEL",
    messages=messages,
    temperature=0.7,
    top_p=0.8,
    max_tokens=2048,
    extra_body={
        "top_k": 20,
        "min_p": 0.0,
        "presence_penalty": 1.5,
        "repetition_penalty": 1.0,
        "chat_template_kwargs": {"enable_thinking": False},
        "structured_outputs": {"json": json_schema},
    },
)
```

Thinking mode for a precise coding task:

```python
completion = client.chat.completions.create(
    model="MY_MODEL",
    messages=messages,
    temperature=0.6,
    top_p=0.95,
    max_tokens=8192,
    extra_body={
        "top_k": 20,
        "min_p": 0.0,
        "presence_penalty": 0.0,
        "repetition_penalty": 1.0,
        "chat_template_kwargs": {
            "enable_thinking": True,
            "preserve_thinking": False,
        },
    },
)
```

Both output limits above are application starting points. Increase them only when observed truncation or task quality warrants the additional latency and context reservation.
