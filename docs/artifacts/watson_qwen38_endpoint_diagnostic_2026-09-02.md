# Watson Qwen3.8 Endpoint Diagnostic

Date: 2 September 2026 (Europe/Madrid)

## Endpoint

- Base URL: `http://10.88.140.94:4000`
- Chat route: `/v1/chat/completions`
- Model requested: `qwen3.8`
- Gateway: LiteLLM over the Watson ZeroTier link
- Client tested from the NAO host, outside the running ROS container

## Discovery And Health

`GET /v1/models` returned HTTP 200 and advertised these model IDs:

```text
qwen3.8
qwen36-turbo-hermes
qwen36-turbo-hermes-llama
qwen36-turbo-hermes-spec
```

`GET /health` initially returned HTTP 200 with four healthy Qwen3.8
backends and zero unhealthy backends. After the chat probes, it returned HTTP
200 with three healthy and one unhealthy backend. The healthy entries reported
`model=openai/qwen3.8` and `max_tokens=16`.

## Chat Probes

All requests used `stream:false`, `temperature:0.2`, and the OpenAI-compatible
JSON shape. A minimal request was:

```json
{
  "model": "qwen3.8",
  "messages": [
    {"role": "user", "content": "Reply with exactly OK."}
  ],
  "max_tokens": 256,
  "temperature": 0.2,
  "stream": false
}
```

The endpoint returned HTTP 500. The stable part of the response was:

```json
{
  "error": {
    "message": "litellm.InternalServerError: InternalServerError: OpenAIException - Failed to parse input at pos 41: <corrupted/generated token text>. Received Model Group=qwen3.8",
    "type": null,
    "param": null,
    "code": "500"
  }
}
```

The latest verification in this review used `max_tokens=64` and
`chat_template_kwargs.enable_thinking=false`. It took 16.6 seconds and
returned HTTP 500 with this representative LiteLLM message:

```text
litellm.InternalServerError: InternalServerError: OpenAIException - Failed to parse input at pos 41: Blasio Garancaffold依WebpackPlugin以上的ằn产品介绍ínc��巨石รำETINGachelroc�agliouscgeist袱[textSYMhop有味想说雯ỵensp�最低何等essi拉德עת乾AVEesuitemapTalypad裕ets.nihacco说来inton蛯ovesholmKitolv盈盈 bravantisuverostarester kudadditolinstral paras COLL. Received Model Group=qwen3.8
Available Model Group Fallbacks=None
```

The response still contained no `choices[0].message.content` field.

The same failure reproduced when the request included:

```json
"chat_template_kwargs": {"enable_thinking": false}
```

The error text contained varying multilingual and replacement-character
sequences on each attempt. This looks like a backend request/parser or
generation-path failure, not an HTTP reachability failure and not a normal
model response. The endpoint did not return an assistant `content` value in
any successful probe.

## Interpretation

The Watson service is discoverable and partially healthy, but `qwen3.8` is not
currently qualified as a ROS stack backend. A model-list response or health
response is insufficient: the minimal OpenAI chat contract must return a valid
assistant message before the stack is launched against it.

The failure occurs before the NAO stack and is independent of grounded
context, planner prompts, KnowledgeCore, fake skills, or report-result logic.
Do not attribute this HTTP 500 to the frozen image. Do not add a stack fallback
for it.

## PC-Side Handoff

Please check the Watson host in this order:

1. Inspect the llama.cpp server log for the request corresponding to the
   `pos 41` parse failure, including whether the request reaches the model
   process and whether the error is emitted while parsing the prompt or while
   decoding the response.
2. Test the underlying llama.cpp endpoint directly, bypassing LiteLLM, with a
   one-turn request and a small token budget. Compare its response with the
   LiteLLM response.
3. Verify the configured chat template and Qwen3.8 tokenizer match the served
   model. The varying corrupted text suggests a tokenizer/template, binary
   compatibility, or concurrent-request problem.
4. Check whether the four health backends share one overloaded or unhealthy
   worker. Temporarily route to one worker at a time and record the worker ID.
5. Confirm that `max_tokens` is not being rewritten to `16` by the health
   configuration and that the model accepts the chosen context length.
6. Repeat the exact minimal request until it returns a JSON object containing
   `choices[0].message.content`. Only then retry a structured JSON planner
   request.

Suggested post-fix acceptance command:

```bash
curl -sS -m 60 -X POST \
  http://10.88.140.94:4000/v1/chat/completions \
  -H 'Content-Type: application/json' \
  -d '{
    "model":"qwen3.8",
    "messages":[
      {"role":"system","content":"Reply with exactly OK."},
      {"role":"user","content":"Reply with exactly OK."}
    ],
    "max_tokens":64,
    "temperature":0,
    "stream":false
  }'
```

Acceptance requires HTTP 200, a non-null `choices[0].message.content`, no
`error` member, and a second successful request without relying on a cached
response.

## Current Disposition

`endpoint_reachable`: yes  
`model_discoverable`: yes  
`health_check`: partial (3/4 healthy after probes)  
`minimal_chat_contract`: fail (HTTP 500)  
`ROS stack qualification`: blocked pending Watson repair
