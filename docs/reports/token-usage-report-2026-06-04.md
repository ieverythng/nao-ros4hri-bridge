# Token Usage Report — 2026-06-04

Generated: 2026-06-04T20:37:08

## Today Summary (since midnight)

| Metric | Value |
|---|---|
| Sessions | 48 |
| Input tokens | 2,236,199 |
| Output tokens | 137,979 |
| Cache read tokens | 29,532,292 |
| Reasoning tokens | 10,553 |
| **Total billed** | **2,384,731** |

### By Model

| Model | Provider | Input | Output | Cache Read | Cost | Sessions |
|---|---|---|---|---|---|---|
| qwen36-turbo-hermes | custom (local) | 1,668,328 | 84,381 | 24,382,596 | $0.00 | 33 |
| gpt-5.4 | openai-codex | 567,871 | 53,598 | 5,149,696 | $0.00 | 7 |
| gpt-5.5-low | — | 0 | 0 | 0 | — | 8 |

## 5-Hour Window (last 5 hours)

| Metric | Value |
|---|---|
| Sessions | 32 |
| Input tokens | 1,836,449 |
| Output tokens | 112,741 |
| **Total** | **1,949,190** |

## 7-Day Weekly Summary

| Metric | Value |
|---|---|
| Sessions | 188 |
| Input tokens | 5,319,732 |
| Output tokens | 380,603 |
| **Total billed** | **5,700,335** |

### Weekly By Model

| Model | Provider | Input | Output | Sessions |
|---|---|---|---|---|
| qwen36-turbo-hermes | custom (local) | 4,645,107 | 310,397 | 146 |
| gpt-5.4 | openai-codex | 567,871 | 53,598 | 7 |
| gpt-5.3-codex | — | 106,754 | 16,608 | 17 |
| gpt-5.5-low | — | 0 | 0 | 17 |

## Key Observations

1. **qwen36-turbo-hermes dominates** — 92% of input tokens today, all local/free
2. **gpt-5.4 (Codex)** — 25% of today's input tokens, used for delegation/subagents
3. **Cache read is massive** — 29.5M cache reads today means prompt caching is working well
4. **Output tokens are low relative to input** — good ratio (~6%), means we're not over-generating
5. **$0.00 cost tracked** — local model is free, Codex OAuth billing not captured in DB

## Recommendations

1. **Set token budget alerts** — configure max tokens per 5hr window for paid models
2. **Prefer local model** — qwen36 handles most work; reserve Codex for tasks that need it
3. **Track subagent token cost** — each delegation should report its token footprint
4. **Reduce output length** — the main source of token waste is long chat responses hitting max output length
