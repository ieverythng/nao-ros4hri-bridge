# v1 Freeze Commit Review

Primary root range: `317e319^..b618eff`

The three commits marked `PREFLIGHT, NEED TO CHECK` contain the source used by
the final v34 image together with runtime evidence and qualification tooling.
The review used the repository standards, final runtime plan, launch contract,
and Chapters 5 to 9 of the final thesis.

## Findings and disposition

| Finding | Severity | Disposition |
| --- | --- | --- |
| Harness inferred speech from chatbot acknowledgement and result summaries instead of correlated speech events | P1 | Closed by the current questionnaire correlation changes and focused tests |
| Trace preflight accepted stale JSONL files without proving that the trace writer was active | P2 | Closed by requiring both a trace file and a visible `interaction_trace_viewer` node |
| New sampling and timeout launch controls were not documented | P3 | Closed in `docs/launch_profiles.md` |
| The generic OpenAI-compatible provider sends optional vLLM-style fields | P2 limitation | Retained for v1 because it is part of the qualified vLLM tuple; documented as a v2 provider-adapter hardening item |
| Runtime code, qualification evidence, and historical JSON were mixed in one large commit | Scope-quality issue | History is reworded with an explicit subject and body; the frozen content is not split because doing so would increase provenance risk |

## Replacement commit messages

### `317e319`

Subject: `feat(runtime): harden target selection and failure evidence`

Body:

- add canonical target-selection validation and recovery helpers;
- strengthen plan/report outcome contracts and planner capability checks;
- expand the runtime questionnaire and retain the supporting stress artifacts.

### `f25d36b`

Subject: `feat(llm): expose backend preflight and generation controls`

Body:

- add launch parameters for chatbot and planner sampling, budgets, and timeouts;
- expose realistic LLM preflight behavior and provider settings;
- update runtime trackers for the Qwen qualification campaign.

### `b618eff`

Subject: `test(runtime): harden questionnaire provenance and scoring`

Body:

- capture loaded chatbot and planner parameters in each artifact;
- improve case timing and clarification/closure correlation;
- add regression coverage for runtime metadata and scored speech evidence.

## Nested `chatbot_llm` history

Two adjacent placeholder commits were also reviewed and reworded:

- `85a6693` became `feat(planner): add grounded routing and handoff contracts`;
- `40a5239` became `feat(llm): expose Ollama generation controls`.

The follow-up target-selection repair was committed separately as
`fix(planner): repair invalid retry-exhausted target selections`. The rewritten
branch was pushed with `--force-with-lease` because the placeholder commits had
already been published.

The older root commit `66f8c83` retains its historical subject. It predates the
final freeze by 53 commits and sits before subsequent merge history. Rewriting
that ancestry would change unrelated collaborative provenance and was therefore
outside this bounded final-history correction.

## Validation

- planner engine: 66 passed;
- runtime questionnaire: 74 passed after the review fixes;
- model-priority resolver: 5 passed;
- chatbot planner-request adapter and Ollama transport: 66 passed in the frozen container;
- dialogue manager planner-act and Say paths: 27 passed in the frozen container;
- `nao_look_at`: 8 passed in the frozen container.

No prompt policy was changed during this review.
