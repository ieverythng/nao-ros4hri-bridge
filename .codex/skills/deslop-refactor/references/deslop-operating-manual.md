# Deslop Operating Manual

Use this file as the default working rubric for deslop requests. It condenses the upstream command into a shorter Codex-friendly playbook.

## Table of Contents

1. Scope and safety
2. Core principles
3. High-value refactor patterns
4. Repo-wide pass sequence
5. Extending the rubric

## Scope and Safety

- Confirm whether the target is first-party, vendored, generated, or a nested repo before editing.
- Prefer behavior-preserving cleanup unless the current code is obviously buggy or contradictory.
- Read the nearest tests, package entrypoints, and recent diffs before changing structure.
- Avoid broad rewrites when a helper extraction or control-flow cleanup will solve the problem.

## Core Principles

### KISS

Favor the simplest structure that still makes the domain obvious. Remove branches, parameters, or wrappers that do not buy clarity.

### YAGNI

Delete speculative abstraction and placeholder logic that is not carrying live behavior yet.

### Cognitive Load

Reduce the number of ideas a reader must hold at once. Long functions often want named helpers, not more comments.

### Single Level of Abstraction

Keep orchestration code focused on routing and sequencing. Push parsing, normalization, and backend-specific details into helpers or adapters.

### Self-Documenting Code

Prefer names that reveal intent: what is being normalized, dispatched, refreshed, or retried.

### Documentation Discipline

Use comments for why, constraints, or integration context. Do not narrate straightforward code.

### DRY

Unify logic only when the duplicated code represents the same concept or contract.

### Separation of Concerns

Split normalization, state updates, transport calls, and formatting into different seams when they change for different reasons.

### Command-Query Separation

Keep functions that mutate state distinct from those that compute or format data whenever practical.

### Fail-Fast

Validate unsupported backends, empty payloads, and missing dependencies explicitly rather than silently choosing a surprising fallback.

### Least Surprise

Make supported cases obvious. If only two backends are supported, the code should read that way without hidden defaults.

### Boy Scout Rule

Leave a file cleaner than you found it, but stop before cleanup turns into churn.

## High-Value Refactor Patterns

### Good candidates

- Repeated "wait for service / warn once / parse JSON" logic.
- Repeated "append turn to history" logic.
- Branches that classify a route and then repeat the same dispatch steps in multiple call sites.
- Backend-selection logic split across several functions with inconsistent defaults.
- Mutable objects updated field-by-field in multiple places.

### Usually not worth it

- Renaming everything in a stable public API.
- Creating a base class for one subclass.
- Splitting tiny functions just to hit an arbitrary line count.
- Replacing a clear literal with a registry when there are only one or two trivial cases.

## Repo-Wide Pass Sequence

1. Read recent commits or the touched diff to find the newest or riskiest code.
2. Start with hot-path first-party packages.
3. Refactor one package at a time so validation stays narrow.
4. Run targeted tests immediately after each package or after a coherent batch.
5. End with a short summary of the principles applied and anything intentionally skipped.

## Extending the Rubric

When the user wants a new principle added:

1. Read `upstream-addtodeslop.md`.
2. Check whether the principle is already covered here or in `upstream-deslop.md`.
3. Research the smallest useful set of sources to confirm the principle is distinct and established.
4. Add a concise section here with:
   - One-line definition
   - When to apply it
   - Common violation
   - One practical example
5. Keep this file lean. Put long source material in a separate reference instead of expanding the skill body.
