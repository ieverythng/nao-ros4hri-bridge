---
name: deslop-refactor
description: Run a deslop-style refactor pass to remove AI-generated code smells and improve readability without changing intended behavior. Use when Codex is asked to deslop, de-slop, clean up, simplify, reduce duplication, improve naming, tighten control flow, or apply the llm-public-utils `deslop` / `addtodeslop` methodology to a file, package, or repo.
---

# Deslop Refactor

## Overview

Apply a behavior-preserving cleanup pass that favors simpler control flow, clearer names, smaller seams, and justified comments over broad rewrites. Keep the review grounded in the upstream `llm-public-utils` workflow, but adapt it to Codex skills and normal code editing instead of Claude/OpenCode slash commands.

## Quick Start

1. Define scope and guardrails before editing.
2. Read the touched code and its nearby tests before proposing abstractions.
3. Load `references/deslop-operating-manual.md` for the working rubric.
4. Load `references/upstream-deslop.md` only when the condensed rubric is not enough or the user explicitly wants the upstream wording.
5. Make the smallest high-value changes first, then validate with targeted tests or linters.

## Workflow

### 1. Understand the real boundary
- Confirm which directories are first-party and which are vendored, generated, or separate repos.
- Prefer the user's recent touch points, failing tests, or hot paths over broad churn.
- Preserve behavior unless the current behavior is clearly unsafe or self-contradictory.

### 2. Diagnose slop by principle
- Look first for KISS, YAGNI, cognitive load, self-documenting code, single level of abstraction, DRY, separation of concerns, command-query separation, fail-fast, and least-surprise issues.
- Treat duplicated structure as a problem only when the code is expressing the same concept, not merely similar syntax.
- Prefer deleting dead branches, collapsing repeated logic, and isolating backend-specific seams over inventing new framework layers.

### 3. Refactor in low-risk increments
- Extract helpers when they remove real duplication or separate concepts that were mixed together.
- Rename things when the old names hide intent or domain meaning.
- Replace magic branching with explicit tables, registries, or helper methods when that makes supported cases obvious.
- Keep comments rare and purposeful: explain non-obvious intent, constraints, or integration seams.

### 4. Validate the cleanup
- Run the narrowest useful test set first.
- If there are no tests, add focused tests only where they materially reduce risk.
- Summarize what changed, what was intentionally left alone, and any residual risks.

## Preferred Transformations

- Collapse repeated condition trees into one helper or lookup table.
- Replace repeated "build history / append turn / trim history" code with one reusable function.
- Move backend-specific parsing into adapters so node logic stays backend-agnostic.
- Separate normalization from execution so tests can stay mostly pure.
- Delete unused state, one-off flags, or placeholder branches that no longer pull their weight.

## Stop Conditions

- Do not chase cosmetic churn once the main readability or maintainability hazards are resolved.
- Do not create abstractions for a single call site unless they also isolate a concept or a risk.
- Do not "deslop" vendored code, generated code, or separate repos unless the user explicitly includes them.

## Extending the Principle Set

If the user wants to expand the rubric itself, follow the upstream `addtodeslop` idea in a Codex-native way:

1. Read `references/upstream-addtodeslop.md`.
2. Compare the requested principle against `references/deslop-operating-manual.md` and `references/upstream-deslop.md`.
3. Research only enough to confirm the principle is distinct and battle-tested.
4. Add a concise new section to `references/deslop-operating-manual.md` instead of bloating `SKILL.md`.
5. Keep the new guidance actionable, balanced, and example-driven.

## References

- `references/deslop-operating-manual.md` — condensed Codex-native workflow and principle checklist.
- `references/upstream-deslop.md` — vendored upstream command reference from `Theta-Tech-AI/llm-public-utils`.
- `references/upstream-addtodeslop.md` — vendored upstream workflow for adding new principles to the rubric.
