---
name: skillopt-skill-iteration
description: Apply a SkillOpt-style optimization loop to iteratively improve another skill or prompt pack using bounded edits, validation gates, and rollback-safe acceptance criteria.
---

# SkillOpt Skill Iteration

Use this skill when a user asks to improve an existing skill or prompt set through disciplined iteration instead of ad-hoc edits.

## Use Cases

- Hardening one skill (`SKILL.md`) through repeated, measurable improvement.
- Improving prompt packs while avoiding prompt bloat.
- Converting "small line here and there" edits into a controlled mutation loop.

## Inputs You Must Define First

Before editing, lock these 5 items:

1. `target_artifact`: exact file(s) to optimize.
2. `objective`: what should improve (e.g., fewer planner misroutes, less over-questioning).
3. `train_set`: scenarios used to generate candidate edits.
4. `holdout_set`: scenarios never used while drafting edits.
5. `acceptance_gate`: clear pass/fail metric (or metric set).

If any item is missing, ask for it or make explicit assumptions.

## SkillOpt Loop (Text-Space Optimization)

### Step 1: Baseline Snapshot

- Record current behavior against train + holdout sets.
- Save a short baseline table (`case`, `expected`, `actual`, `pass/fail`).

### Step 2: Bounded Mutation Proposal

- Propose one small batch of edits only.
- Generalize from train-set failures to the smallest stable invariant. Do not
  hardcode a failing phrase, object name, fixture id, user example, or benchmark
  case into normative prompt rules unless that literal is part of the public
  contract.
- Examples may show concrete shapes, but the rule text should teach the semantic
  behavior (for example, "resolve user-facing labels to grounded entity ids")
  rather than naming one scenario from the run.
- Allowed mutation types:
  - `add`: add one focused rule.
  - `replace`: tighten/clarify one rule.
  - `delete`: remove redundant/conflicting text.
- Keep mutation budget small per iteration:
  - max 3 edits
  - max ~12 changed lines per file unless user approves a bigger pass

### Step 3: Train-Set Check

- Validate candidate on train set.
- If regression appears in core behavior, reject immediately.

### Step 4: Holdout Gate

- Validate on holdout set.
- Accept mutation only if holdout improves or remains stable while objective improves.

### Step 5: Accept/Reject + Changelog

- If accepted: keep edit and log why.
- If rejected: rollback and log rejection reason.
- Maintain a concise iteration log (see reference template).

### Step 6: Slow Update Rule

Every 3 accepted iterations:

- Consolidate wording.
- Remove duplicated rules.
- Preserve behavior.

This prevents prompt/skill drift and instruction clutter.

## Guardrails

- Do not optimize multiple independent behaviors in one iteration.
- Do not broaden scope from one skill to whole stack without user approval.
- Prefer structural clarity over adding many examples.
- Do not turn a train case into a special-case instruction. Convert concrete
  runtime evidence into general wording, and use representative examples only
  when they clarify the contract shape.
- Keep one-question clarification behavior when uncertainty blocks safe execution.

## Integration Hook For Other Skills

When updating any local skill in this repo, add this one-line process reminder inside that skill's workflow section:

`Run a bounded SkillOpt-style iteration (baseline -> mutate -> holdout gate -> accept/reject log) before finalizing major wording changes.`

## Output Format

When reporting results, include:

1. `Objective`
2. `Mutation batch`
3. `Train results`
4. `Holdout results`
5. `Decision (accept/reject)`
6. `Next mutation hypothesis`

## References

- `references/skillopt-loop-template.md`
