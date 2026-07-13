---
name: seam-hypothesis-audit
description: Use for difficult cross-seam investigations with multiple plausible causes, uncertain runtime behavior, or competing architecture and implementation routes. Maintains an explicit hypothesis registry, runs discriminating probes, marks theorem-strength gaps as blocked, and requires adversarial evidence before accepting a fix or declaring a bounded handoff.
---

# Seam Hypothesis Audit

## Purpose

Use this skill when a repository problem spans several packages, contracts,
prompts, registries, runtime nodes, or evidence sources and the first plausible
explanation may be incomplete. It adapts a research-style portfolio search to
software work without importing model-specific claims, unlimited persistence, or
the assumption that a solution must exist.

This is an investigation protocol, not a replacement for the repository's
ROS4HRI, SkillOpt, runtime-review, or normal test workflows.

## Activation Boundary

Use it for:

- failures with several plausible upstream or downstream causes;
- architecture decisions where a change may move ownership across seams;
- planner, chatbot, registry, grounding, execution, or trace problems whose
  current evidence does not identify one cause;
- research-to-implementation questions that need competing formulations and a
  defensible acceptance gate.

Do not use it for a local typo, a single obvious test failure, or a bounded
mechanical refactor. Keep the investigation proportional to the task and set a
time, evidence, and change budget before opening broad search.

## Workflow

### 1. Freeze the target contract

Write the target contract before proposing a fix:

- exact problem statement and affected user/runtime behavior;
- required outcome and explicit non-goals;
- seams and owners that must not change;
- acceptance tests, traces, or observable runtime facts;
- evidence and time budget for this investigation.

Use the repository precedence order: current source/tests/launch files and
package guidance first, accepted architecture contracts next, active workflow
and masterplan after that, and historical or generated documents as evidence
only. Call out drift instead of silently resolving it.

### 2. Establish a baseline

Record the current behavior before editing. Include the smallest reproducible
test, relevant trace or log identifiers, current registry/prompt version, and
the failure and success paths that must remain intact. If live ROS, the robot,
KnowledgeCore, or a simulator is unavailable, record that as an evidence gap.

Run the narrowest relevant checks first. For ROS4HRI work, run the repository
change audit and inspect nested repository state before touching a sensitive
package.

### 3. Build the approach registry

Create an investigation registry using
`references/hypothesis-registry-template.md`. Each route must have a materially
different mechanism, not merely different wording. Useful families include:

- contract or payload mismatch;
- runtime wiring, launch, lifecycle, or interface selection;
- prompt, model, or dialogue policy;
- registry, AB-level, alias, or capability projection;
- perception, grounding, KB, or evidence freshness;
- executor, adapter, action result, or recovery behavior;
- observability, trace correlation, or test-fixture error.

For every route record its assumptions, affected owners, discriminating probe,
expected observation, evidence references, status, and exact gap. Do not allow
an elegant reduction to dominate simply because it is easy to explain.

When independent agents or separate review passes are available, keep early
passes independent and do not leak the current favorite to all reviewers. If
parallel agents are unavailable, perform independent passes sequentially and
label them as independent rather than pretending they are separate evidence.

### 4. Run discriminating probes

Prefer probes that distinguish routes rather than merely exercising the happy
path. A probe should return a concrete artifact: a failing assertion, payload
diff, trace event, launch graph, registry comparison, minimal reproduction,
counterexample, or source-level call path.

Reject status-only conclusions, vague confidence, and claims that a missing
compatibility or global-consistency step is "routine." A candidate is useful
only when it explains an observation and yields a testable next step.

### 5. Manage blocked routes explicitly

Mark a route `blocked` when it reaches a missing lemma, unavailable runtime
fact, unsupported interface assumption, or other gap as strong as the original
problem. Keep the exact gap in the registry. Reopen it only when a materially
new mechanism, invariant, source artifact, or runtime observation appears.

Do not relabel a blocked route as promising because it has an elegant partial
reduction. Do not delete it: the negative result is part of the evidence.

### 6. Run the adversarial gate

Before accepting a change, challenge the leading route against the target
contract and the known ROS4HRI failure modes:

- ownership still matches `AGENTS.md` and the active architecture contract;
- existing ROS interfaces are reused and topic/service/action choice is sound;
- goal, plan, version, and step lineage remain correlated;
- no duplicate speech authority or hidden executable plan was introduced;
- no perception, proximity, KB fact, action result, or completion was
  fabricated;
- planning-time context is not treated as execution-time proof;
- people and objects remain semantically distinct;
- AB=0 interfaces/primitives, AB=1 runtime skills, and AB>=2 proposals keep
  their documented meaning;
- success, failure, cancellation, clarification, supersede, and replan paths
  are covered where relevant;
- nested or upstream-sensitive packages were changed only at the required
  seam;
- canonical registry and all runtime/planner/docs projections agree when the
  registry is in scope.

For prompts or LLM-facing policy, invoke the existing SkillOpt workflow and
record baseline, mutation, train result, holdout result, and accept/reject
decision before accepting wording changes.

### 7. Synthesize and decide

The root investigation pass must compare routes, evidence, residual risk, and
the smallest safe change. Accept only when the acceptance gate passes and the
adversarial review finds no unresolved contract violation. If the task remains
unresolved, return the strongest supported finding, exact remaining gap, and
the next discriminating probe as a bounded handoff; never claim completion from
effort, elapsed time, or an attractive hypothesis.

For durable investigations, keep the registry and decision in
`docs/artifacts/` or the active plan selected by `docs/README.md`. Do not put a
temporary research log in the canonical runtime registry.

## Required Output

Report these sections in the final investigation artifact or response:

1. `Target contract`
2. `Baseline evidence`
3. `Approach registry`
4. `Discriminating probes and results`
5. `Adversarial audit`
6. `Decision: accept, reject, or bounded handoff`
7. `Residual risk and next probe`

When this investigation changes code, finish with focused tests, compilation
or launch checks as applicable, registry consistency checks when relevant, and
the final ROS4HRI change audit. State clearly what still requires live ROS,
KnowledgeCore, simulator, or robot validation.

Run a bounded SkillOpt-style iteration (baseline -> mutate -> holdout gate -> accept/reject log) before finalizing major wording changes.

