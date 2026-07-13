# Hypothesis Registry Template

This is an investigation registry, not the canonical AB/skill registry. Keep it
close to the plan or artifact for the problem being investigated.

## Target Contract

- Problem:
- Required outcome:
- Non-goals:
- Protected seams and owners:
- Acceptance gate:
- Evidence and time budget:

## Baseline

| Observation | Source or command | Expected | Actual | Evidence reference |
| --- | --- | --- | --- | --- |
|  |  |  |  |  |

## Approach Registry

Use one row per materially different mechanism. Do not create rows that differ
only in wording.

| ID | Family | Mechanism | Affected seams | Discriminating probe | Expected observation | Status | Exact gap or reopen condition |
| --- | --- | --- | --- | --- | --- | --- | --- |
| H-01 |  |  |  |  |  | candidate |  |

Allowed statuses: `candidate`, `active`, `blocked`, `rejected`, `accepted`.

Each active route should also record:

- assumptions;
- source, test, trace, or log references;
- concrete artifact produced by the probe;
- whether the route changes code, prompt policy, registry, launch wiring, or
  documentation.

## Round Log

| Round | Routes selected | New evidence | Redirect or rejection | Next probe |
| --- | --- | --- | --- | --- |
| 0 | contract and baseline |  |  |  |
| 1 | independent families |  |  |  |
| 2 | discriminating probes |  |  |  |
| 3 | adversarial audit |  |  |  |

## Adversarial Audit

- [ ] Ownership boundaries remain unchanged or are explicitly accepted.
- [ ] ROS interfaces and topic/service/action choices remain valid.
- [ ] Goal, plan, version, and step lineage remain correlated.
- [ ] No duplicate speech or hidden executable plan was introduced.
- [ ] No unsupported perception, KB, proximity, result, or completion claim was
      introduced.
- [ ] AB level and canonical registry semantics remain correct.
- [ ] Success and relevant failure/recovery paths are tested.
- [ ] Nested/upstream-sensitive changes are narrow and justified.

## Decision

- Decision: `accept` / `reject` / `bounded handoff`
- Chosen route:
- Evidence that satisfies the gate:
- Residual risk:
- Exact next probe if unresolved:
