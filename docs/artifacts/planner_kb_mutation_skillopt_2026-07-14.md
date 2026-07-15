# Planner KB Mutation SkillOpt Ledger

**Date:** 2026-07-14
**Target artifact:** `src/planner_llm/config/planner_prompt_pack.yaml`
**Objective:** Convert explicit natural-language KB changes into valid KnowledgeCore
statements without requiring users to provide RDF syntax or permitting questions to
mutate state.

## Frozen Evaluation

### Train Set

| Case | Expected | Baseline | Pass |
|---|---|---|---|
| `Add a red cup to your KB.` | `kb_add` plan with concrete triples, verified mutation, natural report | Planner returned `fail`, stating that the user had not supplied `subject predicate object` syntax | No |

### Holdout Set

| Case | Protected behavior |
|---|---|
| `What do you remember about the marker?` | Knowledge query only, no mutation |
| Explicit subject-predicate-object add | Preserve supplied facts without semantic expansion |
| Explicit remove request | Use `kb_remove` and verify absence |
| Grouped kitchen delivery | Preserve target selection and final reporting policy |
| Ordinary dialogue | No planner handoff or KB mutation |

## Mutation Batch 1

- **Type:** replace one task-policy rule.
- **Budget:** one rule, at most four prompt lines.
- **Invariant:** an explicit natural-language mutation authorizes translation of only
  the stated facts into concrete triples. It does not authorize unstated properties,
  unsupported perception claims, or mutation from a question.

## Results

Mutation batch 1 changed the train case from an immediate `fail` decision to a
`kb_add` plan attempt, but the model emitted `cup has_color red`. Strict planner
validation rejected the unqualified predicate, and the retry did not improve it.

## Mutation Batch 2

- Require namespace-qualified predicates in the same canonical task-policy rule.
- Make planner validation feedback name the missing namespace and provide predicate
  shape examples. Do not normalize or accept the invalid statement.

The direct train probe still failed. The model emitted
`rdf:type cup dbp:color red`, reversing subject and predicate, and prepended
unrequested `scan` and `find_object` steps.

## Mutation Batch 3

- State the triple token order with a generic example and forbid perception steps for
  explicit facts that do not depend on current observation.
- Make retry feedback name the required subject-first order. This is the final mutation
  in the bounded round.

The clean v4 runtime passed the train case with zero fallback markers. The
planner emitted concrete `object_1 rdf:type Cup` and `object_1 dbp:color red`
statements, KnowledgeCore accepted them, and the chatbot reported, "I've added a
red cup to the knowledge base."

Protected live holdouts also passed:

- query-only turns did not mutate KnowledgeCore;
- explicit add, revise, query, and remove preserved state isolation;
- grouped kitchen delivery retained grounded members, ALEX, and final reporting;
- ordinary dialogue remained outside planner execution.

## Decision

**Accepted.** Batch 3 passed the train and protected live holdouts. The strict
statement validator remains authoritative; invalid function-style or
predicate-first statements are retried rather than normalized into mutations.
