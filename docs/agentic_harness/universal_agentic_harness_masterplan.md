# Universal Agentic Harness: Implementation Masterplan

**Status:** Canonical implementation plan; H0 contract proof implemented, live integration pending
**Date:** 2026-07-22
**Scope:** Model-agnostic and task-agnostic harness kernel with domain-specific AB frames and adapters
**Extends:** `universal_agentic_harness_foundation.md` and `neural_workbench_adaptive_ab_harness.md`
**Decision owner:** Neural Workbench research track; NAO remains the first reference environment

## 1. Executive Decision

The project should build a Universal Agentic Harness, but not by replacing the
model loop, tool runtime, sandbox, session store, or provider layer of every
existing agent.

The defensible architecture is a hybrid:

> Own the AB-semantic kernel, task projection, evidence gate, trace grammar,
> evaluation protocol, and promotion lifecycle. Reuse or adapt mature provider,
> session, extension, sandbox, and worker runtimes behind conformance adapters.

This gives the project a genuinely different basis vector from Hermes, Pi,
OpenClaw, OpenHands, Codex, or Claude Code. Those systems primarily organize
tools, context, permissions, sessions, and execution. Our kernel additionally
asks:

```text
Which abstraction frame applies?
Which AB objects may this role inspect, propose, or control for this task?
What decomposition and effect signatures make those objects real?
What evidence closes the claimed effect?
What trace changed our capability belief?
When may a repeated structure become a reviewed higher-level object?
```

The release strategy is deliberately conservative:

```text
H0  AB contract spine and deterministic gate             IMPLEMENTED PROOF
H1  Executable harness runtime and complete lifecycle     NEXT ENGINEERING TARGET
H2  Cooperative NAO adapter and parity ablation           FIRST REAL ENVIRONMENT
H3  Trace-adaptive Neural Workbench                       OFFLINE-FIRST ADAPTATION
H4  Crystallization and reviewed AB promotion             QUARANTINED LEARNING
H5  Cross-runtime federation and conformance              UNIVERSAL HARNESS v1
H6  AB5 policy-foundry hypothesis                         RESEARCH ONLY, NOT COMMITTED
```

The true Universal Agentic Harness is reached at H5, not when every feature of
every existing harness has been copied. H5 means that the unchanged semantic
kernel can govern more than one model family, runtime style, and task domain
through explicit adapters and comparable traces.

## 2. Target Contract

### Required outcome

Build a portable harness that can turn a model into a bounded subsystem agent
by compiling a task-specific interaction module from a versioned AB object
graph. The resulting system must:

- remain independent of ROS, NAO, any single model provider, and any single
  agent runtime;
- preserve domain ownership instead of absorbing the environment into the
  harness;
- expose the minimum sufficient AB closure for the current role and task;
- distinguish inspection, proposal, control, execution, and effect-claim
  authority;
- preserve an append-only, replayable, versioned execution trace;
- bind completion to deterministic evidence obligations;
- support local models, API models, and external frontier harness workers;
- adapt search and context from measured traces without silently rewriting the
  trusted runtime;
- propose reusable AB2+ interaction objects through quarantine, holdout, and
  owner review;
- compare model-harness configurations under frozen task and environment
  contracts.

### Non-goals

- A global ontology in which one absolute AB number has the same meaning in all
  systems.
- A universal bag of all tools, skills, prompts, and memories.
- Another LLM layer inserted between the current chatbot and planner.
- Automatic prompt rewriting without a bounded SkillOpt train/holdout ledger.
- Online publication of agent-authored skills into a trusted registry.
- Treating MCP discovery as authorization, effect truth, or evidence closure.
- Treating a lower energy score or lower entropy proxy as proof of correctness.
- Moving dialogue, planning, execution, knowledge, perception, or speech
  ownership out of their current NAO packages.
- Calling an adaptive AB4 system AB5 merely because it becomes more capable.

### Protected owners in the NAO reference environment

| Seam | Authoritative owner | Harness relationship |
| --- | --- | --- |
| Dialogue lifecycle and speech | `dialogue_manager` | Observe and relay typed dialogue outputs; never create a second speech authority |
| Route selection and planner handoff | `chatbot_llm` | Compile role projection and validate output reach; do not create executable plans |
| Planning and supervision | `planner_llm` | Validate plan object reach and evidence obligations; do not execute skills |
| Admission, deterministic dispatch, lineage, feedback | `nao_orchestrator` | Consume accepted plans and return authoritative execution events |
| KB query and mutation transport | `kb_skills` | Expose typed capabilities; never permit direct hidden writes |
| Scene facts and detector normalization | `nao_scene_grounding` | Supply bounded observations; never treat model claims as perception |
| Runtime effect evidence | AB1 skill owner | Close effects using fresh execution-time evidence |
| Canonical AB object graph | Neural Workbench `skill_common` registry | Read by snapshot/hash; changed only through registry governance |

### Acceptance definition

The implementation is accepted only when the same task contract can be replayed
with fixed models and environments and demonstrates:

```text
correct scope + correct ownership + valid output + evidence-complete effect
+ reconstructable trace + bounded cost/latency + no protected-path regression
```

## 3. Baseline Evidence

### Repository state checked on 2026-07-22

- Parent checkout: `feat/TFM-LLM_planner`, clean and synchronized with origin.
- `src/chatbot_llm`: `refactor/IRR-turn-engine`, clean and synchronized.
- Neural Workbench: `feat/base-implementation`; one user-owned documentation
  relocation is present and deliberately untouched.
- `python3 scripts/ros4hri_change_audit.py --mode working`: no ROS package
  changes detected before this documentation pass.
- `PYTHONPATH=src/ab_harness .venv/bin/python -m pytest -q
  src/ab_harness/test`: seven tests passed.
- `.venv/bin/python scripts/check_skill_registry_consistency.py`: canonical
  registry and projections passed consistency checks.
- System `python3` lacks PyYAML; this is an interpreter-environment gap, not a
  registry inconsistency.

### What H0 actually implements

The parent-only `src/ab_harness` package is a real portable contract proof. It
contains no ROS or NAO imports in the core and currently proves:

| Implemented seam | Source | Current proof |
| --- | --- | --- |
| Frame-relative control band | `contracts.py` | Ordering, direct-control range, inspectable range |
| Read-only canonical registry snapshot | `registry.py` | JSON load, object views, SHA-256 content identity |
| Task projection and decomposition closure | `projection.py` | Requested objects plus inspectable lower decomposition |
| Deterministic role/output gate | `gate.py` | Output ownership, reachability, direct AB level, effect-claim rejection |
| Append-only trace proof | `trace.py` | JSONL append and round-trip reconstruction |
| NAO compatibility views | `nao_h0.py` | Chatbot route and planner-step mapping without nested package imports |
| Focused fail-closed tests | `test_h0_nao_harness.py` | Accepted paths, unknown object, inspection-only AB0, effect claim, trace replay |

### What H0 does not yet implement

The current proof is intentionally narrower than the desired H0 contract in the
earlier foundation document. These are open seams, not failures:

- no serialized `HarnessSpec`, `TaskSpec`, `ModelProfile`, permission policy,
  environment identity, or versioned schema envelope;
- registry object views do not yet carry full input/output schemas, owner
  authority, side-effect class, freshness, or evidence obligation types;
- projection starts from requested object IDs rather than compiling them from a
  complete task/effect contract;
- the output gate does not yet validate payload schemas, canonical aliases,
  preconditions, permissions, budgets, or evidence closure;
- the trace captures projection and gate decisions, not the full observation,
  model-call, validation, handoff, execution, feedback, cancellation, and
  terminal-evidence lifecycle;
- the chatbot adapter assumes `user_intent.type` can be interpreted as one
  object reference, which is not sufficient for all multi-intent or target
  cases;
- no live chatbot, planner, ROS, simulator, or external worker path invokes the
  package;
- no same-model harness ablation has measured uplift.

Therefore the correct status is:

> H0 contract proof implemented; H0 lifecycle completion and H1 runtime
> integration remain open.

### Existing seams worth preserving

`chatbot_llm` and `planner_llm` already paid much of the software-engineering
cost of a harness. Current source includes prompt-pack loading, provider
configuration, structured schemas, JSON cleanup, retries, skill projection,
grounded-context construction, route/plan validation, supervision, and trace
stages. The extraction objective is not to rewrite those working domain rules.
It is to separate provider-neutral mechanisms behind compatibility adapters and
measure parity one seam at a time.

## 4. AB Semantics: Capability Is Not Abstraction Order

### Frame-relative coordinates

An AB level is a structural coordinate relative to an explicit abstraction
frame, not a global intelligence score:

```text
F_AB = (frame_id, substrate, atomicity_rule, owners, registry_version)

coord_F(o) =
  (level, kind, input_signature, output_signature, effect_signature,
   decomposition, evidence_obligations)
```

In the NAO runtime frame, AB0 effect primitives compose AB1 callable skills. In
the ML architecture frame, tensor primitives and learned operators compose AB2
motifs, AB3 models, and AB4 coupled systems. Cross-frame relations must be
explicit mappings, never inferred from equal numbers.

### Why an LLM plus harness is AB4

The served LLM is an AB3 model object in the ML architecture frame. A harness
couples it to state, tools, memory, environment, verifiers, execution policy,
and feedback:

```text
H_AB4 = (O, Delta, M, E, V, R, Pi, Phi)

O      admitted AB objects and interaction modules
Delta  allowed graph, adapter, prompt, policy, or model transformations
M      trace, artifact, and capability memory
E      environment and runtime adapters
V      schema, effect, safety, and task verifiers
R      provider/model router
Pi     lifecycle and control policy
Phi    promotion, rollback, and governance policy
```

The coupled system is AB4 because the task-facing object is no longer the model
alone. It is the coordinated model-environment policy with typed effects.

### Static and dynamic learning inside AB4

The system can learn in two distinct channels:

| Channel | Examples | Representation |
| --- | --- | --- |
| Static/offline | model weights, adapters, fine-tuning, post-training | versioned `delta_AB` over model or policy objects |
| Dynamic/runtime-derived | trace retrieval, capability posterior, heuristic priors, context projection | versioned harness state and reviewed interaction objects |

Both can improve the same AB4 system. They may increase task success, reduce
uncertainty, reduce latency, improve calibration, or expand the verified
feasible region. None of those improvements automatically changes the AB level.

Use a maturity vector instead of abusing the level:

```text
maturity(o, F, t) =
  (coverage, reliability, calibration, evidence_completeness,
   recovery_quality, portability, cost_efficiency, safety)
```

### When AB5 would be a defensible claim

AB5 requires a new object boundary. A candidate AB5 object would govern a
family of AB4 systems by creating, comparing, revising, and retiring their
policies or abstraction frames under an independent contract:

```text
AB4: executes and adapts one coupled agent-environment policy

AB5 candidate: designs/governs a population of AB4 policies or Workbenches,
               proves cross-system effects, and maintains their lifecycle
```

The minimum AB5 evidence gate is:

1. the object accepts AB4 systems or policies as typed inputs;
2. it produces a materially new AB4 arrangement, policy, or frame mapping;
3. an independent evaluator measures effects across held-out systems/tasks;
4. the creator cannot publish its own proposal directly;
5. rollback and provenance preserve the replaced AB4 state;
6. the behavior cannot be equivalently described as ordinary search within one
   fixed AB4 policy space.

Until those conditions are met, the honest description is an increasingly
capable, adaptive, and efficient AB4 system.

## 5. Canonical System Architecture

```text
                         UNIVERSAL SEMANTIC PLANE

 TaskContract + AgentRole + ABFrame + RegistrySnapshot + EnvironmentIdentity
                                  |
                                  v
                    Task Interaction Compiler
       projection + decomposition + permissions + evidence closure
                                  |
                                  v
                     InteractionModuleSpec
                                  |
              +-------------------+-------------------+
              |                                       |
              v                                       v
      Model/Harness Adapter                      Runtime Adapter
 local API | remote API | Pi | Codex       ROS | shell | browser | MCP
 Claude | Hermes | OpenClaw | OpenHands    API | sandbox | simulator
              |                                       |
              +-------------------+-------------------+
                                  v
                     Deterministic Control Plane
          schema gate + AB reach + permissions + budgets + approvals
                                  |
                                  v
                    Environment-owned execution
                                  |
                                  v
              Observation + EffectEvidence + TerminalState
                                  |
                                  v
                       Append-only Trace Store
                                  |
                   +--------------+--------------+
                   |                             |
                   v                             v
             Evaluation Plane             Neural Workbench
       milestones/minefields/holdout   posterior/search/crystallization
```

### Six planes

| Plane | Owns | Must not own |
| --- | --- | --- |
| Semantic | AB frames, objects, effects, decomposition, role authority | Provider transport or domain execution |
| Interaction compiler | Minimal task projection and evidence closure | Free-form prompt policy |
| Runtime | Lifecycle, cancellation, budgets, approvals, adapter calls | Domain truth not returned by owners |
| Trace | Versioned events, artifacts, lineage, environment identity | Unverified summaries as source of truth |
| Evaluation | Milestones, minefields, terminal acceptance, cost and process metrics | Self-reported model success |
| Workbench | Candidate portfolios, priors, uncertainty, proposal quarantine | Direct publication or trusted execution |

### Core contracts

The first stable grammar should contain:

| Contract | Purpose |
| --- | --- |
| `HarnessSpec` | Kernel version, policies, adapters, trace/eval configuration |
| `TaskSpec` | Goal, role, frame, required effects, prohibited effects, budgets, terminal checks |
| `AbstractionFrame` | Substrate, atomicity rule, owners, registry version |
| `ABObjectSpec` | Typed object, decomposition, effects, observables, permissions, callable status |
| `ABControlBand` | Inspect, propose, direct-control, and effect-claim bounds |
| `InteractionModuleSpec` | Closed task projection presented to one role |
| `ModelProfile` | Provider protocol, schema/tool capabilities, context, latency, trust tier |
| `EnvironmentProfile` | Runtime identity, adapters, resources, security and freshness policy |
| `ActionProposal` | Model-proposed typed operation without effect truth |
| `GateDecision` | Deterministic acceptance/rejection with machine-readable reasons |
| `EffectEvidence` | Owner-issued observation satisfying a declared effect obligation |
| `TraceEvent` | Append-only lifecycle event with lineage and artifact references |
| `CapabilityProfile` | Failure-aware empirical belief over object/task/environment slices |
| `InteractionSkillProposal` | Quarantined higher-order composition with proofs and counterexamples |

## 6. H0-H6 Delivery Spine

### H0: AB contract spine

**Purpose:** Prove that one model role can be scoped to a real task-relative AB
projection and rejected deterministically when it exceeds that projection.

**Current state:** The parent package proves the smallest core, but not the full
lifecycle.

**Complete H0 deliverables:**

- serialize and version `HarnessSpec`, `TaskSpec`, `ModelProfile`, and
  `EnvironmentProfile`;
- expand `ABObjectView` with input/output, effect, evidence, freshness, side
  effect, permission, and owner fields;
- compile projection from required task effects and role policy, not only a
  caller-provided list of object IDs;
- distinguish `inspect`, `propose`, `direct`, `execute`, and `claim_effect`
  authority;
- add payload-schema and canonical-alias validation;
- define complete lifecycle event types and trace identity;
- create golden fixtures for dialogue, KB query, execution handoff, rejected
  reach, cancellation, and failed evidence closure.

**Acceptance gate:**

```text
all current H0 tests
+ schema round-trip/version tests
+ fail-closed permission and evidence tests
+ one complete synthetic lifecycle replay
+ no ROS or nested LLM import in core
```

### H1: Executable runtime kernel

**Purpose:** Turn the contracts into a small runnable harness without migrating
NAO nodes.

**Deliverables:**

- deterministic lifecycle state machine: compile, observe, propose, gate,
  dispatch, receive, verify, recover, terminate;
- provider interface for local and OpenAI-compatible API calls;
- structured-output adapter with explicit parse/repair provenance;
- runtime adapter protocol for shell/API/simulator/MCP/worker calls;
- cancellation, time, token, tool, retry, and cost budgets;
- human approval and policy-intervention events;
- append-only event store with artifact addressing;
- reference CLI and synthetic deterministic environment;
- replay runner that can replace the model with recorded proposals.

**Key design rule:** H1 owns control mechanics, not environment semantics. A
runtime adapter returns typed observations; only the environment contract says
what those observations prove.

**Acceptance gate:** A frozen synthetic task suite must cover success, invalid
proposal, unavailable tool, timeout, cancellation, retry exhaustion, stale
evidence, and false completion. Every case must be reconstructable from events.

### H2: Cooperative NAO adapter

**Purpose:** Prove the harness against a real, already-engineered multi-node
system without moving ownership or degrading behavior.

**Migration order:**

1. read-only registry projection and trace correlation;
2. chatbot output shadow-gating;
3. planner output shadow-gating;
4. provider capability normalization behind current callers;
5. optional cooperative gate for one low-risk path;
6. failure, cancellation, supersede, replan, and duplicate-speech validation;
7. live robot validation only after fake/sim parity.

**Shadow mode:** The harness computes projection and decisions but cannot block
or dispatch. Differences against current validators are logged. This gives us
counterexamples before authority changes.

**Cooperative mode:** One current node explicitly calls a compatibility adapter
at a narrow seam. Existing node validation remains authoritative until the
ablation proves equivalence.

**Required task set:**

| Family | Success case | Adversarial/failure case |
| --- | --- | --- |
| Dialogue | ordinary social turn | action wording must not create execution |
| Knowledge | current scene query | stale memory must not become current truth |
| Execution | grounded single-skill request | unknown/ambiguous target clarification |
| Plan | valid multi-step skill plan | unregistered or inspection-only AB0 step |
| Recovery | retryable skill failure | exhausted retry or non-retryable failure |
| Speech | one acknowledgement and one terminal result | no duplicate utterance authority |

**Acceptance gate:** Same model, prompt pack, task set, fixture, and launch
profile produce behavioral parity or an explicitly reviewed improvement. No
prompt wording change enters this phase without SkillOpt.

### H3: Trace-adaptive Neural Workbench

**Purpose:** Let measured experience shape candidate generation, context, and
recovery without modifying trusted runtime policy online.

**Deliverables:**

- normalized complete traces indexed by task, AB object, mechanism,
  environment version, model-harness configuration, and failure class;
- conservative capability posterior with explicit unknown state;
- retrieval of supporting and counterexample traces;
- mechanism-diverse candidate portfolio rather than paraphrased duplicates;
- deterministic graph verifier and hard-constraint filter;
- Pareto vector for success, risk, latency, cost, evidence, and uncertainty;
- symbolic entropy proxy only for declared observable variables;
- reviewed pulse heuristics with provenance and expiry;
- exploration floor so early trace errors cannot permanently collapse search.

**Running uncertainty:**

```text
H_proxy(task, t) =
  w1 * missing_required_inputs
+ w2 * unresolved_target_count
+ w3 * unknown_preconditions
+ w4 * (1 - grounded_confidence)
+ w5 * unresolved_effect_obligations
+ w6 * failure_or_staleness_uncertainty

delta_H_proxy = H_proxy_before - H_proxy_after
```

This is a measured proxy, not Shannon entropy unless a calibrated probability
distribution exists. Every term must name its observable and owner.

**Acceptance gate:** On held-out tasks, success-plus-failure retrieval must beat
no-memory and success-only baselines without degrading calibration, scope,
evidence completeness, or latency beyond budget.

### H4: Crystallization and reviewed AB promotion

**Purpose:** Convert repeated, causally supported interaction structures into
maintained higher-level AB proposals.

**Candidate lifecycle:**

```text
observed fragment
  -> normalized candidate
  -> decomposition/effect closure
  -> removal and substitution counterfactuals
  -> replay across supporting and opposing traces
  -> held-out task/environment tests
  -> security and owner review
  -> quarantine
  -> approved registry proposal
  -> monitored activation
  -> retain, revise, deprecate, or roll back
```

**Promotion is blocked by:**

- frequency without causal or counterfactual support;
- hidden side effects or unowned evidence;
- compression that erases a recovery or cancellation seam;
- performance measured only on training traces;
- proposer acting as sole verifier;
- unversioned model, prompt, environment, or registry state;
- absent rollback/decompression path.

**Acceptance gate:** At least one reviewed AB2+ proposal must compress a real
solution family while preserving effects, observability, recovery, and holdout
performance. Automatic runtime publication remains prohibited.

### H5: Universal federation and conformance

**Purpose:** Prove that the semantic kernel is independent of model provider,
agent runtime, and task domain.

**Required adapter classes:**

| Adapter | Minimum proof |
| --- | --- |
| Direct local/API model | capability handshake, structured output, token/cost/latency trace |
| Minimal embedded worker | Pi SDK or RPC adapter with explicit active tools |
| Sandboxed execution worker | OpenHands-style action/observation adapter with reproducible environment identity |
| Persistent personal-agent worker | Hermes or OpenClaw adapter with bounded toolset, memory scope, and session lineage |
| Frontier coding worker | Codex or Claude Code process/SDK adapter with permissions and artifact extraction |
| Protocol bridge | MCP discovery mapped into AB objects without granting implicit authority |
| Non-NAO domain | iTrader, Watson, or a synthetic AB4 system using the unchanged core |

**Conformance suite:**

- adapter capability negotiation;
- task projection equivalence;
- permission and approval equivalence;
- cancellation and timeout behavior;
- artifact and trace normalization;
- effect-evidence mapping;
- model-harness matrix evaluation;
- environment/version replay;
- provider/runtime failure isolation;
- no domain imports in the semantic core.

**Acceptance gate:** The same kernel version must run at least two model styles,
two runtime styles, and two domains, including one non-NAO AB4 task. Results are
reported as model-harness-environment configurations, not model scores alone.

### H6: Optional AB5 policy foundry

**Purpose:** Test whether the Workbench can govern a population of AB4 harnesses
or policies rather than merely tune one.

**Status:** Research hypothesis. It is not required for Universal Harness v1.

**Candidate experiments:**

- propose different AB4 harness configurations for a task distribution;
- train on one model/runtime subset and hold out other systems;
- compare, retire, and roll back whole harness policies;
- infer or revise cross-frame mappings with independent validation;
- test whether the operation is genuinely higher-order rather than ordinary
  H3/H4 search over a fixed AB4 space.

**Acceptance gate:** All six AB5 conditions in Section 4 must pass. Otherwise
the result remains an advanced AB4 Workbench.

## 7. Mapping Existing Phase Names

The earlier documents use three phase systems. They are retained for
provenance, but the H-series is the canonical product spine.

| Product release | Parent extraction phases | Adaptive research phases | Meaning |
| --- | --- | --- | --- |
| H0 | P0-P2 in part | A0-A1 in part | Contracts, frames, projection, deterministic gate, initial trace |
| H1 | P1-P2 | A2-A3 | Runtime state machine, graph verifier, complete trace |
| H2 | P3 | A3 and A7 | Cooperative NAO migration and parity ablation |
| H3 | P4 and P7 in part | A4-A5 | Capability posterior, entropy proxy, retrieval, reviewed heuristics |
| H4 | P7 | A6 and A9 | Counterfactual crystallization and versioned deltas |
| H5 | P5-P6 | A8 | External workers, serving control, and cross-domain proof |
| H6 | future | beyond A9 | Higher-order AB4 policy governance research |

The parent `P#` phases describe extraction/integration work. The adaptive `A#`
phases describe research mechanisms. The `H#` releases describe usable system
capability and are what implementation status should report.

## 8. Package and Subsystem Plan

The first implementation remains in this repository to reduce migration risk.
Relocation occurs only after H2 parity and one non-NAO adapter prove that moving
the package changes packaging, not meaning.

| Proposed module | Responsibility | H release |
| --- | --- | --- |
| `ab_harness.contracts` | Frozen portable schemas and versions | H0 |
| `ab_harness.registry` | Read-only snapshots, frame maps, object resolution | H0 |
| `ab_harness.compiler` | Task/effect to minimal AB closure | H0-H1 |
| `ab_harness.policy` | Role authority, permissions, budgets, approvals | H1 |
| `ab_harness.gate` | Schema, reach, effect, evidence, and terminal checks | H0-H1 |
| `ab_harness.runtime` | Lifecycle state machine and adapter orchestration | H1 |
| `ab_harness.trace` | Event grammar, JSONL/artifact stores, replay | H0-H1 |
| `ab_harness.eval` | Milestones, minefields, acceptance, cost/process metrics | H1-H2 |
| `ab_harness.providers` | Direct local/API model capability normalization | H1 |
| `ab_harness.adapters.nao` | Temporary cooperative chatbot/planner/ROS views | H2 |
| `ab_harness.adapters.workers` | Pi, OpenHands, Hermes, OpenClaw, Codex, Claude | H5 |
| `ab_harness.adapters.mcp` | Protocol discovery and transport mapping | H5 |
| `neural_workbench.search` | Candidate families, graph verifier, scoring | H3 |
| `neural_workbench.capability` | Posterior, calibration, entropy proxies | H3 |
| `neural_workbench.crystallization` | Counterfactuals, quarantine, promotion | H4 |
| `skill_common` | Canonical governed multi-frame AB object graph | all |

### Dependency rule

```text
portable contracts -> no ROS, NAO, provider SDK, or frontier harness imports
domain adapters     -> may depend on domain contracts
worker adapters     -> may depend on worker protocol/SDK
Workbench learning  -> consumes traces and registry snapshots, never executor internals
```

## 9. Build, Borrow, or Wrap

### Decision matrix

| System | Strong mechanism to reuse | Why not use as semantic core | Integration decision |
| --- | --- | --- | --- |
| Pi | Small loop, four-tool default, JSONL session tree, SDK/RPC, event interception, explicit tool allowlist | Security and orchestration intentionally delegated to extensions/environment; no AB/effect semantics | Prototype first embedded-worker adapter; borrow event/session patterns |
| OpenHands | Typed action-observation tools, isolated runtime, reproducible images, evaluator controller | Heavy software-engineering and container assumptions; no AB promotion semantics | Reuse as optional sandbox/runtime adapter, not kernel |
| Hermes Agent | Broad providers, toolsets, skills, memory, delegates, ACP/JSON-RPC/API integration | Large personal-agent surface and autonomous skill behavior exceed minimum trusted core | Wrap bounded sessions/toolsets; study skill lifecycle but require our quarantine |
| OpenClaw | Reusable agent core, harness registry, lifecycle, sessions, plugins, multi-channel persistence | Product-scale personal-agent policy and plugin ecosystem are broader than task projection | Adapter for persistent-agent tasks; borrow registry/lifecycle concepts |
| Codex | Scoped repository instructions, skills, MCP, sandbox/approval policy, worktrees, subagents | Closed product runtime; repository semantics are not our AB registry | External worker with task package, artifact contract, and trace bridge |
| Claude Code | CLI/SDK, tool permissions, structured streaming, sessions, MCP | Closed product runtime and provider-specific policy | External worker with bounded tools, turns, permissions, and artifact extraction |
| MCP | Dynamic capability discovery, tools/resources/prompts, local/remote transport | Protocol explicitly leaves context policy and model behavior to the host | Transport adapter only; map discoveries into reviewed AB objects |
| AgentSpec | Trigger-predicate-enforcement rule model | Research DSL, not full harness/runtime | Borrow policy-rule concepts and validate generated rules independently |
| Meta-Harness | Outer-loop search over harness code using scores and traces | Search can overfit or mutate trusted code without our governance | H4 offline optimizer candidate behind quarantine and holdout |
| AutoHarness | Environment-feedback synthesis of deterministic guard/policy code | Extreme synthesis can replace policy and overfit one environment | H4 bounded candidate generator, never direct activation |

### Cannibalization rule

Before copying source rather than adapting a public interface:

1. verify the exact repository, revision, and license;
2. identify the smallest independently testable mechanism;
3. preserve attribution and license obligations;
4. wrap it behind a harness-owned protocol;
5. add conformance tests before making it required;
6. retain a local reference implementation so the semantic kernel is not held
   hostage by one external runtime;
7. reject the dependency if AB closure or evidence ownership requires an
   invasive fork.

Pi and Hermes currently declare MIT licenses in their official repositories.
The exact licenses and dependency obligations of every selected revision must
still be recorded in an adoption ADR before source reuse.

## 10. Research Synthesis

### What recent evidence supports

| Evidence | Finding | Consequence for this roadmap |
| --- | --- | --- |
| [Harness-Bench](https://arxiv.org/abs/2605.27922) | Model-harness pairings differ substantially across completion, process, cost, and failures | Report configuration-level results and preserve complete traces |
| [Claw-SWE-Bench](https://arxiv.org/abs/2606.12344) | With a fixed model, adapter design can produce very large performance differences | Treat adapters as evaluated code, not glue |
| [Life-Harness](https://arxiv.org/abs/2605.22166) | Training-trajectory interface interventions can transfer across frozen models | H3 may learn reusable environment-side structures without weight updates |
| [Harness design and post-training](https://arxiv.org/abs/2606.25447) | Harness choices affect post-training and out-of-distribution behavior | Co-design future training with the harness; do not begin there |
| [Meta-Harness](https://arxiv.org/abs/2603.28052) | Outer-loop harness code search benefits from complete prior code, scores, and traces | H4 optimizer should see artifacted candidates and holdouts |
| [AutoHarness](https://arxiv.org/abs/2603.03329) | Synthesized code harnesses can prevent invalid actions and outperform larger models in bounded games | Crystallize deterministic checks where environment rules are provable |
| [Natural-Language Agent Harnesses](https://arxiv.org/abs/2603.25723) | Control logic can be externalized behind explicit contracts and durable artifacts | Natural language may express proposals/policy, but deterministic gates remain code |
| [AgentSpec](https://arxiv.org/abs/2503.18666) | Runtime trigger/predicate/enforcement rules can be lightweight and effective; generated rules still miss cases | Keep policy typed, inspectable, and independently tested |
| [SafeHarness](https://arxiv.org/abs/2604.13630) | Harness centrality makes it a security attack surface requiring lifecycle defenses | Integrate filtering, causal checks, privilege separation, rollback, and degradation |
| [ToolSandbox](https://arxiv.org/abs/2408.04682) | Stateful tool use needs intermediate milestones and minefields, not final text scoring | Make process acceptance first-class in `ab_harness.eval` |
| [DeployBench](https://arxiv.org/abs/2606.05238) | Agents often self-stop after checking a weaker target than the task requires | Compile terminal acceptance from `TaskSpec`; never trust self-declared completion |
| [SWE-agent ACI](https://arxiv.org/abs/2405.15793) | Agent-computer interface design materially changes behavior | AB projection is an interface design experiment, not ontology decoration |
| [Architectural Design Decisions](https://arxiv.org/abs/2604.18071) | Public harnesses vary repeatedly along subagents, context, tools, safety, and orchestration; high-assurance audit is rare | Keep these dimensions separate and make audit/evidence a differentiator |
| [Agent Systems with Harness Engineering](https://openreview.net/forum?id=nM5tDHrQsx) | Harness design spans workflows, memory, skills, orchestration, context, safety, and evaluation | The Workbench belongs in a broader lifecycle, not just a planner |
| [Code as Agent Harness](https://arxiv.org/abs/2605.18747) | Code supports action, environment modeling, verification, memory, and coordination | Keep executable contracts and verifiers as durable artifacts |
| [What makes a harness a harness](https://arxiv.org/abs/2606.10106) | A harness needs loop, tool interface, context management, and control, and is distinct from an SDK or evaluator | H0 alone is a contract proof; H1 is where the system becomes a runnable harness |
| [Quine](https://arxiv.org/abs/2603.18030) | POSIX processes already provide lifecycle, isolation, composition, and resource controls | Prefer OS/runtime primitives over custom orchestration where sufficient |

### What the literature does not prove for us

- No cited result proves that AB projection improves NAO, iTrader, or Watson.
- No cited result validates our AB levels as universal constants.
- No cited result makes an entropy proxy a calibrated probability measure.
- Cross-model transfer in one benchmark does not prove cross-domain transfer.
- Harness code search does not prove safe online self-modification.
- Better final-task performance does not prove correct effect ownership.
- A personal or coding harness succeeding on software tasks does not prove it
  can safely own embodied execution.

These remain explicit experiments in H2-H6.

## 11. Evaluation and Ablation Program

### Configuration identity

Every result must identify:

```text
model + model revision + provider/runtime + harness version + adapter version
+ prompt pack hash + registry hash + environment image/state + task suite version
```

### Core metrics

| Dimension | Metrics |
| --- | --- |
| Task | terminal acceptance, milestone completion, minefield violations |
| Scope | projected object count, out-of-band proposals, permission denials |
| Evidence | obligation closure, stale evidence, fabricated claims, owner correctness |
| Process | retries, recovery success, cancellation latency, completion-judgment errors |
| Model | parse failures, repair attempts, context tokens, output tokens |
| Runtime | wall time, tool latency, sandbox startup, provider failures |
| Cost | API cost, compute time, repeated work, evaluator cost |
| Adaptation | posterior calibration, retrieval precision, delta-H proxy, counterexample use |
| Portability | adapter conformance, task-spec reuse, trace-schema equivalence |
| Safety | approval violations, privilege escalation, rollback success, unsafe behavior rate |

### Required ablations

1. Flat tool catalog versus AB-closed task projection.
2. Scalar desired level versus frame-relative role control band.
3. Current validator only versus shadow harness gate.
4. One candidate versus mechanism-diverse candidate portfolio.
5. No memory versus success-only retrieval versus success-and-failure posterior.
6. Final-result check versus compiled milestones, minefields, and terminal proof.
7. Symbolic energy only versus hard constraints, Pareto filtering, and energy.
8. No entropy term versus symbolic proxy versus calibrated distribution where
   available.
9. Frequency macro proposal versus counterfactual crystallization.
10. Direct model call versus Pi/OpenHands/frontier worker under the same task.
11. Static harness versus H3 adaptation under in-distribution and shifted tools.
12. NAO AB1 task versus non-NAO AB4 task under unchanged core schemas.

### Acceptance statistics

- freeze train/development and holdout splits before mutation;
- report per-task and aggregate results, not only averages;
- retain failed trajectories and counterexamples;
- use paired comparisons where model/task/environment are held fixed;
- report uncertainty intervals when sample size allows;
- reject changes that improve success while materially worsening evidence,
  safety, or protected-path behavior;
- do not promote from one anecdotal trace.

## 12. Approach Registry

| ID | Architecture family | Discriminating probe | Status | Exact gap or rejection reason |
| --- | --- | --- | --- | --- |
| UAH-01 | Build every harness layer from scratch | Compare development cost and conformance against a Pi/OpenHands adapter | Rejected as default | Reimplements mature sessions, providers, runtimes, and sandboxes without strengthening AB semantics |
| UAH-02 | Fork Hermes or OpenClaw as the whole product | Attempt AB closure/evidence gate without invasive changes | Bounded alternative | Attractive breadth, but product policy and plugin surfaces may dominate the semantic kernel |
| UAH-03 | Use Pi as the kernel | Implement AB projection/gate as extension and inspect trace/effect completeness | Active adapter hypothesis | Minimal core is promising; security and effect semantics must remain ours |
| UAH-04 | Use OpenHands as the kernel | Run same task through action-observation adapter and compare environment identity/replay | Active runtime hypothesis | Strong sandbox, but domain and container assumptions may be too heavy |
| UAH-05 | MCP-first object model | Encode ownership, freshness, effect proof, AB decomposition, and promotion in MCP alone | Rejected as semantic core | MCP intentionally standardizes exchange, not host context policy or effect truth |
| UAH-06 | Own semantic kernel, wrap runtimes | Complete H1 synthetic suite, then Pi/OpenHands conformance | Accepted leading route | Requires disciplined adapter boundary and independent eval suite |
| UAH-07 | Natural-language harness as trusted policy | Mutate policy prose and test deterministic safety/effect guarantees | Rejected for enforcement | Useful as proposal/configuration language, insufficient as sole trusted gate |
| UAH-08 | Online self-modifying Workbench | Let traces directly alter runtime and run adversarial replay | Rejected | Violates review, rollback, holdout, and owner boundaries |
| UAH-09 | Offline dual-loop Workbench | Compare frozen baseline to reviewed trace-derived candidates on holdout | Accepted research route | Needs representative traces and calibrated uncertainty |
| UAH-10 | Capability improvement implies higher AB | Improve an AB4 configuration and test for a new object boundary | Rejected | Performance and abstraction order are different axes |
| UAH-11 | AB5 policy foundry | Govern held-out populations of AB4 systems under independent evaluation | Blocked research route | No implemented system or evidence yet satisfies the AB5 gate |

## 13. Discriminating Probes and Results

| Probe | Observation | Route changed |
| --- | --- | --- |
| Inspect and test parent `ab_harness` | Seven tests pass; registry projection, gate, and trace proof are real | H0 is implemented in part, not merely planned |
| Compare desired H0 grammar to source | Task/provider/environment specs and complete lifecycle are absent | H0 must be completed before calling H1 operational |
| Run registry checker in system Python | Fails because PyYAML is absent | Environment issue; not evidence of registry drift |
| Run registry checker in repo venv | Consistency passes | Canonical registry remains a sound H0 substrate |
| Audit ROS package changes | No working ROS package changes | Documentation work can proceed without runtime interference |
| Inspect chatbot/planner source seams | Both already contain mature, package-owned harness mechanisms | Extraction must be cooperative and parity-gated, not a rewrite |
| Compare MCP specification to AB needs | MCP leaves model/context use to host and exposes transport primitives | Reject MCP as semantic core; retain it as adapter |
| Compare Pi, OpenHands, Hermes, OpenClaw | Each solves different runtime mechanics; none supplies our effect/promotion calculus | Accept hybrid semantic-core plus worker/runtime adapters |
| Review recent harness benchmarks | Same model changes materially across harnesses/adapters | Require frozen model-harness matrix and process traces |
| Apply AB5 boundary test | Current adaptive design tunes one AB4 system | Keep AB5 blocked until a governed population-of-AB4 object exists |

## 14. Adversarial Audit

- [x] The semantic core remains model-, ROS-, and NAO-independent.
- [x] NAO dialogue, planner, orchestrator, KB, perception, execution, and speech
  ownership remain unchanged.
- [x] The current H0 proof is separated from unimplemented lifecycle claims.
- [x] AB level is separated from capability, maturity, and performance.
- [x] AB5 has a structural evidence gate and is not used as marketing language.
- [x] MCP is transport/discovery, not authorization or effect truth.
- [x] External harnesses are adapters or substrates, not hidden semantic owners.
- [x] Completion requires task-owned evidence rather than model self-report.
- [x] Prompt changes remain under SkillOpt and are excluded from H0-H2 by
  default.
- [x] Online adaptation and offline promotion are separate loops.
- [x] Failed traces and counterevidence remain first-class.
- [x] The proposer cannot become its sole verifier.
- [x] Registry promotion remains reviewed, versioned, reversible, and
  decomposable.
- [x] Model-harness configuration is the evaluation unit.
- [ ] Full H0 schemas and lifecycle traces are not implemented.
- [ ] No synthetic H1 runtime suite exists.
- [ ] No NAO shadow-mode adapter has been run.
- [ ] No external harness adapter has passed conformance.
- [ ] Entropy proxies and capability posteriors are not calibrated.
- [ ] No crystallized AB2+ interaction object has passed counterfactual holdout.
- [ ] No non-NAO AB4 implementation has proved universality.
- [ ] No AB5 candidate has passed the structural boundary test.

## 15. Decision

**Accept** the hybrid semantic-kernel architecture and H0-H5 release spine.

**Reject** a wholesale fork of a broad agent product as the default core,
MCP-first semantics, online self-publication, and AB-level inflation from
performance alone.

**Bounded handoff:** Implement only the remaining H0 contract/lifecycle schemas
and H1 synthetic runtime before touching live chatbot/planner behavior. In
parallel, produce throwaway Pi RPC and OpenHands action-observation adapter
spikes only after the adapter protocol is frozen. Neither spike becomes a
dependency until it passes conformance.

## 16. Ordered Implementation Queue

### Now: finish H0

1. Freeze schema versioning and serialization conventions.
2. Add `HarnessSpec`, `TaskSpec`, `ModelProfile`, and `EnvironmentProfile`.
3. Expand AB object effect/evidence/permission fields through a read-only
   adapter over the canonical registry.
4. Compile one synthetic task from required effects to a closed object graph.
5. Extend the deterministic gate and reason codes.
6. Define full lifecycle `TraceEvent` variants and replay.
7. Add success, rejection, cancellation, and stale-evidence fixtures.
8. Keep the existing H0 API behind compatibility exports while tests migrate.

### Next: H1 executable kernel

1. Implement the minimal lifecycle state machine.
2. Add direct local/API model and synthetic runtime adapters.
3. Add budget, approval, cancellation, and terminal-evidence controls.
4. Implement milestones and minefields.
5. Run the frozen synthetic conformance suite.
6. Deslop only after behavior is covered; avoid framework-building beyond
   tested needs.

### Then: H2 cooperative NAO proof

1. Record frozen chatbot/planner fixtures and current test results.
2. Add read-only projection and trace bridge.
3. Run shadow gates and classify disagreements.
4. Integrate one low-risk path behind a launch/config flag.
5. Run same-model standalone versus harness-backed ablations.
6. Validate fake/sim success and failure paths.
7. Run live robot tests only when the operator and robot are available.

### After parity: H3-H5

1. Normalize complete traces and implement failure-aware posterior.
2. Add reviewed retrieval and symbolic uncertainty experiments.
3. Build counterfactual crystallization quarantine.
4. Prototype Pi and OpenHands adapters.
5. Add one persistent-agent or frontier-worker adapter.
6. Prove a non-NAO AB4 task with the unchanged core.
7. Publish a conformance matrix and declare Universal Harness v1 only after H5.

## 17. Key Targets and Success Criteria

| Target | Near-term measure | v1 success |
| --- | --- | --- |
| Scope precision | projected graph smaller than flat catalog without missing required objects | minimal sufficient projection across domains |
| Effect integrity | no model-only effect claims accepted | all terminal effects closed by owner evidence |
| Traceability | synthetic task reconstructable | cross-runtime comparable lifecycle traces |
| Portability | no domain imports in core | two domains, two runtimes, two model styles |
| Recovery | explicit failure/cancel paths | measured recovery without hidden retries or duplicate effects |
| Adaptation | posterior preserves unknown and counterexamples | holdout uplift without safety/evidence regression |
| Crystallization | proposal quarantine exists | one reviewed AB2+ object with rollback |
| Efficiency | bounded context/tool surface | improved success-cost frontier against flat baseline |
| Governance | versioned policies and approvals | reproducible activation, deprecation, and rollback |

## 18. Continuous Research Points

### AB frame comparability

How can different frames remain locally meaningful without becoming arbitrary?
The next theorem-sized task is to define frame invariants and partial mappings
that preserve effect signatures and decomposition depth where possible.

### Projection optimality

Minimal tool exposure may remove useful affordances. We need projection
precision and recall, not only smaller catalogs. Counterfactual task replays can
measure whether omitted objects were genuinely unnecessary.

### Entropy calibration

Symbolic uncertainty is useful only if it correlates with terminal acceptance
and human/domain labels. The project must distinguish count-based proxies,
Bayesian/posterior uncertainty, model uncertainty, and world-state uncertainty.

### Harness-model co-adaptation

H3 should first adapt the interface around frozen models. Later work may compare
harness-aware post-training against generic post-training under tool and task
shift. Weight changes remain versioned deltas and never erase harness evidence.

### Completion judgment

The terminal verifier must compile the exact required effect and artifact
checks from the task contract. A model saying it is finished is an action
proposal, not a terminal fact.

### Security and untrusted context

AB scope reduces reach but does not eliminate prompt injection, malicious tool
descriptions, credential leakage, or poisoned memories. Security must follow
the lifecycle: input provenance, causal/action checks, privilege separation,
artifact scanning, rollback, and degradation.

### Multi-agent and AB4 composition

Subagents are not automatically higher AB objects. A multi-agent system is AB4
when the coupled coordination policy, memory, environment, and verification
form the task-facing object. More agents can also mean more uncorrelated state,
cost, and authority; orchestration must earn its complexity in ablation.

### AB5 boundary

The decisive experiment is not “can the system optimize itself?” It is “does a
new governed object operate over a family of AB4 systems with independently
verified cross-system effects?” Until then, research should report AB4 maturity
and `delta_AB`, not AB5.

## 19. Residual Risk and Next Probe

| Risk | Current evidence gap | Next discriminating probe |
| --- | --- | --- |
| Core becomes another overbuilt framework | H1 does not exist | Implement one synthetic loop with deletion budget and no plugin system |
| AB projection harms model flexibility | No same-model ablation | Flat versus projected task suite with missing-object analysis |
| External harness semantics leak inward | No adapter conformance | Pi RPC spike using only frozen contracts and normalized events |
| Sandbox dominates latency | No measured runtime matrix | Direct local runtime versus OpenHands sandbox on identical tasks |
| Trace priors amplify early errors | No posterior calibration | Inject controlled false successes and measure recovery with counterevidence |
| Entropy proxy rewards easy observables | No correlation study | Compare proxy delta to terminal acceptance and domain labels |
| Crystallization hides unsafe detail | No promoted object | Decompress candidate and verify every effect/recovery seam on replay |
| Frontier worker cannot expose complete trace | Product APIs differ | Define minimum artifact/event contract and classify unavailable fields |
| AB5 remains relabeled optimization | No higher-order object | Require held-out population-of-AB4 governance experiment |

The immediate next probe is H0 lifecycle completion: serialize a `TaskSpec`,
compile one synthetic object closure, pass one action through the expanded gate,
and reconstruct success and stale-evidence failure from append-only events. This
tests the semantic center without touching the live NAO stack or committing to
an external harness.

## 20. Primary Sources and Implementation References

### Research

- [Harness-Bench](https://arxiv.org/abs/2605.27922)
- [Claw-SWE-Bench](https://arxiv.org/abs/2606.12344)
- [Adapting the Interface, Not the Model / Life-Harness](https://arxiv.org/abs/2605.22166)
- [The Interplay of Harness Design and Post-Training](https://arxiv.org/abs/2606.25447)
- [Meta-Harness](https://arxiv.org/abs/2603.28052) and its
  [reference implementation](https://github.com/stanford-iris-lab/meta-harness)
- [AutoHarness](https://arxiv.org/abs/2603.03329)
- [Natural-Language Agent Harnesses](https://arxiv.org/abs/2603.25723)
- [AgentSpec](https://arxiv.org/abs/2503.18666)
- [SafeHarness](https://arxiv.org/abs/2604.13630)
- [ToolSandbox](https://arxiv.org/abs/2408.04682)
- [DeployBench](https://arxiv.org/abs/2606.05238)
- [SWE-agent Agent-Computer Interface](https://arxiv.org/abs/2405.15793)
- [Architectural Design Decisions in AI Agent Harnesses](https://arxiv.org/abs/2604.18071)
- [Agent Systems with Harness Engineering](https://openreview.net/forum?id=nM5tDHrQsx)
- [Code as Agent Harness](https://arxiv.org/abs/2605.18747)
- [What makes a harness a harness](https://arxiv.org/abs/2606.10106)
- [Quine: LLM Agents as Native POSIX Processes](https://arxiv.org/abs/2603.18030)
- [OSWorld](https://arxiv.org/abs/2404.07972)

### Harnesses and protocols

- Pi [coding-agent README](https://github.com/earendil-works/pi/blob/main/packages/coding-agent/README.md)
  and [extensions](https://github.com/earendil-works/pi/blob/main/packages/coding-agent/docs/extensions.md)
- OpenHands [runtime architecture](https://docs.openhands.dev/openhands/usage/architecture/runtime)
  and [typed tool system](https://docs.openhands.dev/sdk/arch/tool-system)
- Hermes Agent [repository](https://github.com/NousResearch/hermes-agent) and
  [programmatic integration](https://github.com/NousResearch/hermes-agent/blob/main/website/docs/developer-guide/programmatic-integration.md)
- OpenClaw [runtime architecture](https://docs.openclaw.ai/agent-runtime-architecture)
  and [repository](https://github.com/openclaw/openclaw)
- Model Context Protocol [architecture](https://modelcontextprotocol.io/docs/learn/architecture)
- Anthropic [Claude Code CLI and permissions](https://docs.anthropic.com/en/docs/claude-code/cli-usage)
- OpenAI Codex documentation: [AGENTS.md](https://developers.openai.com/codex/guides/agents-md),
  [skills](https://developers.openai.com/codex/skills), and
  [security](https://developers.openai.com/codex/security)

### Internal foundations

- `docs/agentic_harness/universal_agentic_harness_foundation.md`
- `docs/agentic_harness/neural_workbench_adaptive_ab_harness.md`
- `src/Neural-Wokbench/docs/neural_workbench/08_entropy_machines_and_capability_space.md`
- `src/Neural-Wokbench/docs/neural_workbench/Neural_Workbench_AB_ML_Object_Theory.html`
- `src/Neural-Wokbench/docs/plans/Neural_Workbench_Formal_Masterplan_Extended.html`
- `src/Neural-Wokbench/docs/plans/neural_workbench_codex_handoff.md`
