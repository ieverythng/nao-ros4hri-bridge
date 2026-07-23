# Universal Agentic Harness: AB-Aware Foundation and Implementation Plan

**Status:** Architecture baseline; parent-repo H0 proof implemented
**Date:** 2026-07-13
**Branch baseline:** `refactor/deslop_repo` at `4604343`
**Seed artifact:** `Universal Agentic Harness Blueprint.html` (user-provided,
2026-07-12)
**Primary reference subsystem:** NAO ROS4HRI + Neural Workbench
**Canonical delivery status:** `universal_agentic_harness_masterplan.md` (2026-07-22)

The phase tables in this foundation preserve the original extraction plan. Use
the canonical masterplan for current H0-H5 implementation status, acceptance
gates, and the AB4/AB5 boundary.

## 1. Project Claim

For bounded domain work, the practical capability of an LLM depends as much on
its interaction harness as on the model itself. The reusable system object is
not a prompt and is not a global bag of tools. It is a typed, observable,
permissioned, evaluated compiler from a task and an environment capability graph
to the smallest interaction surface that lets a model act and be verified.

The Neural Workbench supplies the missing semantic basis:

```text
AB0 = effect primitives, observations, validators, memory atoms, and transport seams
AB1 = runtime abilities with declared effects and observable success
AB2 = bounded composite abilities assembled from lower AB objects
AB3 = task strategies, recovery policies, and role policies
AB4 = subsystem operating profile over a capability graph
```

AB levels are relative to an environment contract. An HTTP call, ROS action,
shell command, or GUI event can ground an AB0 interaction atom, but the endpoint
name alone is not the semantic object. The semantic object describes the effect,
evidence, risk, ownership, and composition constraints that matter to reasoning.

The resulting thesis is:

> A universal agentic harness should compile per-task interaction modules from
> an AB capability graph. The model sees the capabilities, observations,
> constraints, and evidence paths required for the current task, rather than a
> globally exposed environment.

The deeper adaptive path is specified in
`neural_workbench_adaptive_ab_harness.md`. It extends static task projection
with frame-relative AB control bands, candidate pulse graphs, empirical
capability profiles, maintained interaction skills, and reviewed
trace-to-crystallization. The foundation remains the minimum portable kernel;
the extension is staged behind it rather than required all at once.

## 2. Target Contract

### Required outcome

Define and stage a reusable harness architecture that can:

- load different local or remote model providers without changing domain code;
- expose a task-specific subset of tools, resources, prompts, memory, and
  validators;
- represent those interactions as AB objects and typed decomposition edges;
- preserve environment ownership and deterministic execution boundaries;
- trace every model decision, tool call, result, approval, and validation step;
- measure model capability and harness uplift independently;
- support NAO first, then iTrader, Gamma/RESEARCH-GLOBAL, WatsonOW, and other
  subsystems through adapters.
- permit a temporary reference port in the NAO repository without allowing ROS
  or NAO imports into the universal contracts or kernel.

### Non-goals for the first implementation

- Do not replace `chatbot_llm`, `planner_llm`, or `nao_orchestrator` with one
  global agent.
- Do not expose every available tool to every task.
- Do not make MCP the internal semantic model; MCP is one transport adapter.
- Do not auto-promote learned AB2+ objects into runtime-callable abilities.
- Do not move prompt policy before behavior parity and SkillOpt gates exist.
- Do not claim self-improvement from stored traces until an evaluation shows a
  measurable probability or energy change.

### Protected ownership

| Seam | Current owner | Harness relationship |
| --- | --- | --- |
| Dialogue lifecycle and speaking | `dialogue_manager` | Harness may request a dialogue act; it does not speak directly. |
| User-facing dialogue and route declaration | `chatbot_llm` | Supplies a dialogue task adapter and structured output contract. |
| Planning and supervision | `planner_llm` | Supplies a planning task adapter and owns retry/replan policy. |
| Runtime payload normalization | `planner_common` | Supplies ROS-facing compatibility contracts. |
| Canonical AB graph | Neural Workbench `skill_common` | Becomes the capability substrate. |
| Deterministic dispatch and feedback | `nao_orchestrator` | Remains the execution authority. |
| Fresh action evidence | AB1 skills | Remains the proof of effects. |
| KB transport | `kb_skills` | Remains the KnowledgeCore boundary. |

## 3. Formal Model

Let the complete environment capability graph be:

```text
G_AB = (V, E_d, E_g, E_e, E_p)

V   = AB objects
E_d = decomposition edges
E_g = grounding edges to environment transports
E_e = evidence edges from action to observable success/failure
E_p = policy edges for permission, risk, ownership, and approval
```

For task `tau`, current state `x`, and policy profile `rho`, the harness compiler
builds a task-specific interaction module:

```text
I_tau = Project(G_AB, tau, x, rho)
```

`I_tau` is the smallest closed subgraph that contains:

1. candidate abilities able to produce the requested effect;
2. all lower-level objects required by their decomposition;
3. observations needed to establish preconditions;
4. validators and evidence surfaces needed to prove the result;
5. recovery, escalation, and approval paths allowed by policy.

The full agentic system is:

```text
S = (M, H, G_AB, I_tau, C, P, R, Phi)

M     = model or model pool
H     = harness kernel and task compiler
G_AB  = environment capability graph
I_tau = compiled per-task interaction module
C     = context, state, and memory projection
P     = policy, permissions, approvals, and budgets
R     = runtime adapter and execution substrate
Phi   = measured capability profile
```

This separates four things that many harnesses merge:

- capability graph: what the environment can do and observe;
- task interaction graph: what this model may use for this task;
- execution graph: what actually ran;
- evidence graph: what proves or disproves the claimed effect.

## 4. Core Architecture

```mermaid
flowchart TB
    Task["Task request + acceptance contract"] --> Compiler["Task compiler"]
    State["Current state + memory + traces"] --> Compiler
    AB["Canonical AB capability graph"] --> Compiler
    Policy["Risk, scope, permissions, approvals"] --> Compiler

    Compiler --> Module["InteractionModuleSpec: task-scoped AB subgraph"]
    Module --> Context["Context and visibility projection"]
    Module --> Tools["Tool/resource/validator projection"]
    Module --> ModelRouter["Model capability router"]

    ModelRouter --> Model["Local or remote LLM"]
    Context --> Model
    Tools --> Model
    Model --> Candidate["Structured candidate action/plan"]

    Candidate --> Verifier["Schema + AB + policy verifier"]
    Verifier --> Runtime["Environment adapter"]
    Runtime --> Evidence["Typed result + observations"]
    Evidence --> Trace["Append-only trace and eval bus"]
    Trace --> State
    Trace --> Profile["Capability and entropy profile"]
```

### Kernel planes

| Plane | Responsibility | Must remain replaceable |
| --- | --- | --- |
| Task contract | Objective, non-goals, acceptance, budgets, stop conditions | Task author and subsystem adapter |
| Capability | AB objects, decomposition, effects, evidence, risk | Registry backend |
| Interaction | Per-task tool/resource/prompt/validator projection | Compiler strategy |
| Model | Provider, model, structured output, context limits | Local/remote model backend |
| Context | Bounded state, memory, traces, source authority, freshness | Retrieval/context adapter |
| Policy | Scope, approvals, side effects, secrets, escalation | Deployment policy |
| Runtime | Shell, ROS, browser, API, MCP, sandbox, worker | Execution adapter |
| Trace/eval | Events, lineage, artifacts, scores, regression gates | Storage and evaluator |

## 5. Proposed Contract Grammar

### HarnessSpec

```yaml
schema_version: ab_harness/v0
name: nao_research_harness
subsystem: nao_ros4hri
capability_registry:
  provider: skill_common
  source: defaults/ab_registry.json
model_policy:
  allowed_profiles: [dialogue_fast, planner_structured, reviewer_deep]
  fallback_requires_capability_equivalence: true
context_policy:
  max_input_tokens: 24000
  source_precedence: [runtime, contracts, active_docs, traces, history]
  freshness_required_for: [perception, execution_result, kb_state]
execution_policy:
  default_scope: task_projection
  side_effects_require_verification: true
  approval_by_risk: true
trace_policy:
  append_only: true
  record_prompt_hash: true
  record_capability_projection: true
```

### TaskSpec

```yaml
task_id: turn_123
task_kind: execute_and_report
goal: find the red cup and report what is observed
requested_effects: [target_observed, grounded_report_available]
constraints:
  max_ab_level: 2
  allowed_side_effects: [robot_motion, perception_refresh]
  denied_side_effects: [direct_kb_write, direct_speech]
acceptance:
  required_evidence: [fresh_scene_summary, target_entity_id]
  failure_paths: [target_absent, backend_unavailable, unsafe_motion]
budgets:
  wall_time_sec: 90
  model_calls: 3
  tool_calls: 12
```

### InteractionModuleSpec

```yaml
task_id: turn_123
selected_ab_objects:
  - scan
  - find_object
  - select_scan_pattern
  - dispatch_scan_motion
  - wait_for_perception
  - read_scene_summary
  - confirm_target_visibility
  - verify_evidence_payload
exposed_actions: [scan, find_object]
exposed_resources: [scene_summary, target_reference, recent_relevant_traces]
required_validators: [registry_validation, evidence_payload_validation]
approval_points: [robot_motion]
completion_evidence: [fresh_scene_summary, target_entity_id]
```

### ModelProfile

```yaml
profile_id: planner_structured
provider: openai_compatible
endpoint_ref: local_vllm_primary
model_ref: qwen_planner
capabilities:
  tool_calling: true
  structured_output: json_schema
  reasoning_control: optional
  max_context_tokens: 65536
service_objectives:
  first_token_p95_ms: 2500
  completion_p95_ms: 12000
  max_concurrency: 8
routing:
  fallback_profiles: [planner_structured_backup]
  require_same_output_schema: true
```

### TraceEvent

```json
{
  "trace_id": "trace_123",
  "task_id": "turn_123",
  "event_id": "evt_009",
  "parent_event_id": "evt_008",
  "stage": "runtime_result",
  "ab_object_id": "find_object",
  "model_profile": "planner_structured",
  "capability_projection_hash": "sha256:...",
  "status": "succeeded",
  "input_ref": "artifact://...",
  "output_ref": "artifact://...",
  "evidence": {"entity_id": "cup_1", "captured_at_sec": 1783900000.0},
  "risk": {"class": "robot_motion", "approved": true},
  "timing": {"started_ms": 0, "finished_ms": 1840}
}
```

## 6. Current Stack: Reusable Harness Mechanisms Already Implemented

The current NAO stack has already paid for substantial harness engineering. The
correct first implementation is extraction and consolidation, not a parallel
rewrite.

| Existing implementation | Reusable mechanism | Keep domain-local | Proposed destination |
| --- | --- | --- | --- |
| `planner_llm/providers.py` | Provider interface, Ollama/OpenAI-compatible transport, timeout/error normalization, reasoning suppression | Planner-specific config defaults | `ab_harness.providers` |
| `chatbot_llm/ollama_transport.py` | Provider-shape detection, structured response format, preflight, model inventory, token/context controls | Speech-latency defaults | `ab_harness.providers` + chatbot adapter |
| Both `prompt_pack.py` files | Versioned YAML loading, packaged default resolution, validation, controlled overrides | Actual planner/chatbot policy text | `ab_harness.prompt_packs` |
| `chatbot_llm/response_parser.py` and planner JSON parsing | Structured output schema, extraction, parse failure handling | Chatbot/plan schemas | `ab_harness.structured_output` |
| `skill_common.ABRegistry` | Canonical capability lookup and planner/chatbot/workbench/dashboard projections | Canonical registry ownership | Extend in `skill_common`; do not copy |
| `planner_llm/skill_registry.py` and chatbot skill catalog | Bounded model-facing capability projection | Role-specific wording | `InteractionModuleCompiler` adapters |
| `chatbot_llm/knowledge_snapshot.py` and planner request projection | Bounded context, source shaping, freshness/role distinctions | NAO KB and scene semantics | `ContextAdapter` interface |
| `chatbot_llm/turn_engine.py` | Route consistency, fallback ladder, output sanitization, trace stages | Dialogue policy and speech constraints | Keep local; extract only generic validators |
| `planner_llm/planner_engine.py` | Generate, parse, validate, one repair, deterministic fallback | Planning heuristics and plan semantics | Keep local; use kernel provider/output APIs |
| `planner_llm/supervisor.py` | Goal/plan lineage, retry, cancellation, supersede, dialogue-act decisions | Planner ownership | Keep local; expose trace/state adapter |
| `interaction_trace_viewer` and Workbench trace memory | Structured timeline plus append-only trace storage | ROS subscription wiring | `TraceEvent` adapter and storage backend |

### Concrete duplication evidence

- Both LLM packages independently resolve YAML prompt packs and package-share
  defaults.
- Both implement Ollama/OpenAI-compatible response handling and Qwen
  no-thinking behavior.
- Both build bounded model messages and parse JSON under provider failure.
- Both project the canonical skill registry into model-facing text or JSON.
- Both implement fallback behavior after invalid or unavailable model output.
- Both emit trace stages but lack one shared event contract.

These are appropriate extraction candidates. The hundreds of tests around
route semantics, grounding, report wording, planner retry, lineage, and
duplicate speech are evidence that those policies are domain contracts, not
generic kernel code.

The migration target is cooperation, not replacement. `chatbot_llm` and
`planner_llm` remain domain agents while progressively delegating provider
probing, structured-output transport, prompt-pack mechanics, task capability
projection, and trace emission to the harness. This lets the current node
behavior serve as the oracle for each extraction slice.

## 7. Primary-Source Harness Survey

| Harness | Clear win | Adopt | Avoid copying blindly |
| --- | --- | --- | --- |
| Codex | Hierarchical `AGENTS.md`, progressively loaded skills, configurable providers/MCP permissions, sandbox and approval boundaries | Scoped instructions, skill discovery, provider/tool approval metadata | Treating repository instructions as the capability graph |
| Cursor | Scoped rules, dynamic skills and subagents, hooks, visible review, sandbox-aware tool errors, isolated background work | Task-local context, independent worker contexts, before/after hooks, explicit sandbox diagnostics | Global auto-approval or IDE-specific policy in the kernel |
| Pi | Minimal stable loop, four core tools, JSONL session tree, SDK/RPC modes, hot-reloadable extensions, tool overrides | Small kernel, event interception, explicit active-tool set, embeddable worker API | Assuming containers/permissions/plan mode are someone else's problem |
| OpenHands | Client/server sandbox runtime, action-to-observation interface, reproducible runtime images, evaluation controller | Runtime adapter contract, isolated workspaces, artifacted environment identity | Binding the universal kernel to Docker or software engineering only |
| SWE-agent | Agent-Computer Interface as a measurable design axis; exact thought/action/observation trajectories and replayable config | Evaluate interaction design independently from model choice | Benchmark-specific commands as universal tools |
| Hermes Agent | Provider independence, task/platform toolsets, persistent skills/memory, isolated delegates, multiple terminal backends | Toolset profiles, learned procedure proposals, backend adapters | Exposing a very large default tool surface or auto-writing trusted skills |
| AutoGen | Runtime manages identity/lifecycle/message delivery; same agent API across standalone and distributed runtimes | Separate agent definition from runtime placement | Making conversation topology the primary capability ontology |
| Pydantic AI | Typed contracts, durable execution integrations, streaming/MCP compatibility | Typed schemas and pluggable durability | Adopting a workflow engine before task/effect contracts stabilize |
| MCP | Dynamic discovery, capability negotiation, tools/resources/prompts, standard local/remote transports | Transport adapter and external capability discovery | Assuming MCP decides context policy, authorization, or effect truth |

### Survey conclusions

1. Interface design is a first-class performance variable. SWE-agent's ACI
   result supports measuring the harness separately from model quality.
2. Small kernels age better. Pi's minimal loop and extension system are more
   reusable than a global fixed agent graph.
3. The runtime must be replaceable. OpenHands and AutoGen both separate agent
   logic from execution placement.
4. Context should be progressively disclosed. Codex skills and Cursor skills
   reduce always-on prompt load.
5. Sandbox state must be visible to the model. Cursor reports better recovery
   after surfacing the exact permission constraint in tool results.
6. Exact trajectories are essential. SWE-agent, OpenHands, Pi, and Hermes all
   retain enough state to replay or inspect runs.
7. MCP is necessary but insufficient. Its own architecture explicitly leaves
   model/context use to the host application.
8. Toolsets are too coarse unless grounded in effects and evidence. The AB graph
   supplies the semantic closure and verification path that named tool bundles
   lack.

## 8. Hypothesis Registry

| ID | Architecture family | Mechanism | Discriminating probe | Status | Exact gap or reopen condition |
| --- | --- | --- | --- | --- | --- |
| H-01 | Shared utilities inside NAO repo | Extract duplicated provider/prompt/parser code into `planner_common` or another parent package | Import from both nodes and run existing suites unchanged | Rejected as universal boundary | `planner_common` is intentionally planner/ROS contract focused and would couple non-ROS consumers to NAO structure. |
| H-02 | Neural Workbench `ab_harness` kernel | Build a pure-Python package beside `skill_common`; compile task interaction modules from AB graph | Same NAO task produces minimal closed AB projection and behavior parity | Accepted for P0 design | Implementation waits on schema review and parity tests. |
| H-03 | Pi or OpenHands as the kernel | Embed an existing harness and add AB tools/resources around it | Run identical task through Pi/OpenHands adapters and compare trace/eval coverage | Active adapter hypothesis | Reopen as kernel only if it can enforce AB closure and evidence without invasive forks. |
| H-04 | MCP-first universal harness | Represent every ability as MCP tools/resources/prompts | Test whether effect, ownership, freshness, and evidence closure are expressible and enforceable | Rejected as semantic core | MCP standardizes exchange but deliberately does not define host context or agent policy. |
| H-05 | Node-specific harnesses only | Continue evolving chatbot and planner independently | Measure duplication growth and inconsistent provider behavior over another iteration | Blocked | Already duplicates provider, prompt, parsing, projection, and trace mechanisms. |
| H-06 | Global all-tools agent | Give one agent every tool and rely on prompt policy | Compare tool precision, context load, unsafe-call rate, and recovery against task projection | Rejected | Violates least capability, increases prompt/tool entropy, and weakens ownership. |

### Decision

Proceed with H-02 as the architecture baseline, H-03 as a runtime-adapter track,
and a narrow extraction slice from H-01. Reject H-04 and H-06 as the semantic
core. Keep current nodes operational while the compatibility layer is proven.

## 9. Proposed Package Boundary

The first package should live in the Neural Workbench repository because the AB
registry is its substrate and the package must remain independent of ROS.

```text
src/Neural-Wokbench/src/ab_harness/
  package.xml                  # optional ROS packaging wrapper, no ROS imports in core
  setup.py
  ab_harness/
    contracts.py               # HarnessSpec, TaskSpec, InteractionModuleSpec
    capability_projection.py   # AB graph closure and task projection
    structured_output.py       # schema-aware parse/repair result types
    prompt_packs.py             # versioned pack loading, no domain prompt text
    providers/
      base.py
      ollama.py
      openai_compatible.py
      capability_probe.py
    policy/
      risk.py
      approvals.py
      budgets.py
    runtime/
      adapter.py
      result.py
    trace/
      events.py
      store.py
    evals/
      runner.py
      metrics.py
```

Adapters remain outside the core:

```text
adapters/
  nao_ros4hri/
  codex/
  pi/
  openhands/
  mcp/
  shell_sandbox/
```

`ab_harness` depends on `skill_common`. It must not own or duplicate
`ab_registry.json`.

### Reference-port rule

The first implementation may be staged directly in the NAO repository when
that makes parity testing and review easier. A core module qualifies for later
promotion into Neural Workbench only if:

1. it imports no ROS, NAO, dialogue, planner, or orchestrator package;
2. its public schemas contain no NAO-specific field names;
3. both LLM nodes can consume it through thin compatibility adapters;
4. at least one synthetic non-NAO adapter can use the same API; and
5. moving the module changes only package metadata and imports, not semantics.

This reverses the dependency direction that created the current SWE cost:
domain nodes configure and consume the harness; the harness does not import or
coordinate domain nodes.

## 10. Model Serving and Fast Provider Management

Provider management should be driven by measured capability profiles, not model
name conditionals spread across nodes.

### Startup handshake

Every served endpoint should be probed for:

- protocol shape: Ollama, OpenAI-compatible Responses/Chat, or custom;
- model inventory and exact model identifier;
- structured JSON/schema support;
- tool-calling support and tool schema limits;
- context window and effective output limit;
- reasoning/thinking controls;
- streaming shape and timeout behavior;
- image/audio support where relevant;
- warmup latency, first-token latency, decode rate, and concurrency.

The probe produces a versioned `ProviderCapabilityRecord`. Runtime code selects
features from that record instead of checking whether the model name starts with
`qwen`, `gemma`, or another family.

### Routing policy

```text
TaskSpec required model capabilities
  -> eligible ModelProfiles
  -> filter by context, schema, tool, privacy, and locality
  -> rank by measured latency, reliability, cost, and task score
  -> dispatch
  -> fallback only to a capability-equivalent profile
```

Use separate workload pools:

| Workload | Priority | Typical profile |
| --- | --- | --- |
| Spoken dialogue | First-token latency and concise output | Small warm local model |
| Intent/route decision | Schema reliability and low token count | Fast structured model |
| Planner generation | Tool/registry adherence and longer context | Strong structured model |
| Review/research | Reasoning depth and source handling | Larger remote or local model |
| Trace summarization | Throughput and low cost | Batch local model |

### Serving optimizations

- Keep stable harness instructions and schemas at the beginning of prompts so
  vLLM-style automatic prefix caching can reuse prefill computation.
- Place volatile task state and fresh evidence late in the context.
- Use bounded projections rather than transmitting the complete AB registry.
- Warm the active model profiles with representative schema calls.
- Track first-token latency, total latency, input/output tokens, cache-hit
  behavior, retries, fallbacks, and schema failures per profile.
- Route high concurrency using least-busy or latency-aware policies only after
  measuring queue behavior.
- Treat retries, context-window fallbacks, and provider fallbacks as separate
  events with separate budgets.
- Tune local serving memory for the actual concurrency/context mix; SGLang and
  similar servers expose KV-cache and static-memory controls that should be
  benchmarked rather than guessed.

Prefix caching reduces shared-prefix prefill cost, not token generation time.
It therefore benefits stable harness/schema prefixes and repeated task classes,
but it does not excuse verbose output or unbounded context.

## 11. Control Loop

```text
compile(task, subsystem):
    harness_spec = load_harness_spec(subsystem)
    task_spec = normalize_task(task, harness_spec)
    state = context_adapter.snapshot(task_spec)

    interaction = project_ab_subgraph(
        registry=canonical_ab_registry,
        requested_effects=task_spec.requested_effects,
        state=state,
        policy=harness_spec.execution_policy,
    )

    model_profile = model_router.select(task_spec, interaction)
    context = context_router.build(task_spec, interaction, state)
    candidate = model.generate(context, interaction.schemas)

    verified = verifier.check(candidate, interaction, task_spec)
    if verified.requires_approval:
        approval = policy_engine.request(verified.risk)
    result = runtime_adapter.execute(verified)
    evidence = verifier.check_result(result, task_spec.acceptance)

    trace_bus.append(task_spec, interaction, candidate, result, evidence)
    eval_runner.score(task_spec, trace_bus.current_trace())
    return evidence
```

The model never receives direct execution authority. The runtime adapter accepts
only verified operations represented in the compiled interaction module.

## 12. Evaluation and Ablation Protocol

### Core metrics

| Metric | Meaning |
| --- | --- |
| Harness uplift | Same model and task set, AB-aware harness score minus generic harness score |
| Task projection precision | Exposed AB objects actually useful for the task divided by all exposed objects |
| Task projection recall | Required successful-path and recovery objects present in the projection |
| Tool precision | Correct action with valid arguments |
| Evidence completeness | Claimed effects backed by required observations/results |
| Scope discipline | Writes and calls remain inside declared task scope |
| Context efficiency | Useful task/context tokens divided by total input tokens |
| Recovery quality | Failures move to a valid retry, replan, clarification, or terminal state |
| Trace completeness | Run can be reconstructed from versioned events and artifacts |
| Portability | Same task contract runs through another model/runtime adapter |
| Regression rate | Previously passing golden tasks remain passing |

### Required ablations

1. Same model, generic all-tools harness versus AB task projection.
2. Same model and tools, full registry versus bounded capability projection.
3. Same task, local served model versus remote model under the same harness.
4. Same task, prompt-only guard versus deterministic schema/policy verifier.
5. Same task, no trace retrieval versus trace-derived priors.
6. Same task, symbolic energy only versus measured entropy/failure profile.
7. Existing standalone node internals versus cooperative harness adapters,
   first one mechanism at a time and then as a combined path.

No harness improvement is accepted from a single anecdotal run. Prompt or
tool-description changes use a train/holdout ledger. Provider changes must run
the same task set and preserve trace schema.

## 13. Implementation Phases

| Phase | Goal | Deliverable | Acceptance gate |
| --- | --- | --- | --- |
| P0 | Freeze grammar and ownership | This foundation, schemas, hypothesis registry | Review accepts package boundary and non-goals |
| P1 | Extract provider-neutral substrate | `ab_harness` contracts, prompt loader, provider interface, structured-output result types | Existing planner/chatbot suites pass through adapters |
| P2 | Compile per-task interaction modules | AB closure, policy filtering, evidence closure | Golden NAO tasks expose minimal sufficient graphs |
| P3 | NAO cooperative migration | Planner/chatbot provider adapters and trace bridge, optionally staged in this repo | No ROS ownership movement; standalone-versus-harness ablations and fake/live tests preserve behavior |
| P4 | Served-model control plane | Capability probe, profiles, router, warmup, metrics | Local/remote failover is schema- and capability-safe |
| P5 | External workers | Codex, Pi, OpenHands, and MCP adapters | Same TaskSpec produces comparable traces |
| P6 | Cross-domain proof | iTrader and Gamma subsystem profiles | Same kernel, different AB registry/profile, measured uplift |
| P7 | Learning loop | Trace-derived priors and human-reviewed AB2 proposals | Holdout improves; no automatic runtime promotion |

Implementation follows the simpler release ladder defined by the adaptive
extension:

```text
H0 AB-grounded role and output gate + complete trace
H1 candidate and recovery Workbench
H2 trace-adaptive capability profiles and reviewed heuristics
H3 counterfactual crystallization and promotion quarantine
H4 non-NAO AB4 system proof and learned deltas
```

H0 is the first engineering target. It wraps the current good chatbot/planner
behavior and constrains the AB3 model-agent inside the NAO AB4 system without
adding another reasoning layer.

## 14. Immediate Work Queue

1. Review and name the five core schemas.
2. Add `ABRegistry.task_projection(...)` or a separate compiler prototype with
   no runtime behavior change.
3. Build golden projection cases for dialogue, knowledge query, scan/find,
   navigation failure, and grouped delivery.
4. Define the shared provider capability record from the union of current
   chatbot and planner transport behavior.
5. Extract a generic prompt-pack loader behind compatibility wrappers.
6. Define one append-only trace event schema and adapters from
   `chatbot_turn_trace`, planner decisions, orchestrator feedback, and Workbench
   traces.
7. Run current chatbot/planner tests as the behavior baseline.
8. Add same-model generic-versus-AB-projected harness ablations.
9. Run standalone-versus-cooperative ablations for each migrated chatbot and
   planner mechanism; reject slices that change domain behavior.
10. Prototype Pi RPC and OpenHands sandbox adapters only after the kernel schemas
   stabilize.
11. Decide whether `ab_harness` remains a package in Neural Workbench or becomes
    a standalone repository after the first non-NAO adapter succeeds.

## 15. Adversarial Audit

- [x] The design preserves dialogue, planner, orchestrator, KB, grounding, and
  skill ownership.
- [x] The universal kernel is ROS-independent and model-independent.
- [x] MCP is treated as transport/discovery, not effect truth or authorization.
- [x] Per-task capability projection replaces a global tool bag.
- [x] Action claims require explicit evidence closure.
- [x] Provider fallback requires capability and schema compatibility.
- [x] Current node-specific tests are retained as parity gates.
- [x] Learned composites remain proposal-only until reviewed.
- [~] Core H0 frame, band, role, projection, gate, and trace schemas are
  implemented and tested; the full task/provider/environment/lifecycle grammar
  remains open.
- [ ] No same-model harness ablation has yet measured uplift.
- [ ] No external Pi/OpenHands adapter has yet been prototyped.
- [ ] Live ROS and robot behavior remain outside this documentation-only pass.

**Decision:** accept the architecture and implemented H0 proof. Complete the H0
lifecycle grammar before H1, then gate H2 cooperative integration by
behavior-parity tests and same-model ablations. The canonical masterplan owns
current status.

## 16. Primary Sources

- OpenAI Codex: [AGENTS.md discovery](https://developers.openai.com/codex/guides/agents-md),
  [skills and progressive disclosure](https://developers.openai.com/codex/skills),
  [security](https://developers.openai.com/codex/security), and
  [configuration reference](https://developers.openai.com/codex/config-reference).
- Cursor: [rules](https://docs.cursor.com/context/rules-for-ai),
  [CLI and command approval](https://docs.cursor.com/en/cli/using),
  [subagents and skills](https://cursor.com/changelog/2-4), and
  [sandbox implementation](https://cursor.com/blog/agent-sandboxing).
- Pi: [coding-agent README](https://github.com/earendil-works/pi/blob/main/packages/coding-agent/README.md)
  and [extension API](https://github.com/earendil-works/pi/blob/main/packages/coding-agent/docs/extensions.md).
- OpenHands: [runtime architecture](https://docs.openhands.dev/openhands/usage/architecture/runtime)
  and [evaluation harness](https://docs.openhands.dev/openhands/usage/developers/evaluation-harness).
- SWE-agent: [Agent-Computer Interface paper](https://arxiv.org/abs/2405.15793)
  and [trajectory format](https://github.com/SWE-agent/SWE-agent/blob/main/docs/usage/trajectories.md).
- Hermes Agent: [repository](https://github.com/NousResearch/hermes-agent) and
  [toolset registry](https://github.com/NousResearch/hermes-agent/blob/main/website/docs/reference/toolsets-reference.md).
- AutoGen: [agent runtime architecture](https://microsoft.github.io/autogen/stable/user-guide/core-user-guide/core-concepts/architecture.html).
- Pydantic AI: [durable execution](https://pydantic.dev/docs/ai/integrations/durable_execution/overview/).
- MCP: [architecture and primitives](https://modelcontextprotocol.io/docs/learn/architecture).
- vLLM: [automatic prefix caching](https://docs.vllm.ai/en/latest/features/automatic_prefix_caching/).
- LiteLLM: [load balancing](https://docs.litellm.ai/docs/proxy/load_balancing)
  and [provider fallbacks](https://docs.litellm.ai/docs/proxy/reliability).
- SGLang: [serving memory and concurrency tuning](https://docs.sglang.ai/advanced_features/hyperparameter_tuning.html).
