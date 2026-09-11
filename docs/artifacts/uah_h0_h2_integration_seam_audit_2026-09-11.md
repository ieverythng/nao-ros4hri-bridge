# UAH H0-H2 Integration Seam Audit

**Date:** 2026-09-11  
**UAH repository:** `/home/juanbeck/universal-agentic-harness`  
**Branch:** `feat/pre-commit-queue`  
**Reviewed commits:** `5379b46` and `3401367`  
**Fixed point:** `cf90a7e`  
**NAO compatibility baseline:** annotated tag `v1.0.0`, peeled commit
`ebffe93a74be4e013ce0f60fdfc41268dba73fc3`  
**Chatbot baseline:** `a2ecca79600efafeeef75aab4440d18b860b974f`  
**NeuralWorkbench intended revision:** `e76ba7eafbd90f9ed239a65f741d2598ecd033cb`

## Decision

Retain both commits as working-branch progress, but do not describe the result
as H2-qualified or as an authority-ready NAO adapter.

The accurate release statement is:

> UAH has an incomplete H0 contract spine, several narrow H1 seams, and an
> H2-shaped ROS-free NAO compatibility package. `ab_harness_nao` is importable
> and cleanly separated from the portable kernel, but it is not yet registered
> into an environment lifecycle, does not implement NAO planner parity, and can
> currently produce a false-positive qualification result.

This matches the owner's description that development is in H1 while closing
remaining H0 contracts. H2 is not closeable until the false-completion defect,
complete lifecycle replay, source-pinned NAO normalization, explicit authority
modes, and planner parity are green.

The canonical UAH contract permits recorded and fake/simulated closure for H2.
This thread adopts a stricter final integration condition: after those H2 gates
pass, the cycle closes only after a clean full-stack launch completes one
successful NAO planner/chatbot run without duplicate speech or fabricated
effects.

## Release position

| Phase | Current state | Audit judgment |
| --- | --- | --- |
| H0, contract spine | Frame-relative AB views, projection, gates, bindings, task acceptance, in-memory environment profile/run registration, and minimal trace round trips exist | Real but incomplete. Serialization, full identity grammar, task/effect compilation, complete object schemas, and lifecycle events remain open. |
| H1, executable kernel | Fake owner, recorded canary, task-acceptance evaluator, environment registration, configuration identity, and bounded Workbench retrieval exist | Partial vertical slices only. There is no complete runtime state machine, task ingress to terminal ledger, replay-equivalent digest, provider/model lifecycle, cancellation, timeout, or retry-exhaustion suite. |
| H2, cooperative NAO | `ab_harness_nao` contains role projections, candidate AB0 bindings, recorded qualification, and a canary | Not qualified. No explicit `legacy | uah | shadow` runner, frozen NAO fixtures, full lineage, dialogue relay, `report_result` delegation, fake/sim parity report, or source-schema conformance exists. |
| H3, NeuralWorkbench | Transport-neutral protocol and candidate-only retrieval seams exist | Correctly quarantined. H3 must remain optional and shadow-only until H2 replay and parity pass. |
| H4, crystallization | Counterfactual, holdout, review, provenance, and rollback contract is documented | Design only, as intended. No promotion claim is supported. |

## Review of the latest two commits

### `5379b46`, documentation and design-report update

The documentation changes describe the intended package split and environment
registration boundary accurately when read together with the following commit.
The generated Markdown and HTML topology is coherent at branch head.

The commit is not independently coherent:

- it changes the advertised command to `python -m ab_harness_nao` before that
  package exists;
- it reports 73 passing tests while the exact commit has 50;
- it states that environment-run registration and the separate NAO package are
  implemented, although their source arrives in `3401367`.

An exact archive of `5379b46` produced 50 passing tests. The legacy
`python -m ab_harness smoke` command passed, while
`python -m ab_harness_nao` failed because the package was absent. The issue is
commit atomicity, not the combined branch-head behavior. The documentation and
implementation should have been one atomic commit, or the first commit should
have used prospective wording.

### `3401367`, package split and environment registration

The strongest change is the dependency correction:

- NAO contracts, qualification, canary, and CLI moved out of `ab_harness` into
  `ab_harness_nao`;
- the portable kernel no longer imports NAO compatibility code;
- a regression test rejects imports from `ab_harness_nao` inside the kernel;
- `EnvironmentProfileRegistry` and `EnvironmentRunRegistry` establish a narrow
  in-memory profile/activation seam;
- duplicate run IDs, duplicate attestation IDs, missing scalar lineage,
  missing readiness references, unknown profiles, and profile mismatches fail
  before insertion.

The exact commit passed 73 tests. A wheel built from an archive installed in a
fresh virtual environment, and `python -m ab_harness_nao` passed its accepted,
rejected, and Workbench retrieval canaries. Regenerating canonical documents
produced no diff. `git diff --check cf90a7e...3401367` also passed.

The commit combines two independently reviewable changes, the NAO package move
and the environment-run API. This does not invalidate the code, but it weakens
regression attribution and makes the preceding documentation commit dependent
on it.

## What “registered adapter” currently means

Three meanings must remain separate:

| Meaning | State |
| --- | --- |
| Importable Python package | Yes. `find_packages(where="src")` includes `ab_harness_nao`, and the installed module runs. |
| Independent distribution or declared adapter entry point | No. The wheel and ROS package are still named `ab_harness`; no adapter entry-point or adapter registry exists. `importlib.metadata` maps `ab_harness_nao` to the `ab_harness` distribution. |
| Registered environment/runtime adapter | No. The canary constructs its registry, binding, fake owner, and qualification harness directly. It does not create an `EnvironmentProfile`, `EnvironmentRunAttestation`, or `EnvironmentRun`, and it has no NAO task-ingress or agent-run lifecycle. |

`ab_harness_nao` is therefore the initial explicit reference adapter package,
not yet a registered H1/H2 adapter. Keeping both packages in one distribution
is acceptable through H2 if the dependency rule remains tested. A separate
distribution is not required until a second domain or deployment lifecycle
demonstrates a need. A small adapter descriptor and environment-registration
seam are required before using “registered adapter” as a runtime claim.

## Blocking findings

### 1. False-positive NAO qualification

`ab_harness_nao.contracts.planner_output` records references only for steps
whose type is `skill`. The NAO v1 contract also admits `look_at`, and hostile or
regression fixtures must expose `say` so duplicate-speech policy can reject it.
Both are currently invisible to the UAH gate. The qualification runner also
allows a case with no required observables and no effect obligations, then
defines success as the absence of missing observables.

The exact commit was probed with a plan containing only unprojected `look_at`
and `say` steps. The result was:

```text
referenced_objects = ()
gate = accepted
qualification = passed
evidence = ()
acceptance = None
```

This violates the hard H2 scope, evidence, duplicate-speech, and
false-completion gates. It must be repaired before any UAH authority mode is
enabled.

Required correction:

1. Normalize every supported NAO plan-step type into a typed referenced
   operation, including `look_at`.
2. Reject unknown, ignored, or speech-producing planner steps explicitly.
3. Require a non-empty immutable effect-obligation set for H2 qualification.
4. Treat `required_observables` only as a temporary compatibility input that is
   compiled into obligations, never as an alternative success calculus.
5. Add public-seam cases proving that ignored or empty plans cannot pass and
   that no rejected step reaches the owner.

### 2. Authority-mode vocabulary differs from the H2 contract

Candidate bindings declare `('shadow', 'cooperative')`. The canonical runtime
contract requires exactly `legacy | uah | shadow`, with no silent fallback.
Binding resolution is string-exact, so a later approved binding would reject
`uah` while accepting the undeclared `cooperative` mode.

Use one enum or validated value object across configuration, bindings, parity
fixtures, traces, and the NAO launch adapter. “Cooperative” may describe the H2
migration strategy, but it should not be a fourth runtime authority mode.

### 3. The NAO frame identifier is inconsistent

The adapter emits `frame_id='nao_ros4hri'`, while the foundation, adaptive
Workbench contract, and examples use `nao_runtime`. Frame identity is part of
every AB coordinate and later Workbench trace key. This mismatch would split
one semantic frame into two histories and undermine H3 retrieval and H4
crystallization evidence.

Freeze one identifier in the DomainContractPack before generating more traces.
The documented `nao_runtime` name is preferable because it distinguishes the
AB frame from the repository or domain name.

### 4. Candidate binding tests do not prove source parity

`test_latest_nao_contract_seams_preserve_planner_gate_boundary` supplies
`a2ecca7c` and comparative branch revision `9da89c0c`. The frozen baselines are
chatbot `a2ecca796...` and NAO `v1.0.0` at `ebffe93...`. The binding factory
stores caller strings without resolving the revision or loading source-owned
fixtures, so the test proves literal catalog construction only.

The declared locators inspected in this audit exist at the intended source
revisions. That is useful evidence, but it is not schema parity. H2 still needs
package-owned golden fixtures generated from the pinned contracts and checked
for `goal_id`, `request_id`, `plan_id`, `plan_version`, stable `step_id`, result
payloads, and error paths.

### 5. Protected NAO seams remain absent

The candidate map covers six AB0 objects but omits or does not implement:

- the orchestrator relay from `/planner/dialogue_act` to
  `/nao_orchestrator/planner_dialogue_act`;
- the `report_result` AB1 operation, evidence verification, explicit chatbot
  delegation, return artifact, native communication dispatch, and speech
  evidence;
- supervisor retry, cancellation, supersession, and replan transitions;
- planner request and plan lineage across the full causal trace;
- explicit separation of raw output, proposal, semantic admission, domain
  lease, owner result, evidence, and terminal task acceptance.

These omissions are acceptable for an H0 reference canary. They prevent H2
parity and any claim that the current adapter covers the planner/chatbot loop.

### 6. Environment registration proves matching, not owner authenticity

The current registry checks that caller-supplied owner, runtime, and
DomainContractPack strings match the stored profile and that at least one
non-empty readiness reference exists. It does not authenticate the attestation
issuer, resolve evidence artifacts, apply an interface-specific readiness
policy, or verify signatures. The documentation acknowledges the missing
signature and persistence work, but phrases such as “profile-verified
owner-attested activation” can be read too strongly.

For the current H0 seam, describe this as **profile-matched in-memory
registration of an externally owner-issued attestation**. Before H2, the NAO
adapter must validate the attestation against the frozen profile's required
interfaces and evidence rules. Cryptographic signatures may remain later scope
if the trust boundary is process-local and explicitly documented.

`EnvironmentProfileRegistry` also accepts empty, blank, or duplicate
`required_interface_ids` and `agent_handle_ids`. The profile is thus not yet a
complete NAO environment contract or stable chatbot/planner roster.

### 7. Documentation and CI retain small integrity gaps

- The development log still records the removed
  `.venv\Scripts\python.exe -m ab_harness smoke` command as passing.
- The historical H2 review also retains the removed command. Historical
  evidence may remain, but it must be labelled as applying to the earlier
  commit rather than the current CLI.
- The committed CI renderer runs all document generation but checks diffs only
  under `docs/agentic_harness`, not the canonical `docs/plans`,
  `docs/architecture`, and `docs/artifacts` paths. The current uncommitted queue
  appears to address this with a non-mutating `--check` mode, but those changes
  were outside the reviewed commits.
- The package-boundary regression test prohibits only `ab_harness_nao` imports.
  It does not enforce the complete ROS, NAO, provider-SDK, and runtime-product
  denylist stated by the portable-kernel contract.

## Seam parity matrix

| Seam | NAO v1 source behavior | UAH branch-head behavior | State |
| --- | --- | --- | --- |
| Kernel dependency direction | NAO semantics remain outside a portable core | NAO files are isolated in `ab_harness_nao`; core import direction is tested | Green |
| Planner gate ingress | Chatbot publishes `/nao_orchestrator/planner_request`; orchestrator gates before `/planner/request` | Candidate locators preserve both AB0 objects | Amber, declaration only |
| Planner request normalization | `planner_common.PlannerRequest` owns the contract and native lineage | Contract locator is declared; no source fixture or task ingress exists | Red for H2 |
| Planner output | Planner publishes typed plans on `/intents`, including supported step types and lineage | Adapter extracts only `skill` names and can ignore other steps | Red, false-positive risk |
| Deterministic dispatch | `nao_orchestrator` validates and dispatches AB1 skills | One in-process fake `find_object` owner is exercised | Amber, synthetic slice |
| Effect evidence | Invoked AB1 owner supplies fresh execution evidence | Owner and declared-observable checks exist for the fake method | Amber, no freshness/lifecycle proof |
| Task acceptance | Completion must follow required and best-effort effects | Pure evaluator exists, but the NAO canary can bypass it | Red for qualification |
| Execution feedback | Orchestrator emits structured feedback to planner supervision | AB0 contract pointer only | Red for H2 |
| Cancellation, supersede, replan | Planner supervisor and orchestrator preserve native state and lineage | No adapter behavior or lifecycle events | Red |
| Planner dialogue relay | Planner act passes through the orchestrator relay to the dialogue owner | Relay topic and behavior are absent from the adapter map | Red |
| `report_result` | AB1 planner operation verifies evidence, delegates wording to chatbot, and returns to native speech ownership | No operation/delegation adapter or parity fixture | Red |
| Single utterance authority | Dialogue path prevents planner/executor duplicate speech | No duplicate-speech qualification case; `say` may be ignored | Red |
| Environment activation | Native owner/lifecycle establishes readiness | In-memory profile-matched registration exists but is not used by the NAO canary | Amber, isolated H1 seam |
| Authority mode | NAO repository must expose `legacy`, `uah`, or `shadow` explicitly | Candidate binding modes are `shadow` and `cooperative`; smoke uses `smoke` | Red |
| Full ledger and replay | H2 requires a multi-actor, model-free reconstructable lifecycle | Minimal `HarnessTrace` and Workbench experience only | Red |
| Startup/model preflight | Existing chatbot/planner lifecycle probes gate activation | No agent registry, lease, startup preflight, or model invocation | Red |
| H3/H4 quarantine | Workbench proposes candidates; UAH gates; owner executes; promotion is reviewed | Protocol and documentation preserve this boundary | Green as architecture, not implementation |

## Two-axis code review

### Standards

1. **High, authority and evidence contract:** `EnvironmentRunRegistry.register`
   trusts caller-supplied owner and evidence-reference strings. It performs
   profile matching, not issuer authentication or evidence-policy validation.
   This must remain a bounded H0 claim because runtime discovery and strings are
   not authorization or evidence.
2. **Medium, incomplete environment profile:** the profile is not yet
   content-addressed and has no readiness policy. Interface and handle members
   are unvalidated. Repeated loose authority/revision strings also create a
   primitive-obsession and data-clump risk that should be resolved through the
   serialized contract grammar rather than another helper layer.
3. **Medium, commit atomicity:** `5379b46` claims implementation that is supplied
   only by `3401367`. The combined head is usable, but the first commit cannot
   substantiate its own documentation.

Standards axis: three findings. The worst issue is treating profile-matched
strings as stronger owner/evidence validation than the code performs.

### Specification

1. **P1:** an empty or ignored-step plan can pass NAO qualification without
   evidence.
2. **P1:** binding runtime modes do not match the required
   `legacy | uah | shadow` vocabulary.
3. **P1:** `ab_harness_nao` is separated but is not registered into the new
   environment lifecycle and is not H2-qualified.
4. **P1:** the adapter omits dialogue relay, `report_result` delegation, full
   lineage, and source-pinned parity fixtures.
5. **P2:** `3401367` mixes package movement with a separate environment-run
   feature, and the development dashboard retains a removed command.

Specification axis: five findings. The worst issue is false-positive
qualification with no dispatched operation or effect evidence.

## Recommended H0-H2 closure sequence

The next tests should remain at public seams. This audit proposes the order but
does not authorize test implementation in place of the ongoing owner-confirmed
TDD work.

### Gate 0, repair the existing NAO canary

1. Make planner-output normalization exhaustive over the frozen NAO plan-step
   grammar.
2. Reject empty, ignored, unsupported, direct-speech, and out-of-projection
   plans.
3. Require task effect obligations and route all closure through
   `TaskAcceptanceEvaluator`.
4. Freeze `nao_runtime` and `legacy | uah | shadow` as canonical identifiers.

Exit evidence: the current false-positive probe is red before the fix and green
after it, with zero owner calls and a failed qualification result.

### Gate 1, finish the H0 contract spine needed by H1

1. Serialize and version environment profile/run, task, operation, obligation,
   proposal, admission, lease, and event envelopes.
2. Validate complete interface and stable handle rosters.
3. Compile obligations from `TaskSpec`, DomainContractPack evidence rules, and
   task-ingress lineage.
4. Enforce the full portable-kernel forbidden-import policy.

Exit evidence: schema round trips, rejected incomplete contracts, one complete
synthetic task contract, and no domain imports in the core.

### Gate 2, connect the H1 lifecycle tracer

1. Register an attested synthetic environment run.
2. Classify immutable ingress.
3. Attach the relevant agent run.
4. Normalize a recorded proposal, admit it, request a domain lease, execute the
   fake owner, evaluate obligations, and append every stage.
5. Derive a model-free `VerifiedTraceDigest` and replay to the same decision.
6. Add stale evidence, unavailable owner, timeout, cancellation, retry
   exhaustion, required failure, best-effort deficit, and false-completion
   cases.

Exit evidence: the frozen synthetic suite reconstructs every terminal decision
without a model.

### Gate 3, make `ab_harness_nao` a registered reference adapter

1. Add a declarative adapter descriptor tied to the frozen NAO
   DomainContractPack and environment profile.
2. Register named chatbot and planner handles without importing ROS into the
   kernel.
3. Generate golden fixtures from NAO `v1.0.0` and chatbot `a2ecca796`.
4. Normalize planner request, plan, feedback, dialogue act, relay, and native
   lineage into the H1 event grammar.
5. Reconcile `report_result` through an owner-reviewed DomainContractPack
   revision.

Exit evidence: clean-install recorded qualification uses the environment-run
and task-ingress path rather than constructing an isolated canary graph.

### Gate 4, qualify H2 planner parity

1. Run the same frozen cases under explicit `legacy`, `shadow`, and `uah`
   modes.
2. Cover accepted, rejected, clarification, recovery, cancellation,
   supersession, stale evidence, and false completion.
3. Preserve native lineage and separate every proposal, admission, lease,
   result, evidence, and terminal artifact.
4. Prove `report_result` cross-frame chatbot delegation and one final speech
   authority.
5. Classify and review every disagreement before UAH becomes authoritative.

Exit evidence: model-free replay equivalence, a parity/disagreement report, and
fake/simulated success and failure paths with no duplicate speech.

### Gate 5, close this integration thread with NAO

After canonical H2 qualification:

1. Build a clean container from the intended NAO base.
2. Stop and relaunch the full selected profile once.
3. Verify core-node uniqueness and lifecycle readiness.
4. Run one successful end-to-end planner/chatbot task on NAO, including owner
   evidence and the terminal report.
5. Run one failure, cancellation, or clarification case.
6. Archive the exact image, model, prompt, registry, adapter, environment,
   evaluator, and trace identities.

This final gate is stricter than the UAH H2 minimum and satisfies the stated
goal for this integration thread.

## H3-H4 protection rules

The H0-H2 work should preserve these later-stage invariants:

- NeuralWorkbench remains an optional companion and is not an abstraction
  frame.
- H3 receives complete immutable traces and returns candidate context or shadow
  proposals only.
- The UAH gate and environment owner remain the admission, execution, and
  evidence authorities.
- Frame IDs, configuration IDs, task IDs, actor IDs, and operation edges must be
  stable before H3 retrieval data accumulates.
- H4 crystallization cannot promote from frequency. Replay, opposing traces,
  disjoint holdout, owner review, provenance, decompression, and rollback remain
  mandatory.
- The missing NeuralWorkbench gitlink at `e76ba7e` must be restored and verified
  before the companion can be treated as mounted rather than intended.

The package split in `3401367` helps these goals because it prevents NAO
normalization from becoming part of the semantic kernel. The false-completion,
frame-name, and lifecycle gaps must be corrected before current canary traces
are allowed to become H3 capability evidence.

## Evidence and limitations

Reproduced checks:

- exact `3401367` archive: 73 tests passed;
- exact `3401367` archive: `python -m ab_harness_nao` passed all canaries;
- exact `3401367` wheel: built, installed in a fresh virtual environment, and
  exposed `ab_harness_nao` from the `ab_harness` distribution;
- exact `3401367`: canonical documentation regenerated without a diff;
- `git diff --check cf90a7e...3401367`: passed;
- exact `5379b46` archive: 50 tests passed, legacy CLI passed, new adapter CLI
  absent;
- NAO tag `v1.0.0`: declared planner, feedback, dialogue, and scene contract
  locators were confirmed in source;
- chatbot revision `a2ecca796`: planner request adapter locator was confirmed.

No live ROS stack, simulator, container, model endpoint, or robot was launched
or disturbed. The UAH working tree contained uncommitted pre-commit tooling and
environment-ingress work; those changes were deliberately excluded from the
two-commit judgment.

The parent GitNexus index was stale. The SocialMinds bootstrap refreshed most
sources but stopped because the local `motions_skills` branch has diverged and
cannot fast-forward. GitNexus MCP exposed no indexed repositories in this
session. All parity findings were therefore confirmed through direct docs,
commit archives, tests, and pinned source reads rather than the graph layer.
