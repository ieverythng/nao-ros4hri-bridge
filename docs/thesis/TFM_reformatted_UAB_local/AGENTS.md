# TFM Local Writing Guide

This folder contains the local LaTeX project for the master thesis. The guide
applies whenever an agent edits the thesis source, figures, appendix, or
thesis-facing plan artifacts.

## Scope And Working Rule

- Preserve the user's working-tree edits. Inspect `git diff` before touching a
  file and make the smallest requested change.
- Treat the thesis as a description of the complete evaluated NAO interaction
  and execution stack, not as a planner study or a catalogue of every internal
  research tool in the repository.
- Keep implementation claims tied to current source, tests, launch profiles,
  contracts, and dated runtime evidence.
- Do not change ROS package code while editing this folder unless the user asks
  for a separate implementation task.
- Compile locally after meaningful edits and inspect affected PDF pages visually.

## TFM Vocabulary

- Use **NAO interaction and execution stack**, **integrated NAO stack**, or a
  more specific component boundary. Do not present ROS4HRI as the system or as
  a framework. The term is reserved for direct discussion of the cited
  interface proposal, aligned packages, or literal repository/package names.
- Use abstract roles such as **dialogue manager**, **chatbot**, **planner**, and
  **orchestrator** through Chapters 1 and 2. Introduce concrete package/node
  names in Chapter 3 before using them in later implementation chapters.
- Use **skill** or **skill registry** for the thesis-facing capability model.
- Do not use internal registry decomposition labels or internal workbench names
  in thesis prose, captions, tables, or thesis-facing diagrams.
- Use `grounded_context` without an internal schema suffix. Describe it as the
  compact KB-to-chatbot and planner projection.
- Introduce **knowledge base (KB)** at its first conceptual use, then use `KB`.
  Keep **KnowledgeCore** as the name of the concrete transport/backend seam.
- Use “multi-step deterministic fake-skill validation”, never “deep fake” in
  thesis-facing prose.
- Qualify claims with “implemented”, “observed”, “supported by the source”, or
  “future work”. Do not turn an architectural constraint into an empirical
  guarantee without a corresponding evidence artifact.
- Avoid em dashes, marketing language, bare “first” claims, meta-commentary,
  and unexplained generic terms. Prefer direct technical sentences that match
  the surrounding thesis prose. Do not use abstract labels such as
  “substrate”, “evidential role”, or “operationalise” when communication,
  ownership, validation, or execution describes the point more clearly.
- Use `\parencite{...}` for parenthetical references. Use
  `\textcite{...}` only when the cited author is the grammatical subject.
- Introduce the purpose of every section before an enumeration, table, or
  figure. Name the artifact in the preceding prose and state what the reader
  should obtain from it.

## Ownership And Contracts

Preserve these boundaries in every chapter and figure:

- `dialogue_manager` owns the tracked-person dialogue lifecycle, per-person and
  group conversation state, history, turn coordination, and final speech
  realization through its reusable HRI interfaces.
- `chatbot_llm` owns user-facing language generation, route selection, grounded
  context construction, and candidate planner ingress. It does not create
  executable plans.
- `planner_llm` owns plan generation and a goal-keyed helper that maintains
  coherent goal state when execution feedback requires a planning response. The
  helper is part of the node; do not sell it as an independent execution
  supervisor or robot monitor.
- `nao_orchestrator` owns planner admission, deterministic plan validation,
  dispatch, lineage, execution feedback, and planner-dialogue relay. It does
  not become an LLM policy or replacement planner.
- `kb_skills` owns KnowledgeCore query and revision transport.
- `nao_scene_grounding` owns detector normalization, detector-derived scene
  facts, and `/scene/summary`.
- Runtime skills own fresh evidence for effects that depend on the changing
  robot or scene state.

Use these cross-links when describing enforcement:

- dialogue versus execution: FR2, FR3, Contract 1, and Contract 2;
- skill admissibility and deterministic plan validation: FR5, Contracts 4 and
  5, and the orchestrator section;
- compact grounding: Contract 3 and the grounded-context pipeline figure;
- state-changing skill evidence and verified KB effects: Contract 6 and the
  grounding/knowledge section;
- feedback-driven planning: FR6, FR7, and Contract 7;
- speech ownership and duplicate prevention: the ownership map, Contract 8,
  and the validation speech metric;
- raw scene, KB, action, and trace evidence: Contract 9 and the evidence bundle.

## Planner And Fallback Description

Chapter 5 must describe the normal model path and the bounded deterministic
guards around it. Keep their responsibilities distinct:

1. `chatbot_llm` builds compact grounded context on every user turn, calls the
   configured response/intent stages, normalizes route decisions, repairs
   route contradictions, and publishes candidate planner requests only for
   execution-eligible turns.
2. `planner_llm` builds the planner prompt from the admitted request, grounded
   context, skill registry, allowed step types, output schema, and feedback.
   It extracts and validates model JSON, performs one bounded validation retry,
   and returns a typed failure or clarification when the output remains invalid.
3. Narrow planner fallbacks may handle an obvious primitive motion, a grounded
   location-group delivery request, or a grounded look-at/report request. They
   must not silently convert composite or target-ambiguous language into an
   incomplete plan.
4. `nao_orchestrator` validates and dispatches the resulting plan. Its report
   fallback derives wording from execution evidence and must not fabricate
   success. Adapter fallbacks are profile-dependent and remain execution
   details, not planner policy.

Every fallback described in the thesis must state its trigger, scope, output
mode, evidence/trace marker, and failure behavior. A fallback is not a hidden
second planner.

## Skill Contract Shape

When introducing the shared skill contract, use the same fields for all skills:

- name and category;
- aliases and planner-facing parameters;
- required parameters and grounding/preconditions;
- expected effects;
- observable success evidence;
- known failure modes;
- planner guidance and permitted recovery policy;
- result schema;
- safety flags;
- runtime availability and implementation status;
- executor or adapter mapping;
- fake-policy support where applicable.

The canonical registry and its generated projections remain implementation
evidence. The TFM should present a compact skill table and place the detailed
machine-readable inventory in an appendix or referenced artifact. Do not copy
internal-only decomposition metadata into the main prose.

## Diagram Rules

- Use the established runtime colours: dialogue blue, grounding orange,
  planning green, execution purple, skills neutral grey, future work dashed.
- Keep arrows outside labels and boxes. Use a wider bus or an orthogonal route
  when an action path would cross a contract label.
- Keep `interaction_sim` and preloaded KB fixtures as explicit validation inputs
  when the figure describes grounding or evaluation.
- Keep the World Model Enricher as a dashed future extension. Do not include
  internal workbench names in TFM-facing figures.
- Render from the TikZ source, crop without embedded captions or page numbers,
  and inspect at thesis width and grayscale-equivalent contrast.

## Validation Commands

From the thesis folder:

```bash
latexmk -pdf main.tex
```

From the repository root:

```bash
python3 scripts/ros4hri_change_audit.py --mode working
git diff --check
```

When the skill inventory changes, check the canonical registry projections with
the repository's registry consistency scripts. Do not update runtime registries
as collateral to a prose-only task.

## SkillOpt Ledger

Run a bounded SkillOpt-style iteration (baseline -> mutate -> holdout gate ->
accept/reject log) before finalizing major wording changes.

### Iteration 2026-07-13: thesis writing guard

- **Target artifact:** this `AGENTS.md`.
- **Objective:** reduce thesis drift in terminology, ownership, fallback claims,
  and AI-like academic prose while preserving the user's surgical editing loop.
- **Train set:** the current Chapter 5 requests covering internal registry
  vocabulary, KB first use, planner helper wording, deterministic fallbacks,
  skill contracts, and diagram readability.
- **Holdout set:** Chapters 3, 4, 6, 7, and Appendix edits involving contract
  links, validation claims, figures, and registry tables.
- **Acceptance gate:** the guide must provide explicit rules for each train-set
  seam; a static scan must find no forbidden internal vocabulary in the
  TFM-facing source; the repository audit and thesis compile must remain clean.

**Baseline:** no thesis-local `AGENTS.md`; terminology, fallback scope, and
cross-link requirements were repeated in conversation and could drift between
agents.

**Mutation batch:**

1. Add a TFM vocabulary section with explicit forbidden/internal terms.
2. Add ownership, fallback, skill-contract, and diagram guardrails tied to the
   current runtime contracts.
3. Add a compact baseline, train set, holdout set, and acceptance gate.

**Train result:** accepted by inspection. The requested Chapter 5 seams are
covered by explicit rules and source-grounded terms.

**Holdout result:** accepted by static review. The rules apply to Chapters 3,
4, 6, 7, the appendix, and figures without assigning runtime ownership to the
wrong node or requiring internal registry vocabulary in the manuscript.

**Decision:** accept. No prompt-pack or ROS source mutation was made.

**Next mutation hypothesis:** after three accepted thesis-writing iterations,
consolidate duplicated vocabulary rules and add only a focused table/figure
check if a repeated formatting defect appears.

### Iteration 2026-07-13: contracts, skills, and runtime profiles

- **Target artifacts:** Chapters 4 and 5, the runtime-skill appendix, and the
  synchronized completion-plan pair.
- **Objective:** make the Contract 4 skill seam, orchestrator ownership,
  perception/KB effects, fake validation, and launch profiles precise enough to
  support Chapters 6 and 7 without internal registry terminology.
- **Train set:** the requested Contract 4 renumbering, complete runtime-skill
  table, direct and planner orchestrator ingress, custom YOLO grounding,
  post-skill KB effects, fake-skill scope, and simulator/robot profile prose.
- **Holdout set:** dialogue/speech ownership, all contract references outside
  Chapter 5, canonical-versus-generated registry consistency, and landscape
  appendix layout.
- **Acceptance gate:** Contracts 1--9 remain sequential; all sixteen canonical
  runtime skills are represented; no forbidden thesis vocabulary returns;
  Chapters 4 and 5 compile without overfull boxes; affected pages pass visual
  inspection; unresolved runtime drift is tracked rather than hidden.

**Baseline:** the shared entry had been inserted as Contract 3A, later contracts
still used the old numbering, the orchestrator prose was planner-centred, and
the launch section repeated prompt-policy material without naming the two
integrated profiles.

**Mutation batch:**

1. Renumber the contract map and add typed skill-result and raw-evidence
   contracts around the canonical Contract 4 entry.
2. Add the complete skill surface and expand the orchestrator, detector,
   verified KB-effect, and fake-skill implementation descriptions.
3. Replace the runtime-configuration prose with source-derived simulator and
   robot/RViz profiles, lifecycle sequencing, and model-provider defaults.

**Train result:** accepted. The source compiles to a 72-page PDF, and visual
inspection found no overlap or clipping in the changed contract and Chapter 5
tables.

**Holdout result:** accepted with one external implementation gate. The
canonical inventory contains all sixteen skills and the thesis references are
consistent. Registry scripts identified stale generated planner projections for
`walk_to` and `wave_greet`; this source drift is tracked as `TFM-I22` and was not
silently repaired during the prose-only pass.

**Decision:** accept the thesis mutation batch. Keep the final scored-source
freeze conditional on `TFM-I22` passing both registry consistency checks.

**Next mutation hypothesis:** define Chapter 6 acceptance denominators and the
frozen case manifest against Contracts 1--9 before replacing Chapter 7's
preliminary results.

### Iteration 2026-07-13: evaluation protocol and evidence-set results

- **Target artifacts:** Chapters 6 and 7 and the synchronized completion-plan
  pair.
- **Objective:** convert the validation scaffold and historical status narrative
  into a reproducible evaluation protocol and evidence-based results chapter.
- **Train set:** the full and deterministic-fake validation ledgers, the
  twenty-case questionnaire, nine-case canonical fake-skill manifest, accepted
  June/July run records, and the runtime-review scorecard.
- **Holdout set:** ROS4HRI speech ownership, planner admission, KB transport,
  detector-profile separation, full-sweep versus isolated denominators, and the
  ban on development-status language in the manuscript.
- **Acceptance gate:** no unsupported result is introduced; the historical
  eight-case sweeps remain distinct from the nine-case protocol; reviewer scores
  support one-decimal values; Chapters 6 and 7 compile without overfull boxes;
  affected pages pass visual inspection.

**Baseline:** Chapter 6 lacked fixed manifests, denominators, reset rules, and a
formal score. Chapter 7 was organized as a development-status report and did
not reconcile the accepted full, fake, strict-KB, and alternate-model evidence.

**Mutation batch:**

1. Add the unit of analysis, execution profiles, acceptance gates, exact case
   manifests, reset policy, metrics, status rules, and evidence-bundle shape.
2. Define a weighted eight-dimension 1--10 score with critical-failure caps and
   explicit treatment of fallback counters as pressure telemetry.
3. Populate Chapter 7 with F28, F08, F10, and F11 evidence sets, preserving
   full-sweep denominators and adding a ROS4HRI contract-results table.

**Train result:** accepted. The protocol distinguishes the twenty-case main
questionnaire, nine-case canonical fake matrix, historical eight-case F08
sweeps, isolated diagnostic probes, and alternate-model records.

**Holdout result:** accepted. Speech ownership, planner-gate ownership, skill
feedback lineage, KB mutation ownership, and detector-score separation remain
explicit. Static review found no banned status phrases in Chapters 6 or 7.

**Decision:** accept. MacTeX produced a 75-page PDF after the score-table
addition. Chapters 6 and 7 have no overfull boxes, and visual inspection found
no clipped text, overlap, or broken continued-table headers.

**Next mutation hypothesis:** freeze the holdout prompts and stable evidence
hashes, then generate Chapter 7 plots from checked-in aggregate tables.

### Iteration 2026-07-14: semantic validation depth and durable evidence

- **Target artifacts:** Chapters 6--10, the runtime-review questionnaire, and
  the TFM evidence-bundle builder.
- **Objective:** prevent ROS trajectory completion from being reported as task
  correctness when selected entities, roles, postconditions, or final reports
  disagree with the user request.
- **Train set:** the F13 ordered-target false success, incomplete grouped
  report, stale KB postconditions, non-action execution leak, and the F14
  ordered, work-table, kitchen, and gold-apple focused probes.
- **Holdout set:** ROS4HRI speech and planner-gate ownership, detector-profile
  separation, historical versus diagnostic denominators, fake-versus-physical
  claim boundaries, and academic writing rules.
- **Acceptance gate:** target-selection counterexamples fail; F14 exact-member
  cases retain their observed verdicts; main, environment, fake, and robustness
  manifest counts are fixed; evidence files are hash-addressed; registry checks,
  Python tests, static LaTeX structure, and repository whitespace checks pass.

**Baseline:** the main questionnaire could mark a case pass from route,
execution, terminal, and speech breadcrumbs without proving that the selected
or reported target set matched the request. F13 therefore contained a 20/20
trajectory result alongside semantic false success.

**Mutation batch:**

1. Add exact selected-member, recipient, and report-policy expectations to the
   questionnaire and freeze a five-case robustness set.
2. Add validation-depth boundaries, semantic metrics, RQ maps, and the F13/F14
   interpretation to the thesis.
3. Add immutable evidence bundles with original and bundled SHA-256 hashes,
   normalized case rows, and explicit historical or diagnostic standing.

**Train result:** accepted. Twenty-five focused Python tests passed. Regrading
the saved F14 artifacts preserved the ordered, work-table, and kitchen passes
and retained the gold-apple failure because its authoritative selection was
absent.

**Holdout result:** accepted with external visual QA pending. Registry
consistency and generated-view checks passed, static LaTeX brace, environment,
label, and reference checks passed, and `git diff --check` was clean. A TeX
engine was unavailable on this machine, so compilation and page rendering are
assigned to the TeX-equipped handoff.

**Decision:** accept the semantic-oracle and manuscript mutation. Do not assign
a final runtime score until one clean fingerprinted run completes the frozen
main, environment, deterministic-policy, robustness, and detector programme.

**Next mutation hypothesis:** generate final profile, policy, phase, and latency
figures only from the frozen aggregate bundle.
