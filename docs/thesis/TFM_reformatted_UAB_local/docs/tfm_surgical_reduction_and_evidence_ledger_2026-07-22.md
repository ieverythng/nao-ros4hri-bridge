# TFM Surgical Reduction and Evidence Ledger

Date: 2026-07-22

Manuscript base: `TFM_reformatted_UAB_Overleaf_2026-07-15.zip`

Locally compiled baseline: 82 PDF pages with MacTeX (`latexmk`, pdfLaTeX and Biber)

## Purpose

This ledger defines a conservative editing pass. The manuscript has already been
reviewed closely, so revisions must preserve its architecture, ownership rules,
contract semantics, evidence boundaries, and supervisor-approved vocabulary.
The aim is to reduce repetition and printed supporting material while making
Chapters 5--9 read as one validation argument.

The current local pagination is:

| Material | Physical PDF pages |
| --- | ---: |
| Front matter | 1--12 |
| Chapter 1, Introduction | 13--15 |
| Chapter 2, Background | 16--20 |
| Chapter 3, System Design | 21--24 |
| Chapter 4, Implementation, including runtime contracts | 25--44 |
| Chapter 5, Validation Methodology | 45--53 |
| Chapter 6, Results | 54--61 |
| Chapter 7, Discussion | 62--65 |
| Chapter 8, Limitations and Future Work | 66--68 |
| Chapter 9, Conclusion | 69 |
| Appendices A--C | 70--74 |
| Appendix D, dataset and printed case index | 75--80 |
| Bibliography | 81--82 |

The supplied change log reports 75 pages. The same source compiles to 82 pages
locally. Use the local 82-page build as the reduction baseline until Overleaf is
compiled from the reconciled source and the difference is explained. Do not
claim a page saving against the older number.

## Non-Negotiable Editorial Rules

1. Treat this as a validation of a bounded integrated NAO system, not a general
   evaluation of unrestricted autonomy.
2. Use five thesis-facing validation modes: simple dialogue, KB interaction,
   simple skill, composite skill, and failure management.
3. Within failure management, distinguish beginning-, middle-, and end-stage
   evidence. Do not imply a balanced injected three-position experiment. The
   retained evidence has controlled beginning and middle failures; end-stage
   evidence concerns final action/report closure rather than a dedicated
   last-skill injection.
4. Keep the primary Qwen3-VL qualification and the later Ollama model-invariance
   campaign as separate evidence scopes and denominators.
5. `not_scored` and unresolved timeout rows do not enter pass-rate denominators.
   `degraded` remains scoreable but is not a pass. Missing mandatory terminal
   evidence is a failure unless correlation itself is unavailable.
6. Replace thesis-facing `fake_deep` with "multi-step deterministic fake-skill
   validation". Retain `fake_deep` only where it is an exact source-data label.
7. Do not introduce internal abstraction-boundary terminology or Neural
   Workbench into the thesis. Refer to runtime skills and the skill registry.
8. Describe ROS4HRI only when naming a concrete package, interface, or cited
   body of work. It is an interoperability boundary, not the name of the thesis
   architecture.
9. Preserve component ownership: the dialogue manager owns dialogue lifecycle
   and speech; the chatbot owns response generation, route selection, grounding
   projection, and planner handoff; the planner produces and revises plans; the
   orchestrator validates and dispatches deterministically; skills own fresh
   execution evidence; `kb_skills` owns KB transport.
10. Use `\textcite{...}` only when an author is the grammatical subject. Use
    `\cite{...}` for parenthetical support.

## Reduction Ledger

| Priority | Source location | Surgical operation | Preserve explicitly | Estimated saving | Risk |
| --- | --- | --- | --- | ---: | --- |
| P0 | Appendix D, `11_appendix.tex`, "Complete Case Index" and `12_dataset_rows.tex` | Remove the printed 108-row index. Keep the embedded CSV, run inventory, field dictionary, status semantics, and SHA-256 value. Replace the index with one sentence directing the reader to the attachment. | Dataset accessibility, provenance, 108-record count, field definitions, checksum. | 5--6 pages | Low |
| P1 | Chapter 4, Contracts 1--5 in `04_runtime_contracts.tex` | Keep the overview map and one representative payload. Replace repeated JSON-plus-field-table pairs with compact boundary tables. Move exhaustive field dictionaries and remaining payload examples to Appendix A if they are necessary. | Contract number, API/topic, producer, validator, consumer, lineage, evidence invariant. | 2--3 pages | Medium |
| P1 | Chapter 5, Validation Levels and Acceptance Criteria | Merge the five-stage enumeration into the five supervisor modes. Keep the three claim levels as a short paragraph or compact table. Combine overlapping criteria such as skill/order coverage and target/role integrity only when their pass rules remain explicit. | Unit of analysis, pass/fail/not-scored policy, software-versus-physical claim boundary. | 0.75--1.25 pages | Medium |
| P1 | Chapter 6, Frozen Evidence Scope and Result Summary | Remove the closing numerical recap after the same values have appeared in the opening table and mode tables. Begin with the principal comparative result, then direct the reader to denominators. | 64/72 primary result, 20/22 questionnaire result, separate campaign scopes. | 0.4--0.7 pages | Low |
| P1 | Chapter 6, per-case prose after the questionnaire table | Replace the list of passing cases with one analytical paragraph on what the mode establishes and one paragraph on the two failure classes. | Red-cup route violation; kitchen recipient-role/closure failure; safe non-dispatch. | 0.4--0.7 pages | Low |
| P1 | Chapter 7, eight short sections | Merge into four sections: model/backend variation; contract containment; grounding, roles, and target sets; failure handling and portability. Refer to Chapter 6 tables instead of repeating scores. | Interpretation, causes that remain hypotheses, safety boundary, no physical-performance claim. | 1--1.5 pages | Medium |
| P2 | Chapter 4, node descriptions in `05_implementation.tex` | Remove sentences that restate the contract field tables or Chapter 3 runtime sequence. Retain implementation mechanisms, lifecycle/ROS APIs, validation points, and diagram introductions. | Exact ownership and deterministic execution boundary. | 0.5--1 page | Medium |
| P2 | Chapter 5, full 22-case questionnaire | Keep the five-mode aggregate manifest in the chapter. Move exact utterances and case IDs to Appendix C or the attached dataset, unless the supervisor requires every utterance in the body. | Reproducible prompt set and expected route/behaviour. | 1--2 pages | Medium |
| P2 | Chapters 8 and 9 | Remove repeated denominator lists and named failure lists already reported in Chapter 6. State limitations by class and conclude with the architecture-level result. | Scenario-bounded scope, simulated-skill limitation, model qualification requirement. | 0.5--0.8 pages | Low |
| P3 | Appendices A--C | Keep the skill inventory, one launch profile, and trace schema. Remove explanatory prose already present in Chapter 4 and obsolete example labels such as `S1--S8` if the dataset uses named cases. | Reproducibility and interface inventory. | 0.3--0.6 pages | Low |

Applying P0 and P1 should bring the local build to approximately 72--75 pages
without deleting a substantive claim. P2 can reduce it further if required.
Do not alter margins, font size, line spacing, float scaling, or bibliography
format merely to meet a page target.

## Chapter 4: Contract-Specific Guidance

The runtime-contract section is valuable because it shows where model output
loses authority and where typed evidence begins. Its current 12-page span is
long because Contracts 1--5 each repeat three forms of the same information:
prose introduction, JSON listing, and field table.

Recommended body structure:

1. Retain Table 4.1 as the ownership map.
2. Retain one end-to-end payload example, preferably the planner request and
   planner output together, because it shows grounding and lineage.
3. Give each contract a compact four-column row: API or payload, producer and
   consumer, deterministic check, and invariant.
4. Keep Contract 4's skill-entry semantics in the body because preconditions,
   expected effects, observable success, known failures, and adapter mapping
   are a central contribution.
5. Move exhaustive field definitions and standalone JSON listings for Contracts
   1, 3, and 5 to Appendix A. Contracts 6--9 are already comparatively concise.
6. Do not transfer all ownership wording to node descriptions. Contract
   ownership is normative; node descriptions explain implementation.

The reduction must retain the distinction between planning-time facts and
execution-time evidence. A grounded entity or declared expected effect supports
planning but cannot establish that a skill succeeded.

## Chapters 5 and 6: Evaluation Spine

Use the following five-mode structure in both methodology and results so that
the reader does not have to translate between "levels", "families", and case
sets:

| Mode | Methodological question | Principal evidence |
| --- | --- | --- |
| Simple dialogue | Does one non-action turn produce one response with no planner or skill activity? | Route, response, planner absence, speech ownership. |
| KB interaction | Are entities, properties, relations, and state updates represented truthfully without accidental robot action? | Grounded context, KB pre/post query, route, response terms. |
| Simple skill | Does one grounded request produce an admissible skill call, typed result, and truthful report? | Plan, dispatch, skill result, execution feedback, closure. |
| Composite skill | Are all requested targets, roles, capabilities, and ordering constraints preserved? | Target selection, ordered plan, execution sequence, postconditions, report coverage. |
| Failure management | Does a failed step stop obsolete work and lead to a bounded replan, clarification, or truthful failure? | Failure position, plan version, preserved completed work, terminal result. |

Chapter 5 should define the mode, unit, fixture, expected route, forbidden
events, mandatory phases, and verdict rule. Chapter 6 should report aggregate
outcomes by mode and then analyse the dominant error classes. Do not reproduce
the complete case manifest in both chapters.

## Chapter 6: Required Evidence Separation

### Primary qualification

Keep the existing primary Qwen3-VL campaign as the thesis's principal
qualification: 72 records, 64 passes and 8 failures. The 22 exercised
end-to-end requirements produced 20 passes and 2 failures. Its environment,
stateful KB, repeated robustness, controlled failure, and capability-extreme
subsets remain the evidence used to answer the research questions.

### Model-invariance supplement

Present the 2026-07-21 Ollama campaign as a later, matched comparison. It has
191 records across three models and must not be added to the 108-record thesis
dataset denominator. The all-record status totals are:

| Model | Records | Pass | Degraded | Fail | Not scored | Unresolved |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `gemma4:31b-cloud` | 69 | 54 | 0 | 10 | 3 | 2 |
| `gemma4:cloud` | 61 | 52 | 0 | 9 | 0 | 0 |
| `nemotron-3-super:cloud` | 61 | 44 | 2 | 11 | 3 | 1 |

For the comparable standard families, report the source report's strict rates:
50/59 (84.7%), 51/60 (85.0%), and 43/56 (76.8%), respectively. These are
descriptive one-run engineering results, not confidence intervals or a general
model ranking.

The strongest cross-model observations are:

- all three models passed the 11-case environment profile;
- both Gemma variants passed all seven scoreable KB-stress cases;
- all-success multi-step deterministic fake-skill validation passed 9/9 for
  both Gemma variants and 8/9 plus one degraded case for Nemotron;
- targeted fail-once navigation passed for `gemma4:cloud` and Nemotron;
- seven failures were shared by all three models, concentrated in demanding
  composition, object-grounded picking, recipient clarification, and recovery;
- Nemotron added KB wording omissions and more high-composition coverage loss.

The supported conclusion is that interfaces and several runtime paths remained
stable across compatible backends, while semantic output was not model
invariant. Shared failures are seam hypotheses, not proof of a single stack
defect. Model-specific failures are variance candidates, not proof that all
other system components were correct.

## Failure-Position Claims

Use a small table rather than implying a factorial experiment:

| Position | Evidence available | Permitted claim |
| --- | --- | --- |
| Beginning | Primary fail-once navigation and targeted Ollama fail-once navigation. | The runtime can stop the initial plan, preserve lineage, replan, and close successfully in the tested navigation case. |
| Middle | Primary pick failure after successful preceding work; blocked delivery diagnostic. | Completed work and plan lineage can be preserved, but terminal recovery was not uniformly reliable. |
| End | Missing terminal report or closure after otherwise meaningful execution/recovery. | Completion was correctly withheld in scored cases; no dedicated symmetric last-skill injection was performed. |

## Incoherences to Resolve Before Submission

1. **Page count:** the supplied change log says 75 pages, but the reconciled
   source builds to 82 locally.
2. **Model evidence:** Chapters 6, 8, and 9 describe Ollama models as narrow
   smoke probes. The 191-record comparison supersedes that description but does
   not supersede the primary Qwen qualification.
3. **Dataset scope:** Appendix D calls the 108-row file the "final dataset".
   Rename it "primary thesis evidence index" if the model comparison is cited,
   and describe the 191-row comparison as a separate supplement.
4. **Failure matrix:** methodology correctly discloses the absence of a
   dedicated end-stage injection. Preserve this qualification in Results,
   Discussion, and Conclusion.
5. **Wave taxonomy:** the trace label `wave_greet` and canonical
   `perform_motion` projection remain inconsistent. Explain once in Results or
   a data note, not in several chapters.
6. **Acknowledgements:** the section is empty. Either complete it or omit it
   according to programme requirements.
7. **Originality form:** verify the faculty/programme text on the embedded form.
8. **Appendix log template:** `S1--S8` does not match the named final cases.
9. **Evidence hashes:** if the primary CSV or package changes, regenerate the
   SHA-256 values rather than copying old values.
10. **Current/future wording:** this is the submission manuscript. Avoid
    "currently", "so far", and promises that validation "will" be run.

## Acceptance Checklist for the Writing Agent

- [ ] The revised PDF compiles without undefined references or citations.
- [ ] Every figure and table is introduced in prose before it appears.
- [ ] The five validation modes use the same names in Chapters 5 and 6.
- [ ] Primary and model-comparison denominators are never pooled.
- [ ] Beginning, middle, and end failure evidence is described asymmetrically.
- [ ] No complete case list is repeated in the body and Appendix.
- [ ] Contract ownership and planning-versus-execution evidence remain explicit.
- [ ] No physical capability is inferred from simulated-skill success.
- [ ] Model-compatible is not used as a synonym for model-qualified.
- [ ] The final page count is measured from the generated PDF and recorded.
