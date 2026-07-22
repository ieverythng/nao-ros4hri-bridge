# Prism Revision Interface

## Task

Revise the supplied LaTeX thesis source conservatively. Reduce repetition and
printed supporting material, then integrate the compact model-comparison
evidence into Validation Methodology, Results, Discussion, Limitations, and
Conclusion. Return compilable LaTeX, not a prose-only rewrite.

Read `tfm_surgical_reduction_and_evidence_ledger_2026-07-22.md` before editing.
It is the governing editorial and evidence contract.

## Inputs

- `01_thesis_source/`: authoritative reconciled manuscript.
- `02_revision_contract/`: supervisor notes and surgical ledger.
- `03_compact_evidence/primary/primary_dataset_108.csv`: primary qualification
  index already represented in the manuscript.
- `03_compact_evidence/model_invariance/model_invariance_dataset_191.csv`:
  separate three-model comparison.
- `03_compact_evidence/model_invariance/model_overall_summary.csv`: all-record
  status totals.
- `03_compact_evidence/model_invariance/model_suite_summary.csv`: suite-level
  status counts.
- `03_compact_evidence/model_invariance/shared_failures_all_three.csv`: strict
  intersection of failures across the three models.
- `03_compact_evidence/model_invariance/REPORT.md` and `METHODOLOGY.md`: source
  interpretation and run contract.

## Mandatory Structure

Use the same five modes in methodology and results:

1. simple dialogue;
2. KB interaction;
3. simple skill;
4. composite skill;
5. failure management, separated into beginning, middle, and end evidence.

The primary Qwen3-VL campaign remains the principal qualification. The Ollama
campaign is a separate model-invariance supplement. Never add their rows,
passes, failures, or denominators together.

## Required Chapter Operations

### Chapter 4, Implementation

- Keep the contract overview and normative ownership.
- Keep Contract 4's shared skill-entry semantics in the body.
- Reduce repeated field explanations and JSON examples; move exhaustive detail
  to Appendix A where needed.
- Keep planning-time context distinct from execution-time evidence.

### Chapter 5, Validation Methodology

- Organise the principal method around the five modes.
- Retain unit of analysis, fixed runtime tuple, fixtures, required and forbidden
  observations, reset assumptions, and verdict semantics.
- State that `degraded` is scoreable but is not a pass; `not_scored` and blank
  timeout rows are excluded.
- Describe beginning, middle, and end failure evidence without claiming a
  balanced three-position injection experiment.

### Chapter 6, Results

- Lead with aggregate findings and failure classes, not a narration of every
  passing case.
- Preserve the primary 64/72 and questionnaire 20/22 results.
- Add a separate table for the three-model campaign using exact model and suite
  summaries from the compact CSVs.
- Report that all models passed environment 11/11, while semantic variation
  remained in composition, grounding-dependent selection, clarification, and
  closure.
- Do not call the comparison a statistical ranking.

### Chapters 7--9

- Discussion interprets results and does not repeat all counts.
- Limitations state the scenario, simulation, model, observability, and
  failure-position boundaries once.
- Conclusion answers the research aim with bounded claims and no new result.

### Appendix D

- Keep dataset scope, run inventory, field dictionary, attachment, and checksum.
- Remove the printed 108-row case index. The attached CSV is the index.
- Call the 108-row dataset the primary thesis evidence index once the separate
  191-row model comparison is included.

## Vocabulary and Style

- Use "integrated NAO architecture" or "integrated NAO stack" for the system.
- Use ROS4HRI only for a concrete package, interface, or literature reference.
- Use "multi-step deterministic fake-skill validation" in thesis prose.
- Use `fake_deep` only as an exact machine-readable source label.
- Do not mention internal abstraction-boundary levels or Neural Workbench.
- Avoid em dashes, marketing language, meta-commentary, and bare novelty claims.
- Use `\textcite{...}` only when authors are grammatical subjects and
  `\cite{...}` for parenthetical citations.
- Introduce every figure and table in the preceding prose.

## Evidence Restrictions

- Do not infer physical navigation, grasping, perception accuracy, or human
  safety from deterministic simulated adapters.
- Do not convert a safe rejection into task success.
- Do not convert missing terminal evidence into success.
- Do not attribute a shared failure to either the model or stack as fact unless
  the evidence contains a discriminating probe.
- Do not describe model endpoint availability as task qualification.

## Required Outputs

1. Complete compilable Overleaf project.
2. PDF compiled from that exact project.
3. `REVISION_LOG.md` with section-level changes and the final page count.
4. `EVIDENCE_CHECK.md` listing every numerical claim and its source file.
5. `UNRESOLVED_ITEMS.md` for claims or edits that require author confirmation.

Before returning the project, run at least two LaTeX passes plus Biber, check
undefined references and citations, inspect all landscape pages and long tables,
and confirm that the PDF attachment still opens or that its external checksum is
reported correctly.
