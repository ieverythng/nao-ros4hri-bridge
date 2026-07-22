# Final thesis change ledger

## Comparison basis

The main before/after comparison uses:

- **Previous supervisor PDF:** `prism-uploads/TFM_Juan_V2_RRE.pdf`
  - Generated July 16, 2026
  - 91 pages
  - Title: *Design, Integration, and Validation of a Modular Language-Model-Enabled Interaction and Execution Stack for the NAO Robot*
- **Final merged PDF:** `main.pdf`
  - Generated July 20, 2026
  - 75 pages
  - Title: *Design, Integration, and Validation of a Modular LLM-Based Interactive Architecture*

The other supplied PDFs are earlier drafts and were not used as the primary baseline:

- `TFM_JUANv0_RRE.pdf`: 38 pages
- `TFM_reformatted_preview_RRE.pdf`: 58 pages
- `Obraoriginal_TFG_202425.pdf`: one-page originality declaration, inserted into the final PDF

The final PDF is 16 pages shorter than the previous supervisor PDF despite adding the declaration and dataset appendix. This is mainly due to the supervisor-requested simplification and consolidation of the architecture, runtime-contract, validation, and results material.

## Supervisor corrections preserved

The final merge preserves the corrections recorded in `docs/supervisor_corrections_TFM_Juan_V2_RRE.md`:

- The shorter, platform-general thesis title is retained.
- NAO is presented as the implementation and evaluation platform rather than as the only possible target of the architecture.
- Chapter 3 remains **System Design**.
- Runtime contracts remain part of **Implementation** instead of returning as a standalone body chapter.
- The body now has nine chapters:
  1. Introduction
  2. Background
  3. System Design
  4. Implementation
  5. Validation Methodology
  6. Results
  7. Discussion
  8. Limitations and Future Work
  9. Conclusion
- The simplified 23-case validation questionnaire and its direct acceptance criteria are retained.
- The evidence-weighted 1–10 score from the runtime report was **not** restored.
- The planner-facing inventory remains twelve robot/interaction skills.
- Knowledge add, revise, and remove operations remain knowledge-component interfaces, not robot skills.
- A wave is presented as a named `perform_motion` behavior rather than as a thirteenth `wave_greet` skill.
- Claims remain bounded to the tested software, fixtures, models, and adapters.

## Final evidence integrated

The following final files were used:

- `prism-uploads/dataset.csv`
- `prism-uploads/nao_ros4hri_model_agnostic_evidence_2026-07-20.zip`
- `prism-uploads/Model_Agnostic_Runtime_Qualification_Report.docx`
- `prism-uploads/Obraoriginal_TFG_202425.pdf`

The external CSV and DOCX are byte-identical to their copies inside the evidence ZIP.

### Consolidated dataset

- 108 case records
- 24 fields
- 33 retained JSON source artifacts
- 86 pass
- 1 degraded
- 19 fail
- 2 not scored

Attribution labels:

- 86 coherent stack successes
- 19 model/backend variance observations
- 2 harness-observability limitations
- 1 stack-contract/runtime limitation

Model-indexed evidence:

| Model | Pass | Degraded | Fail | Not scored | Scope |
|---|---:|---:|---:|---:|---|
| Qwen3-VL | 85 | 1 | 12 | 0 | Frozen v34 campaign plus application ablations |
| Qwen3.5 | 0 | 0 | 4 | 2 | Historical vLLM evidence |
| `gemma4:31b-cloud` | 1 | 0 | 1 | 0 | Current smoke plus historical maximal probe |
| `gemma4:cloud` | 0 | 0 | 1 | 0 | Historical maximal probe |
| `nemotron-3-super:cloud` | 0 | 0 | 1 | 0 | Historical maximal probe |

The final Gemma selection is described as an operational fallback with preflight and one dialogue smoke pass, not as a replacement qualification.

### Frozen primary result

The `current_v34` group contains 72 case records:

- 64 pass
- 1 degraded
- 7 fail

Key sub-results:

- Original main artifact: 21/21 under its historical oracle
- Environment profile: 11/11
- Stateful KB profile: 7/7
- Repeated robustness: 12/15
- Capability-extreme profile: 5/7
- Fake deep all-success profile: 7 pass, 1 degraded, 1 fail
- Fail-once navigation: pass with observed failure, plan-version advancement, recovery, and closure
- Targeted missing-recipient probe: fail because the required clarification was not expressed in correlated speech

### Supervisor-aligned 23-case re-adjudication

The older 21-case main artifact predates the final supervisor questionnaire. It was mapped conservatively to the new requirements:

| Family | Cases | Pass | Fail | Not scored |
|---|---:|---:|---:|---:|
| Dialogue | 5 | 5 | 0 | 0 |
| Knowledge | 8 | 6 | 1 | 1 |
| Atomic execution | 4 | 4 | 0 | 0 |
| Composite execution | 5 | 4 | 1 | 0 |
| Route safety | 1 | 1 | 0 | 0 |
| **Total** | **23** | **20** | **2** | **1** |

The two failures and one unscored requirement are explicit:

1. **Red-cup KB update — fail:** the symbolic update succeeded, but it used the execution planner and `kb_add`; the supervisor criterion requires a knowledge-only route.
2. **Kitchen cup to operator — fail:** the trace assigned the kitchen to the recipient role, clarified, and safely stopped without completing the required task.
3. **Visible red-object filter — not scored:** no retained matching probe exists, so the result was not inferred from fixture facts.

The historical navigate/wave/report behavior remains a pass, with a caveat that its trace uses the old `wave_greet` label and therefore does not independently re-test the renamed twelve-skill projection.

## Chapter-level changes

### Abstract

Added the final evidence volume and bounded headline results:

- 108 records from 33 artifacts
- frozen primary 64 pass, 1 degraded, 7 fail
- final questionnaire 20 pass, 2 fail, 1 not scored
- successful navigation replan
- incomplete maximal plans rejected before execution

### Chapter 5 — Validation Methodology

Added a conservative evidence-alignment rule for the supervisor-revised questionnaire. The method now explains when an older retained case can support a new row, when it must be re-adjudicated, and when the result must remain not scored.

The source `degraded` label is retained in the dataset, but terminal closure remains mandatory in the thesis-level interpretation.

### Chapter 6 — Results

Replaced the previous placeholder/final-run template with the complete July 20 result:

- evidence scope by run group
- supervisor-aligned 23-case result
- environment and stateful KB results
- deterministic fake-skill and recovery results
- exact three-run robustness matrix
- capability-extreme results
- application ablations
- model/backend evidence
- direct answers to all research questions
- bounded result summary

### Chapter 7 — Discussion

Rewritten around the final finding:

- interfaces are model-independent, but behavior is model-dependent
- non-determinism is contained rather than eliminated
- exact questionnaire re-adjudication is discussed
- grounding is separated from authoritative target selection
- safe rejection is separated from task success
- fail-once navigation is the strongest replan evidence
- fake-skill claims are separated from physical robot claims

### Chapter 8 — Limitations and Future Work

Updated with the observed residual failures and evidence boundaries. Future work now includes:

- LoRA or other parameter-efficient post-training
- held-out evaluation for role preservation, coverage, JSON, clarification, and closure
- schema-constrained decoding
- stronger deterministic semantic guards
- deterministic terminal closure
- separate detector and physical-adapter campaigns
- full requalification for every replacement model
- automated report/dataset consistency checks

### Chapter 9 — Conclusion

Replaced the provisional conclusion with quantified final results and the central thesis claim: deterministic contracts do not make the model invariant, but they make variance observable, restrict its execution authority, and provide a repeatable qualification boundary.

## Front matter and appendices

- Inserted `Obraoriginal_TFG_202425.pdf` immediately after the main thesis cover.
- Added the final results paragraph to the Abstract.
- Added **Appendix D: Final Evaluation Dataset**.
- Embedded `dataset.csv` as a PDF file-attachment annotation.
- Added a machine-derived run inventory.
- Added a grouped dictionary for all 24 CSV fields.
- Added SHA-256 identifiers for the CSV, ZIP, and qualification report.
- Added a complete 108-row case index in `sections/12_dataset_rows.tex`.

Evidence hashes:

- CSV: `be4d7d9b0499557cb9a16ba08cba65d58c011863affcaff74211b3e6c57931ce`
- ZIP: `9e49b71bd633a8f2005858bbb145dcb5be4917cbf37c2f34c6e6ee05d65b1992`
- DOCX: `225c38378bfc175c69c8bb9d23b1dcdb90516473607026d8d99d4d4a8d7667c7`

## Source inconsistencies retained transparently

Two supplied summaries disagree with the machine-derived data:

1. **A0 smoke result**
   - CSV: 6/6 expected trajectories
   - Qualification report: 5/6 manual semantic passes, median 3.89 s
   - Final thesis: both are shown; the application setting remains retained.

2. **Compact DOCX run inventory**
   - DOCX: `current_v34=15` source files and `current_ollama_switch=1`
   - Machine `dataset_summary.json`: `current_v34=12` and `current_ollama_switch=2`
   - The DOCX entries sum to 35 despite stating 33 total artifacts.
   - Final thesis: uses the machine-derived 12/13/3/3/2 inventory, which sums to 33.

No silent correction was made to the supplied artifacts.

## Remaining author/SV checks

1. **Acknowledgements are still blank.** This is the only visibly unfinished prose section.
2. **Originality-form faculty mismatch:** the supplied form says *Facultat de Ciències de la Comunicació*, while the thesis cover says *Faculty of Science*. Confirm with the programme or supervisor that this is the correct form before signing.
3. The red-object-filter questionnaire row can only move from not scored after an exact retained rerun.
4. The current Gemma fallback should not be described as fully qualified without the complete frozen campaign.
5. Fake-skill results should not be promoted to physical navigation, grasping, manipulation, perception, or human-safety claims.

## Files changed in this final merge

- `main.tex`
- `sections/00_abstract.tex`
- `sections/06_validation_methodology.tex`
- `sections/07_results.tex`
- `sections/08_discussion.tex`
- `sections/09_limitations_future_work.tex`
- `sections/10_conclusion.tex`
- `sections/11_appendix.tex`
- `sections/12_dataset_rows.tex` — new
- `changes.md` — new

## Verification performed

- Full `latexmk`/Biber build completed successfully.
- Final PDF contains 75 pages.
- No undefined references or citations remain.
- No result placeholders such as “pending final run” remain.
- The originality declaration is the second physical PDF page.
- The dataset is present as a 36,563-byte embedded file-attachment annotation.
- The complete appendix index contains 108 rows.
- External CSV and DOCX match the copies inside the evidence ZIP.
- `git diff --check` passes after the final formatting cleanup.
