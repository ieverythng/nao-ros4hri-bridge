# Compact Model-Invariance Evidence

This directory is the compact, thesis-facing projection of the supplied
`model_invariance_comparison_2026-07-21.zip`. It omits raw trace JSON, rendered
PDF/DOCX reports, and duplicate archive material.

The campaign contains 191 records across `gemma4:31b-cloud`, `gemma4:cloud`,
and `nemotron-3-super:cloud`. It is a separate model-comparison supplement and
must not be pooled with the primary 108-record thesis evidence index.

## Files

- `model_invariance_dataset_191.csv`: per-case evidence index.
- `model_overall_summary.csv`: all-record status totals by model.
- `model_suite_summary.csv`: status totals by model and source suite.
- `shared_failures_all_three.csv`: failure intersection across all models.
- `comparison_summary.json`: full machine-derived aggregate and case matrix.
- `dataset_summary.json`: inventory and attribution totals.
- `source_REPORT.md`: supplied analysis report.
- `source_METHODOLOGY.md`: supplied run and scoring contract.
- `source_PROVENANCE_GAPS.md`: declared evidence limitations.
- `SHA256SUMS.txt`: checksums for the compact files.

## Thesis Vocabulary

The source suite label `fake_deep` is retained only in machine-readable fields.
In thesis prose, call it "multi-step deterministic fake-skill validation".
Likewise, replace broad references to a "ROS4HRI stack" in the source report
with "integrated NAO stack" unless a concrete ROS4HRI package or interface is
being named.

## Scoring

`pass` is a scoreable success. `degraded` is scoreable but is not counted as a
pass. `fail` is a scoreable failure. `not_scored` and blank unresolved timeout
rows are excluded from strict pass-rate denominators.

The source report also defines a narrower set of comparable standard families.
Those strict rates are 50/59 for `gemma4:31b-cloud`, 51/60 for
`gemma4:cloud`, and 43/56 for `nemotron-3-super:cloud`. Do not derive those
figures by treating every row in `model_overall_summary.csv` as comparable.
