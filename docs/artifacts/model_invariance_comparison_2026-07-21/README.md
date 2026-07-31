# Frozen model-invariance comparison

Start with [REPORT.html](REPORT.html) for the readable report or [REPORT.md](REPORT.md) for the Markdown source.

- [Templated DOCX report](report/Model_Invariance_Runtime_Qualification_Report.docx)
- [Rendered PDF report](report/Model_Invariance_Runtime_Qualification_Report.pdf)
- [Comparative summary](analysis/comparison_summary.json)
- [CSV dataset](analysis/dataset.csv)
- [JSONL dataset](analysis/dataset.jsonl)
- [Methodology](METHODOLOGY.md)
- [Endpoint inventory](inventory_probe.md)
- [Provenance gaps](PROVENANCE_GAPS.md)
- [Seam hypothesis audit](../model_invariance_seam_audit_2026-07-21.md)
- [Raw per-model JSON](raw/)

The vLLM semantic cell is blocked because `http://10.7.138.215:8004/v1/models` returned HTTP 000. The three completed cells use `gemma4:31b-cloud`, `nemotron-3-super:cloud`, and `gemma4:cloud` through Ollama on the same frozen image.
