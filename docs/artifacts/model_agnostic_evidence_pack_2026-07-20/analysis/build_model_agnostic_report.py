#!/usr/bin/env python3
"""Create the model-agnostic runtime qualification report from the retained template."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
import zipfile
from pathlib import Path

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
from docx.enum.style import WD_STYLE_TYPE
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt


TITLE = "Model-Agnostic NAO ROS4HRI Runtime Qualification"
DATE = "20 July 2026"


def _set_cell_shading(cell, fill: str) -> None:
    properties = cell._tc.get_or_add_tcPr()
    shading = properties.find(qn("w:shd"))
    if shading is None:
        shading = OxmlElement("w:shd")
        properties.append(shading)
    shading.set(qn("w:fill"), fill)


def _set_cell_text(cell, text: str, *, bold: bool = False, size: float = 8.5) -> None:
    cell.text = ""
    paragraph = cell.paragraphs[0]
    paragraph.paragraph_format.space_after = Pt(2)
    run = paragraph.add_run(str(text))
    run.bold = bold
    run.font.name = "Helvetica Neue"
    run.font.size = Pt(size)
    cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER


def _table(doc: Document, headers: list[str], rows: list[list[str]], widths: list[float]) -> None:
    table = doc.add_table(rows=1, cols=len(headers))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    for index, header in enumerate(headers):
        cell = table.rows[0].cells[index]
        _set_cell_text(cell, header, bold=True, size=8.5)
        _set_cell_shading(cell, "D9E2F3")
        cell.width = Inches(widths[index])
    for row in rows:
        cells = table.add_row().cells
        for index, value in enumerate(row):
            _set_cell_text(cells[index], value, size=8.2)
            cells[index].width = Inches(widths[index])
    doc.add_paragraph("")


def _set_paragraph(paragraph, text: str) -> None:
    paragraph.text = text
    for run in paragraph.runs:
        run.font.name = "Helvetica Neue"


def _set_style_fonts(doc: Document) -> None:
    for style_name in ("Normal", "normal", "Title", "Heading 1", "Heading 2", "List Bullet"):
        try:
            style = doc.styles[style_name]
        except KeyError:
            continue
        style.font.name = "Helvetica Neue"
        if style_name.lower() in {"normal", "list bullet"}:
            style.font.size = Pt(10.5)


def _add_bullets(doc: Document, items: list[str]) -> None:
    for item in items:
        try:
            style = doc.styles["Report Bullet"]
        except KeyError:
            style = doc.styles.add_style("Report Bullet", WD_STYLE_TYPE.PARAGRAPH)
            style.base_style = doc.styles["Normal"]
            style.paragraph_format.left_indent = Inches(0.25)
            style.paragraph_format.first_line_indent = Inches(-0.15)
        paragraph = doc.add_paragraph(style=style)
        paragraph.paragraph_format.space_after = Pt(3)
        paragraph.add_run("• " + item)


def _replace_template_header(output: Path) -> None:
    """Replace text-box placeholders retained outside python-docx paragraphs."""
    with tempfile.NamedTemporaryFile(dir=output.parent, suffix=".docx", delete=False) as temporary:
        temporary_path = Path(temporary.name)
    try:
        with zipfile.ZipFile(output, "r") as source, zipfile.ZipFile(temporary_path, "w") as target:
            for info in source.infolist():
                content = source.read(info.filename)
                if info.filename == "word/header1.xml":
                    content = content.replace(b"Report title", b"Model-Agnostic Runtime Qualification")
                    content = content.replace(b">Date<", b">20 July 2026<")
                target.writestr(info, content)
        os.replace(temporary_path, output)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def build(template: Path, output: Path, summary_path: Path) -> None:
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    doc = Document(str(template))
    _set_style_fonts(doc)
    for section in doc.sections:
        for paragraph in section.header.paragraphs:
            if "Report title" in paragraph.text:
                _set_paragraph(paragraph, "Model-Agnostic Runtime Qualification")
            elif paragraph.text.strip() == "Date":
                _set_paragraph(paragraph, DATE)

    cover_table = doc.tables[0]
    _set_cell_text(cover_table.cell(0, 0), "Runtime evidence, model variance, and fallback qualification", size=10)
    _set_cell_text(cover_table.cell(0, 2), "Prepared for TFM review\n" + DATE, size=9)
    overview_table = doc.tables[1]
    overview_rows = [
        ("Qualification", "v34 Qwen3-VL runtime score 8.4/10", "Qualified primary with bounded model variance"),
        ("Dataset", "108 cases from 33 JSON artifacts", "Raw provenance retained for thesis review"),
        ("Availability", "vLLM unreachable at the final live check", "Ollama fallback selected after probes"),
    ]
    for index, value in enumerate(("Theme", "Observation", "Implication")):
        _set_cell_text(overview_table.cell(0, index), value, bold=True, size=8.5)
        _set_cell_shading(overview_table.cell(0, index), "D9E2F3")
    for row_index, values in enumerate(overview_rows, start=1):
        for column_index, value in enumerate(values):
            _set_cell_text(overview_table.cell(row_index, column_index), value, size=8.2)
    _set_paragraph(doc.paragraphs[2], TITLE)
    _set_paragraph(
        doc.paragraphs[5],
        "Executive summary | Introduction | Key findings | Context and conditions | Patterns in the evidence | Implications | Recommendations | Conclusion | Appendix",
    )

    _set_paragraph(
        doc.paragraphs[7],
        "The frozen v34 NAO ROS4HRI stack reached a qualified runtime score of 8.4/10 with QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ. The result is evidence about a stack configuration under a fixed model and endpoint, not a universal property of every language model. The review separated deterministic contract tests from live ROS runtime cases and from repeated model-sensitive cases.",
    )
    _set_paragraph(
        doc.paragraphs[8],
        "Across the consolidated dataset, 108 questionnaire cases were retained from 33 JSON artifacts. The v34 campaign showed strong dialogue, grounding, KnowledgeCore, planner admission, fake-skill, and recovery behavior. Remaining failures were concentrated in model-authored target selection, maximal capability coverage, missing-recipient wording, and terminal closure. The current vLLM endpoint became unreachable, so the live demonstration was moved to gemma4:31b-cloud through Ollama. That fallback passed preflight and a speech smoke turn, but it is not presented as a replacement qualification.",
    )
    _set_paragraph(doc.paragraphs[10], "Qualified primary: 8.4/10 on the v34 frozen response-first campaign.")
    _set_paragraph(doc.paragraphs[11], "Evidence volume: 108 case records, 33 source JSON artifacts, plus HTML and Markdown reports.")
    _set_paragraph(doc.paragraphs[12], "Current demo: gemma4:31b-cloud, selected after vLLM became unreachable and Ollama probes passed.")
    _set_paragraph(doc.paragraphs[14], "The stack is model-agnostic at the contract boundary, while generated route, target, and plan content remains model-dependent. A defensible evaluation therefore keeps ownership seams fixed and varies the model as an experimental factor. The evidence pack preserves raw artifacts, runtime metadata, speech observations, structured traces, and the assessment applied to each case.")
    _set_paragraph(doc.paragraphs[15], "The analysis treats model variance and stack coherence as related but distinct questions. A malformed intent or incomplete universal selection is attributed to model or backend variance when the deterministic owner and grounded evidence remain valid. A fabricated effect, duplicate goal, or contract violation remains a stack failure regardless of model. Backend reachability is reported as a runtime dependency condition.")

    _set_paragraph(doc.paragraphs[17], "The evidence supports a qualified primary model and a stable stack contract with bounded residual variance. The strongest results came from explicit grounded context, deterministic executor admission, AB=1 effect evidence, and structured trace correlation. The weakest results occurred when a model had to cover many requested actions, preserve every target through a composite plan, or produce closure wording after a failure or clarification.")
    _set_paragraph(doc.paragraphs[19], "The v34 runtime tuple held the container image, launch wiring, prompt packs, sampling parameters, token budgets, timeout values, KnowledgeCore fixtures, fake-skill policies, and questionnaire cases constant. The active vLLM identity was proven during the primary campaign. Later endpoint checks showed a different advertised vLLM model and then no route to port 8004. This temporal drift is why endpoint inventory and named completion probes are part of the operational evidence.")
    _set_paragraph(doc.paragraphs[21], "The model transition pattern is consistent across the raw cases. Ordinary dialogue, grounded state queries, and short skill requests are more stable than maximal multi-intent plans. Repeated robustness cases exposed incomplete universal selections and one dropped report policy without allowing fabricated execution to cross the orchestrator. The same stack can therefore remain coherent while model output quality varies at the planning boundary.")
    _set_paragraph(doc.paragraphs[22], "The current fallback path confirms availability rather than semantic equivalence. Gemma4:31b-cloud passed Ollama liveness, chatbot preflight, planner preflight, and one simple dialogue smoke case. The NAOqi connection timed out independently because the robot endpoint was unreachable. That condition is retained as a network limitation and is not conflated with model behavior.")
    _set_paragraph(doc.paragraphs[23], "Key takeaway. Stack-level claims should be made from invariant owners, grounded evidence, execution lineage, and truthful speech. Model-level claims require a frozen repeated campaign. The evidence supports both statements without collapsing them into one score.")
    _set_paragraph(doc.paragraphs[24], "Attribution framework")
    _set_paragraph(doc.paragraphs[25], "Implications")
    _set_paragraph(doc.paragraphs[26], "The dataset labels successful coherent cases, model or backend variance, runtime dependency failures, and harness observability gaps separately. These labels are interpretive summaries anchored to raw artifacts. They are useful for thesis writing because they identify which results can support a stack conclusion and which results require model-specific qualification.")

    _set_paragraph(doc.paragraphs[28], "The recommended demonstration policy is operationally conservative: query and probe the preferred vLLM endpoint first, then use a tested Ollama candidate if the preferred backend cannot pass readiness. Pin the result for the run. Record the fallback event with the unavailable model, replacement model, backend, timestamp, and reason. Requalify a replacement only through the frozen runtime campaign.")
    _set_paragraph(doc.paragraphs[29], "Preserve the model-independent contract. Keep chatbot dialogue ownership, planner normalization, orchestrator admission, KnowledgeCore transport, scene grounding, and AB=1 effect evidence unchanged while comparing models.")
    _set_paragraph(doc.paragraphs[30], "Separate availability from quality. Inventory endpoints and run small non-thinking probes before launch, but do not interpret HTTP success as evidence of target-selection or multi-step planning quality.")
    _set_paragraph(doc.paragraphs[31], "Repeat the sensitive cases. Re-run missing-recipient clarification, universal target selection, grouped delivery closure, maximal capability coverage, and fail-once recovery at least three times for each candidate.")

    _set_paragraph(doc.paragraphs[33], "The v34 frozen run is suitable as the primary stack qualification with explicit model scope. It demonstrates that the surrounding ROS4HRI architecture can preserve grounding, execution truth, and recovery boundaries under substantial runtime testing. The evidence also shows why a model replacement cannot be accepted from a single successful conversation or endpoint probe.")
    _set_paragraph(doc.paragraphs[34], "The operational selector closes the immediate availability seam while preserving scientific interpretation. It gives the demo a visible, pinned fallback and leaves the thesis result honest: Qwen3-VL is the qualified primary, gemma4:31b-cloud is the current live fallback, and all other reserve models remain candidates until re-evaluated.")
    _set_paragraph(doc.paragraphs[37], "Test semantics. Deterministic unit and source tests establish parser, contract, registry, and ownership behavior. Live questionnaire cases exercise the running ROS graph, endpoint, KnowledgeCore, fake skills, speech, and trace seams. The two categories must not be added together and described as one stress score.")
    _set_paragraph(doc.paragraphs[38], "The 108-case dataset is a provenance index over retained artifacts, not a new benchmark. Case rows link back to source files, while the raw JSON remains authoritative for wording, targets, lineage, and timing. The current fallback has a much smaller qualification scope than the v34 primary.")
    _set_paragraph(doc.paragraphs[40], "NAO ROS4HRI runtime artifacts. Frozen v34 review, ablations, historical endpoint runs, and current fallback smoke evidence. Local project evidence pack, 20 July 2026.")
    _set_paragraph(doc.paragraphs[41], "Robot runtime performance review. REPORT.md and REPORT.html for iiia:nao-runtime-v34-final-frozen-review. Local project artifact, 20 July 2026.")
    _set_paragraph(doc.paragraphs[42], "Model-agnostic questionnaire dataset. dataset.csv, dataset.jsonl, and dataset_summary.json. Generated from retained runtime artifacts, 20 July 2026.")
    _set_paragraph(doc.paragraphs[43], "Operational backend probe. backend_probe_2026-07-20.md and current Ollama launch artifacts. Local runtime evidence, 20 July 2026.")
    _set_paragraph(doc.paragraphs[45], "Template note. This report preserves the retained design-report visual system. Page flow and rendered pages were checked after generation; raw evidence remains in the accompanying ZIP.")

    doc.add_page_break()
    doc.add_heading("Detailed evidence tables", level=1)
    doc.add_paragraph("The following tables provide the compact quantitative view used for thesis review. They are summaries of the retained files and should be read with the raw artifact links in the ZIP.")
    _table(
        doc,
        ["Evidence layer", "Observed result", "Interpretation"],
        [
            ["Deterministic source gates", "Planner, chatbot, registry, compilation, and ROS4HRI audits passed in the source campaign", "Contract and implementation evidence; not runtime stress"],
            ["Live v34 main harness", "21/21 cases completed, with manual wording and closure downgrades retained", "Strong end-to-end stack behavior under the qualified primary"],
            ["Live repeated robustness", "12/15 across three repetitions", "Model-sensitive route, target, and report-closure variance"],
            ["Environment and KB", "11/11 environment, 7/7 stateful KB chain", "Grounding and state mutation seams remained coherent"],
            ["Fake deep and failure recovery", "7 pass, 1 degraded, 1 fail; fail-once navigation passed", "Executor truth and replan lineage protected against false completion"],
            ["Current fallback smoke", "Gemma4 preflights passed; one speech smoke case passed", "Operational demo readiness only"],
        ],
        [1.55, 3.1, 2.15],
    )

    doc.add_heading("Model comparison and reserve status", level=1)
    _table(
        doc,
        ["Rank", "Model and backend", "Evidence-backed status"],
        [
            ["1", "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ\nvLLM", "Qualified primary at 8.4/10 on v34. Strong grounding, delivery, recovery, and ordinary composites; residual universal-selection and maximal-plan variance."],
            ["2", "QuantTrio/Qwen3.6-35B-A3B-AWQ\nvLLM", "Startup evidence existed, but no complete locked score. Requalify only when the named endpoint remains stable for the entire run."],
            ["3", "qwen3-coder:480b-cloud\nOllama", "Strong prior evidence, later blocked by account or service limits. Current inventory still advertises the name, but prior HTTP 410 makes it a reserve only."],
            ["4", "nemotron-3-super:cloud\nOllama", "Current liveness passed. Prior planner capability evidence was incomplete, so it is suitable for narrow probes until requalified."],
            ["5", "gemma4:31b-cloud\nOllama", "Current demo fallback. Preflight and one smoke case passed; historical maximal intent behavior was not sufficient for planner qualification."],
            ["6", "gemma4:cloud\nOllama", "Current liveness passed and historical partial recovery existed. Keep as last resort pending frozen requalification."],
        ],
        [0.45, 2.0, 4.35],
    )

    doc.add_heading("Application sampling ablation", level=1)
    _table(
        doc,
        ["Cell", "Result", "Decision"],
        [
            ["A0 application sampling", "5/6 smoke semantics; median response 3.89 s", "Selected"],
            ["A1 published Qwen sampling", "5/6; median 7.28 s", "Rejected for latency without semantic gain"],
            ["A2 deterministic sampling", "5/6; median 4.16 s", "Rejected because semantics did not improve"],
            ["A3 256/256/1024 token budgets", "Maximal coverage still failed and latency increased", "Rejected"],
            ["A4 60/30/20/45 timeouts", "Smoke completed without transport fallback", "Selected"],
        ],
        [1.65, 3.25, 1.9],
    )

    doc.add_heading("Attribution and model variance", level=1)
    _table(
        doc,
        ["Dataset label", "Count", "Use in the thesis"],
        [
            ["stack_coherent_success", str(summary.get("attribution_counts", {}).get("stack_coherent_success", 0)), "Evidence that the owned seams produced the expected trajectory."],
            ["model_or_backend_variance", str(summary.get("attribution_counts", {}).get("model_or_backend_variance", 0)), "Model-sensitive route, selection, JSON, coverage, or planner behavior."],
            ["harness_observability", str(summary.get("attribution_counts", {}).get("harness_observability", 0)), "Evidence correlation or measurement issue requiring caution."],
            ["stack_contract_or_runtime", str(summary.get("attribution_counts", {}).get("stack_contract_or_runtime", 0)), "Insufficient evidence for a narrower attribution."],
        ],
        [2.0, 0.8, 4.0],
    )

    doc.add_page_break()
    doc.add_heading("Operational selector and reproducibility", level=1)
    doc.add_paragraph("The selector is implemented as a launch-time wrapper around the existing ROS launch profile. It does not alter the frozen default launch arguments. An explicit model supplied through chatbot_model, ollama_model, or planner_llm_model is tried first. If it fails a hard readiness condition, the resolver tries the vLLM inventory and then the ordered Ollama inventory. One successful choice is pinned for the launch.")
    _add_bullets(
        doc,
        [
            "Hard failure conditions include an unreachable inventory endpoint, a non-success HTTP response, an empty assistant message, or a failed readiness probe.",
            "A slow but still responding request does not trigger a mid-run model change. The selector is evaluated at launch so the conversation remains model-consistent.",
            "Fallback emits a human-readable message naming the unavailable preferred model and the selected replacement.",
            "Fallback also appends a JSONL event with the requested model, current model, backend, timestamp, and reason.",
            "No fallback model is promoted to qualified status from availability alone. It must pass the frozen runtime review with repeated sensitive cases.",
        ],
    )
    doc.add_heading("Suggested TFM wording", level=2)
    doc.add_paragraph("The evaluation demonstrates that the stack preserves its principal ROS4HRI and execution contracts across changing language-model backends, while model-generated route selection and multi-step planning remain variable. The qualified v34 result therefore characterizes the stack under a named primary model and frozen runtime tuple. Subsequent model runs are treated as controlled ablations or operational fallbacks until they complete the same review.")

    doc.add_page_break()
    doc.add_heading("Appendix: run inventory", level=1)
    doc.add_paragraph("The full run inventory, per-case dataset, and raw JSON artifacts are included in the ZIP. The compact inventory below records the number of source files and cases used in this report.")
    _table(
        doc,
        ["Run group", "Source files", "Case records", "Main purpose"],
        [
            ["current_v34", "15", "72", "Frozen primary qualification"],
            ["ablations", "13", "26", "Application sampling, token, and timeout ablations"],
            ["historical_vllm", "3", "6", "Qwen3.5 vLLM reserve evidence"],
            ["historical_ollama", "3", "3", "Ollama reserve evidence"],
            ["current_ollama_switch", "1", "1", "Live fallback preflight and smoke"],
        ],
        [1.4, 1.1, 1.1, 3.2],
    )
    doc.add_paragraph("Source paths in the evidence pack are the provenance keys. The report intentionally avoids claiming a universal model ranking from heterogeneous historical runs. The next accepted result should add a new frozen campaign rather than overwrite these artifacts.")

    output.parent.mkdir(parents=True, exist_ok=True)
    doc.save(str(output))
    _replace_template_header(output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    build(args.template, args.output, args.summary)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
