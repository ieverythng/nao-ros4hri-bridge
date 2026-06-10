#!/usr/bin/env python3
"""Generate thesis-facing architecture artifacts in a consistent report style."""

from __future__ import annotations

from dataclasses import dataclass
from html import escape
from pathlib import Path
import re
import textwrap

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import cm, mm
from reportlab.platypus import (
    PageBreak,
    Paragraph,
    Preformatted,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "thesis"

REPORT_CSS = """
@page {
  size: A4;
  margin: 2.6cm 2.2cm 2.4cm 2.2cm;
}
:root {
  --ink: #111;
  --muted: #3a3a3a;
  --line: #1f1f1f;
  --soft: #f3f3f3;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  color: var(--ink);
  background: #fff;
  font-family: "Liberation Serif", "Times New Roman", Times, serif;
  font-size: 11.5pt;
  line-height: 1.42;
}
.doc {
  max-width: 172mm;
  margin: 0 auto;
  padding: 0;
}
.title-page {
  min-height: 247mm;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  page-break-after: always;
}
.title-top {
  text-align: center;
  margin-top: 18mm;
}
.title-top h1 {
  font-size: 20pt;
  line-height: 1.28;
  margin: 20mm 0 8mm;
  font-weight: 700;
}
.title-top h2 {
  font-size: 13pt;
  margin: 0 0 3mm;
  font-weight: 600;
}
.title-top p {
  margin: 2mm 0;
  color: var(--muted);
}
.title-meta {
  width: 100%;
  border-collapse: collapse;
  margin: 0 auto 20mm;
  font-size: 11pt;
}
.title-meta td {
  padding: 2.5mm 0;
  vertical-align: top;
  border: 0;
}
.title-meta td:first-child {
  width: 38mm;
  font-weight: 700;
}
h1, h2, h3 {
  margin: 0 0 3mm;
  line-height: 1.28;
  font-weight: 700;
}
h1 { font-size: 16.5pt; margin-top: 0; }
h2 {
  font-size: 13pt;
  margin-top: 9mm;
  border-bottom: 0.5pt solid var(--line);
  padding-bottom: 1.5mm;
}
h3 {
  font-size: 11.8pt;
  margin-top: 6mm;
}
p { margin: 2.2mm 0; }
ul, ol { margin: 2.5mm 0 2.5mm 6mm; padding: 0; }
li { margin: 1.4mm 0; }
table {
  width: 100%;
  border-collapse: collapse;
  margin: 3mm 0 4mm;
  font-size: 10.8pt;
}
th, td {
  border: 0.6pt solid #333;
  padding: 2.2mm 2.3mm;
  vertical-align: top;
}
th {
  background: #fafafa;
  text-align: left;
  font-weight: 700;
}
.note {
  border: 0.6pt solid #333;
  background: #fcfcfc;
  padding: 2.6mm 3mm;
  margin: 3mm 0;
  font-size: 10.8pt;
}
code.inline, code {
  font-family: "Liberation Mono", "Courier New", monospace;
  background: #efefef;
  padding: 0.2mm 1mm;
  border: 0.4pt solid #d7d7d7;
  border-radius: 2px;
  font-size: 10.3pt;
}
pre {
  margin: 2.5mm 0 4mm;
  background: var(--soft);
  border: 0.6pt solid #666;
  padding: 2.6mm 2.8mm;
  overflow: auto;
  font-family: "Liberation Mono", "Courier New", monospace;
  font-size: 9.2pt;
  line-height: 1.3;
  white-space: pre-wrap;
}
.diagram {
  margin: 4mm 0 5mm;
  border: 0.6pt solid #777;
  padding: 2mm;
  background: #fff;
}
.diagram svg {
  width: 100%;
  height: auto;
  display: block;
}
.caption {
  margin-top: 2mm;
  font-size: 10pt;
  color: #222;
  font-style: italic;
  text-align: center;
}
.small { font-size: 10.3pt; color: #222; }
"""


@dataclass(frozen=True)
class Artifact:
    slug: str
    title: str
    subtitle: str
    scope: str
    body: str


ARCH_DIAGRAM = """<div class="diagram">
  <svg viewBox="0 0 1400 570" role="img" aria-label="Layered planner architecture">
    <defs>
      <marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="#111"></path>
      </marker>
    </defs>
    <rect x="40" y="34" width="1280" height="78" fill="#f7f7f7" stroke="#222"></rect>
    <text x="65" y="80" font-size="28" font-family="Liberation Serif, Times New Roman, serif">Human interaction layer: dialogue_manager + chatbot_llm</text>
    <rect x="40" y="146" width="1280" height="90" fill="#fff" stroke="#222"></rect>
    <text x="65" y="190" font-size="26" font-family="Liberation Serif, Times New Roman, serif">Semantic planning layer: planner_llm + planner_common contracts</text>
    <text x="65" y="218" font-size="18" font-family="Liberation Serif, Times New Roman, serif">Goal supervision, plan generation, replanning, and planner dialogue acts.</text>
    <rect x="40" y="270" width="1280" height="90" fill="#f7f7f7" stroke="#222"></rect>
    <text x="65" y="314" font-size="26" font-family="Liberation Serif, Times New Roman, serif">Deterministic execution layer: nao_orchestrator + skill registry projection</text>
    <text x="65" y="342" font-size="18" font-family="Liberation Serif, Times New Roman, serif">Admission, validation, action dispatch, and execution feedback.</text>
    <rect x="40" y="394" width="600" height="86" fill="#fff" stroke="#222"></rect>
    <text x="65" y="438" font-size="24" font-family="Liberation Serif, Times New Roman, serif">Grounding layer: KB + scene grounding</text>
    <text x="65" y="466" font-size="17" font-family="Liberation Serif, Times New Roman, serif">Knowledge snapshots, scene summaries, and state_t0.</text>
    <rect x="720" y="394" width="600" height="86" fill="#fff" stroke="#222"></rect>
    <text x="745" y="438" font-size="24" font-family="Liberation Serif, Times New Roman, serif">Robot adapter layer: AB=1 skills</text>
    <text x="745" y="466" font-size="17" font-family="Liberation Serif, Times New Roman, serif">say, look_at, scan, report_result, and head motion.</text>
    <g stroke="#111" stroke-width="2" marker-end="url(#arr)">
      <line x1="700" y1="112" x2="700" y2="146"></line>
      <line x1="700" y1="236" x2="700" y2="270"></line>
      <line x1="470" y1="360" x2="470" y2="394"></line>
      <line x1="1010" y1="360" x2="1010" y2="394"></line>
      <line x1="720" y1="437" x2="640" y2="437"></line>
    </g>
    <text x="64" y="530" font-size="17" font-family="Liberation Serif, Times New Roman, serif">Invariant: the LLM proposes structured intent and plans; deterministic ROS nodes own validation, dispatch, and evidence.</text>
  </svg>
  <div class="caption">Figure 1. Layered decomposition of the active NAO ROS4HRI planner stack.</div>
</div>"""


FLOW_DIAGRAM = """<div class="diagram">
  <svg viewBox="0 0 1500 430" role="img" aria-label="Runtime sequence">
    <defs>
      <marker id="arr2" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="#111"></path>
      </marker>
    </defs>
    <g font-family="Liberation Serif, Times New Roman, serif" font-size="15">
      <text x="45" y="35">User</text><text x="190" y="35">dialogue_manager</text><text x="390" y="35">chatbot_llm</text>
      <text x="575" y="35">nao_orchestrator</text><text x="820" y="35">planner_llm</text><text x="1030" y="35">Skills</text><text x="1210" y="35">KB / Scene</text>
    </g>
    <g stroke="#aaa" stroke-dasharray="4 4">
      <line x1="70" y1="45" x2="70" y2="365"></line><line x1="260" y1="45" x2="260" y2="365"></line>
      <line x1="450" y1="45" x2="450" y2="365"></line><line x1="660" y1="45" x2="660" y2="365"></line>
      <line x1="875" y1="45" x2="875" y2="365"></line><line x1="1060" y1="45" x2="1060" y2="365"></line><line x1="1260" y1="45" x2="1260" y2="365"></line>
    </g>
    <g stroke="#111" stroke-width="1.5" fill="none" marker-end="url(#arr2)">
      <line x1="70" y1="75" x2="260" y2="75"></line><line x1="260" y1="105" x2="450" y2="105"></line>
      <line x1="450" y1="135" x2="660" y2="135"></line><line x1="660" y1="165" x2="875" y2="165"></line>
      <line x1="875" y1="195" x2="660" y2="195"></line><line x1="660" y1="225" x2="1060" y2="225"></line>
      <line x1="1060" y1="255" x2="1260" y2="255"></line><line x1="1060" y1="285" x2="660" y2="285"></line>
      <line x1="660" y1="315" x2="875" y2="315"></line><line x1="875" y1="345" x2="260" y2="345"></line>
    </g>
    <g font-family="Liberation Serif, Times New Roman, serif" font-size="13">
      <text x="112" y="68">utterance</text><text x="300" y="98">turn request</text><text x="488" y="128">planner_request</text>
      <text x="700" y="158">admitted request</text><text x="735" y="188">plan JSON</text><text x="790" y="218">dispatch</text>
      <text x="1105" y="248">evidence lookup</text><text x="790" y="278">skill result</text><text x="706" y="308">execution_feedback</text>
      <text x="565" y="338">dialogue_act / completion</text>
    </g>
  </svg>
  <div class="caption">Figure 2. Main runtime sequence from natural-language request to execution feedback.</div>
</div>"""


def inline_html(text: str) -> str:
    value = escape(text)
    value = re.sub(r"`([^`]+)`", r"<code>\1</code>", value)
    value = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", value)
    return value


def markdown_to_html(md: str) -> str:
    html: list[str] = []
    in_pre = False
    in_raw = False
    pre_lines: list[str] = []
    raw_lines: list[str] = []
    list_type: str | None = None
    table_rows: list[str] = []

    def close_list() -> None:
        nonlocal list_type
        if list_type:
            html.append(f"</{list_type}>")
            list_type = None

    def flush_table() -> None:
        nonlocal table_rows
        if not table_rows:
            return
        html.append("<table>")
        for index, row in enumerate(table_rows):
            cells = [cell.strip() for cell in row.strip("|").split("|")]
            if index == 1 and all(set(cell.replace(":", "").replace("-", "")) == set() for cell in cells):
                continue
            tag = "th" if index == 0 else "td"
            html.append("<tr>" + "".join(f"<{tag}>{inline_html(cell)}</{tag}>" for cell in cells) + "</tr>")
        html.append("</table>")
        table_rows = []

    for raw_line in md.splitlines():
        line = raw_line.rstrip()
        if in_raw:
            raw_lines.append(line)
            if line == "</div>":
                html.append("\n".join(raw_lines))
                raw_lines = []
                in_raw = False
            continue
        if line.startswith("```"):
            if in_pre:
                html.append("<pre>" + escape("\n".join(pre_lines)) + "</pre>")
                pre_lines = []
                in_pre = False
            else:
                close_list()
                flush_table()
                in_pre = True
            continue
        if in_pre:
            pre_lines.append(line)
            continue
        if line.startswith('<div class="diagram"'):
            close_list()
            flush_table()
            raw_lines = [line]
            in_raw = True
            continue
        if not line.strip():
            close_list()
            flush_table()
            continue
        if line.startswith("|") and line.endswith("|"):
            close_list()
            table_rows.append(line)
            continue
        flush_table()
        if line.startswith("# "):
            close_list()
            html.append(f"<h1>{inline_html(line[2:])}</h1>")
        elif line.startswith("## "):
            close_list()
            html.append(f"<h2>{inline_html(line[3:])}</h2>")
        elif line.startswith("### "):
            close_list()
            html.append(f"<h3>{inline_html(line[4:])}</h3>")
        elif line.startswith("- "):
            if list_type != "ul":
                close_list()
                list_type = "ul"
                html.append("<ul>")
            html.append(f"<li>{inline_html(line[2:])}</li>")
        elif re.match(r"^\d+\. ", line):
            if list_type != "ol":
                close_list()
                list_type = "ol"
                html.append("<ol>")
            item = re.sub(r"^\d+\. ", "", line)
            html.append(f"<li>{inline_html(item)}</li>")
        elif line.startswith("> "):
            close_list()
            html.append(f'<div class="note">{inline_html(line[2:])}</div>')
        else:
            close_list()
            html.append(f"<p>{inline_html(line)}</p>")
    close_list()
    flush_table()
    return "\n".join(html)


def markdown_blocks(md: str) -> list[tuple[str, object]]:
    blocks: list[tuple[str, object]] = []
    in_pre = False
    in_raw = False
    pre_lines: list[str] = []
    table_rows: list[str] = []

    def flush_table() -> None:
        nonlocal table_rows
        if table_rows:
            rows = [[cell.strip() for cell in row.strip("|").split("|")] for row in table_rows]
            if len(rows) > 1 and all(set(cell.replace(":", "").replace("-", "")) == set() for cell in rows[1]):
                rows.pop(1)
            blocks.append(("table", rows))
            table_rows = []

    for raw_line in md.splitlines():
        line = raw_line.rstrip()
        if in_raw:
            if line == "</div>":
                in_raw = False
            continue
        if line.startswith("```"):
            if in_pre:
                blocks.append(("pre", "\n".join(pre_lines)))
                pre_lines = []
                in_pre = False
            else:
                flush_table()
                in_pre = True
            continue
        if in_pre:
            pre_lines.append(line)
            continue
        if line.startswith("<div"):
            flush_table()
            in_raw = True
            continue
        if not line.strip():
            flush_table()
            continue
        if line.startswith("|") and line.endswith("|"):
            table_rows.append(line)
            continue
        flush_table()
        if line.startswith("# "):
            blocks.append(("h1", line[2:]))
        elif line.startswith("## "):
            blocks.append(("h2", line[3:]))
        elif line.startswith("### "):
            blocks.append(("h3", line[4:]))
        elif line.startswith("- "):
            blocks.append(("li", line[2:]))
        elif re.match(r"^\d+\. ", line):
            blocks.append(("li", re.sub(r"^\d+\. ", "", line)))
        elif line.startswith("> "):
            blocks.append(("note", line[2:]))
        else:
            blocks.append(("p", line))
    flush_table()
    return blocks


def strip_md(text: str) -> str:
    return re.sub(r"`([^`]+)`", r"\1", re.sub(r"\*\*([^*]+)\*\*", r"\1", text))


def write_pdf(path: Path, artifact: Artifact, md: str) -> None:
    styles = getSampleStyleSheet()
    body = ParagraphStyle("Body", parent=styles["BodyText"], fontName="Times-Roman", fontSize=10.7, leading=15, spaceAfter=6)
    h1 = ParagraphStyle("H1", parent=styles["Heading1"], fontName="Times-Bold", fontSize=16, leading=20, spaceBefore=2, spaceAfter=9)
    h2 = ParagraphStyle("H2", parent=styles["Heading2"], fontName="Times-Bold", fontSize=13, leading=16, spaceBefore=14, spaceAfter=7)
    h3 = ParagraphStyle("H3", parent=styles["Heading3"], fontName="Times-Bold", fontSize=11.5, leading=14, spaceBefore=9, spaceAfter=5)
    note = ParagraphStyle("Note", parent=body, backColor=colors.whitesmoke, borderColor=colors.black, borderWidth=0.5, borderPadding=6)
    code = ParagraphStyle("Code", parent=styles["Code"], fontName="Courier", fontSize=8.3, leading=10)
    title = ParagraphStyle("Title", parent=styles["Title"], fontName="Times-Bold", fontSize=20, leading=25, alignment=TA_CENTER)
    centered = ParagraphStyle("Centered", parent=body, alignment=TA_CENTER)
    small = ParagraphStyle("Small", parent=body, fontSize=9.5, leading=12)

    doc = SimpleDocTemplate(
        str(path),
        pagesize=A4,
        rightMargin=2.2 * cm,
        leftMargin=2.2 * cm,
        topMargin=2.6 * cm,
        bottomMargin=2.4 * cm,
    )
    story = [
        Spacer(1, 18 * mm),
        Paragraph("Universitat Autonoma de Barcelona", centered),
        Paragraph("Master Degree in Modelling for Science and Engineering", centered),
        Paragraph("Institution / Lab: IIIA-CSIC", centered),
        Spacer(1, 30 * mm),
        Paragraph(escape(artifact.title), title),
        Paragraph(escape(artifact.subtitle), centered),
        Spacer(1, 55 * mm),
        Table(
            [
                ["Document type:", "Thesis architecture and implementation reference"],
                ["Academic year:", "2025-2026"],
                ["Date:", "2026-06-03"],
                ["Scope:", artifact.scope],
                ["Repository:", "https://github.com/ieverythng/nao-ros4hri-bridge"],
            ],
            colWidths=[38 * mm, 110 * mm],
            style=TableStyle([("FONTNAME", (0, 0), (0, -1), "Times-Bold"), ("FONTNAME", (1, 0), (1, -1), "Times-Roman"), ("FONTSIZE", (0, 0), (-1, -1), 10.5), ("VALIGN", (0, 0), (-1, -1), "TOP"), ("BOTTOMPADDING", (0, 0), (-1, -1), 7)]),
        ),
        PageBreak(),
    ]

    for kind, value in markdown_blocks(md):
        if kind == "h1":
            story.append(Paragraph(escape(str(value)), h1))
        elif kind == "h2":
            story.append(Paragraph(escape(str(value)), h2))
        elif kind == "h3":
            story.append(Paragraph(escape(str(value)), h3))
        elif kind == "p":
            story.append(Paragraph(escape(strip_md(str(value))), body))
        elif kind == "li":
            story.append(Paragraph("• " + escape(strip_md(str(value))), body))
        elif kind == "note":
            story.append(Paragraph(escape(strip_md(str(value))), note))
        elif kind == "pre":
            story.append(Preformatted(str(value), code, maxLineLength=92))
            story.append(Spacer(1, 4))
        elif kind == "table":
            rows = value  # type: ignore[assignment]
            table = Table(rows, repeatRows=1)
            table.setStyle(
                TableStyle(
                    [
                        ("GRID", (0, 0), (-1, -1), 0.45, colors.black),
                        ("BACKGROUND", (0, 0), (-1, 0), colors.whitesmoke),
                        ("FONTNAME", (0, 0), (-1, 0), "Times-Bold"),
                        ("FONTNAME", (0, 1), (-1, -1), "Times-Roman"),
                        ("FONTSIZE", (0, 0), (-1, -1), 8.6),
                        ("VALIGN", (0, 0), (-1, -1), "TOP"),
                        ("LEFTPADDING", (0, 0), (-1, -1), 5),
                        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
                    ]
                )
            )
            story.append(table)
            story.append(Spacer(1, 7))
        if kind not in {"table"}:
            story.append(Spacer(1, 2))
    doc.build(story)


def write_artifact(artifact: Artifact) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    md = (
        f"# {artifact.title}\n\n"
        "Date: 2026-06-03\n"
        "Audience: TFM writing, supervisor review, implementation handoff\n"
        f"Scope: {artifact.scope}\n\n"
        "---\n\n"
        f"{artifact.body.strip()}\n"
    )
    rendered_md = f"# {artifact.title}\n\n{artifact.body.strip()}\n"
    md_path = OUT / f"{artifact.slug}.md"
    html_path = OUT / f"{artifact.slug}.html"
    pdf_path = OUT / f"{artifact.slug}.pdf"
    md_path.write_text(md, encoding="utf-8")
    html_path.write_text(
        textwrap.dedent(
            f"""\
            <!doctype html>
            <html lang="en">
            <head>
              <meta charset="utf-8" />
              <meta name="viewport" content="width=device-width, initial-scale=1" />
              <title>{escape(artifact.title)}</title>
              <style>{REPORT_CSS}</style>
            </head>
            <body>
              <main class="doc">
                <section class="title-page">
                  <div class="title-top">
                    <h2>Universitat Autonoma de Barcelona</h2>
                    <p>Master Degree in Modelling for Science and Engineering</p>
                    <p>Institution / Lab: IIIA-CSIC</p>
                    <h1>{escape(artifact.title)}<br/>{escape(artifact.subtitle)}</h1>
                    <p>Technical thesis artifact generated from the active NAO ROS4HRI planner repository.</p>
                  </div>
                  <table class="title-meta">
                    <tr><td>Document type:</td><td>Thesis architecture and implementation reference</td></tr>
                    <tr><td>Academic year:</td><td>2025-2026</td></tr>
                    <tr><td>Date:</td><td>2026-06-03</td></tr>
                    <tr><td>Scope:</td><td>{escape(artifact.scope)}</td></tr>
                    <tr><td>Repository:</td><td><span class="small">https://github.com/ieverythng/nao-ros4hri-bridge</span></td></tr>
                  </table>
                </section>
                {markdown_to_html(rendered_md)}
              </main>
            </body>
            </html>
            """
        ),
        encoding="utf-8",
    )
    write_pdf(pdf_path, artifact, rendered_md)


ARCH_BODY = f"""
## Purpose and Thesis Positioning

This document turns the current implementation into thesis-ready prose. It supports the architecture, implementation, and discussion chapters of the TFM draft extracted from the Overleaf structure. The system under study is a modular ROS 2 and ROS4HRI stack for the NAO robot in which natural language, symbolic grounding, LLM planning, deterministic execution, and user-facing dialogue are deliberately separated.

The core claim is that LLM-based planning becomes more inspectable and experimentally useful when the language model is not treated as a direct robot controller. Instead, the LLM is placed inside a typed planning layer. It receives bounded context, emits structured JSON, and depends on deterministic ROS nodes for admission, validation, execution, and feedback.

{ARCH_DIAGRAM}

## Architectural Thesis

The architecture follows a layered design. The dialogue layer is responsible for receiving user turns and producing semantic intent frames. The planning layer turns those intent frames into structured plans over an abstract skill registry. The orchestrator validates the plan and dispatches only supported actions. Skill servers interact with robot adapters, scene sources, and the knowledge base, then publish typed feedback that the planner can use for continuation, clarification, or replanning.

This split gives a concrete answer to the problem statement: an LLM planner can be integrated into a ROS4HRI robotic system by making it a supervised, contract-bound component rather than an unconstrained action generator. The planner decides what should happen at the symbolic level, while ROS components decide whether and how it may happen.

## Runtime Ownership Model

| Layer | Main packages | Responsibility | Thesis role |
|---|---|---|---|
| Dialogue | `dialogue_manager`, `chatbot_llm` | Dialogue lifecycle, turn handling, intent declaration, user-facing language | Shows that conversational behavior is separated from robot execution |
| Planning | `planner_llm`, `planner_common` | Plan generation, supervisor state, request/output/feedback contracts | Shows how LLM output is made structured and auditable |
| Execution | `nao_orchestrator`, AB=1 skills | Request admission, plan validation, action dispatch, feedback | Shows deterministic control around the LLM |
| Grounding | `nao_scene_grounding`, `kb_skills`, KnowledgeCore | Detector-to-KB grounding and prompt-facing state | Shows how the planner receives bounded symbolic context |
| Robot adapters | `nao_say_skill`, `nao_look_at`, motion/replay skills | Robot-specific execution details | Shows portability through abstract capability contracts |

## End-to-End Runtime Flow

{FLOW_DIAGRAM}

The runtime flow begins with a human utterance. The dialogue manager forwards the conversational turn to `chatbot_llm`, which classifies the turn as dialogue, knowledge query, or execution. Execution-eligible turns are converted into planner request JSON and published through the orchestrator planner gate. The planner produces a plan, the orchestrator dispatches each step, and feedback is returned as structured JSON. User-facing status is emitted through planner dialogue acts and handled by the dialogue/speech owner.

## Node and Package Responsibilities

### `chatbot_llm`

`chatbot_llm` is the user-facing LLM backend. Its role is not to generate robot plans directly. It interprets the user turn, creates a normalized intent frame, injects knowledge and scene context when available, and publishes planner requests when the turn requires execution. It also remains the appropriate owner for user-facing wording when planner dialogue completion is routed through chatbot-owned language generation.

### `planner_llm`

`planner_llm` is the high-level supervisor. It consumes admitted planner requests, reasons over the skill registry, and emits plan JSON. It tracks goal continuity through `goal_id` and plan lineage through `plan_id` and `plan_version`. It receives execution feedback and may continue, replan, ask for clarification, explain a failure, or emit completion dialogue.

### `planner_common`

`planner_common` is the contract source of truth. It normalizes planner requests, plan steps, execution feedback, scene summaries, and dialogue acts. This package is critical for thesis reproducibility because it makes the JSON interfaces explicit rather than leaving them implicit in prompt text.

### `nao_orchestrator`

`nao_orchestrator` is the deterministic execution boundary. It admits planner requests, validates plan steps, dispatches actions, and publishes feedback. It is intentionally not an LLM policy node. This is the main safety and reproducibility boundary: planner outputs become executable only after deterministic validation.

### `nao_scene_grounding` and `kb_skills`

`nao_scene_grounding` converts detector output into symbolic facts and scene summaries. `kb_skills` keeps KnowledgeCore access behind a package boundary. Together, they provide the grounding path that prevents the planner from relying only on language priors.

### AB=1 skills and robot adapters

Skills such as `say`, `look_at`, `scan`, `report_result`, and head-motion actions form the execution vocabulary. They are represented to the planner as abstract capabilities but implemented through ROS actions and robot-specific adapters. This is the thesis bridge between symbolic planning and embodied robot behavior.

## Architectural Invariants

- The planner emits structured plan JSON, not direct robot API calls.
- The orchestrator validates and dispatches; it does not generate LLM policy.
- The dialogue stack owns user-facing speech and conversational lifecycle.
- Scene and KB state are passed as bounded JSON context, not as raw detector or KB transport dumps.
- Execution feedback is the mechanism that connects embodied outcomes back to the planner.
- People and objects are represented separately in grounded context to avoid treating humans as generic objects.

## Relation to ROS4HRI and SocialMinds Principles

The system follows ROS4HRI-style modularity by keeping public behavior in ROS messages, services, and actions. Continuous observations such as scene state are topic-like, short request-response operations remain service-like, and long-running robot behaviors are action-oriented. The architecture also preserves upstream-sensitive package boundaries: dialogue remains in dialogue packages, grounding remains in scene/KB packages, and robot execution remains in skill/adaptor packages.

For the thesis, this matters because the contribution is not simply that an LLM can produce a plausible task list. The contribution is that the LLM planner is embedded in a robotics architecture where responsibilities are inspectable and replaceable.

## Chapter Integration Guidance

- Chapter 1: motivate the need for modular, grounded, feedback-aware LLM planning.
- Chapter 3: convert the invariants into design principles and non-functional requirements.
- Chapter 4: use the layered diagram and package responsibility table as the system architecture section.
- Chapter 6: expand the node descriptions into implementation subsections.
- Chapter 9: discuss why ownership boundaries reduce brittleness and improve observability.
"""


CONTRACT_BODY = r"""
## Purpose

This document is the thesis-facing contract reference for the planner stack. It records the JSON shapes that connect dialogue, grounding, planning, execution, feedback, and planner dialogue. The goal is to make the implementation auditable and to give the thesis a precise vocabulary for discussing runtime behavior.

## Contract Design Principles

- Payloads should be concise enough to inspect during a live run.
- Fields should be owned by the node that can keep them truthful.
- Planner requests can contain T0 evidence; planner outputs should remain plan-centric.
- Duplicate fields should be removed unless they represent different ownership scopes.
- Human detections must be represented as people, not as generic objects.
- Natural-language summaries may help the LLM, but they should be bounded and secondary to structured JSON.

## Planner Request Envelope

The planner request travels as `hri_actions_msgs/msg/Intent`. The ROS envelope carries transport metadata, while `Intent.data` carries the planner request JSON.

```json
{
  "intent": "planner_request",
  "source": "user_123",
  "modality": "speech",
  "confidence": 0.82,
  "priority": 128,
  "data": {
    "request_id": "turn_123",
    "goal_id": "goal_turn_123",
    "request_kind": "new_goal",
    "goal_text": "look at the person and report completion",
    "normalized_intents": ["look_at"],
    "scene_targets": ["person"],
    "dialogue_context": [],
    "requested_plan": [],
    "grounded_context": {
      "knowledge_snapshot": {},
      "scene_summary": {},
      "state_t0": {}
    },
    "planner_mode": "default",
    "interaction_mode": "speech",
    "dialogue_turn_id": "role:turn"
  }
}
```

| Field | Meaning | Owner |
|---|---|---|
| `request_id` | Unique turn/request identifier | `chatbot_llm` |
| `goal_id` | Logical goal continuity identifier | `chatbot_llm`, admitted by orchestrator |
| `request_kind` | Transition type: `new_goal`, `goal_update`, `clarification_answer`, `cancel_request` | `chatbot_llm` |
| `goal_text` | Planner-facing task objective | `chatbot_llm` |
| `normalized_intents` | Strict intent labels | `chatbot_llm` |
| `scene_targets` | Compact target labels/entities | `chatbot_llm` |
| `grounded_context` | Hybrid Minimal T0 evidence | `chatbot_llm` |

## Knowledge Snapshot

`knowledge_snapshot` is a prompt-facing compact view of symbolic KB facts. It is not a raw KnowledgeCore transport object.

```json
{
  "schema_version": "knowledge_snapshot_v2",
  "captured_at_sec": 1777040000.2,
  "references": [
    {"normalized_name": "cup", "id": "cup_1", "type": "Cup"},
    {"normalized_name": "person", "id": "person_1", "type": "Person"}
  ],
  "counts": {"entities": 2, "people": 1, "objects": 1}
}
```

The `references` array is deliberately small: it gives the planner names, stable identifiers, and types without forcing it to parse large text blocks. The `counts` field supports quick consistency checks and allows prompts to mention cardinality without repeating entity lists.

## Scene Summary

`scene_summary` carries transient detector-grounded evidence. It includes provenance, recency, image-space coordinates, and source confidence. People and objects are separated.

```json
{
  "schema_version": "scene_summary_v2",
  "observer": "myself",
  "backend": "emorobcare_cv",
  "captured_at_sec": 1777040000.2,
  "objects": [
    {
      "entity_id": "cup_1",
      "label": "cup",
      "kb_class": "Cup",
      "score": 0.92,
      "tracker_id": "",
      "source": "emorobcare_cv",
      "center_x": 321.0,
      "center_y": 238.0,
      "last_seen_sec": 1777040000.1
    }
  ],
  "people": [
    {
      "id": "person_1",
      "label": "person",
      "type": "Person",
      "source": "emorobcare_cv",
      "score": 0.81,
      "center_x": 186.0,
      "center_y": 202.0,
      "last_seen_sec": 1777040000.2
    }
  ]
}
```

This representation preserves the useful image-space information needed for future `look_at` arguments while avoiding a duplicate `look_at_candidates` field. The planner prompt can state that every `state_t0.entities[*].id` is a valid candidate for `look_at.target_frame` when the entity is visible.

## State T0

`state_t0` is the canonical planner-facing snapshot for precondition and postcondition reasoning.

```json
{
  "schema_version": "state_t0_v2",
  "observer": "myself",
  "backend": "emorobcare_cv",
  "captured_at_sec": 1777040000.2,
  "entity_counts": {"entities": 2, "people": 1, "objects": 1},
  "entities": [
    {
      "normalized_name": "cup",
      "id": "cup_1",
      "type": "Cup",
      "kind": "object",
      "source": "emorobcare_cv",
      "last_seen_sec": 1777040000.1
    },
    {
      "normalized_name": "person",
      "id": "person_1",
      "type": "Person",
      "kind": "person",
      "source": "emorobcare_cv",
      "last_seen_sec": 1777040000.2
    }
  ]
}
```

The distinction between `scene_summary` and `state_t0` is important. `scene_summary` keeps richer detector metadata. `state_t0` is a normalized reasoning set. The former is closer to perception; the latter is closer to symbolic planning.

## Planner Output

Planner output travels on `/intents` as `hri_actions_msgs/msg/Intent`. The executable part is `Intent.data.plan`.

```json
{
  "grounded_context": {
    "knowledge_snapshot": {},
    "scene_summary": {},
    "state_t0": {}
  },
  "plan": {
    "goal_id": "goal_turn_123",
    "plan_id": "plan_123",
    "plan_version": 1,
    "status": "planning",
    "validation_status": "draft",
    "failure_reason": "",
    "user_facing_reason": "",
    "replan_hint": "",
    "retry_budget": 2,
    "scene_targets": ["person"],
    "communication_policy": {
      "emit_acknowledge": false,
      "emit_progress": false,
      "emit_completion": true,
      "emit_failure": true
    },
    "steps": [
      {
        "id": "step_1",
        "type": "look_at",
        "name": "look_at",
        "args": {"target_frame": "person_1"},
        "requires": [],
        "on_failure": "replan",
        "retry_budget": 1
      }
    ]
  }
}
```

The plan is intentionally nested under `plan` to avoid duplicated top-level planning metadata. Removed legacy fields include `goal_token`, planner `ack_mode`, planner `ack_text`, `world_model_snapshot`, and `world_model_text`.

## Execution Feedback

Execution feedback is the closed-loop signal from `nao_orchestrator` to `planner_llm`.

```json
{
  "goal_id": "goal_turn_123",
  "plan_id": "plan_123",
  "plan_version": 1,
  "intent": "look_at",
  "source": "nao_orchestrator",
  "event_type": "step_failed",
  "status": "failed",
  "reason": "target frame unavailable",
  "validation_status": "draft",
  "replan_hint": "scan for people before retrying look_at",
  "retry_budget": 1,
  "blocking": true,
  "unmet_preconditions": ["target_frame_visible"],
  "needs_user_input": false,
  "scene_targets": ["person"],
  "validation_errors": [],
  "timestamp_sec": 1777040012.4,
  "result_summary": "The requested person target was not available.",
  "result_payload": {
    "target_found": false,
    "target_kind": "person"
  },
  "step": {
    "id": "step_1",
    "type": "look_at",
    "name": "look_at",
    "retry_budget": 1,
    "on_failure": "replan",
    "requires": []
  }
}
```

Feedback is not just logging. It is the planner supervisor's evidence stream. It determines whether a goal can continue, whether a step needs replanning, or whether the user should be asked for clarification.

## Planner Dialogue Act

Planner dialogue acts carry planner-owned conversational intent. They do not make the planner the speech owner.

```json
{
  "goal_id": "goal_turn_123",
  "plan_id": "plan_123",
  "plan_version": 1,
  "act": "ask_clarification",
  "priority": "normal",
  "await_user_response": true,
  "reason": "multiple candidate people visible",
  "text_hint": "Which person should I look at?",
  "slots_needed": ["target_person"],
  "context": {
    "scene_targets": ["person"],
    "status": "waiting_user"
  }
}
```

The thesis should describe this as a separation between dialogue intention and utterance realization. The planner can say that clarification is needed; the dialogue stack remains responsible for interaction lifecycle and speech delivery.

## Semantics of Goal and Plan Identity

`goal_id` represents continuity of the user's objective. `plan_id` and `plan_version` represent the lineage of a particular proposed plan. This enables replanning without pretending that a new plan is a new user goal.

| Identifier | Meaning | Example |
|---|---|---|
| `goal_id` | Stable objective identity | `goal_turn_123` |
| `plan_id` | Plan lineage identity | `plan_123` |
| `plan_version` | Revision number within lineage | `2` |
| `step.id` | Stable step join point | `step_look_person` |
"""


VALIDATION_BODY = """
## Purpose

This document converts the implementation into a thesis validation protocol. It is intended to support the methodology, results, and discussion chapters by making experiments repeatable and by naming the evidence that each run must collect.

## Validation Thesis

The system should be evaluated not only by whether the robot completes a task, but by whether the architecture preserves the intended boundaries while doing so. A successful planner-mediated run should show correct routing, valid contract generation, deterministic admission, skill dispatch, feedback publication, and a single user-facing speech authority.

## Research Questions Covered

| Research question | Observable evidence |
|---|---|
| Can planner-mediated execution improve multi-step task handling? | Plan validity, step completion, replanning events |
| Does the skill registry reduce invalid robot-specific outputs? | Invalid-step rate, unsupported-skill rejection rate |
| Does execution feedback improve recovery? | Replan success, clarification success, failure explanation quality |
| Do architecture boundaries remain inspectable? | Trace completeness, contract validity, ownership violations |
| What limitations remain under real HRI constraints? | Latency, duplicate speech, grounding staleness, model failure cases |

## Scenario Matrix

| Scenario group | Example command | Expected behavior | Primary metrics |
|---|---|---|---|
| Dialogue-only | hello | Remains dialogue, no planner request | false planner-route rate |
| Knowledge query | what can you see? | Uses KB/scene answer, no unnecessary execution | route accuracy, answer groundedness |
| Atomic execution | look left | Single validated skill step | completion, latency |
| Grounded look-at | look at the person | Uses visible person target and `look_at` step | target correctness, person/object separation |
| Multi-step task | scan for a cup and tell me what you found | Planner emits ordered steps | plan validity, step success |
| Failure recovery | target disappears before execution | Feedback triggers replan or failure dialogue | recovery rate |
| Clarification | ambiguous target | Planner asks for missing slot | clarification precision |
| Cancellation/update | stop or changed goal | Goal transition is explicit | supersede/cancel correctness |

## Runtime Evidence to Capture

- user utterance and timestamp;
- chatbot route and `user_intent` JSON;
- planner request envelope and `Intent.data`;
- grounded context snapshot (`knowledge_snapshot`, `scene_summary`, `state_t0`);
- planner output JSON;
- orchestrator validation result;
- dispatched action server and step id;
- execution feedback JSON;
- planner dialogue act, if emitted;
- final user-facing utterance and speech owner.

## Quantitative Metrics

| Metric | Definition | Interpretation |
|---|---|---|
| Route accuracy | Fraction of turns routed to the intended class | Tests chatbot planner gating |
| Plan validity | Fraction of planner outputs accepted by orchestrator | Tests planner contract discipline |
| Unsupported step rate | Fraction of steps rejected as unknown or invalid | Tests registry grounding |
| Task completion | Fraction of scenarios completed | Tests end-to-end behavior |
| Recovery success | Fraction of failures resolved by replan or clarification | Tests closed-loop supervision |
| Duplicate speech rate | Fraction of turns with multiple competing utterances | Tests dialogue ownership |
| Grounding correctness | Fraction of selected targets matching scene/KB evidence | Tests perception-to-planner context |
| Median latency | Time from user utterance to first action or answer | Tests HRI usability |

## Qualitative Analysis Dimensions

Each trace should explain why the chatbot selected the route, what grounded context was available at T0, why the planner selected the steps, whether the orchestrator accepted or rejected each step, how feedback changed planner state, whether the final user-facing response had one owner, and what the trace reveals about limitations of the architecture.

## Ablation Plan

| Ablation | Change | Hypothesis |
|---|---|---|
| Direct execution only | Disable planner-mediated path | Multi-step and failure recovery become less robust |
| No grounded context | Remove `knowledge_snapshot`, `scene_summary`, `state_t0` | Target selection and perception tasks degrade |
| No execution feedback | Suppress feedback to planner | Replanning and failure explanations degrade |
| Minimal registry | Hide some skill metadata | Unsupported or underspecified plan steps increase |
| Chatbot wording disabled | Direct planner dialogue only | Completion wording may become less conversational but easier to attribute |

## Validation Gates Before Experiments

1. Run contract unit tests for `planner_common`, `chatbot_llm`, `planner_llm`, and `nao_orchestrator`.
2. Run registry consistency checks so planner-visible skills match canonical AB entries.
3. Run ROS4HRI structural audit to confirm changed packages and ownership-sensitive files.
4. Launch the sim profile and verify expected action servers are available.
5. Execute one dialogue-only, one knowledge-query, one atomic action, and one failure case before collecting formal results.

## Experiment Log Template

| Field | Value |
|---|---|
| Experiment ID | `EXP-YYYYMMDD-###` |
| Scenario group |  |
| User utterance |  |
| Expected route |  |
| Observed route |  |
| Planner request published | yes/no |
| Planner output valid | yes/no |
| Executed steps |  |
| Feedback events |  |
| Final speech owner |  |
| Outcome | success / failure / partial |
| Notes |  |

## Threats to Validity

The main internal threats are prompt sensitivity, incomplete test coverage for live ROS timing, and possible mismatch between simulated and robot execution. External threats include model availability, cloud/local model differences, and the fact that the validation environment is centered on the NAO stack. Construct validity depends on whether metrics such as plan validity and route accuracy genuinely capture user-perceived task success. The thesis should therefore combine quantitative tables with trace-based qualitative analysis.

## Expected Results Narrative

A strong thesis result would show that planner-mediated execution does not merely add complexity. It should demonstrate that the planner path improves inspectability, makes multi-step execution more explicit, and creates recovery opportunities through feedback. Even when the planner fails, the architecture should make the failure attributable: wrong route, stale grounding, invalid plan, missing skill, execution failure, or dialogue ownership issue.
"""


ARTIFACTS = [
    Artifact(
        slug="tfm_architecture_and_implementation_reference_2026-06-03",
        title="TFM Architecture and Implementation Reference",
        subtitle="ROS4HRI LLM Planner Stack",
        scope="Architecture, package ownership, runtime flow, and thesis chapter integration.",
        body=ARCH_BODY,
    ),
    Artifact(
        slug="tfm_runtime_contracts_and_semantics_2026-06-03",
        title="TFM Runtime Contracts and Semantics Reference",
        subtitle="Planner, Grounding, Feedback, and Dialogue JSON",
        scope="Planner request/output contracts, grounding semantics, execution feedback, and dialogue acts.",
        body=CONTRACT_BODY,
    ),
    Artifact(
        slug="tfm_validation_protocol_2026-06-03",
        title="TFM Validation and Experiment Protocol",
        subtitle="Scenario Matrix, Metrics, and Trace Evidence",
        scope="Validation methodology, experiment design, metrics, ablations, and evidence capture.",
        body=VALIDATION_BODY,
    ),
]


def main() -> None:
    for artifact in ARTIFACTS:
        write_artifact(artifact)
        print(f"generated {artifact.slug}")


if __name__ == "__main__":
    main()
