#!/usr/bin/env python3
"""Render the editable academic CV as a compact, reference-matched A4 PDF."""

from __future__ import annotations

import html
import re
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    HRFlowable,
    PageBreak,
    PageTemplate,
    Paragraph,
    Spacer,
)


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[2]
SOURCE = HERE / "Juan_Bendek_Academic_CV.md"
OUTPUT = REPO_ROOT / "output" / "pdf" / "Juan_Bendek_Academic_CV.pdf"

INK = colors.HexColor("#111111")
LINK = colors.HexColor("#51245F")
MUTED = colors.HexColor("#555555")


def inline_markup(text: str) -> str:
    """Convert the small Markdown subset used by the CV into ReportLab markup."""
    placeholders: dict[str, str] = {}

    def preserve_link(match: re.Match[str]) -> str:
        key = f"@@LINK{len(placeholders)}@@"
        label = html.escape(match.group(1), quote=False)
        href = html.escape(match.group(2), quote=True)
        placeholders[key] = f'<link href="{href}" color="#{LINK.hexval()[2:]}"><u>{label}</u></link>'
        return key

    working = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", preserve_link, text.strip())
    working = html.escape(working, quote=False)
    working = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", working)
    working = re.sub(r"`(.+?)`", r"<font name='Courier'>\1</font>", working)
    working = re.sub(r"\*(.+?)\*", r"<i>\1</i>", working)
    for key, replacement in placeholders.items():
        working = working.replace(key, replacement)
    return working


def parse_blocks() -> list[tuple[str, str]]:
    blocks: list[tuple[str, str]] = []
    for raw in SOURCE.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line:
            continue
        if line == "<!-- pagebreak -->":
            blocks.append(("pagebreak", ""))
        elif line.startswith("# "):
            blocks.append(("name", line[2:]))
        elif line.startswith("## "):
            blocks.append(("section", line[3:]))
        elif line.startswith("### "):
            blocks.append(("entry", line[4:]))
        elif line.startswith("- "):
            blocks.append(("bullet", line[2:]))
        elif line.startswith("**Supervisor:**"):
            blocks.append(("supervisor", line))
        elif line.startswith("*") and line.endswith("*"):
            blocks.append(("meta", line[1:-1]))
        elif line.startswith("[") and "](" in line:
            blocks.append(("project_link", line))
        else:
            blocks.append(("contact" if not blocks or blocks[-1][0] == "name" else "body", line))
    return blocks


def styles() -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "name": ParagraphStyle(
            "AcademicCVName",
            parent=base["Normal"],
            fontName="Times-Bold",
            fontSize=14.0,
            leading=16.0,
            alignment=TA_CENTER,
            textColor=INK,
            spaceAfter=2.2 * mm,
        ),
        "contact": ParagraphStyle(
            "AcademicCVContact",
            parent=base["Normal"],
            fontName="Times-Roman",
            fontSize=8.2,
            leading=9.6,
            alignment=TA_CENTER,
            textColor=INK,
            spaceAfter=1.8 * mm,
        ),
        "section": ParagraphStyle(
            "AcademicCVSection",
            parent=base["Normal"],
            fontName="Times-Bold",
            fontSize=9.25,
            leading=10.8,
            alignment=TA_LEFT,
            textColor=INK,
            spaceBefore=1.5 * mm,
            spaceAfter=0.4 * mm,
        ),
        "entry": ParagraphStyle(
            "AcademicCVEntry",
            parent=base["Normal"],
            fontName="Times-Bold",
            fontSize=8.75,
            leading=10.1,
            alignment=TA_LEFT,
            textColor=INK,
            spaceBefore=1.0 * mm,
            spaceAfter=0.2 * mm,
            keepWithNext=True,
        ),
        "meta": ParagraphStyle(
            "AcademicCVMeta",
            parent=base["Normal"],
            fontName="Times-Italic",
            fontSize=8.15,
            leading=9.0,
            alignment=TA_LEFT,
            textColor=INK,
            spaceAfter=0.15 * mm,
            keepWithNext=True,
        ),
        "supervisor": ParagraphStyle(
            "AcademicCVSupervisor",
            parent=base["Normal"],
            fontName="Times-Roman",
            fontSize=8.15,
            leading=9.0,
            alignment=TA_LEFT,
            textColor=INK,
            spaceAfter=0.15 * mm,
            keepWithNext=True,
        ),
        "project_link": ParagraphStyle(
            "AcademicCVProjectLink",
            parent=base["Normal"],
            fontName="Times-Roman",
            fontSize=7.8,
            leading=8.9,
            alignment=TA_LEFT,
            textColor=LINK,
            spaceAfter=0.35 * mm,
            keepWithNext=True,
        ),
        "body": ParagraphStyle(
            "AcademicCVBody",
            parent=base["Normal"],
            fontName="Times-Roman",
            fontSize=8.15,
            leading=9.35,
            alignment=TA_LEFT,
            textColor=INK,
            spaceAfter=0.7 * mm,
        ),
        "bullet": ParagraphStyle(
            "AcademicCVBullet",
            parent=base["Normal"],
            fontName="Times-Roman",
            fontSize=8.05,
            leading=9.0,
            alignment=TA_LEFT,
            textColor=INK,
            leftIndent=6.0 * mm,
            firstLineIndent=-3.3 * mm,
            rightIndent=0,
            bulletIndent=0,
            spaceBefore=0,
            spaceAfter=0.30 * mm,
        ),
    }


def footer(canvas, doc) -> None:
    canvas.saveState()
    width, _ = A4
    canvas.setFillColor(MUTED)
    canvas.setFont("Times-Roman", 7)
    canvas.drawString(15.5 * mm, 7.5 * mm, "Juan David Bendek Williamson")
    canvas.drawRightString(width - 15.5 * mm, 7.5 * mm, f"Academic CV | {doc.page}")
    canvas.restoreState()


def render() -> Path:
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    doc = BaseDocTemplate(
        str(OUTPUT),
        pagesize=A4,
        leftMargin=15.5 * mm,
        rightMargin=15.5 * mm,
        topMargin=10.5 * mm,
        bottomMargin=12.5 * mm,
        title="Academic Curriculum Vitae - Juan David Bendek Williamson",
        author="Juan David Bendek Williamson",
        subject="Academic curriculum vitae and research experience",
    )
    frame = Frame(
        doc.leftMargin,
        doc.bottomMargin,
        doc.width,
        doc.height,
        leftPadding=0,
        rightPadding=0,
        topPadding=0,
        bottomPadding=0,
    )
    doc.addPageTemplates([PageTemplate(id="academic_cv", frames=[frame], onPage=footer)])

    cv_styles = styles()
    story = []
    for kind, text in parse_blocks():
        if kind == "pagebreak":
            story.append(PageBreak())
        elif kind == "name":
            story.append(Paragraph(inline_markup(text), cv_styles["name"]))
        elif kind == "contact":
            story.append(Paragraph(inline_markup(text), cv_styles["contact"]))
            story.append(HRFlowable(width="100%", thickness=0.55, color=INK, spaceAfter=3.0 * mm))
        elif kind == "section":
            story.append(Paragraph(inline_markup(text), cv_styles["section"]))
            story.append(HRFlowable(width="100%", thickness=0.55, color=INK, spaceAfter=0.8 * mm))
        elif kind == "bullet":
            story.append(Paragraph(inline_markup(text), cv_styles["bullet"], bulletText="•"))
        else:
            story.append(Paragraph(inline_markup(text), cv_styles[kind]))

    story.append(Spacer(1, 1 * mm))
    doc.build(story)
    return OUTPUT


if __name__ == "__main__":
    print(render())
