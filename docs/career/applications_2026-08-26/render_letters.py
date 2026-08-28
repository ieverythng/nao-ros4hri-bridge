#!/usr/bin/env python3
"""Render the editable career application letters as compact A4 PDFs."""

from __future__ import annotations

import html
import re
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
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


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "pdf"
SOURCES = sorted(ROOT.glob("[0-9][1-4]_*.md"))

INK = colors.HexColor("#20252B")
MUTED = colors.HexColor("#5E6873")
ACCENT = colors.HexColor("#A52128")
RULE = colors.HexColor("#D9DDE1")


def inline_markup(text: str) -> str:
    escaped = html.escape(text.strip())
    escaped = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", escaped)
    escaped = re.sub(r"`(.+?)`", r"<font name='Courier'>\1</font>", escaped)
    escaped = escaped.replace("\n", "<br/>")
    return escaped


def parse_blocks(source: Path) -> list[tuple[str, str]]:
    lines = source.read_text(encoding="utf-8").splitlines()
    blocks: list[tuple[str, str]] = []
    paragraph: list[tuple[str, bool]] = []

    def flush() -> None:
        if paragraph:
            text = ""
            for part, hard_break in paragraph:
                text += part.strip()
                text += "\n" if hard_break else " "
            blocks.append(("paragraph", text.strip()))
            paragraph.clear()

    for line in lines:
        stripped = line.strip()
        if not stripped:
            flush()
            continue
        if stripped == "<!-- pagebreak -->":
            flush()
            blocks.append(("pagebreak", ""))
            continue
        if stripped.startswith("# "):
            flush()
            blocks.append(("title", stripped[2:]))
        elif stripped.startswith("## "):
            flush()
            blocks.append(("section", stripped[3:]))
        elif re.match(r"^\d+\.\s", stripped):
            flush()
            blocks.append(("reference", stripped))
        else:
            paragraph.append((stripped, line.endswith("  ")))
    flush()
    return blocks


def footer(canvas, doc) -> None:
    canvas.saveState()
    width, _ = A4
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.5)
    canvas.line(22 * mm, 14 * mm, width - 22 * mm, 14 * mm)
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(MUTED)
    canvas.drawString(22 * mm, 9.5 * mm, "Juan David Bendek Williamson")
    canvas.drawRightString(width - 22 * mm, 9.5 * mm, f"Page {doc.page}")
    canvas.restoreState()


def styles(*, compact: bool = False) -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "title": ParagraphStyle(
            "LetterTitle",
            parent=base["Title"],
            fontName="Helvetica-Bold",
            fontSize=15,
            leading=18,
            textColor=INK,
            alignment=TA_LEFT,
            spaceAfter=5 * mm,
        ),
        "body": ParagraphStyle(
            "LetterBody",
            parent=base["BodyText"],
            fontName="Helvetica",
            fontSize=9.35,
            leading=13.1,
            textColor=INK,
            alignment=TA_LEFT,
            spaceAfter=(2.5 if compact else 3.25) * mm,
        ),
        "section": ParagraphStyle(
            "LetterSection",
            parent=base["Heading2"],
            fontName="Helvetica-Bold",
            fontSize=10.25,
            leading=12.5,
            textColor=ACCENT,
            spaceBefore=2.5 * mm,
            spaceAfter=2.5 * mm,
        ),
        "reference": ParagraphStyle(
            "LetterReference",
            parent=base["BodyText"],
            fontName="Helvetica",
            fontSize=8.6,
            leading=11.5,
            textColor=INK,
            leftIndent=4 * mm,
            firstLineIndent=-4 * mm,
            spaceAfter=1.7 * mm,
        ),
    }


def render(source: Path) -> Path:
    OUTPUT.mkdir(exist_ok=True)
    target = OUTPUT / f"{source.stem}.pdf"
    doc = BaseDocTemplate(
        str(target),
        pagesize=A4,
        leftMargin=22 * mm,
        rightMargin=22 * mm,
        topMargin=18 * mm,
        bottomMargin=19 * mm,
        title=source.stem,
        author="Juan David Bendek Williamson",
        subject="Professional application letter",
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
    doc.addPageTemplates([PageTemplate(id="letter", frames=[frame], onPage=footer)])

    story = []
    style = styles(compact=source.name.startswith(("01_", "04_")))
    for kind, text in parse_blocks(source):
        if kind == "pagebreak":
            story.append(PageBreak())
        elif kind == "title":
            story.append(Paragraph(inline_markup(text), style["title"]))
            story.append(
                HRFlowable(
                    width="100%",
                    thickness=1.2,
                    color=ACCENT,
                    spaceAfter=4 * mm,
                )
            )
        elif kind == "section":
            story.append(Paragraph(inline_markup(text), style["section"]))
        elif kind == "reference":
            story.append(Paragraph(inline_markup(text), style["reference"]))
        else:
            story.append(Paragraph(inline_markup(text), style["body"]))
    story.append(Spacer(1, 2 * mm))
    doc.build(story)
    return target


def main() -> None:
    for source in SOURCES:
        print(render(source))


if __name__ == "__main__":
    main()
