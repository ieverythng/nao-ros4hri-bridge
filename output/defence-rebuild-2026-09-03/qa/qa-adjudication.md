# Revised defence deck — QA assessment

62 slides: 32 main, one Q&A page and 29 appendix pages. All slides rendered and visually inspected; long titles, overlapping headings and overflowing code excerpts were corrected. The exported PPTX was re-imported and rendered again. The ten slightly different round-trip renders were individually inspected. These differences concern font/paragraph spacing and chart rendering, not missing content. Speaker notes contain source references on every slide.

The original PPTX SHA-256 remains unchanged. All displayed JSON examples parse. Read-only package checks confirm XML validity, resolving internal relationships, embedded images, one native editable chart, nonnegative extents and no unresolved template placeholders. The source deck contained negative-height decorative lines: these were normalised through the artifact-tool shape-position API in an intermediate copy before using the required starter-deck helper. The original file was not changed.

## Fidelity checker false positives

The remaining automatic findings label `extract_sources.py` and `verify_package.py` as direct OOXML mutations. Manual source review confirms that both open ZIP archives read-only and never save or rewrite a PPTX. They write only extracted text/JSON QA reports. The checker's broad lexical rule flags any file containing `ZipFile` together with a slide path or a word beginning with `write`; it does not inspect archive mode. These two findings are adjudicated as false positives, not silently removed. All presentation authoring uses JavaScript artifact-tool objects.

Earlier resized-panel warnings on slide 30 were reviewed: those were mapped inherited shapes, not new covers. The final slide uses the same open-column treatment as the new technical slides; no opaque covering panels remain.

## Evidence and scope

The final 76-page thesis, source contracts, supplied photos and retained local logs are the content sources. Schema examples are labelled separately from recorded trace excerpts. Recovery events are sorted and deduplicated by goal-correlated plan/event/step fields; the repeated find in the replacement plan is explicitly acknowledged. The primary 72-case denominator remains separate from 108 provenance records and the 191-record model supplement.

The robot stack was not relaunched or changed. Existing unrelated working-tree edits were preserved. Human supervision/review of agent-generated project code is attributed to the author's statement, not independently inferred from code history.

## HTML review testing

The self-contained review document is regenerated from the exported PPTX renders and has a new deck hash/storage namespace. JavaScript syntax, valid annotation round trips, invalid import rejection, slide IDs/count and image byte correspondence are checked locally. The browser rejected the local file URL under its security policy; no browser workaround was attempted. Interactive browser verification therefore remains unperformed.

## Missing optional material

The supervisor fallback video is not locally supplied and is not embedded. The retained trace is available as an additional fallback. No extra Linux traces were required for this revision.
