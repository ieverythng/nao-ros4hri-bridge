# Qwen3VL failure-position evidence addendum

Run ID: `qwen3vl-failure-position-addendum-2026-07-22`

This addendum packages Qwen3VL traces that were created after the earlier model-agnostic evidence bundle. It is intended to support the thesis analysis of where failures enter a composite planning sequence. The evidence is grouped around three positions: beginning, middle, and end. The package retains the original run directories, manifests, metadata, raw case JSON, compressed stack logs, derived records, and checksums.

The addendum supplements [the 20 July model-agnostic evidence pack](../nao_ros4hri_model_agnostic_evidence_2026-07-20.zip). A ZIP listing check confirmed that the earlier pack did not contain the F13 or F14 run identifiers or their failure-profile artifacts.

## Contents

- `REPORT.md` and `REPORT.html`: thesis-oriented evidence report.
- `METHODOLOGY.md`: definitions, provenance rules, and limitations.
- `analysis/failure_position_dataset.csv`: one row per source case.
- `analysis/failure_position_dataset.jsonl`: the same records in line-oriented JSON.
- `analysis/failure_position_summary.json`: machine-readable counts and stage matrix.
- `analysis/failure_position_trace_excerpts.md`: compact excerpts derived from retained case logs.
- `evidence/F13_semantic_audit/`: 54-case semantic audit from 13 July 2026.
- `evidence/F14_targeted_hardening/`: 7-case targeted hardening run from 14 July 2026.
- `evidence/metadata/`: run-level metadata and image/provenance declarations.
- `evidence/previous_package_boundary.md`: comparison against the earlier ZIP.
- `checksums.sha256`: checksums for every packaged file except the checksum file itself.

## Headline evidence

The derived dataset contains 61 case records:

| Source run | Cases | Pass | Degraded | Fail | Not scored |
| --- | ---: | ---: | ---: | ---: | ---: |
| F13 semantic audit | 54 | 39 | 5 | 4 | 6 |
| F14 targeted hardening | 7 | 6 | 0 | 1 | 0 |
| Combined addendum | 61 | 45 | 5 | 5 | 6 |

The targeted subset contains 13 records. Five are classified at the beginning of the emitted plan, two in the middle, two at the end, two as mixed middle-to-end profiles, and two as unknown because the retained trace does not isolate the configured failure.

The strongest traces are:

```text
beginning: find_object -> failure -> replan/recovery
middle:    find_object -> success -> pick_object -> failure -> retry plan -> success
end:       navigate_to -> success -> find_object -> success -> bring_object -> failure
```

These are observations of stack events and case assessments. They do not establish that every configured fake failure occurred at the requested step when the raw event trace does not expose that step.

## Interpretation boundary

F13 and F14 are diagnostic Qwen3VL runs, not one frozen qualification tuple. They span successive images and worktree states, with the F14 metadata explicitly recording the multi-image provenance. The package therefore supports case-level attribution and failure-position analysis. It should not be reduced to a single model score without preserving the run and image identifiers.

The package contains no new live run. It reconciles existing artifacts and adds a deterministic, provenance-preserving projection for thesis analysis.
