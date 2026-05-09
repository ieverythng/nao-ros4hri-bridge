# Documentation Index

Use this index to keep top-level docs focused and avoid one-off bloat.

## Active docs (update first)

- `current_workflow.md` — canonical architecture and ownership map.
- `contracts.md` — planner/request/feedback contract details.
- `launch_profiles.md` — operator launch matrix and runtime switches.
- `planner_status.md` — current planner behavior and known limits.
- `planner_architecture_current.md` — implementation-facing planner architecture snapshot.

## Archived docs (reference, do not treat as source of truth)

- `artifacts/` — handoffs, temporary checklists, and older design notes.
- `knowledge/` — generated knowledge-layer outputs and wiki snapshots.

## Retention rule

If a note is tied to a specific demo day, branch sweep, or troubleshooting pass,
place it in `docs/artifacts/` and keep the durable conclusions in one of the
active docs above.

## Markdown + HTML Pair Rule

For plan/review docs intended for human sharing:

1. Keep Markdown as canonical source (`.md`).
2. Commit a companion HTML render (`.html`) in the same folder.
3. Generate/update HTML with:

```bash
python3 scripts/render_markdown_html.py docs/<name>.md docs/<name>.html
```

If the Markdown changes, refresh the paired HTML in the same commit.
