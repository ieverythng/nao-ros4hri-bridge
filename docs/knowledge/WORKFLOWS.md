# nao-ros4hri-bridge GitNexus Workflows

Use GitNexus as a narrowing tool, then confirm in source.

Core patterns:

- explore: `query` -> `context` -> source
- debug: `query` symptom -> `context` suspect -> source
- refactor: `context` + `impact` before edits
- refresh: `post_commit_refresh.sh` after commits when you want the graph current
- source coverage: `scripts/bootstrap_socialminds_sources.sh` before the first serious graph pass on this repo

If the web backend is already running, prefer:

```bash
GITNEXUS_USE_HTTP=1 tools/knowledge/codex_with_gitnexus.sh
```

In that mode, `codex mcp list` will show the shared backend as `gitnexus_http`.
Also avoid direct local CLI graph calls against the same running repo database, or you may hit a LadybugDB file lock.
