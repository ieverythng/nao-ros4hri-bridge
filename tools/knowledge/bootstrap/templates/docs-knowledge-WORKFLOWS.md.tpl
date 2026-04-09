# __REPO_NAME__ GitNexus Workflows

Use GitNexus as a narrowing tool, then confirm in source.

Core patterns:

- explore: `query` -> `context` -> source
- debug: `query` symptom -> `context` suspect -> source
- refactor: `context` + `impact` before edits
- refresh: `post_commit_refresh.sh` after commits when you want the graph current

If the web backend is already running, prefer:

```bash
GITNEXUS_USE_HTTP=1 tools/knowledge/codex_with_gitnexus.sh
```

In that mode, `codex mcp list` will show the shared backend as `gitnexus_http`.
