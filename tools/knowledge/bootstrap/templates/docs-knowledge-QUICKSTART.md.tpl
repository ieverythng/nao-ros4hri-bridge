# __REPO_NAME__ GitNexus Quickstart

Run:

- `tools/knowledge/setup_gitnexus.sh`
- `tools/knowledge/index_repo.sh`
- `tools/knowledge/status.sh`

Useful test paths:

- CLI: `./tools/knowledge/gitnexus.sh query <symbol>`
- Codex: `tools/knowledge/codex_with_gitnexus.sh`
- Web UI backend: `tools/knowledge/serve_http_mcp.sh`
- Shared UI + Codex backend: `GITNEXUS_USE_HTTP=1 tools/knowledge/codex_with_gitnexus.sh`
- In that HTTP-backed mode, `codex mcp list` will show the shared backend as `gitnexus_http`

Optional refresh automation:

- `tools/knowledge/post_commit_refresh.sh`

Wiki generation needs an OpenAI-compatible API key or saved GitNexus config.

For graph-guided agent playbooks, read `docs/knowledge/WORKFLOWS.md`.
