# nao-ros4hri-bridge GitNexus Quickstart

Run:

- `scripts/bootstrap_socialminds_sources.sh`
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

Important refresh note:

- `tools/knowledge/pre_commit_advisory.sh` is advisory only and does not refresh the graph
- `tools/knowledge/index_repo.sh --force` or `tools/knowledge/post_commit_refresh.sh` refresh the index
- if the local web UI/backend is already serving this repo, restart it after a refresh if you want the UI to pick up the newest graph state immediately

Source coverage note:

- this repo intentionally keeps several graph-relevant packages ignored in git
- `tools/knowledge/index_repo.sh` uses `.gitnexusignore` instead of `.gitignore` so GitNexus still includes those source trees
- `scripts/bootstrap_socialminds_sources.sh` clones reference-only upstream packages under `ref_src/knowledge_sources/` and lightweight ROS4HRI contracts under `src/`
- `tools/knowledge/index_repo.sh` also regenerates `docs/knowledge/ros_runtime_proxy.py` and related ROS graph artifacts before each analyze run
- `tools/knowledge/ros_graph_overrides.json` lets this repo pin runtime-specific ROS endpoint rewrites when the launch surface differs from what raw source alone would suggest

Wiki generation needs an OpenAI-compatible API key or saved GitNexus config.

For graph-guided agent playbooks, read `docs/knowledge/WORKFLOWS.md`.
