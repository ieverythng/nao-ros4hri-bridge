# nao-ros4hri-bridge Knowledge Layer

This repo uses GitNexus as a local code-intelligence backend plus a tracked `docs/knowledge/` surface for humans and agents.

Start with `QUICKSTART.md` for the shortest path and `WORKFLOWS.md` for the graph-guided agent playbooks.

Primary commands:

- `tools/knowledge/setup_gitnexus.sh`
- `tools/knowledge/index_repo.sh`
- `tools/knowledge/status.sh`
- `tools/knowledge/generate_wiki.sh`
- `tools/knowledge/serve_http_mcp.sh`
- `tools/knowledge/codex_with_gitnexus.sh`
- `tools/knowledge/post_commit_refresh.sh`

Notes:

- first install can take a couple of minutes on Apple Silicon
- wiki generation needs an OpenAI-compatible API key or a saved `~/.gitnexus/config.json`
- the bootstrap skips existing guidance and config files instead of overwriting them
- `QUICKSTART.md` is the fastest path for a new repo
- `WORKFLOWS.md` mirrors the most useful GitNexus graph workflows in repo-owned form
- `GITNEXUS_USE_HTTP=1 tools/knowledge/codex_with_gitnexus.sh` lets Codex share the already-running GitNexus web backend
- In that HTTP-backed mode, `codex mcp list` will show the shared backend as `gitnexus_http`
- this repo uses `.gitnexusignore` and `GITNEXUS_NO_GITIGNORE=1` during indexing so GitNexus can include ignored graph-relevant source trees such as `src/chatbot_llm`, `src/dialogue_manager`, and any bootstrapped ROS4HRI packages
- run `scripts/bootstrap_socialminds_sources.sh` before indexing if you want the graph to include the upstream KB stack and the lightweight ROS4HRI workspace contracts
- if the GitNexus UI/backend is already running, avoid direct local CLI graph calls against the same repo database; reindex first, then restart the server or use the HTTP-backed MCP path
- once multiple repos are indexed, the same GitNexus registry can expose all of them and the UI can switch projects without a separate install per repo
