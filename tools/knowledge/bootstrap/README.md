# Knowledge Bootstrap

This folder is the reusable bootstrap layer for future repos.

Use `init_repo_knowledge.sh <target-repo-path>` to seed:

- `tools/knowledge/` runtime scripts
- `AGENTS.md`
- `.codex/config.toml`
- `.cursor/mcp.json`
- `docs/knowledge/` skeleton files
- `docs/knowledge/QUICKSTART.md`
- `docs/knowledge/WORKFLOWS.md`

Design goals:

- keep the runtime self-contained
- keep tracked docs portable across repos
- avoid forcing heavy or LLM-backed work into normal commit flows
- make GitNexus an advisory layer first, not an irreversible workflow switch
- refresh runtime scripts without clobbering an existing repo's guidance files
