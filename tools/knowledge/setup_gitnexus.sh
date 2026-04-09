#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib.sh"

ensure_gitnexus_installed

log "GitNexus is ready for $(repo_name)"
log "Version: ${GITNEXUS_VERSION}"
log "Binary: ${GITNEXUS_BIN}"

if [[ -f "${REPO_ROOT}/.codex/config.toml" ]]; then
  log "Codex project MCP config is tracked at .codex/config.toml"
fi

if [[ -f "${REPO_ROOT}/.cursor/mcp.json" ]]; then
  log "Cursor project MCP config is tracked at .cursor/mcp.json"
fi

sync_knowledge_artifacts

log "Next steps:"
log "  1. tools/knowledge/index_repo.sh"
log "  2. tools/knowledge/status.sh"
log "  3. tools/knowledge/generate_wiki.sh   (after LLM config is available)"
