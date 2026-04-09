#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib.sh"

codex_args=(codex -C "${REPO_ROOT}")

http_mcp_url=""
if [[ -n "${GITNEXUS_MCP_URL:-}" ]]; then
  http_mcp_url="${GITNEXUS_MCP_URL}"
elif [[ "${GITNEXUS_USE_HTTP:-0}" == "1" ]]; then
  http_mcp_url="http://127.0.0.1:4747/api/mcp"
fi

if [[ -n "${http_mcp_url}" ]]; then
  codex_args+=(
    -c 'mcp_servers.gitnexus.enabled=false'
    -c "mcp_servers.gitnexus_http.url=\"${http_mcp_url}\""
  )
else
  codex_args+=(
    -c "mcp_servers.gitnexus.command=\"./tools/knowledge/gitnexus.sh\""
    -c 'mcp_servers.gitnexus.args=["mcp"]'
  )
fi

exec "${codex_args[@]}" "$@"
