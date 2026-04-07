#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib.sh"

HOST="${GITNEXUS_HTTP_HOST:-0.0.0.0}"
PORT="${GITNEXUS_HTTP_PORT:-4747}"

cd "${REPO_ROOT}"
log "Starting GitNexus HTTP server on ${HOST}:${PORT}"
log "MCP endpoint: http://${HOST}:${PORT}/api/mcp"
run_gitnexus serve --host "${HOST}" --port "${PORT}"
