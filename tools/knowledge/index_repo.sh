#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib.sh"

cd "${REPO_ROOT}"
if [[ -x "${SCRIPT_DIR}/generate_ros_graph_proxy.py" ]]; then
  python3 "${SCRIPT_DIR}/generate_ros_graph_proxy.py" --repo-root "${REPO_ROOT}"
fi
if [[ -f "${REPO_ROOT}/.gitnexusignore" ]]; then
  export GITNEXUS_NO_GITIGNORE=1
fi
run_gitnexus analyze "${REPO_ROOT}" --skip-agents-md "$@"
sync_knowledge_artifacts

log "Index refreshed for $(repo_name)"
log "Run tools/knowledge/status.sh to inspect freshness and sync state."
