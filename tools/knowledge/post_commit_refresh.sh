#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib.sh"

if ! git rev-parse --git-dir >/dev/null 2>&1; then
  die "This script must run inside a git repository."
fi

log "Refreshing GitNexus after commit"
cd "${REPO_ROOT}"
if [[ -f "${REPO_ROOT}/.gitnexusignore" ]]; then
  export GITNEXUS_NO_GITIGNORE=1
fi
run_gitnexus analyze "${REPO_ROOT}" --skip-agents-md "$@"
sync_knowledge_artifacts
log "GitNexus refresh complete"
