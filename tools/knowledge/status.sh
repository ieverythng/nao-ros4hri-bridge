#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib.sh"

advisory_mode=0
if [[ "${1:-}" == "--advisory" ]]; then
  advisory_mode=1
  shift
fi

cd "${REPO_ROOT}"

if [[ ! -f "${REPO_ROOT}/.gitnexus/meta.json" ]]; then
  log "No GitNexus index found yet. Run tools/knowledge/index_repo.sh"
  exit "${advisory_mode}"
fi

run_gitnexus status "$@" || true

python3 "${SCRIPT_DIR}/status_summary.py" --repo-root "${REPO_ROOT}"

staged_files="$(git diff --cached --name-only)"
if [[ -n "${staged_files}" ]]; then
  log "Staged files detected. After commit, the local GitNexus index may need a refresh."
fi
