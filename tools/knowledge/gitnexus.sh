#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib.sh"

subcommand="${1:-}"

if [[ "${subcommand}" == "wiki" ]]; then
  shift

  if ! has_wiki_llm_config; then
    die "Wiki generation needs an OpenAI-compatible API key or a saved ~/.gitnexus/config.json. Configure GitNexus wiki access first."
  fi

  cd "${REPO_ROOT}"
  run_gitnexus wiki "${REPO_ROOT}" "$@"
  sync_knowledge_artifacts
  log "Wiki artifacts synced into docs/knowledge/"
  exit 0
fi

cd "${REPO_ROOT}"
run_gitnexus "$@"
