#!/usr/bin/env bash

set -euo pipefail

KNOWLEDGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${KNOWLEDGE_DIR}/../.." && pwd)"

GITNEXUS_VERSION="${GITNEXUS_VERSION:-1.5.3}"
GITNEXUS_INSTALL_ROOT="${GITNEXUS_INSTALL_ROOT:-${HOME}/.gitnexus-tooling/gitnexus-${GITNEXUS_VERSION}}"
GITNEXUS_BIN="${GITNEXUS_INSTALL_ROOT}/node_modules/.bin/gitnexus"
GITNEXUS_CONFIG_FILE="${GITNEXUS_CONFIG_FILE:-${HOME}/.gitnexus/config.json}"

log() {
  printf '[knowledge] %s\n' "$*"
}

die() {
  printf '[knowledge] %s\n' "$*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"
}

ensure_node_version() {
  require_cmd node
  require_cmd npm

  local version major
  version="$(node -p 'process.versions.node')"
  major="${version%%.*}"

  [[ "${major}" =~ ^[0-9]+$ ]] || die "Could not parse Node.js version: ${version}"
  (( major >= 18 )) || die "GitNexus requires Node.js >= 18; found ${version}"
}

gitnexus_is_installed() {
  [[ -x "${GITNEXUS_BIN}" ]] && "${GITNEXUS_BIN}" --version >/dev/null 2>&1
}

ensure_gitnexus_installed() {
  ensure_node_version

  if gitnexus_is_installed; then
    return
  fi

  log "Installing gitnexus@${GITNEXUS_VERSION} into ${GITNEXUS_INSTALL_ROOT}"
  rm -rf "${GITNEXUS_INSTALL_ROOT}/node_modules" "${GITNEXUS_INSTALL_ROOT}/package-lock.json"
  mkdir -p "${GITNEXUS_INSTALL_ROOT}"

  local -a install_args
  install_args=(
    install
    --prefix "${GITNEXUS_INSTALL_ROOT}"
    --no-audit
    --no-fund
    "gitnexus@${GITNEXUS_VERSION}"
  )
  npm "${install_args[@]}"
}

run_gitnexus() {
  ensure_gitnexus_installed
  "${GITNEXUS_BIN}" "$@"
}

sync_knowledge_artifacts() {
  require_cmd python3
  python3 "${KNOWLEDGE_DIR}/sync_artifacts.py" --repo-root "${REPO_ROOT}"
}

repo_name() {
  basename "${REPO_ROOT}"
}

has_saved_gitnexus_config() {
  [[ -f "${GITNEXUS_CONFIG_FILE}" ]]
}

has_wiki_llm_config() {
  [[ -n "${OPENAI_API_KEY:-}" ]] ||
    [[ -n "${GITNEXUS_API_KEY:-}" ]] ||
    [[ -n "${AZURE_OPENAI_API_KEY:-}" ]] ||
    has_saved_gitnexus_config
}
