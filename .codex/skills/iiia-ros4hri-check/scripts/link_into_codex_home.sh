#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET_ROOT="${CODEX_HOME:-$HOME/.codex}/skills"
TARGET_PATH="${TARGET_ROOT}/iiia-ros4hri-check"
MODE="${1:-copy}"

mkdir -p "${TARGET_ROOT}"
case "${MODE}" in
  symlink)
    rm -rf "${TARGET_PATH}"
    ln -sfn "${SKILL_DIR}" "${TARGET_PATH}"
    echo "symlinked ${SKILL_DIR} -> ${TARGET_PATH}"
    ;;
  copy)
    rm -rf "${TARGET_PATH}"
    mkdir -p "${TARGET_PATH}"
    cp -a "${SKILL_DIR}/." "${TARGET_PATH}/"
    echo "copied ${SKILL_DIR} -> ${TARGET_PATH}"
    ;;
  *)
    echo "Usage: $0 [copy|symlink]" >&2
    exit 2
    ;;
esac
