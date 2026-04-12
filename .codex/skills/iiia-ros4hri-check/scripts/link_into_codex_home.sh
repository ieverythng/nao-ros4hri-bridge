#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET_ROOT="${CODEX_HOME:-$HOME/.codex}/skills"
TARGET_PATH="${TARGET_ROOT}/iiia-ros4hri-check"

mkdir -p "${TARGET_ROOT}"
ln -sfn "${SKILL_DIR}" "${TARGET_PATH}"
echo "linked ${SKILL_DIR} -> ${TARGET_PATH}"
