#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-symlink}"
SKILL_NAME="rgctl-operations"
SOURCE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TARGET_ROOT="${CODEX_HOME:-$HOME/.codex}/skills"
TARGET_DIR="$TARGET_ROOT/$SKILL_NAME"

mkdir -p "$TARGET_ROOT"

if [[ "$MODE" == "copy" ]]; then
  rm -rf "$TARGET_DIR"
  cp -R "$SOURCE_DIR" "$TARGET_DIR"
  echo "Installed $SKILL_NAME to $TARGET_DIR (copy mode)."
else
  ln -sfn "$SOURCE_DIR" "$TARGET_DIR"
  echo "Installed $SKILL_NAME to $TARGET_DIR (symlink mode)."
fi
