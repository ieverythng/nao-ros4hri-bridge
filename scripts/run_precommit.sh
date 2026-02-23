#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [ ! -x ".venv/bin/python" ]; then
  echo "Missing .venv. Run:"
  echo "  ./scripts/setup_dev_tools.sh"
  exit 1
fi

source .venv/bin/activate
python -m pre_commit run --all-files
