#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is not installed. Install python3 first."
  exit 1
fi

if [ ! -d ".venv" ]; then
  echo "Creating .venv ..."
  if ! python3 -m venv .venv; then
    echo "Failed to create .venv."
    echo "Install required packages, then rerun:"
    echo "  sudo apt-get update && sudo apt-get install -y python3-venv python3-pip"
    exit 1
  fi
fi

source .venv/bin/activate

python -m pip install --upgrade pip
python -m pip install -r requirements-dev.txt

python -m pre_commit install --install-hooks --hook-type pre-commit
python -m pre_commit install --hook-type pre-push

echo "Dev tools ready."
echo "Use these commands from VS Code terminal:"
echo "  source .venv/bin/activate"
echo "  python -m pre_commit run --all-files"
