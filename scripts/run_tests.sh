#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [[ $# -gt 0 ]]; then
  echo "Usage: $0"
  exit 2
fi

echo "[1/7] Syntax checks"
python3 - <<'PY'
from pathlib import Path
import py_compile

paths = []
for pattern in (
    "src/nao_chatbot/launch/*.launch.py",
    "src/nao_chatbot/nao_chatbot/*.py",
    "src/asr_vosk/launch/*.launch.py",
    "src/asr_vosk/asr_vosk/*.py",
    "src/dialogue_manager/dialogue_manager/*.py",
    "src/nao_skill_servers/nao_skill_servers/*.py",
    "src/simple_audio_capture/launch/*.launch.py",
    "src/simple_audio_capture/simple_audio_capture/*.py",
):
    paths.extend(sorted(Path().glob(pattern)))

compiled = 0
for path in paths:
    if path.name.startswith("test_"):
        continue
    py_compile.compile(str(path), doraise=True)
    compiled += 1
print(f"Compiled {compiled} python files")
PY

echo "[2/7] nao_chatbot unit tests"
PYTHONPATH=src/nao_chatbot pytest -q src/nao_chatbot/test/unit

echo "[3/7] asr_vosk unit tests"
PYTHONPATH=src/asr_vosk pytest -q src/asr_vosk/test/unit

echo "[4/7] simple_audio_capture unit tests"
PYTHONPATH=src/simple_audio_capture pytest -q src/simple_audio_capture/test/unit

echo "[5/7] dialogue_manager bridge unit tests"
PYTHONPATH=src/dialogue_manager pytest -q \
  src/dialogue_manager/test/test_nao_asr_utils.py \
  src/dialogue_manager/test/test_nao_dialogue_manager_unit.py

echo "[6/7] nao_skill_servers unit tests"
PYTHONPATH=src/nao_skill_servers pytest -q src/nao_skill_servers/test/unit

echo "[7/7] Done"
echo "All available tests passed."
