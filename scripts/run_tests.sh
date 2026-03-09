#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

STRICT_DIALOGUE=false
if [[ "${1:-}" == "--strict-dialogue" ]]; then
  STRICT_DIALOGUE=true
  shift
fi
if [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--strict-dialogue]"
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
    "src/simple_audio_capture/launch/*.launch.py",
    "src/simple_audio_capture/simple_audio_capture/*.py",
    "src/nao_posture_bridge/nao_posture_bridge/*.py",
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

echo "[6/7] dialogue_manager unit tests (conditional)"
if python3 - <<'PY'
import importlib.util
import sys

sys.exit(0 if importlib.util.find_spec("chatbot_msgs") is not None else 1)
PY
then
  PYTHONPATH=src/dialogue_manager pytest -q \
    src/dialogue_manager/test/test_chatbot_client.py \
    src/dialogue_manager/test/test_dialogue.py \
    src/dialogue_manager/test/test_skill_servers.py \
    src/dialogue_manager/test/test_speech_handler.py \
    src/dialogue_manager/test/test_tts_client.py
else
  if [[ "${STRICT_DIALOGUE}" == "true" ]]; then
    echo "dialogue_manager tests require chatbot_msgs, but it is unavailable in this shell."
    exit 1
  fi
  echo "Skipping dialogue_manager unit tests: chatbot_msgs is unavailable in this shell."
fi

echo "[7/7] Done"
echo "All available tests passed."
