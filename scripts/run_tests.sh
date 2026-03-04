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

echo "[1/4] Syntax checks"
python3 -m py_compile src/nao_chatbot/launch/*.launch.py
python3 -m py_compile \
  src/nao_chatbot/nao_chatbot/asr_utils.py \
  src/nao_chatbot/nao_chatbot/asr_vosk.py \
  src/nao_posture_bridge/nao_posture_bridge/posture_skill_server.py

echo "[2/4] nao_chatbot unit tests"
PYTHONPATH=src/nao_chatbot pytest -q src/nao_chatbot/test/unit

echo "[3/4] dialogue_manager unit tests (conditional)"
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

echo "[4/4] Done"
echo "All available tests passed."
