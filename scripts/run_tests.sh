#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [[ $# -gt 0 ]]; then
  echo "Usage: $0"
  exit 2
fi

set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # Load ROS message packages for migration-package unit tests.
  source /opt/ros/jazzy/setup.bash
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi
set -u

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
    "src/nao_skill_servers/nao_skill_servers/*.py",
    "src/nao_look_at/nao_look_at/*.py",
    "src/nao_orchestrator/nao_orchestrator/*.py",
    "src/nao_say_skill/nao_say_skill/*.py",
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
PYTHONPATH="src/nao_chatbot:${PYTHONPATH:-}" pytest -q src/nao_chatbot/test/unit

echo "[3/7] asr_vosk unit tests"
PYTHONPATH="src/asr_vosk:${PYTHONPATH:-}" pytest -q src/asr_vosk/test/unit

echo "[4/7] simple_audio_capture unit tests"
PYTHONPATH="src/simple_audio_capture:${PYTHONPATH:-}" pytest -q src/simple_audio_capture/test/unit

echo "[5/7] migration package unit tests"
PYTHONPATH="src/nao_look_at:src/nao_orchestrator:src/nao_say_skill:${PYTHONPATH:-}" pytest -q \
  src/nao_look_at/test/test_nao_look_at_unit.py \
  src/nao_orchestrator/test/test_nao_orchestrator_intent_rules.py \
  src/nao_say_skill/test/test_nao_say_skill_unit.py

echo "[6/7] nao_skill_servers unit tests"
PYTHONPATH="src/nao_skill_servers:${PYTHONPATH:-}" pytest -q src/nao_skill_servers/test/unit

echo "[7/7] Done"
echo "All available tests passed."
