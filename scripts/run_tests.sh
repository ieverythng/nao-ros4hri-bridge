#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTEST_ADDOPTS="${PYTEST_ADDOPTS:-} -p no:cacheprovider"
export PYTHONPYCACHEPREFIX="$(mktemp -d)"
trap 'rm -rf "${PYTHONPYCACHEPREFIX}"' EXIT

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

have_python_module() {
  python3 - "$1" <<'PY'
import importlib.util
import sys

module_name = sys.argv[1]
raise SystemExit(0 if importlib.util.find_spec(module_name) is not None else 1)
PY
}

echo "[1/10] Syntax checks"
python3 - <<'PY'
from pathlib import Path
import py_compile

paths = []
for pattern in (
    "src/nao_chatbot/launch/*.launch.py",
    "src/nao_chatbot/nao_chatbot/*.py",
    "src/chatbot_llm/chatbot_llm/*.py",
    "src/kb_skills/kb_skills/*.py",
    "src/dialogue_manager/dialogue_manager/*.py",
    "src/planner_common/planner_common/*.py",
    "src/planner_llm/planner_llm/*.py",
    "src/asr_vosk/launch/*.launch.py",
    "src/asr_vosk/asr_vosk/*.py",
    "src/nao_look_at/nao_look_at/*.py",
    "src/nao_orchestrator/nao_orchestrator/*.py",
    "src/nao_replay_motion/nao_replay_motion/*.py",
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

echo "[2/10] nao_chatbot unit tests"
PYTHONPATH="src/nao_chatbot:${PYTHONPATH:-}" python3 -m pytest -q \
  src/nao_chatbot/test/unit/test_asr_push_to_talk_cli.py \
  src/nao_chatbot/test/unit/test_robot_speech_debug.py

echo "[3/10] kb_skills unit tests"
PYTHONPATH="src/kb_skills:${PYTHONPATH:-}" python3 -m pytest -q \
  src/kb_skills/test/test_query_client.py

echo "[4/10] planner package unit tests"
PYTHONPATH="src/planner_common:src/planner_llm:${PYTHONPATH:-}" python3 -m pytest -q \
  src/planner_common/test/test_contracts.py \
  src/planner_llm/test/test_planner_engine.py

echo "[5/10] chatbot_llm unit tests"
if have_python_module hri_actions_msgs && have_python_module chatbot_msgs; then
  PYTHONPATH="src/kb_skills:src/chatbot_llm:${PYTHONPATH:-}" python3 -m pytest -q \
    src/chatbot_llm/test/test_intent_adapter.py \
    src/chatbot_llm/test/test_knowledge_snapshot.py \
    src/chatbot_llm/test/test_skill_catalog.py \
    src/chatbot_llm/test/test_turn_engine.py
else
  echo "Skipping chatbot_llm ROS contract tests because required ROS message modules are unavailable."
fi

echo "[6/10] dialogue_manager unit tests"
if have_python_module numpy; then
  PYTHONPATH="src/dialogue_manager:${PYTHONPATH:-}" python3 -m pytest -q \
    src/dialogue_manager/test/test_chatbot_client.py \
    src/dialogue_manager/test/test_dialogue.py \
    src/dialogue_manager/test/test_integration.py \
    src/dialogue_manager/test/test_manager_node.py \
    src/dialogue_manager/test/test_skill_servers.py \
    src/dialogue_manager/test/test_speech_handler.py \
    src/dialogue_manager/test/test_tts_client.py
else
  echo "Skipping dialogue_manager unit tests because python3 module 'numpy' is unavailable."
fi

echo "[7/10] asr_vosk unit tests"
PYTHONPATH="src/asr_vosk:${PYTHONPATH:-}" python3 -m pytest -q src/asr_vosk/test/unit

echo "[8/10] simple_audio_capture unit tests"
PYTHONPATH="src/simple_audio_capture:${PYTHONPATH:-}" python3 -m pytest -q src/simple_audio_capture/test/unit

echo "[9/10] migration package unit tests"
if have_python_module numpy; then
  PYTHONPATH="src/planner_common:src/kb_skills:src/nao_look_at:src/nao_orchestrator:src/nao_replay_motion:src/nao_say_skill:${PYTHONPATH:-}" python3 -m pytest -q \
    src/nao_look_at/test/test_nao_look_at_unit.py \
    src/nao_orchestrator/test/test_nao_orchestrator_intent_rules.py \
    src/nao_replay_motion/test/test_nao_replay_motion_unit.py \
    src/nao_say_skill/test/test_nao_say_skill_unit.py
else
  echo "Skipping ROS action-based migration unit tests because python3 module 'numpy' is unavailable."
  PYTHONPATH="src/planner_common:src/kb_skills:src/nao_orchestrator:${PYTHONPATH:-}" python3 -m pytest -q \
    src/nao_orchestrator/test/test_nao_orchestrator_intent_rules.py
fi

echo "[10/10] launch smoke"
if [[ -f install/setup.bash ]]; then
  ros2 launch nao_chatbot nao_chatbot_sim.launch.py --show-args >/dev/null
  ros2 launch nao_chatbot nao_chatbot_sim_asr.launch.py --show-args >/dev/null
  ros2 launch nao_chatbot nao_chatbot_robot.launch.py --show-args >/dev/null
  ros2 launch nao_chatbot nao_chatbot_robot_asr.launch.py --show-args >/dev/null
  ros2 launch nao_chatbot nao_chatbot_planner_local.launch.py --show-args >/dev/null
  ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py --show-args >/dev/null
else
  echo "Skipping launch smoke because install/setup.bash is not available."
fi

echo "Done"
echo "All available tests passed."
