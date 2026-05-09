#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

# Prefer project venv when present (numpy, ruff, etc.); still inherits ROS from sourced setup.bash.
PYTHON_BIN="python3"
if [[ -x "${REPO_ROOT}/.venv/bin/python3" ]]; then
  PYTHON_BIN="${REPO_ROOT}/.venv/bin/python3"
fi

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
  # Load ROS message packages for migration-package unit tests and launch imports.
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f install/setup.bash ]]; then
  # shellcheck source=/dev/null
  source install/setup.bash
fi
set -u

have_python_module() {
  "${PYTHON_BIN}" - "$1" <<'PY'
import importlib.util
import sys

module_name = sys.argv[1]
raise SystemExit(0 if importlib.util.find_spec(module_name) is not None else 1)
PY
}

echo "[1/10] Syntax checks"
"${PYTHON_BIN}" - <<'PY'
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
PYTHONPATH="src/nao_chatbot:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q \
  src/nao_chatbot/test/unit/test_asr_push_to_talk_cli.py \
  src/nao_chatbot/test/unit/test_robot_speech_debug.py

if have_python_module launch; then
  PYTHONPATH="src/nao_chatbot:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q \
    src/nao_chatbot/test/unit/test_launch_profiles.py
else
  echo "Skipping nao_chatbot/test/unit/test_launch_profiles.py (need ROS 'launch' on PYTHONPATH; source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash before run_tests.sh)."
fi

echo "[3/10] kb_skills unit tests"
PYTHONPATH="src/kb_skills:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q \
  src/kb_skills/test/test_query_client.py

echo "[4/10] planner package unit tests"
PYTHONPATH="src/planner_common:src/planner_llm:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q \
  src/planner_common/test/test_contracts.py \
  src/planner_llm/test/test_planner_engine.py

echo "[5/10] chatbot_llm unit tests"
if have_python_module hri_actions_msgs && have_python_module chatbot_msgs; then
  PYTHONPATH="src/kb_skills:src/chatbot_llm:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q \
    src/chatbot_llm/test/test_intent_adapter.py \
    src/chatbot_llm/test/test_knowledge_snapshot.py \
    src/chatbot_llm/test/test_skill_catalog.py \
    src/chatbot_llm/test/test_turn_engine.py
else
  echo "Skipping chatbot_llm ROS contract tests because required ROS message modules are unavailable."
fi

echo "[6/10] dialogue_manager unit tests"
if have_python_module numpy; then
  PYTHONPATH="src/dialogue_manager:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q \
    src/dialogue_manager/test/test_chatbot_client.py \
    src/dialogue_manager/test/test_dialogue.py \
    src/dialogue_manager/test/test_integration.py \
    src/dialogue_manager/test/test_manager_node.py \
    src/dialogue_manager/test/test_skill_servers.py \
    src/dialogue_manager/test/test_speech_handler.py \
    src/dialogue_manager/test/test_tts_client.py
else
  echo "Skipping dialogue_manager unit tests because python module 'numpy' is unavailable (pip install -r requirements-dev.txt in .venv)."
fi

echo "[7/10] asr_vosk unit tests"
PYTHONPATH="src/asr_vosk:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q src/asr_vosk/test/unit

echo "[8/10] simple_audio_capture unit tests"
PYTHONPATH="src/simple_audio_capture:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q src/simple_audio_capture/test/unit

echo "[9/10] migration package unit tests"
if have_python_module numpy; then
  PYTHONPATH="src/planner_common:src/kb_skills:src/nao_look_at:src/nao_orchestrator:src/nao_replay_motion:src/nao_say_skill:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q \
    src/nao_look_at/test/test_nao_look_at_unit.py \
    src/nao_orchestrator/test/test_nao_orchestrator_intent_rules.py \
    src/nao_replay_motion/test/test_nao_replay_motion_unit.py \
    src/nao_say_skill/test/test_nao_say_skill_unit.py
else
  echo "Skipping ROS action-based migration unit tests because python module 'numpy' is unavailable (pip install -r requirements-dev.txt in .venv)."
  PYTHONPATH="src/planner_common:src/kb_skills:src/nao_orchestrator:${PYTHONPATH:-}" "${PYTHON_BIN}" -m pytest -q \
    src/nao_orchestrator/test/test_nao_orchestrator_intent_rules.py
fi

echo "[10/10] launch smoke"
if [[ -f install/setup.bash ]]; then
  # Use source launch entrypoints so smoke checks always track current edits
  # even when install/share contains stale launch symlinks.
  run_launch_smoke_if_tracked() {
    local launch_path="$1"
    if git ls-files --error-unmatch "${launch_path}" >/dev/null 2>&1; then
      ros2 launch "${launch_path}" --show-args >/dev/null
    else
      echo "Skipping launch smoke for untracked file: ${launch_path}"
    fi
  }

  run_launch_smoke_if_tracked src/nao_chatbot/launch/nao_chatbot_sim.launch.py
  run_launch_smoke_if_tracked src/nao_chatbot/launch/nao_chatbot_robot.launch.py
  run_launch_smoke_if_tracked src/nao_chatbot/launch/nao_chatbot_demo.launch.py
  run_launch_smoke_if_tracked src/nao_chatbot/launch/nao_chatbot_asr_only.launch.py
else
  echo "Skipping launch smoke because install/setup.bash is not available."
fi

echo "Done"
echo "All available tests passed."
