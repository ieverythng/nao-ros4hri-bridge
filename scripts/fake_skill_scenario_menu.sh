#!/usr/bin/env bash
set -euo pipefail

NODE_NAME="${1:-/fake_skill_server}"

echo "[fake-skill-menu] target node: ${NODE_NAME}"
echo

if ! ros2 param get "${NODE_NAME}" available_scenario_ids >/dev/null 2>&1; then
  echo "Could not read available_scenario_ids from ${NODE_NAME}."
  echo "Make sure fake_skill_server is running in the same ROS environment."
  exit 1
fi

current_value="$(ros2 param get "${NODE_NAME}" active_scenario_id | tail -n 1 | sed 's/^.*value: //')"
raw_ids="$(ros2 param get "${NODE_NAME}" available_scenario_ids | tail -n 1 | sed 's/^.*value: //')"

echo "Current active_scenario_id: ${current_value}"
echo "Available scenario ids: ${raw_ids}"
echo
echo "Set one of the following options:"
echo "  0) <none> (reset to defaults)"

python3 - "$raw_ids" <<'PY'
import ast
import sys

raw = sys.argv[1] if len(sys.argv) > 1 else "[]"
try:
    values = ast.literal_eval(raw)
except Exception:
    values = []
if not isinstance(values, list):
    values = []

for index, value in enumerate(values, start=1):
    print(f"  {index}) {value}")
PY

echo
read -r -p "Selection number (or q to quit): " selection

if [[ "${selection}" == "q" || "${selection}" == "Q" ]]; then
  echo "No changes made."
  exit 0
fi

if [[ "${selection}" == "0" ]]; then
  ros2 param set "${NODE_NAME}" active_scenario_id ""
  exit 0
fi

chosen_value="$(python3 - "$raw_ids" "$selection" <<'PY'
import ast
import sys

raw = sys.argv[1] if len(sys.argv) > 1 else "[]"
selection = sys.argv[2] if len(sys.argv) > 2 else ""
try:
    index = int(selection)
except Exception:
    print("")
    raise SystemExit(0)

try:
    values = ast.literal_eval(raw)
except Exception:
    values = []
if not isinstance(values, list):
    values = []

if index <= 0 or index > len(values):
    print("")
else:
    print(str(values[index - 1]))
PY
)"

if [[ -z "${chosen_value}" ]]; then
  echo "Invalid selection."
  exit 1
fi

ros2 param set "${NODE_NAME}" active_scenario_id "${chosen_value}"
