#!/usr/bin/env bash
set -euo pipefail

NODE_NAME="${1:-/fake_skill_server}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your ROS environment first." >&2
  exit 1
fi

parse_string_array_param() {
  local raw="$1"
  python3 - "$raw" <<'PY'
import ast
import re
import sys

text = sys.argv[1]
match = re.search(r"\[(.*)\]", text, flags=re.DOTALL)
if not match:
    sys.exit(0)
value = "[" + match.group(1) + "]"
try:
    parsed = ast.literal_eval(value)
except Exception:
    sys.exit(0)
if not isinstance(parsed, list):
    sys.exit(0)
for item in parsed:
    if isinstance(item, str) and item.strip():
        print(item.strip())
PY
}

available_raw="$(ros2 param get "$NODE_NAME" available_scenario_ids 2>/dev/null || true)"
if [[ -z "${available_raw}" ]]; then
  echo "Could not read available_scenario_ids from ${NODE_NAME}." >&2
  echo "Is fake_skill_server running in this ROS domain?" >&2
  exit 1
fi

mapfile -t scenario_ids < <(parse_string_array_param "${available_raw}")
current_raw="$(ros2 param get "$NODE_NAME" active_scenario_id 2>/dev/null || true)"
echo "Node: ${NODE_NAME}"
echo "Current active_scenario_id:"
echo "${current_raw:-<unavailable>}"
echo

options=("<defaults>")
for scenario_id in "${scenario_ids[@]}"; do
  options+=("${scenario_id}")
done
options+=("Quit")

PS3="Select active fake-skill scenario: "
select selected in "${options[@]}"; do
  if [[ -z "${selected:-}" ]]; then
    echo "Invalid selection."
    continue
  fi
  if [[ "${selected}" == "Quit" ]]; then
    exit 0
  fi
  if [[ "${selected}" == "<defaults>" ]]; then
    ros2 param set "$NODE_NAME" active_scenario_id ""
  else
    ros2 param set "$NODE_NAME" active_scenario_id "$selected"
  fi
  echo
  ros2 param get "$NODE_NAME" active_scenario_id
  exit 0
done
