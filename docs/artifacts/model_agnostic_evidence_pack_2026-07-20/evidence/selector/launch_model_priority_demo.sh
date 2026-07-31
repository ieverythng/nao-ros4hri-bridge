#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESOLVER="${ROOT_DIR}/scripts/resolve_demo_model.py"

if [ ! -r "${RESOLVER}" ]; then
  echo "model resolver is missing: ${RESOLVER}" >&2
  exit 2
fi

CLI_MODEL_OVERRIDE=""
for launch_arg in "$@"; do
  case "${launch_arg}" in
    chatbot_model:=*|ollama_model:=*|planner_llm_model:=*)
      CLI_MODEL_OVERRIDE="${launch_arg#*:=}"
      ;;
  esac
done

RESOLUTION_JSON="$(mktemp /tmp/nao_model_resolution.XXXXXX.json)"
trap 'rm -f "${RESOLUTION_JSON}"' EXIT
RESOLVER_ARGS=(--json --log-selection)
if [ -n "${CLI_MODEL_OVERRIDE}" ]; then
  RESOLVER_ARGS+=(--model-override "${CLI_MODEL_OVERRIDE}")
fi
python3 "${RESOLVER}" "${RESOLVER_ARGS[@]}" >"${RESOLUTION_JSON}"

mapfile -t MODEL_LAUNCH_ARGS < <(
  python3 - "${RESOLUTION_JSON}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    payload = json.load(stream)
for launch_arg in payload["launch_args"]:
    print(launch_arg)
PY
)
if [ "${#MODEL_LAUNCH_ARGS[@]}" -eq 0 ]; then
  echo "model resolver returned no launch arguments" >&2
  exit 1
fi

python3 - "${RESOLUTION_JSON}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    payload = json.load(stream)
choice = payload["choice"]
print("[model resolver] backend=%s model=%s" % (choice["backend"], choice["model"]))
print("[model resolver] chatbot=%s" % payload["chatbot_server_url"])
for diagnostic in payload.get("diagnostics", []):
    print("[model resolver] %s" % diagnostic, file=sys.stderr)
PY
printf '%s\n' "[model priority launch] starting nao_chatbot with resolved backend"

exec ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  "$@" \
  "${MODEL_LAUNCH_ARGS[@]}"
