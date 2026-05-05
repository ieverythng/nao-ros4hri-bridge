#!/usr/bin/env bash
set -euo pipefail

CHATBOT_HOST="${CHATBOT_OLLAMA_HOST:-127.0.0.1:11434}"
PLANNER_HOST="${PLANNER_OLLAMA_HOST:-127.0.0.1:11435}"
MODEL="${OLLAMA_MODEL:-gemma4:31b-cloud}"
LOG_DIR="${OLLAMA_LOG_DIR:-/tmp/nao_demo_ollama}"
SYSTEM_OLLAMA_MODELS="/usr/share/ollama/.ollama/models"
if [ -z "${OLLAMA_MODELS:-}" ] && [ -d "${SYSTEM_OLLAMA_MODELS}/manifests" ]; then
  export OLLAMA_MODELS="${SYSTEM_OLLAMA_MODELS}"
fi
RUN_PLANNER_AS_OLLAMA_USER="${RUN_PLANNER_AS_OLLAMA_USER:-auto}"

mkdir -p "${LOG_DIR}"

endpoint_ready() {
  local host="$1"
  python3 - "$host" <<'PY'
import sys
import urllib.request

host = sys.argv[1]
try:
    with urllib.request.urlopen(f"http://{host}/api/tags", timeout=2.0) as response:
        raise SystemExit(0 if response.status == 200 else 1)
except Exception:
    raise SystemExit(1)
PY
}

start_endpoint() {
  local name="$1"
  local host="$2"
  if endpoint_ready "${host}"; then
    echo "[ollama:${name}] ready at ${host}"
    return
  fi

  if ! command -v ollama >/dev/null 2>&1; then
    echo "ollama executable not found on host PATH" >&2
    exit 127
  fi

  echo "[ollama:${name}] starting at ${host}"
  if should_run_as_ollama_user "${name}"; then
    echo "[ollama:${name}] using system ollama identity via sudo -u ollama"
    sudo -u ollama env \
      HOME=/usr/share/ollama \
      OLLAMA_HOST="${host}" \
      OLLAMA_MODELS="${OLLAMA_MODELS:-${SYSTEM_OLLAMA_MODELS}}" \
      nohup ollama serve >"${LOG_DIR}/${name}.log" 2>&1 &
  else
    OLLAMA_HOST="${host}" nohup ollama serve >"${LOG_DIR}/${name}.log" 2>&1 &
  fi
  echo "$!" >"${LOG_DIR}/${name}.pid"

  for _ in $(seq 1 30); do
    if endpoint_ready "${host}"; then
      echo "[ollama:${name}] ready at ${host} | pid=$(cat "${LOG_DIR}/${name}.pid")"
      return
    fi
    sleep 1
  done

  echo "[ollama:${name}] did not become ready; see ${LOG_DIR}/${name}.log" >&2
  exit 1
}

should_run_as_ollama_user() {
  local name="$1"
  if [ "${name}" != "planner" ]; then
    return 1
  fi
  case "${RUN_PLANNER_AS_OLLAMA_USER}" in
    1|true|yes|on)
      return 0
      ;;
    0|false|no|off)
      return 1
      ;;
  esac
  [ -r /usr/share/ollama/.ollama/id_ed25519 ] && return 1
  [ -d "${SYSTEM_OLLAMA_MODELS}/manifests" ] && command -v sudo >/dev/null 2>&1
}

probe_model() {
  local name="$1"
  local host="$2"
  echo "[ollama:${name}] probing model ${MODEL}"
  python3 - "$host" "$MODEL" <<'PY'
import json
import sys
import urllib.request

host, model = sys.argv[1], sys.argv[2]
payload = {
    "model": model,
    "messages": [
        {"role": "system", "content": "Reply only with JSON. No prose."},
        {"role": "user", "content": "Return {\"ready\":true}."},
    ],
    "stream": False,
    "think": False,
    "options": {"temperature": 0.0, "num_predict": 32},
}
request = urllib.request.Request(
    f"http://{host}/api/chat",
    data=json.dumps(payload).encode("utf-8"),
    method="POST",
    headers={"Content-Type": "application/json"},
)
with urllib.request.urlopen(request, timeout=90.0) as response:
    text = response.read().decode("utf-8", "replace")
print(text[:300])
PY
}

start_endpoint chatbot "${CHATBOT_HOST}"
start_endpoint planner "${PLANNER_HOST}"
probe_model chatbot "${CHATBOT_HOST}"
probe_model planner "${PLANNER_HOST}"

echo "Both Ollama endpoints are ready:"
echo "  chatbot: http://${CHATBOT_HOST}/api/chat"
echo "  planner: http://${PLANNER_HOST}"
