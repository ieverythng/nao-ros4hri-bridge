#!/usr/bin/env python3
"""Inject ROS4HRI speech turns and collect runtime evidence per case."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


DEFAULT_CONTAINER = "nao_ros2"
VOICE_ID = "anonymous_speaker"
VOICE_TRACKED_TOPIC = "/nao_chatbot/humans/voices/tracked"
VOICE_SPEECH_TOPIC = "/nao_chatbot/humans/voices/anonymous_speaker/speech"
TOPIC_SAMPLE_TIMEOUT_SEC = 1.5
DEFAULT_GLOBAL_TIMEOUT_SEC = 420
KB_PROBE_OBJECT_ID = "codex_probe_cup"


@dataclass(frozen=True)
class KbInjection:
    object_id: str
    statements: tuple[str, ...]
    query_patterns: tuple[str, ...]
    query_vars: tuple[str, ...]


@dataclass(frozen=True)
class ProbeCase:
    name: str
    category: str
    text: str
    wait_sec: float = 10.0
    mode: str = "speech"
    setup: KbInjection | None = None


SMOKE_CASES = (
    ProbeCase("simple_dialogue_hey", "simple_dialogue", "Hey, how are you?", 8.0),
    ProbeCase("kb_visible_now", "kb_query_dialogue", "What can you see?", 10.0),
    ProbeCase(
        "kb_injected_object_name",
        "kb_query_dialogue",
        "What is the name and color of the probe cup?",
        12.0,
        setup=KbInjection(
            object_id=KB_PROBE_OBJECT_ID,
            statements=(
                f"myself sees {KB_PROBE_OBJECT_ID}",
                f"{KB_PROBE_OBJECT_ID} rdf:type Cup",
                f"{KB_PROBE_OBJECT_ID} dbp:name TITAS",
                f"{KB_PROBE_OBJECT_ID} dbp:color gold",
                f"{KB_PROBE_OBJECT_ID} oro:isOn table_1",
            ),
            query_patterns=(f"{KB_PROBE_OBJECT_ID} ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase("simple_wave", "simple_skill_execution", "Wave at me.", 14.0),
    ProbeCase(
        "composite_head_wave",
        "composite_skill_execution",
        "Move your head in all directions and then wave at me.",
        24.0,
    ),
    ProbeCase(
        "reflective_followup",
        "simple_dialogue",
        "How many directions did you move your head?",
        10.0,
    ),
)

TOPICS_TO_SAMPLE = (
    "/chatbot_llm/turn_trace",
    "/planner/request",
    "/planner/execution_feedback",
    "/nao_orchestrator/planner_dialogue_act",
    "/debug/nao_say/speech",
    "/speech",
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--container", default=DEFAULT_CONTAINER)
    parser.add_argument("--case-set", default="smoke", choices=("smoke",))
    parser.add_argument("--out", default="/tmp/nao_active_questionnaire.json")
    parser.add_argument("--since-sec", type=int, default=90)
    parser.add_argument("--global-timeout-sec", type=int, default=DEFAULT_GLOBAL_TIMEOUT_SEC)
    parser.add_argument(
        "--sample-topics",
        action="store_true",
        help="Collect one-shot topic samples after each case. Slower, but useful for E2E speech runs.",
    )
    parser.add_argument(
        "--mode",
        default="speech",
        choices=("speech", "chatbot_service"),
        help="Turn injection seam. Speech is full ROS4HRI E2E; service is chatbot-only.",
    )
    args = parser.parse_args()

    cases = list(SMOKE_CASES)
    results = []
    service_history: list[dict[str, str]] = []
    started_at = time.time()
    for case in cases:
        if time.time() - started_at > max(30, args.global_timeout_sec):
            results.append(
                {
                    "name": "global_timeout",
                    "category": "observability",
                    "text": "",
                    "mode": args.mode,
                    "turn_result": "global timeout reached before remaining cases",
                "started_at_unix_sec": time.time(),
                "wait_sec": 0.0,
                "injection_scope": args.mode,
                "topic_samples": {},
                "log_excerpt": recent_logs(args.container, args.since_sec),
            }
            )
            write_payload(args.out, args.container, args.case_set, started_at, results)
            return 2

        case_start = time.time()
        setup_result = None
        if case.setup is not None:
            setup_result = inject_kb_probe(args.container, case.setup)

        mode = args.mode or case.mode
        if mode == "speech":
            turn_result = publish_voice_turn(args.container, case.text)
        else:
            turn_result = call_chatbot_turn(
                args.container,
                case.text,
                len(results) + 1,
                history=service_history,
            )
            service_history.append({"speaker": VOICE_ID, "text": case.text})
            response_text = extract_service_response(turn_result)
            if response_text:
                service_history.append({"speaker": "__assistant__", "text": response_text})
        time.sleep(max(0.0, case.wait_sec))
        results.append(
            {
                "name": case.name,
                "category": case.category,
                "text": case.text,
                "mode": mode,
                "setup_result": setup_result,
                "turn_result": turn_result,
                "started_at_unix_sec": case_start,
                "wait_sec": case.wait_sec,
                "injection_scope": injection_scope(mode),
                "topic_samples": sample_topics(args.container) if args.sample_topics else {},
                "log_excerpt": recent_logs(args.container, args.since_sec),
            }
        )
        write_payload(args.out, args.container, args.case_set, started_at, results)

    write_payload(args.out, args.container, args.case_set, started_at, results)
    print(args.out)
    return 0


def write_payload(
    out_path: str,
    container: str,
    case_set: str,
    started_at: float,
    results: list[dict],
) -> None:
    payload = {
        "container": container,
        "case_set": case_set,
        "started_at_unix_sec": started_at,
        "finished_at_unix_sec": time.time(),
        "cases": results,
    }
    Path(out_path).write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def injection_scope(mode: str) -> str:
    if mode == "speech":
        return "full_ros4hri_dialogue_ingress"
    if mode == "chatbot_service":
        return "chatbot_llm_only_no_dialogue_manager_or_orchestrator"
    return mode


def publish_voice_turn(container: str, text: str) -> str:
    escaped_text = text.replace("\\", "\\\\").replace('"', '\\"')
    script = f"""
set -e
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
timeout 8 ros2 topic pub --once -w 1 {VOICE_TRACKED_TOPIC} hri_msgs/msg/IdsList "{{ids: ['{VOICE_ID}']}}" >/tmp/nao_questionnaire_voice.log 2>&1 || true
sleep 1
timeout 8 ros2 topic pub --once -w 1 {VOICE_SPEECH_TOPIC} hri_msgs/msg/LiveSpeech "{{final: \\"{escaped_text}\\", confidence: 1.0, locale: \\"en_US\\"}}" >/tmp/nao_questionnaire_speech.log 2>&1 || true
cat /tmp/nao_questionnaire_voice.log /tmp/nao_questionnaire_speech.log 2>/dev/null || true
"""
    return run(["docker", "exec", container, "bash", "-lc", script], timeout=20, check=False)


def inject_kb_probe(container: str, injection: KbInjection) -> dict[str, str]:
    statements_yaml = "\n".join("  - '%s'" % item for item in injection.statements)
    patterns_yaml = "\n".join("  - '%s'" % item for item in injection.query_patterns)
    vars_yaml = "\n".join("  - '%s'" % item for item in injection.query_vars)
    revise_request = f"""
method: update
statements:
{statements_yaml}
models:
  - default
lifespan:
  sec: 300
  nanosec: 0
"""
    query_request = f"""
patterns:
{patterns_yaml}
vars:
{vars_yaml}
models:
  - default
"""
    service_probe = run(
        [
            "docker",
            "exec",
            container,
            "bash",
            "-lc",
            (
                "source /opt/ros/jazzy/setup.bash && "
                "source /home/ubuntu/ws/install/setup.bash 2>/dev/null || true; "
                "ros2 service list -t | grep -E '/kb/(revise|query)' || true"
            ),
        ],
        timeout=10,
        check=False,
    )
    revise_output = call_ros_service(
        container,
        "/kb/revise",
        "kb_msgs/srv/Revise",
        revise_request,
        timeout_sec=20,
    )
    time.sleep(1.0)
    query_output = call_ros_service(
        container,
        "/kb/query",
        "kb_msgs/srv/Query",
        query_request,
        timeout_sec=20,
    )
    return {
        "object_id": injection.object_id,
        "service_probe": service_probe,
        "revise_output": revise_output,
        "query_output": query_output,
    }


def call_ros_service(
    container: str,
    service_name: str,
    service_type: str,
    request_yaml: str,
    *,
    timeout_sec: int,
) -> str:
    script = f"""
set -e
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
cat >/tmp/nao_questionnaire_service_request.yaml
timeout {timeout_sec} ros2 service call --stdin {service_name} {service_type} < /tmp/nao_questionnaire_service_request.yaml
"""
    return run(
        ["docker", "exec", "-i", container, "bash", "-lc", script],
        timeout=timeout_sec + 5,
        check=False,
        input_text=request_yaml,
    )


def call_chatbot_turn(
    container: str,
    text: str,
    sequence: int,
    *,
    history: list[dict[str, str]],
) -> str:
    escaped_text = text.replace("'", "''")
    uuid_tail = max(1, min(255, sequence))
    history_items = list(history) + [{"speaker": VOICE_ID, "text": text}]
    history_yaml = "\n".join(
        "- speaker: \"%s\"\n  text: '%s'\n  timestamp: 0.0"
        % (
            item["speaker"].replace('"', '\\"'),
            item["text"].replace("'", "''"),
        )
        for item in history_items
    )
    request = f"""
dialogue_id:
  uuid: [{uuid_tail}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {uuid_tail}]
role:
  name: "__default__"
  configuration: "{{}}"
locale: "en_US"
summary: ""
history:
{history_yaml}
"""
    script = """
set -e
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
cat >/tmp/nao_questionnaire_request.yaml
timeout 90 ros2 service call --stdin /chatbot_llm/dialogue_interaction chatbot_msgs/srv/DialogueInteraction < /tmp/nao_questionnaire_request.yaml
"""
    return run(
        ["docker", "exec", "-i", container, "bash", "-lc", script],
        timeout=100,
        check=False,
        input_text=request,
    )


def extract_service_response(output: str) -> str:
    match = re.search(r"response=(['\"])(.*?)\1, intents=", output, flags=re.DOTALL)
    if not match:
        return ""
    return match.group(2).encode("utf-8").decode("unicode_escape").strip()


def sample_topics(container: str) -> dict[str, str]:
    samples = {}
    for topic in TOPICS_TO_SAMPLE:
        script = f"""
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
timeout {TOPIC_SAMPLE_TIMEOUT_SEC} ros2 topic echo --once {topic} 2>/dev/null || true
"""
        samples[topic] = run(
            ["docker", "exec", container, "bash", "-lc", script],
            timeout=TOPIC_SAMPLE_TIMEOUT_SEC + 3,
            check=False,
        )
    return samples


def recent_logs(container: str, since_sec: int) -> str:
    output = run(
        ["docker", "logs", "--since", f"{max(1, since_sec)}s", container],
        timeout=15,
        check=False,
    )
    interesting = []
    markers = (
        "SPEECH INPUT",
        "CHATBOT",
        "ROUTE_RESOLVED",
        "PLANNER_REQUEST",
        "planner_llm",
        "execution_feedback",
        "report_result",
        "DEBUG_SPEECH",
        "ROBOT OUTPUT",
        "GROUNDED_CONTEXT",
        "ERROR",
        "WARN",
    )
    for line in output.splitlines():
        if any(marker in line for marker in markers):
            interesting.append(line)
    return "\n".join(interesting[-260:])


def run(
    cmd: list[str],
    *,
    timeout: float,
    check: bool = True,
    input_text: str | None = None,
) -> str:
    try:
        completed = subprocess.run(
            cmd,
            input=input_text,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as err:
        partial = err.stdout or ""
        if isinstance(partial, bytes):
            partial = partial.decode(errors="replace")
        return "%s\n[TIMEOUT after %.1fs]" % (str(partial).strip(), timeout)
    if check and completed.returncode != 0:
        raise RuntimeError(
            "command failed (%s):\n%s" % (" ".join(cmd), completed.stdout)
        )
    return completed.stdout.strip()


if __name__ == "__main__":
    raise SystemExit(main())
