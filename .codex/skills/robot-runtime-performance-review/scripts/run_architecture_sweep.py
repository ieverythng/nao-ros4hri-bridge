#!/usr/bin/env python3
"""Run focused architecture-seam checks for the NAO ROS4HRI runtime stack."""

from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_CONTAINER = "nao_ros2"
QUESTIONNAIRE_SCRIPT = Path(__file__).with_name("run_active_questionnaire.py")
MARKER_ID = "codex_arch_marker"
ROS_CLI_PREAMBLE = """
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
""".strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--container", default=DEFAULT_CONTAINER)
    parser.add_argument("--out", default="/tmp/nao_architecture_sweep.json")
    parser.add_argument("--include-maximal", action="store_true")
    args = parser.parse_args()

    questionnaire = _load_questionnaire_module()
    started_at = time.time()
    results: dict[str, object] = {
        "container": args.container,
        "started_at_unix_sec": started_at,
        "scope": "architecture_service_and_direct_action_sweep",
        "runtime_state": collect_runtime_state(args.container),
    }

    cleanup_marker(args.container, MARKER_ID)
    results["kb_mutation"] = run_kb_mutation_sweep(args.container, questionnaire)
    results["fake_skill_guard"] = run_fake_guard_sweep(args.container)
    if args.include_maximal:
        results["maximal_person_delivery"] = run_maximal_person_delivery(
            args.container,
            questionnaire,
        )

    results["finished_at_unix_sec"] = time.time()
    Path(args.out).write_text(json.dumps(results, indent=2, sort_keys=True), encoding="utf-8")
    print(args.out)
    return 0


def _load_questionnaire_module():
    spec = importlib.util.spec_from_file_location("runtime_questionnaire", QUESTIONNAIRE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("Unable to load %s" % QUESTIONNAIRE_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules["runtime_questionnaire"] = module
    spec.loader.exec_module(module)
    return module


def collect_runtime_state(container: str) -> dict[str, str]:
    commands = {
        "docker_ps": ["docker", "ps", "--format", "{{.Names}} {{.Status}}"],
        "core_nodes": docker_ros_cmd(
            "ros2 node list | sort | grep -E 'chatbot_llm|dialogue_manager|planner_llm|nao_orchestrator|fake_skill_server|knowledge_core|scan_skill|report_result' || true"
        ),
        "dialogue_state": docker_ros_cmd("timeout 5 ros2 lifecycle get /dialogue_manager || true"),
        "orchestrator_state": docker_ros_cmd("timeout 5 ros2 lifecycle get /nao_orchestrator || true"),
        "speech_subscription": docker_ros_cmd(
            "timeout 8 ros2 topic info -v /nao_chatbot/humans/voices/anonymous_speaker/speech || true"
        ),
        "fake_actions": docker_ros_cmd(
            "timeout 8 ros2 action list | grep -E '/skill/fake/(navigate_to|find_object|pick_object|place_object|bring_object|look_at)' || true"
        ),
    }
    state = {}
    for key, cmd in commands.items():
        state[key] = run(cmd if key == "docker_ps" else ["docker", "exec", container, "bash", "-lc", cmd], timeout=15)
    return state


def run_kb_mutation_sweep(container: str, questionnaire) -> dict[str, object]:
    history: list[dict[str, str]] = []
    turns = []
    for sequence, text in (
        (
            1,
            "Add this to your knowledge base: %s rdf:type Cube, %s dbp:name NOVA, and %s dbp:color green."
            % (MARKER_ID, MARKER_ID, MARKER_ID),
        ),
        (
            2,
            "Update %s so its dbp:color is blue in your knowledge base." % MARKER_ID,
        ),
        (
            3,
            "What do you remember about %s?" % MARKER_ID,
        ),
        (
            4,
            "Remove %s from your knowledge base." % MARKER_ID,
        ),
    ):
        output = questionnaire.call_chatbot_turn(
            container,
            text,
            sequence,
            voice_id="codex_architecture_sweep",
            history=history,
        )
        time.sleep(12 if sequence != 3 else 4)
        turns.append(
            {
                "sequence": sequence,
                "text": text,
                "service_output": output,
                "query_after": query_subject(container, MARKER_ID),
            }
        )
    return {
        "mode": "chatbot_service_to_planner_orchestrator_kb",
        "marker_id": MARKER_ID,
        "turns": turns,
    }


def run_fake_guard_sweep(container: str) -> dict[str, str]:
    seed_output = revise(
        container,
        "update",
        (
            "codex_arch_cup rdf:type Cup",
            "codex_arch_cup dbp:name ARCH_CUP",
            "codex_arch_person rdf:type Human",
            "codex_arch_person dbp:name ALEX",
        ),
    )
    missing_recipient = send_fake_action(
        container,
        "/skill/fake/bring_object",
        "{target: codex_arch_cup, target_kind: object, result_mode: success}",
    )
    grounded_recipient = send_fake_action(
        container,
        "/skill/fake/bring_object",
        (
            "{target: codex_arch_cup, target_kind: object, result_mode: success, "
            "evidence_policy: '{\"recipient_id\":\"codex_arch_person\"}'}"
        ),
    )
    return {
        "seed_output": seed_output,
        "missing_recipient": missing_recipient,
        "grounded_recipient": grounded_recipient,
    }


def run_maximal_person_delivery(container: str, questionnaire) -> dict[str, str]:
    case = next(
        item
        for item in questionnaire.COMPOSITE_CASES
        if item.name == "maximal_kitchen_cup_to_person"
    )
    setup = questionnaire.inject_kb_probe(container, case.setup)
    output = questionnaire.call_chatbot_turn(
        container,
        case.text,
        5,
        voice_id="codex_architecture_maximal",
        history=[],
    )
    time.sleep(45)
    feedback = run(
        ["docker", "exec", container, "bash", "-lc", "tail -n 200 /tmp/nao_runtime_review_launch.log"],
        timeout=15,
    )
    return {"setup": setup, "service_output": output, "recent_launch_log": feedback}


def cleanup_marker(container: str, subject: str) -> None:
    rows = query_subject_json(container, subject)
    statements = [
        "%s %s %s" % (subject, row.get("predicate", ""), row.get("object", ""))
        for row in rows
        if row.get("predicate") and row.get("object")
    ]
    if statements:
        revise(container, "retract", statements)


def query_subject(container: str, subject: str) -> str:
    request = """
patterns:
  - "%s ?predicate ?object"
vars:
  - "?predicate"
  - "?object"
models:
  - default
""" % subject
    return call_ros_service(container, "/kb/query", "kb_msgs/srv/Query", request, timeout_sec=20)


def query_subject_json(container: str, subject: str) -> list[dict]:
    output = query_subject(container, subject)
    marker = "json='"
    start = output.find(marker)
    if start < 0:
        return []
    start += len(marker)
    end = output.find("'", start)
    if end < 0:
        return []
    try:
        return json.loads(output[start:end])
    except json.JSONDecodeError:
        return []


def revise(container: str, method: str, statements) -> str:
    if isinstance(statements, str):
        statements = [statements]
    statements_yaml = "\n".join("  - '%s'" % item for item in statements)
    request = """
method: %s
statements:
%s
models:
  - default
lifespan:
  sec: 300
  nanosec: 0
""" % (method, statements_yaml)
    return call_ros_service(container, "/kb/revise", "kb_msgs/srv/Revise", request, timeout_sec=20)


def send_fake_action(container: str, action_name: str, goal: str) -> str:
    command = docker_ros_cmd(
        "timeout 25 ros2 action send_goal %s nao_skills/action/ScanScene \"%s\" --feedback"
        % (action_name, goal.replace('"', '\\"'))
    )
    return run(["docker", "exec", container, "bash", "-lc", command], timeout=30)


def call_ros_service(
    container: str,
    service_name: str,
    service_type: str,
    request_yaml: str,
    *,
    timeout_sec: int,
) -> str:
    command = docker_ros_cmd(
        "cat >/tmp/arch_sweep_request.yaml && timeout %d ros2 service call --stdin %s %s < /tmp/arch_sweep_request.yaml"
        % (timeout_sec, service_name, service_type)
    )
    return run(
        ["docker", "exec", "-i", container, "bash", "-lc", command],
        timeout=timeout_sec + 5,
        input_text=request_yaml,
    )


def docker_ros_cmd(command: str) -> str:
    return "%s\n%s" % (ROS_CLI_PREAMBLE, command)


def run(cmd: list[str], *, timeout: float, input_text: str | None = None) -> str:
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
        return "%s\n[TIMEOUT after %.1fs]" % (partial.strip(), timeout)
    return completed.stdout.strip()


if __name__ == "__main__":
    raise SystemExit(main())
