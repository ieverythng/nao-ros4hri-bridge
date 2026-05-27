#!/usr/bin/env python3
"""Run live fake-skill scenario probes in a ROS container and write a trace report."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
ARTIFACTS_DIR = REPO_ROOT / "docs" / "artifacts"
DEFAULT_CONTAINER = "nao_ros2"
DEFAULT_TOPIC = "/planner/request"


@dataclass(frozen=True)
class ProbeCase:
    case_id: str
    mode: str
    scenario_id: str
    goal_text: str
    normalized_intents: tuple[str, ...]
    scene_targets: tuple[str, ...]
    random_failure_prob: float = 0.5
    wait_sec: float = 11.0


CASES: tuple[ProbeCase, ...] = (
    ProbeCase(
        case_id="success_default",
        mode="scenario",
        scenario_id="",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
    ),
    ProbeCase(
        case_id="deterministic_ambiguous",
        mode="scenario",
        scenario_id="ambiguous_cup",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
    ),
    ProbeCase(
        case_id="deterministic_path_blocked",
        mode="scenario",
        scenario_id="path_blocked",
        goal_text="navigate to the cup",
        normalized_intents=("move_to", "navigate_to"),
        scene_targets=("cup",),
    ),
    ProbeCase(
        case_id="random_seeded_stress",
        mode="random_seeded",
        scenario_id="",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
        random_failure_prob=0.95,
    ),
    ProbeCase(
        case_id="all_fail_always",
        mode="always_fail",
        scenario_id="",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
    ),
)


def _run(
    cmd: list[str],
    *,
    check: bool = True,
    timeout_sec: float | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        text=True,
        capture_output=True,
        check=check,
        timeout=timeout_sec,
    )


def _docker_exec(
    container: str,
    bash_cmd: str,
    *,
    check: bool = True,
    timeout_sec: float | None = None,
) -> subprocess.CompletedProcess[str]:
    cmd = ["docker", "exec", container, "bash", "-lc", bash_cmd]
    return _run(cmd, check=check, timeout_sec=timeout_sec)


def _docker_exec_env(
    container: str,
    bash_cmd: str,
    env_vars: dict[str, str],
    *,
    check: bool = True,
    timeout_sec: float | None = None,
) -> subprocess.CompletedProcess[str]:
    cmd = ["docker", "exec"]
    for key, value in env_vars.items():
        cmd.extend(["-e", f"{key}={value}"])
    cmd.extend([container, "bash", "-lc", bash_cmd])
    return _run(cmd, check=check, timeout_sec=timeout_sec)


def _ros_preamble() -> str:
    return "source /opt/ros/jazzy/setup.bash && source /home/ubuntu/ws/install/setup.bash"


def _start_trace_viewer(container: str, trace_dir: str) -> int:
    bash_cmd = (
        f"{_ros_preamble()} && mkdir -p {shlex.quote(trace_dir)} && "
        "python3 - <<'PY'\n"
        "import subprocess\n"
        "from pathlib import Path\n"
        f"trace_dir = Path({trace_dir!r})\n"
        "log_path = trace_dir / 'trace_node.log'\n"
        "cmd = [\n"
        "    'ros2', 'run', 'interaction_trace_viewer', 'trace_node', '--ros-args',\n"
        "    '-p', 'compact_mode:=false',\n"
        "    '-p', 'include_raw_payloads:=false',\n"
        "    '-p', 'write_jsonl:=true',\n"
        f"    '-p', 'jsonl_output_dir:={trace_dir}',\n"
        "    '-p', 'write_html_on_shutdown:=false',\n"
        "    '-p', 'enable_scene_summary_channel:=false',\n"
        "    '-p', 'rosout_min_level:=warn',\n"
        "    '-p', 'include_channels_csv:=planner/request,intents,planner/execution_feedback,planner/dialogue_act,nao_orchestrator/planner_dialogue_act,chatbot_llm/turn_trace,fake_skills/events,world_model/enriched_snapshot,world_model/enriched_text',\n"
        "    '-p', 'include_event_types_csv:=planner_request,planner_output,execution_feedback,planner_dialogue_act,chatbot_turn_trace,skill_result,kb_snapshot',\n"
        "]\n"
        "with log_path.open('a', encoding='utf-8') as log_handle:\n"
        "    proc = subprocess.Popen(cmd, stdout=log_handle, stderr=log_handle, start_new_session=True)\n"
        "print(proc.pid)\n"
        "PY"
    )
    result = _docker_exec(container, bash_cmd)
    stdout = result.stdout.strip()
    if not stdout:
        raise RuntimeError("Failed to start trace viewer; empty PID output.")
    return int(stdout.splitlines()[-1].strip())


def _stop_process(container: str, pid: int) -> None:
    _docker_exec(container, f"kill -TERM -{pid} >/dev/null 2>&1 || kill {pid} >/dev/null 2>&1 || true", check=False)


def _sync_files_to_container(container: str) -> None:
    mappings = (
        (
            REPO_ROOT / "src" / "nao_orchestrator" / "nao_orchestrator" / "orchestrator.py",
            "/home/ubuntu/ws/src/nao_orchestrator/nao_orchestrator/orchestrator.py",
        ),
        (
            REPO_ROOT / "src" / "nao_chatbot" / "nao_chatbot" / "stack_launch.py",
            "/home/ubuntu/ws/src/nao_chatbot/nao_chatbot/stack_launch.py",
        ),
        (
            REPO_ROOT / "src" / "interaction_trace_viewer" / "interaction_trace_viewer" / "payload_normalizer.py",
            "/home/ubuntu/ws/src/interaction_trace_viewer/interaction_trace_viewer/payload_normalizer.py",
        ),
        (
            REPO_ROOT / "src" / "interaction_trace_viewer" / "interaction_trace_viewer" / "trace_node.py",
            "/home/ubuntu/ws/src/interaction_trace_viewer/interaction_trace_viewer/trace_node.py",
        ),
        (
            REPO_ROOT / "src" / "fake_skills" / "config" / "fake_skill_scenarios.yaml",
            "/home/ubuntu/ws/src/fake_skills/config/fake_skill_scenarios.yaml",
        ),
        (
            REPO_ROOT / "scripts" / "fake_skill_scenario_menu.sh",
            "/home/ubuntu/ws/scripts/fake_skill_scenario_menu.sh",
        ),
    )
    for source_path, target_path in mappings:
        _docker_exec(
            container,
            f"mkdir -p {shlex.quote(str(Path(target_path).parent))}",
        )
        _run(["docker", "cp", str(source_path), f"{container}:{target_path}"])


def _build_container_workspace(container: str) -> None:
    build_cmd = (
        f"{_ros_preamble()} && cd /home/ubuntu/ws && "
        "colcon build --packages-select nao_orchestrator interaction_trace_viewer fake_skills nao_chatbot"
    )
    _docker_exec(container, build_cmd)


def _relaunch_stack(container: str) -> None:
    relaunch_cmd = (
        "pkill -f \"ros2 launch nao_chatbot nao_chatbot_sim.launch.py\" >/dev/null 2>&1 || true && "
        f"{_ros_preamble()} && python3 - <<'PY'\n"
        "import subprocess\n"
        "from pathlib import Path\n"
        "log_path = Path('/tmp/codex_stack_relaunch.log')\n"
        "cmd = [\n"
        "    'ros2', 'launch', 'nao_chatbot', 'nao_chatbot_sim.launch.py',\n"
        "    'sim_use_laptop_tts:=false',\n"
        "    'posture_bridge_wake_up_on_connect:=true',\n"
        "    'start_naoqi_driver:=true',\n"
        "    'start_object_detection:=true',\n"
        "    'start_scene_grounding:=true',\n"
        "    'object_detection_backend:=emorobcare_cv',\n"
        "    'nao_ip:=172.26.112.130',\n"
        "    'network_interface:=wlp1s0',\n"
        "    'start_planner_llm:=true',\n"
        "    'chatbot_planner_mode_enabled:=true',\n"
        "    'chatbot_server_url:=http://10.7.138.215:8004/v1/chat/completions',\n"
        "    'planner_llm_provider:=openai_compatible',\n"
        "    'planner_llm_base_url:=http://10.7.138.215:8004',\n"
        "    'planner_llm_model:=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ',\n"
        "    'planner_llm_api_key_env:=VLLM_API_KEY',\n"
        "    'start_fake_skills:=true',\n"
        "    'start_interaction_trace_viewer:=true',\n"
        "    'start_demo_log_window:=true',\n"
        "]\n"
        "with log_path.open('a', encoding='utf-8') as log_handle:\n"
        "    proc = subprocess.Popen(cmd, stdout=log_handle, stderr=log_handle, start_new_session=True)\n"
        "print(proc.pid)\n"
        "PY"
    )
    _docker_exec(container, relaunch_cmd, check=False)
    time.sleep(18.0)


def _set_fake_skill_policy(container: str, case: ProbeCase) -> None:
    scenario_literal = case.scenario_id if case.scenario_id else '""'
    bash_cmd = (
        f"{_ros_preamble()} && "
        f"timeout 8 ros2 param set /fake_skill_server global_mode {shlex.quote(case.mode)} >/dev/null && "
        f"timeout 8 ros2 param set /fake_skill_server random_failure_prob {case.random_failure_prob:.2f} >/dev/null && "
        f"timeout 8 ros2 param set /fake_skill_server active_scenario_id {scenario_literal} >/dev/null"
    )
    _docker_exec(container, bash_cmd, timeout_sec=20.0)


def _publish_planner_request(
    container: str,
    case: ProbeCase,
    *,
    sequence: int,
    planner_request_topic: str,
) -> tuple[str, str]:
    turn_id = f"probe_turn_{sequence:02d}_{case.case_id}"
    goal_id = f"goal_probe_{sequence:02d}_{case.case_id}"
    goal_token = f"{goal_id}:{turn_id}"
    payload = {
        "request_id": turn_id,
        "goal_id": goal_id,
        "goal_token": goal_token,
        "request_kind": "new_goal",
        "goal_text": case.goal_text,
        "normalized_intents": list(case.normalized_intents),
        "ack_text": f"I will {case.goal_text}.",
        "ack_mode": "say",
        "scene_targets": list(case.scene_targets),
        "dialogue_context": [f"probe case: {case.case_id}"],
        "grounded_context": {
            "knowledge_snapshot": {"summary_text": "probe context"},
            "scene_summary": {},
            "world_model_snapshot": {},
            "world_model_text": "",
        },
        "planner_mode": "default",
        "interaction_mode": "speech",
        "dialogue_turn_id": turn_id,
    }
    intent_publish_cmd = (
        f"{_ros_preamble()} && python3 - <<'PY'\n"
        "import json\n"
        "import os\n"
        "import rclpy\n"
        "from hri_actions_msgs.msg import Intent\n"
        "from rclpy.node import Node\n"
        "topic = os.environ['PROBE_TOPIC']\n"
        "payload = json.loads(os.environ['PROBE_PAYLOAD_JSON'])\n"
        "rclpy.init()\n"
        "node = Node('scenario_probe_publisher')\n"
        "pub = node.create_publisher(Intent, topic, 1)\n"
        "deadline = node.get_clock().now().nanoseconds / 1e9 + 3.0\n"
        "while pub.get_subscription_count() == 0 and (node.get_clock().now().nanoseconds / 1e9) < deadline:\n"
        "    rclpy.spin_once(node, timeout_sec=0.05)\n"
        "msg = Intent()\n"
        "msg.intent = 'planner_request'\n"
        "msg.source = Intent.REMOTE_SUPERVISOR\n"
        "msg.modality = Intent.MODALITY_SPEECH\n"
        "msg.priority = 128\n"
        "msg.confidence = 0.95\n"
        "msg.data = json.dumps(payload, separators=(',', ':'))\n"
        "pub.publish(msg)\n"
        "rclpy.spin_once(node, timeout_sec=0.1)\n"
        "node.destroy_node()\n"
        "rclpy.shutdown()\n"
        "PY"
    )
    _docker_exec_env(
        container,
        intent_publish_cmd,
        {
            "PROBE_TOPIC": planner_request_topic,
            "PROBE_PAYLOAD_JSON": json.dumps(payload, separators=(",", ":")),
        },
        timeout_sec=30.0,
    )
    return turn_id, goal_id


def _copy_trace_jsonl(container: str, trace_dir: str, output_dir: Path) -> Path:
    ls_result = _docker_exec(
        container,
        f"ls -1 {shlex.quote(trace_dir)}/*.jsonl 2>/dev/null | tail -n 1",
    )
    remote_jsonl = ls_result.stdout.strip()
    if not remote_jsonl:
        raise RuntimeError(f"No JSONL traces found in {trace_dir}")
    local_jsonl = output_dir / Path(remote_jsonl).name
    _run(["docker", "cp", f"{container}:{remote_jsonl}", str(local_jsonl)])
    return local_jsonl


def _load_events(path: Path) -> list[dict]:
    events: list[dict] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        events.append(json.loads(line))
    return events


def _case_events(events: list[dict], goal_id: str) -> list[dict]:
    matched: list[dict] = []
    for event in events:
        trace_id = str(event.get("trace_id", "")).strip()
        payload = event.get("payload", {})
        if trace_id == goal_id:
            matched.append(event)
            continue
        if isinstance(payload, dict) and str(payload.get("goal_id", "")).strip() == goal_id:
            matched.append(event)
    return matched


def _first_payload(events: list[dict], *, channel: str) -> dict | None:
    for event in events:
        if str(event.get("channel", "")).strip() == channel:
            payload = event.get("payload")
            if isinstance(payload, dict):
                return payload
    return None


def _format_event_line(event: dict) -> str:
    ts = float(event.get("timestamp", 0.0) or 0.0)
    channel = str(event.get("channel", "")).strip()
    event_type = str(event.get("event_type", "")).strip()
    summary = str(event.get("summary", "")).strip()
    return f"- `{ts:.3f}` `{event_type}` `{channel}`: {summary}"


def _write_report(
    *,
    report_path: Path,
    trace_jsonl_path: Path,
    trace_dir: str,
    planner_request_topic: str,
    cases: list[tuple[ProbeCase, str, str]],
    all_events: list[dict],
) -> None:
    lines: list[str] = []
    now_utc = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    lines.append("# Live Fake-Skill Scenario Probe Report")
    lines.append("")
    lines.append(f"- Generated: {now_utc}")
    lines.append(f"- Container: `{DEFAULT_CONTAINER}`")
    lines.append(f"- Planner request topic: `{planner_request_topic}`")
    lines.append(f"- Trace dir (container): `{trace_dir}`")
    lines.append(f"- Trace JSONL (local copy): `{trace_jsonl_path}`")
    lines.append("")

    for case, turn_id, goal_id in cases:
        case_events = _case_events(all_events, goal_id)
        lines.append(f"## Case `{case.case_id}`")
        lines.append("")
        lines.append(f"- `mode`: `{case.mode}`")
        lines.append(f"- `scenario_id`: `{case.scenario_id or '<none>'}`")
        lines.append(f"- `goal_id`: `{goal_id}`")
        lines.append(f"- `turn_id`: `{turn_id}`")
        lines.append(f"- `goal_text`: `{case.goal_text}`")
        lines.append(f"- `event_count`: `{len(case_events)}`")

        plan_payload = _first_payload(case_events, channel="/intents")
        feedback_events = [e for e in case_events if e.get("channel") == "/planner/execution_feedback"]
        final_feedback = feedback_events[-1]["payload"] if feedback_events else {}
        dialogue_payload = _first_payload(case_events, channel="/planner/dialogue_act")

        lines.append("")
        lines.append("### Flow")
        lines.append("")
        if case_events:
            for event in case_events:
                lines.append(_format_event_line(event))
        else:
            lines.append("- No events matched this goal id in trace file.")

        lines.append("")
        lines.append("### JSON Excerpts")
        lines.append("")
        if isinstance(plan_payload, dict):
            lines.append("`planner_output` payload:")
            lines.append("```json")
            lines.append(json.dumps(plan_payload, indent=2, ensure_ascii=True))
            lines.append("```")
        if isinstance(final_feedback, dict) and final_feedback:
            lines.append("`final_execution_feedback` payload:")
            lines.append("```json")
            lines.append(json.dumps(final_feedback, indent=2, ensure_ascii=True))
            lines.append("```")
        if isinstance(dialogue_payload, dict) and dialogue_payload:
            lines.append("`planner_dialogue_act` payload:")
            lines.append("```json")
            lines.append(json.dumps(dialogue_payload, indent=2, ensure_ascii=True))
            lines.append("```")

        lines.append("")

    report_path.write_text("\n".join(lines).strip() + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run live fake-skill scenario probes in container.")
    parser.add_argument("--container", default=os.environ.get("PROBE_CONTAINER", DEFAULT_CONTAINER))
    parser.add_argument("--topic", default=os.environ.get("PROBE_TOPIC", DEFAULT_TOPIC))
    parser.add_argument("--skip-sync-rebuild", action="store_true")
    parser.add_argument("--skip-relaunch", action="store_true")
    parsed = parser.parse_args()

    container = parsed.container
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    trace_dir = f"/tmp/codex_fake_skill_probe_{stamp}"
    ARTIFACTS_DIR.mkdir(parents=True, exist_ok=True)
    report_path = ARTIFACTS_DIR / f"fake_skill_scenario_probe_report_{stamp}.md"
    local_trace_dir = ARTIFACTS_DIR / "trace_captures"
    local_trace_dir.mkdir(parents=True, exist_ok=True)

    started_cases: list[tuple[ProbeCase, str, str]] = []
    trace_pid = 0
    try:
        if not parsed.skip_sync_rebuild:
            _sync_files_to_container(container)
            _build_container_workspace(container)
        if not parsed.skip_relaunch:
            _relaunch_stack(container)
        trace_pid = _start_trace_viewer(container, trace_dir)
        time.sleep(2.0)

        for idx, case in enumerate(CASES, start=1):
            _set_fake_skill_policy(container, case)
            turn_id, goal_id = _publish_planner_request(
                container,
                case,
                sequence=idx,
                planner_request_topic=parsed.topic,
            )
            started_cases.append((case, turn_id, goal_id))
            time.sleep(case.wait_sec)

        time.sleep(2.0)
    finally:
        if trace_pid > 0:
            _stop_process(container, trace_pid)
            time.sleep(0.5)

    trace_jsonl_path = _copy_trace_jsonl(container, trace_dir, local_trace_dir)
    events = _load_events(trace_jsonl_path)
    _write_report(
        report_path=report_path,
        trace_jsonl_path=trace_jsonl_path,
        trace_dir=trace_dir,
        planner_request_topic=parsed.topic,
        cases=started_cases,
        all_events=events,
    )

    print(str(report_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
