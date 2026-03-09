from __future__ import annotations

import argparse
import select
import sys
import time
from typing import NamedTuple

try:
    import rclpy
    from std_msgs.msg import Bool
except ModuleNotFoundError:  # pragma: no cover - exercised only outside ROS env
    rclpy = None
    Bool = None


INTERACTIVE_HELP = (
    "Controls: [space]/t toggle, o open, c close, q quit, ? help"
)


class KeyAction(NamedTuple):
    state: bool
    should_exit: bool
    should_print_help: bool


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish push-to-talk Bool commands for asr_vosk."
    )
    parser.add_argument(
        "--topic",
        default="/asr_vosk/push_to_talk",
        help="Bool topic consumed by asr_vosk push-to-talk mode.",
    )
    parser.add_argument(
        "--node-name",
        default="asr_push_to_talk_cli",
        help="ROS node name for this operator utility.",
    )
    parser.add_argument(
        "--sleep-after-publish",
        type=float,
        default=0.15,
        help="Seconds to wait after one-shot publishes before shutdown.",
    )
    parser.add_argument(
        "--close-on-exit",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="In interactive mode, publish false when the utility exits.",
    )

    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--open",
        action="store_true",
        help="Publish true once and exit.",
    )
    mode_group.add_argument(
        "--close",
        action="store_true",
        help="Publish false once and exit.",
    )
    mode_group.add_argument(
        "--pulse",
        type=float,
        metavar="SECONDS",
        help="Publish true, wait the given seconds, then publish false and exit.",
    )

    return parser


def interpret_key(current_state: bool, key: str) -> KeyAction:
    if key in (" ", "t", "T"):
        return KeyAction(not current_state, False, False)
    if key in ("o", "O", "+"):
        return KeyAction(True, False, False)
    if key in ("c", "C", "-"):
        return KeyAction(False, False, False)
    if key in ("?", "h", "H"):
        return KeyAction(current_state, False, True)
    if key in ("q", "Q", "\x03"):
        return KeyAction(current_state, True, False)
    return KeyAction(current_state, False, False)


def _require_ros() -> None:
    if rclpy is None or Bool is None:
        raise RuntimeError(
            "rclpy/std_msgs are required to run asr_push_to_talk_cli inside a ROS 2 environment."
        )


def _publish_state(publisher, enabled: bool) -> None:
    msg = Bool()
    msg.data = bool(enabled)
    for _ in range(3):
        publisher.publish(msg)
        time.sleep(0.03)


def _print_state(enabled: bool) -> None:
    sys.stdout.write(f"ASR listening {'ON' if enabled else 'OFF'}\n")
    sys.stdout.flush()


def _run_interactive(node, publisher, close_on_exit: bool) -> int:
    import termios
    import tty

    if not sys.stdin.isatty():
        sys.stderr.write(
            "Interactive mode requires a TTY. Use --open, --close, or --pulse from non-interactive shells.\n"
        )
        return 2

    state = False
    _publish_state(publisher, state)
    sys.stdout.write(INTERACTIVE_HELP + "\n")
    _print_state(state)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            rclpy.spin_once(node, timeout_sec=0.0)
            if not ready:
                continue

            key = sys.stdin.read(1)
            action = interpret_key(state, key)
            if action.should_print_help:
                sys.stdout.write(INTERACTIVE_HELP + "\n")
                sys.stdout.flush()
                continue
            if action.state != state:
                state = action.state
                _publish_state(publisher, state)
                _print_state(state)
            if action.should_exit:
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if close_on_exit and state:
            _publish_state(publisher, False)
            _print_state(False)

    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    _require_ros()
    rclpy.init(args=None)
    node = rclpy.create_node(args.node_name)
    publisher = node.create_publisher(Bool, args.topic, 1)

    try:
        if args.open:
            _publish_state(publisher, True)
            _print_state(True)
            time.sleep(max(0.0, float(args.sleep_after_publish)))
            return 0

        if args.close:
            _publish_state(publisher, False)
            _print_state(False)
            time.sleep(max(0.0, float(args.sleep_after_publish)))
            return 0

        if args.pulse is not None:
            duration = max(0.0, float(args.pulse))
            _publish_state(publisher, True)
            _print_state(True)
            time.sleep(duration)
            _publish_state(publisher, False)
            _print_state(False)
            time.sleep(max(0.0, float(args.sleep_after_publish)))
            return 0

        return _run_interactive(node, publisher, args.close_on_exit)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
