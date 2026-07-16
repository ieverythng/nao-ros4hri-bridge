"""Preload deterministic KnowledgeCore environment fixtures."""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from kb_skills.mutation_client import KnowledgeCoreMutationClient
from nao_chatbot.environment_fixtures import validate_environment_fixture
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


DEFAULT_FIXTURE_FILE = "preloaded_environments.json"


class EnvironmentPreloader(Node):
    """One-shot node that writes named semantic fixtures through /kb/revise."""

    def __init__(self) -> None:
        super().__init__("preloaded_environment")
        self.declare_parameter("environment_ids", "")
        self.declare_parameter("fixture_path", "")
        self.declare_parameter("kb_revise_service_name", "/kb/revise")
        self.declare_parameter("kb_revise_timeout_sec", 10.0)
        self.declare_parameter("kb_lifespan_sec", 1800.0)
        self.declare_parameter("kb_models", "")

        self._mutation_client = KnowledgeCoreMutationClient(
            node=self,
            service_name=str(self.get_parameter("kb_revise_service_name").value),
            timeout_sec=float(self.get_parameter("kb_revise_timeout_sec").value),
        )

    def run(self) -> int:
        environment_ids = _parse_csv(self.get_parameter("environment_ids").value)
        if not environment_ids:
            self.get_logger().info("No preloaded environment ids requested.")
            return 0

        fixture_path = _resolve_fixture_path(str(self.get_parameter("fixture_path").value))
        fixtures = _load_fixtures(fixture_path)
        models = _parse_csv(self.get_parameter("kb_models").value)
        lifespan_sec = float(self.get_parameter("kb_lifespan_sec").value)
        if not self._wait_for_mutation_service():
            self.get_logger().error(
                "KnowledgeCore revise service did not become ready for environment preload."
            )
            return 2

        failures = []
        for environment_id in environment_ids:
            fixture = fixtures.get(environment_id)
            if fixture is None:
                failures.append("unknown fixture '%s'" % environment_id)
                continue
            validation_errors = validate_environment_fixture(environment_id, fixture)
            if validation_errors:
                failures.append(
                    "%s: %s" % (environment_id, '; '.join(validation_errors))
                )
                continue
            statements = [
                str(item).strip()
                for item in fixture.get("statements", [])
                if str(item).strip()
            ]
            result = self._mutation_client.add_facts(
                statements,
                models=models,
                lifespan_sec=lifespan_sec,
            )
            if not result.success:
                failures.append("%s: %s" % (environment_id, result.error_msg))
                continue
            self.get_logger().info(
                "Preloaded environment '%s' with %d KnowledgeCore statements from %s"
                % (environment_id, result.statement_count, fixture_path)
            )

        if failures:
            for failure in failures:
                self.get_logger().error("Preloaded environment failed: %s" % failure)
            return 2
        return 0

    def _wait_for_mutation_service(self) -> bool:
        client = getattr(self._mutation_client, "_client", None)
        if client is None:
            return False
        deadline = time.time() + float(self.get_parameter("kb_revise_timeout_sec").value)
        while time.time() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                return True
        return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--environment-ids", default="")
    parser.add_argument("--fixture-path", default="")
    parser.add_argument("--kb-lifespan-sec", default="1800.0")
    parser.add_argument("--kb-models", default="")
    parsed, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = EnvironmentPreloader()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.set_parameters(
            [
                Parameter(
                    "environment_ids",
                    Parameter.Type.STRING,
                    parsed.environment_ids,
                ),
                Parameter(
                    "fixture_path",
                    Parameter.Type.STRING,
                    parsed.fixture_path,
                ),
                Parameter(
                    "kb_lifespan_sec",
                    Parameter.Type.DOUBLE,
                    float(parsed.kb_lifespan_sec),
                ),
                Parameter(
                    "kb_models",
                    Parameter.Type.STRING,
                    parsed.kb_models,
                ),
            ]
        )
        return node.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def _load_fixtures(path: Path) -> dict[str, dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    fixtures = payload.get("environments", {}) if isinstance(payload, dict) else {}
    if not isinstance(fixtures, dict):
        raise RuntimeError("Fixture file does not contain an 'environments' object: %s" % path)
    return {str(key).strip(): value for key, value in fixtures.items() if isinstance(value, dict)}


def _resolve_fixture_path(raw_path: str) -> Path:
    if raw_path.strip():
        return Path(raw_path).expanduser().resolve()
    try:
        share = get_package_share_directory("nao_chatbot")
    except PackageNotFoundError:
        return (Path(__file__).resolve().parent.parent / "config" / DEFAULT_FIXTURE_FILE).resolve()
    return Path(share, "config", DEFAULT_FIXTURE_FILE)


def _parse_csv(value) -> list[str]:
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


if __name__ == "__main__":
    raise SystemExit(main())
