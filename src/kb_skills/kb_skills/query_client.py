"""Reusable read-only client helpers for KnowledgeCore queries."""

from __future__ import annotations

import json
import threading

try:  # pragma: no cover - optional dependency in unit tests
    from kb_msgs.srv import Query
except ImportError:  # pragma: no cover - optional dependency in unit tests
    Query = None


class KnowledgeCoreQueryClient:
    """Thin ROS client wrapper around the canonical `/kb/query` service."""

    def __init__(self, node, callback_group, service_name: str, timeout_sec: float) -> None:
        self._node = node
        self._service_name = str(service_name or "/kb/query").strip() or "/kb/query"
        self._timeout_sec = max(0.05, float(timeout_sec))
        self._client = None
        self._warned_import = False
        self._warned_unavailable = False

        if Query is None:
            self._warned_import = True
            self._node.get_logger().warn(
                "kb_msgs is unavailable; KnowledgeCore query client is disabled"
            )
            return

        self._client = self._node.create_client(
            Query,
            self._service_name,
            callback_group=callback_group,
        )

    # -------------------------------------------------------------------------
    # Public query surface
    # -------------------------------------------------------------------------

    @property
    def service_name(self) -> str:
        """Expose the resolved service name for diagnostics."""
        return self._service_name

    def query_rows(
        self,
        *,
        patterns: list[str],
        query_vars: list[str],
        models: list[str],
        turn_id: str = "",
        trace=None,
        trace_stage: str = "KB_QUERY",
    ) -> list[dict]:
        """Execute one query and return parsed response rows."""
        if self._client is None:
            return []

        if not self._client.service_is_ready():
            if not self._warned_unavailable:
                self._node.get_logger().warn(
                    "KnowledgeCore query service is unavailable at %s"
                    % self._service_name
                )
                self._warned_unavailable = True
            return []

        response = self._query_once(
            patterns=patterns,
            query_vars=query_vars,
            models=models,
            turn_id=turn_id,
            trace=trace,
            trace_stage=trace_stage,
        )
        if response is None:
            return []
        return self.parse_response_rows(getattr(response, "json", ""))

    # -------------------------------------------------------------------------
    # ROS query execution
    # -------------------------------------------------------------------------

    def _query_once(
        self,
        *,
        patterns: list[str],
        query_vars: list[str],
        models: list[str],
        turn_id: str,
        trace=None,
        trace_stage: str,
    ):
        request = Query.Request()
        request.patterns = list(patterns)
        request.vars = list(query_vars)
        request.models = list(models)

        future = self._client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())

        if not completed.wait(timeout=self._timeout_sec):
            future.cancel()
            self._trace(
                trace,
                turn_id,
                trace_stage,
                "timeout waiting for %s" % self._service_name,
                level="warn",
            )
            return None

        try:
            response = future.result()
        except Exception as err:  # pragma: no cover - rclpy failure path
            self._trace(
                trace,
                turn_id,
                trace_stage,
                "query failure: %s" % err,
                level="warn",
            )
            return None

        if not getattr(response, "success", False):
            self._trace(
                trace,
                turn_id,
                trace_stage,
                "query returned failure: %s" % getattr(response, "error_msg", ""),
                level="warn",
            )
            return None
        return response

    # -------------------------------------------------------------------------
    # Payload normalization helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def parse_response_rows(json_payload: str) -> list[dict]:
        """Parse KnowledgeCore JSON bindings into a stable list of rows."""
        payload = str(json_payload or "").strip()
        if not payload:
            return []
        try:
            parsed = json.loads(payload)
        except json.JSONDecodeError:
            return []
        if isinstance(parsed, dict):
            parsed = [parsed]
        if not isinstance(parsed, list):
            return []
        return [row for row in parsed if isinstance(row, dict)]

    @staticmethod
    def dedupe_rows(rows: list[dict]) -> list[dict]:
        """Remove duplicated rows while preserving their original order."""
        deduped: list[dict] = []
        seen: set[str] = set()
        for row in rows:
            key = json.dumps(row, sort_keys=True, default=str)
            if key in seen:
                continue
            seen.add(key)
            deduped.append(row)
        return deduped

    @staticmethod
    def _trace(trace, turn_id: str, stage: str, message: str, level: str = "info") -> None:
        if callable(trace):
            trace(turn_id, stage, message, level=level)
