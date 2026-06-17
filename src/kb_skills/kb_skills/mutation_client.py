"""Reusable write-side client helpers for KnowledgeCore mutations."""

from __future__ import annotations

from dataclasses import dataclass
import math
import sys
import threading

try:  # pragma: no cover - optional dependency in unit tests
    from kb_msgs.srv import Revise
except ImportError:  # pragma: no cover - optional dependency in unit tests
    Revise = None

_MUTATION_RESULT_DATACLASS_KWARGS = {"frozen": True}
if sys.version_info >= (3, 10):  # pragma: no branch - interpreter version check
    _MUTATION_RESULT_DATACLASS_KWARGS["slots"] = True


@dataclass(**_MUTATION_RESULT_DATACLASS_KWARGS)
class MutationResult:
    """Normalized result returned by `KnowledgeCoreMutationClient`."""

    success: bool
    operation: str
    dispatched: bool
    error_msg: str = ""
    statement_count: int = 0


class KnowledgeCoreMutationClient:
    """Thin ROS client wrapper around the canonical `/kb/revise` service."""

    _TRACE_STAGES = {
        "add": "KB_ADD",
        "remove": "KB_REMOVE",
        "update": "KB_REVISE",
    }

    def __init__(
        self,
        node=None,
        callback_group=None,
        service_name: str = "/kb/revise",
        timeout_sec: float = 0.5,
    ) -> None:
        self._node = node
        self._service_name = str(service_name or "/kb/revise").strip() or "/kb/revise"
        self._timeout_sec = max(0.05, float(timeout_sec))
        self._client = None
        self._warned_unavailable = False

        if node is None or Revise is None:
            if node is not None and Revise is None:
                node.get_logger().warn(
                    "kb_msgs is unavailable; KnowledgeCore mutation client is disabled"
                )
            return

        self._client = node.create_client(
            Revise,
            self._service_name,
            callback_group=callback_group,
        )

    @property
    def service_name(self) -> str:
        return self._service_name

    def close(self) -> None:
        """Release the ROS service client owned by this wrapper."""
        if self._node is not None and self._client is not None:
            self._node.destroy_client(self._client)
        self._client = None

    def add_fact(
        self,
        statement: str,
        *,
        models: list[str] | None = None,
        lifespan_sec: float = 0.0,
        turn_id: str = "",
        trace=None,
    ) -> MutationResult:
        return self._mutate_one(
            operation="add",
            statement=statement,
            models=models,
            lifespan_sec=lifespan_sec,
            turn_id=turn_id,
            trace=trace,
        )

    def add_facts(
        self,
        statements,
        *,
        models: list[str] | None = None,
        lifespan_sec: float = 0.0,
        wait_for_result: bool = True,
        turn_id: str = "",
        trace=None,
    ) -> MutationResult:
        return self._mutate_many(
            operation="add",
            statements=statements,
            models=models,
            lifespan_sec=lifespan_sec,
            wait_for_result=wait_for_result,
            turn_id=turn_id,
            trace=trace,
        )

    def revise_fact(
        self,
        statement: str,
        *,
        models: list[str] | None = None,
        lifespan_sec: float = 0.0,
        turn_id: str = "",
        trace=None,
    ) -> MutationResult:
        return self._mutate_one(
            operation="update",
            statement=statement,
            models=models,
            lifespan_sec=lifespan_sec,
            turn_id=turn_id,
            trace=trace,
        )

    def revise_facts(
        self,
        statements,
        *,
        models: list[str] | None = None,
        lifespan_sec: float = 0.0,
        wait_for_result: bool = True,
        turn_id: str = "",
        trace=None,
    ) -> MutationResult:
        return self._mutate_many(
            operation="update",
            statements=statements,
            models=models,
            lifespan_sec=lifespan_sec,
            wait_for_result=wait_for_result,
            turn_id=turn_id,
            trace=trace,
        )

    def remove_fact(
        self,
        statement: str,
        *,
        models: list[str] | None = None,
        turn_id: str = "",
        trace=None,
    ) -> MutationResult:
        return self._mutate_one(
            operation="remove",
            statement=statement,
            models=models,
            turn_id=turn_id,
            trace=trace,
        )

    def remove_facts(
        self,
        statements,
        *,
        models: list[str] | None = None,
        wait_for_result: bool = True,
        turn_id: str = "",
        trace=None,
    ) -> MutationResult:
        return self._mutate_many(
            operation="remove",
            statements=statements,
            models=models,
            lifespan_sec=0.0,
            wait_for_result=wait_for_result,
            turn_id=turn_id,
            trace=trace,
        )

    def _mutate_one(
        self,
        *,
        operation: str,
        statement: str,
        models: list[str] | None = None,
        lifespan_sec: float = 0.0,
        turn_id: str = "",
        trace=None,
    ) -> MutationResult:
        return self._mutate_many(
            operation=operation,
            statements=[statement],
            models=models,
            lifespan_sec=lifespan_sec,
            wait_for_result=True,
            turn_id=turn_id,
            trace=trace,
        )

    def _mutate_many(
        self,
        *,
        operation: str,
        statements,
        models: list[str] | None = None,
        lifespan_sec: float = 0.0,
        wait_for_result: bool = True,
        turn_id: str = "",
        trace=None,
    ) -> MutationResult:
        return self.mutate(
            operation=operation,
            statements=statements,
            models=models,
            lifespan_sec=lifespan_sec,
            wait_for_result=wait_for_result,
            turn_id=turn_id,
            trace=trace,
            trace_stage=self._TRACE_STAGES.get(operation, "KB_MUTATE"),
        )

    def mutate(
        self,
        *,
        operation: str,
        statements,
        models: list[str] | None = None,
        lifespan_sec: float = 0.0,
        wait_for_result: bool = True,
        turn_id: str = "",
        trace=None,
        trace_stage: str = "KB_MUTATE",
    ) -> MutationResult:
        clean_statements = self.coerce_statements(statements)
        statement_count = len(clean_statements)
        if not clean_statements:
            return self._result(
                success=False,
                operation=operation,
                dispatched=False,
                error_msg="No statements were provided",
                statement_count=statement_count,
            )

        if not self._service_is_ready():
            return self._result(
                success=False,
                operation=operation,
                dispatched=False,
                error_msg="KnowledgeCore revise service is unavailable",
                statement_count=statement_count,
            )

        request = self._build_request(
            operation=operation,
            statements=clean_statements,
            models=list(models or []),
            lifespan_sec=lifespan_sec,
        )
        future = self._client.call_async(request)

        if not wait_for_result:
            return self._result(
                success=True,
                operation=operation,
                dispatched=True,
                statement_count=statement_count,
            )

        response = self._await_response(
            future,
            turn_id=turn_id,
            trace=trace,
            trace_stage=trace_stage,
        )
        if response is None:
            return self._result(
                success=False,
                operation=operation,
                dispatched=False,
                error_msg=f"Timeout waiting for {self._service_name}",
                statement_count=statement_count,
            )

        if not getattr(response, "success", False):
            error_msg = str(getattr(response, "error_msg", "")).strip()
            self._trace(
                trace,
                turn_id,
                trace_stage,
                "mutation returned failure: %s" % error_msg,
                level="warn",
            )
            return self._result(
                success=False,
                operation=operation,
                dispatched=True,
                error_msg=error_msg,
                statement_count=statement_count,
            )

        return self._result(
            success=True,
            operation=operation,
            dispatched=True,
            statement_count=statement_count,
        )

    def _service_is_ready(self) -> bool:
        if self._client is None:
            return False
        if self._client.service_is_ready():
            self._warned_unavailable = False
            return True
        if not self._warned_unavailable and self._node is not None:
            self._node.get_logger().warn(
                "KnowledgeCore revise service is unavailable at %s"
                % self._service_name
            )
            self._warned_unavailable = True
        return False

    def _await_response(
        self,
        future,
        *,
        turn_id: str,
        trace,
        trace_stage: str,
    ):
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
            return future.result()
        except Exception as err:  # pragma: no cover - rclpy failure path
            self._trace(
                trace,
                turn_id,
                trace_stage,
                "mutation failure: %s" % err,
                level="warn",
            )
            return None

    @staticmethod
    def coerce_statements(statements) -> list[str]:
        if isinstance(statements, str):
            statements = [statements]
        if not isinstance(statements, (list, tuple)):
            return []
        return [str(item).strip() for item in statements if str(item).strip()]

    @staticmethod
    def _build_request(
        *,
        operation: str,
        statements: list[str],
        models: list[str],
        lifespan_sec: float,
    ):
        if Revise is None:  # pragma: no cover - guarded by caller
            raise RuntimeError("kb_msgs.srv.Revise is unavailable")
        request = Revise.Request()
        request.method = _revise_method_for_operation(operation)
        request.statements = list(statements)
        request.models = list(models)
        lifespan = max(0.0, float(lifespan_sec))
        request.lifespan.sec = int(math.floor(lifespan))
        request.lifespan.nanosec = int(
            (lifespan - request.lifespan.sec) * 1_000_000_000
        )
        return request

    @staticmethod
    def _result(
        *,
        success: bool,
        operation: str,
        dispatched: bool,
        error_msg: str = "",
        statement_count: int = 0,
    ) -> MutationResult:
        return MutationResult(
            success=success,
            operation=operation,
            dispatched=dispatched,
            error_msg=error_msg,
            statement_count=statement_count,
        )

    @staticmethod
    def _trace(trace, turn_id: str, stage: str, message: str, level: str = "info") -> None:
        if callable(trace):
            trace(turn_id, stage, message, level=level)


def _revise_method_for_operation(operation: str) -> str:
    clean = str(operation or '').strip().lower()
    if clean == 'remove':
        return 'retract'
    return clean or 'update'
