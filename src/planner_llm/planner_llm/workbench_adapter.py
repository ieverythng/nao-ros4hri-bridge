"""Optional Neural Workbench seam for planner_llm.

The Workbench remains a companion research package. This adapter lets
planner_llm ask it for candidate abstract skill programs without making the ROS
stack depend on the package at import time or launch time.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
import sys
from typing import Any

from planner_common import ExecutionFeedback
from planner_common import PlannerRequest


@dataclass(frozen=True)
class WorkbenchAdapterConfig:
    enabled: bool = False
    required: bool = False
    desired_ab_level: int = 1
    python_path: str = ''
    trace_candidates: bool = True


@dataclass(frozen=True)
class WorkbenchPlanCandidate:
    planner_output: dict
    raw_output: str
    metadata: dict


class WorkbenchPlannerAdapter:
    """Lazy facade around the external Neural Workbench client."""

    def __init__(
        self,
        config: WorkbenchAdapterConfig,
        *,
        client: Any | None = None,
        logger=None,
    ) -> None:
        self._config = config
        self._client = client
        self._logger = logger
        self._load_error = ''
        self._warned_unavailable = False

    @property
    def enabled(self) -> bool:
        return bool(self._config.enabled)

    @property
    def required(self) -> bool:
        return bool(self._config.required)

    @property
    def load_error(self) -> str:
        return self._load_error

    def propose(
        self,
        request: PlannerRequest,
        *,
        world_model_text: str = '',
        world_model_snapshot: dict | None = None,
        feedback: ExecutionFeedback | None = None,
    ) -> WorkbenchPlanCandidate | None:
        if not self.enabled:
            return None

        client = self._resolve_client()
        if client is None:
            return None

        decision = client.propose(
            goal_text=request.goal_text,
            context=self._context_payload(
                request,
                world_model_text=world_model_text,
                world_model_snapshot=world_model_snapshot or {},
                feedback=feedback,
            ),
            desired_ab_level=max(0, int(self._config.desired_ab_level)),
        )
        decision_payload = self._decision_to_dict(decision)
        planner_output = dict(decision_payload.get('planner_output', {}) or {})
        if not planner_output:
            return None

        metadata = self._metadata_payload(decision_payload)
        if metadata:
            planner_output['neural_workbench'] = metadata
        return WorkbenchPlanCandidate(
            planner_output=planner_output,
            raw_output=json.dumps(planner_output, sort_keys=True),
            metadata=metadata,
        )

    def _resolve_client(self):
        if self._client is not None:
            return self._client

        self._extend_python_path()
        try:
            from neural_workbench import NeuralWorkbenchClient
            from skill_common import load_default_registry
        except Exception as err:  # pragma: no cover - depends on companion repo install
            self._load_error = str(err)
            self._warn('Neural Workbench unavailable: %s' % err)
            return None

        try:
            self._client = NeuralWorkbenchClient(load_default_registry())
        except Exception as err:  # pragma: no cover - depends on companion repo install
            self._load_error = str(err)
            self._warn('Neural Workbench failed to initialize: %s' % err)
            return None
        return self._client

    def _extend_python_path(self) -> None:
        for path in _split_paths(self._config.python_path):
            if path not in sys.path:
                sys.path.insert(0, path)

    def _context_payload(
        self,
        request: PlannerRequest,
        *,
        world_model_text: str,
        world_model_snapshot: dict,
        feedback: ExecutionFeedback | None,
    ) -> dict:
        return {
            'request_id': request.request_id,
            'goal_id': request.goal_id,
            'goal_text': request.goal_text,
            'normalized_intents': list(request.normalized_intents),
            'scene_targets': list(request.scene_targets),
            'grounded_context': dict(request.grounded_context or {}),
            'world_model_text': str(world_model_text or '').strip(),
            'world_model_snapshot': dict(world_model_snapshot or {}),
            'execution_feedback': self._feedback_payload(feedback),
        }

    @staticmethod
    def _feedback_payload(feedback: ExecutionFeedback | None) -> dict:
        if feedback is None:
            return {}
        return {
            'goal_id': feedback.goal_id,
            'plan_id': feedback.plan_id,
            'plan_version': feedback.plan_version,
            'event_type': feedback.event_type,
            'status': feedback.status,
            'reason': feedback.reason,
            'step_id': feedback.step_id,
            'step_name': feedback.step_name,
            'retry_budget': feedback.retry_budget,
        }

    @staticmethod
    def _decision_to_dict(decision) -> dict:
        if hasattr(decision, 'to_dict'):
            return dict(decision.to_dict())
        if isinstance(decision, dict):
            return dict(decision)
        return {}

    def _metadata_payload(self, decision_payload: dict) -> dict:
        selected_program = decision_payload.get('selected_program')
        metadata = {
            'source': 'neural_workbench',
            'desired_ab_level': max(0, int(self._config.desired_ab_level)),
            'selected_program_id': '',
            'selected_program': selected_program if isinstance(selected_program, dict) else {},
            'trace_ids': list(decision_payload.get('trace_ids', []) or []),
        }
        if isinstance(selected_program, dict):
            metadata['selected_program_id'] = str(selected_program.get('program_id', '')).strip()
        if self._config.trace_candidates:
            metadata['candidate_programs'] = list(decision_payload.get('candidate_programs', []) or [])
        return metadata

    def _warn(self, message: str) -> None:
        if self._warned_unavailable:
            return
        self._warned_unavailable = True
        if self._logger is not None:
            self._logger.warn(message)


def _split_paths(value: str) -> tuple[str, ...]:
    paths: list[str] = []
    for item in str(value or '').split(os.pathsep):
        clean_path = item.strip()
        if clean_path:
            paths.append(clean_path)
    return tuple(paths)
