"""Pure planning engine for planner_llm."""

from __future__ import annotations

from dataclasses import dataclass
import json

from planner_common import ExecutionFeedback
from planner_common import PlannerRequest
from planner_common import build_plan_payload
from planner_common import extract_json_object

from planner_llm.providers import BasePlannerProvider
from planner_llm.providers import PlannerProviderError

try:  # pragma: no cover - runtime dependency
    from hri_actions_msgs.msg import Intent
except ImportError:  # pragma: no cover - import-light unit tests
    class Intent:  # type: ignore[no-redef]
        RAW_USER_INPUT = 'raw_user_input'
        SAY = 'say'
        GREET = 'greet'
        PERFORM_MOTION = 'perform_motion'


_RULE_BASED_MOTIONS = {
    'head_center': 'head_center',
    'head_look_left': 'head_look_left',
    'head_look_right': 'head_look_right',
    'head_look_up': 'head_look_up',
    'head_look_down': 'head_look_down',
    'posture_stand': 'stand',
    'posture_sit': 'sit',
    'posture_kneel': 'kneel',
}

_ALLOWED_STEP_TYPES = ('noop', 'say', 'skill', 'look_at')
_ALLOWED_SKILL_NAMES = ('perform_motion', 'motion', 'look_at')
_ALLOWED_MOTION_OBJECTS = tuple(_RULE_BASED_MOTIONS.values())
_SYSTEM_PROMPT = (
    'You are planner_llm for a ROS4HRI robot. Reply with one JSON object only. '
    'Return fields ack_text, ack_mode, decision, validation_status, failure_reason, '
    'replan_hint, retry_budget, scene_targets, and steps. Each step must contain '
    'type, name, args, requires, on_failure, and retry_budget. Use only the allowed '
    'step types and skill names. Prefer short executable plans. normalized_intents may be '
    'empty or partial, so infer the executable request from user_text, context, and any '
    'planner hints instead of requiring a perfect upstream label. If the task is ambiguous '
    'or blocked, set decision to clarify and include clarification_text.'
)


@dataclass(frozen=True)
class PlannerDecision:
    intent_name: str
    payload: dict
    plan_id: str
    raw_model_output: str = ''
    mode: str = 'plan'


class PlannerEngine:
    """Turn planner requests and WME context into executable plan envelopes."""

    def __init__(
        self,
        provider: BasePlannerProvider,
        *,
        default_intent_name: str = Intent.RAW_USER_INPUT,
        default_retry_budget: int = 1,
    ) -> None:
        self._provider = provider
        self._default_intent_name = str(default_intent_name or Intent.RAW_USER_INPUT).strip()
        self._default_retry_budget = max(0, int(default_retry_budget))

    def plan_request(
        self,
        request: PlannerRequest,
        *,
        world_model_text: str = '',
        world_model_snapshot: dict | None = None,
        feedback: ExecutionFeedback | None = None,
    ) -> PlannerDecision:
        if feedback is not None and feedback.status in ('failed', 'invalid') and feedback.retry_budget <= 0:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='retry budget exhausted',
                mode='clarify',
            )

        rule_based_decision = self._rule_based_decision(request, feedback=feedback)
        if rule_based_decision is not None:
            return rule_based_decision

        try:
            raw_model_output = self._provider.generate(
                self._build_messages(
                    request,
                    world_model_text=world_model_text,
                    world_model_snapshot=world_model_snapshot or {},
                    feedback=feedback,
                )
            )
        except PlannerProviderError as err:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='planner backend unavailable: %s' % err,
                mode='clarify',
            )

        decision = self._decision_from_model_output(
            request,
            raw_model_output,
            feedback=feedback,
        )
        if decision is not None:
            return decision
        return self._clarification_decision(
            request,
            feedback=feedback,
            reason='planner output did not contain a valid executable plan',
            raw_model_output=raw_model_output,
            mode='clarify',
        )

    def _build_messages(
        self,
        request: PlannerRequest,
        *,
        world_model_text: str,
        world_model_snapshot: dict,
        feedback: ExecutionFeedback | None,
    ) -> list[dict[str, str]]:
        prompt = {
            'request': self._request_payload(request),
            'world_model_text': str(world_model_text or '').strip(),
            'world_model_snapshot': world_model_snapshot,
            'execution_feedback': self._feedback_payload(feedback),
            'allowed_step_types': list(_ALLOWED_STEP_TYPES),
            'allowed_skill_names': list(_ALLOWED_SKILL_NAMES),
            'allowed_motion_objects': list(_ALLOWED_MOTION_OBJECTS),
        }
        return [
            {
                'role': 'system',
                'content': _SYSTEM_PROMPT,
            },
            {
                'role': 'user',
                'content': json.dumps(prompt, sort_keys=True),
            },
        ]

    def _decision_from_model_output(
        self,
        request: PlannerRequest,
        raw_model_output: str,
        *,
        feedback: ExecutionFeedback | None,
    ) -> PlannerDecision | None:
        parsed = extract_json_object(raw_model_output)
        if not parsed:
            return None

        decision_mode = str(parsed.get('decision', parsed.get('mode', 'plan'))).strip().lower()
        if decision_mode in ('clarify', 'clarification'):
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason=str(parsed.get('clarification_text', parsed.get('failure_reason', ''))).strip(),
                raw_model_output=raw_model_output,
                mode='clarify',
            )

        if decision_mode == 'fail':
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason=str(parsed.get('failure_reason', 'Unable to continue safely.')).strip(),
                raw_model_output=raw_model_output,
                mode='fail',
            )

        steps = self._extract_plan_steps(parsed)
        if not steps:
            return None

        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=steps,
            ack_text=str(parsed.get('ack_text', '')).strip(),
            ack_mode=str(parsed.get('ack_mode', request.ack_mode)).strip(),
            validation_status=str(parsed.get('validation_status', 'draft')).strip() or 'draft',
            failure_reason=str(parsed.get('failure_reason', '')).strip(),
            replan_hint=str(parsed.get('replan_hint', '')).strip(),
            retry_budget=self._resolved_retry_budget(parsed, feedback),
            scene_targets=self._scene_targets_for_decision(request, feedback, parsed),
            plan_id=str(parsed.get('plan_id', parsed.get('id', ''))).strip(),
            raw_model_output=raw_model_output,
            mode='replan' if feedback is not None else 'plan',
        )

    def _clarification_decision(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
        reason: str,
        raw_model_output: str = '',
        mode: str = 'clarify',
    ) -> PlannerDecision:
        clean_reason = str(reason or '').strip() or 'I need a bit more detail before I can continue.'
        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=[
                self._step(
                    step_type='say',
                    name='say',
                    args={'text': clean_reason},
                    on_failure='fail',
                )
            ],
            ack_text=request.ack_text,
            ack_mode=request.ack_mode,
            validation_status='draft',
            failure_reason=clean_reason if mode == 'fail' else '',
            replan_hint='clarify_user',
            retry_budget=0,
            scene_targets=self._scene_targets_for_decision(request, feedback, {}),
            raw_model_output=raw_model_output,
            mode=mode,
        )

    def _rule_based_decision(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
    ) -> PlannerDecision | None:
        if str(request.planner_mode or '').strip().lower() in (
            'multi_step',
            'multistep',
            'composite',
            'sequenced',
        ):
            return None
        if len(request.normalized_intents) > 1:
            return None

        retry_budget = self._resolved_retry_budget({}, feedback)
        scene_targets = self._scene_targets_for_decision(request, feedback, {})

        for normalized_intent in request.normalized_intents:
            motion_name = _RULE_BASED_MOTIONS.get(normalized_intent)
            if motion_name:
                return self._build_decision(
                    request=request,
                    feedback=feedback,
                    steps=[
                        self._step(
                            step_type='skill',
                            name='perform_motion',
                            args={'object': motion_name},
                            on_failure='replan',
                        )
                    ],
                    ack_text=request.ack_text,
                    ack_mode=request.ack_mode,
                    validation_status='draft',
                    retry_budget=retry_budget,
                    scene_targets=scene_targets,
                    mode='rule',
                )

        if any(intent_name in ('greet', Intent.GREET) for intent_name in request.normalized_intents):
            return self._build_decision(
                request=request,
                feedback=feedback,
                steps=[
                    self._step(
                        step_type='say',
                        name='say',
                        args={'text': request.ack_text or 'Hello!'},
                    )
                ],
                ack_text=request.ack_text,
                ack_mode=request.ack_mode,
                validation_status='draft',
                retry_budget=retry_budget,
                scene_targets=scene_targets,
                mode='rule',
            )
        return None

    @staticmethod
    def _request_payload(request: PlannerRequest) -> dict:
        return {
            'request_id': request.request_id,
            'user_text': request.user_text,
            'normalized_intents': list(request.normalized_intents),
            'ack_text': request.ack_text,
            'ack_mode': request.ack_mode,
            'scene_targets': list(request.scene_targets),
            'dialogue_context': list(request.dialogue_context),
            'grounded_context': request.grounded_context,
            'planner_mode': request.planner_mode,
        }

    @staticmethod
    def _feedback_payload(feedback: ExecutionFeedback | None) -> dict:
        if feedback is None:
            return {}
        return {
            'plan_id': feedback.plan_id,
            'status': feedback.status,
            'reason': feedback.reason,
            'replan_hint': feedback.replan_hint,
            'retry_budget': feedback.retry_budget,
            'scene_targets': list(feedback.scene_targets),
            'step_id': feedback.step_id,
            'step_type': feedback.step_type,
            'step_name': feedback.step_name,
        }

    @staticmethod
    def _extract_plan_steps(parsed: dict) -> list[dict]:
        steps = parsed.get('steps')
        if isinstance(steps, list) and steps:
            return steps
        plan_payload = parsed.get('plan', {})
        if not isinstance(plan_payload, dict):
            return steps if isinstance(steps, list) else []
        nested_steps = plan_payload.get('steps', [])
        if isinstance(nested_steps, list):
            return nested_steps
        return steps if isinstance(steps, list) else []

    @staticmethod
    def _step(
        *,
        step_type: str,
        name: str,
        args: dict,
        on_failure: str = 'fail',
        retry_budget: int = 0,
    ) -> dict:
        return {
            'type': step_type,
            'name': name,
            'args': args,
            'requires': [],
            'on_failure': on_failure,
            'retry_budget': retry_budget,
        }

    def _build_decision(
        self,
        *,
        request: PlannerRequest,
        feedback: ExecutionFeedback | None,
        steps: list[dict],
        ack_text: str,
        ack_mode: str,
        validation_status: str,
        failure_reason: str = '',
        replan_hint: str = '',
        retry_budget: int = 0,
        scene_targets: list[str] | None = None,
        plan_id: str = '',
        raw_model_output: str = '',
        mode: str = 'plan',
    ) -> PlannerDecision:
        payload = build_plan_payload(
            request=request,
            ack_text=ack_text,
            ack_mode=ack_mode,
            validation_status=validation_status,
            failure_reason=failure_reason,
            replan_hint=replan_hint,
            retry_budget=retry_budget,
            scene_targets=scene_targets or self._scene_targets_for_decision(request, feedback, {}),
            steps=steps,
            plan_id=plan_id,
        )
        return PlannerDecision(
            intent_name=self._default_intent_name,
            payload=payload,
            plan_id=payload['plan']['plan_id'],
            raw_model_output=raw_model_output,
            mode=mode,
        )

    def _resolved_retry_budget(self, parsed: dict, feedback: ExecutionFeedback | None) -> int:
        if 'retry_budget' in parsed:
            try:
                return max(0, int(parsed.get('retry_budget', 0)))
            except (TypeError, ValueError):
                return 0
        if feedback is not None:
            return max(0, int(feedback.retry_budget) - 1)
        return self._default_retry_budget

    def _scene_targets_for_decision(
        self,
        request: PlannerRequest,
        feedback: ExecutionFeedback | None,
        parsed: dict,
    ) -> list[str]:
        parsed_targets = parsed.get('scene_targets', [])
        if isinstance(parsed_targets, list) and parsed_targets:
            return [str(item).strip() for item in parsed_targets if str(item).strip()]
        if feedback is not None and feedback.scene_targets:
            return list(feedback.scene_targets)
        return list(request.scene_targets)
