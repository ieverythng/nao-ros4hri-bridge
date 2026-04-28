"""Pure planning engine for planner_llm."""

from __future__ import annotations

from dataclasses import dataclass
import json

from planner_common import ExecutionFeedback
from planner_common import PlannerRequest
from planner_common import build_plan_payload
from planner_common import extract_json_object
from planner_common import normalize_communication_policy
from planner_common import normalize_plan_steps

from planner_llm.providers import BasePlannerProvider
from planner_llm.providers import PlannerProviderError
from planner_llm.skill_registry import SkillRegistry

try:  # pragma: no cover - runtime dependency
    from hri_actions_msgs.msg import Intent
except ImportError:  # pragma: no cover - import-light unit tests
    class Intent:  # type: ignore[no-redef]
        RAW_USER_INPUT = 'raw_user_input'
        GREET = 'greet'


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
_SYSTEM_PROMPT = (
    'You are planner_llm for a ROS4HRI robot. Reply with one JSON object only. '
    'Return fields ack_text, ack_mode, decision, validation_status, failure_reason, '
    'replan_hint, retry_budget, scene_targets, communication_policy, and steps. '
    'Each step must contain type, name, args, requires, on_failure, and retry_budget. '
    'Plan only over the supplied abstract skill registry and allowed step types. '
    'Do not reference robot-specific topics, NAOqi APIs, or direct hardware calls. '
    'normalized_intents may be incomplete, so infer the executable request from goal_text, '
    'grounded context, and execution feedback. Treat requested_plan as a compatibility '
    'fallback only, never as higher priority than goal_text or allowed skills. '
    'For scan-style requests such as "look around and tell me what you see", '
    'prefer a short sequence of perform_motion sweep steps followed by a scene-inspection '
    'skill from the supplied registry when available. '
    'If the task is ambiguous or blocked, set '
    'decision to clarify and include clarification_text. If no safe continuation exists, '
    'set decision to fail and explain why.'
)


@dataclass(frozen=True)
class PlannerDecision:
    intent_name: str
    payload: dict
    plan_id: str
    raw_model_output: str = ''
    mode: str = 'plan'


class PlannerEngine:
    """Turn planner requests and context into executable plan envelopes."""

    def __init__(
        self,
        provider: BasePlannerProvider,
        skill_registry: SkillRegistry,
        *,
        default_intent_name: str = Intent.RAW_USER_INPUT,
        default_retry_budget: int = 1,
    ) -> None:
        self._provider = provider
        self._skill_registry = skill_registry
        self._default_intent_name = str(default_intent_name or Intent.RAW_USER_INPUT).strip()
        self._default_retry_budget = max(0, int(default_retry_budget))

    def plan_request(
        self,
        request: PlannerRequest,
        *,
        world_model_text: str = '',
        world_model_snapshot: dict | None = None,
        feedback: ExecutionFeedback | None = None,
        goal_id: str = '',
        plan_version: int = 1,
        status: str = 'planning',
        communication_policy: dict | None = None,
    ) -> PlannerDecision:
        resolved_goal_id = str(goal_id or request.goal_id).strip()
        resolved_plan_version = max(1, int(plan_version or 1))
        resolved_policy = normalize_communication_policy(communication_policy)

        if feedback is not None and feedback.status in ('failed', 'invalid') and feedback.retry_budget <= 0:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='retry budget exhausted',
                mode='clarify',
                goal_id=resolved_goal_id,
                plan_version=resolved_plan_version,
                status='waiting_user',
                communication_policy=resolved_policy,
            )

        rule_based_decision = self._rule_based_decision(
            request,
            feedback=feedback,
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status=status,
            communication_policy=resolved_policy,
        )
        if rule_based_decision is not None:
            return rule_based_decision

        try:
            raw_model_output = self._provider.generate(
                self._build_messages(
                    request,
                    world_model_text=world_model_text,
                    world_model_snapshot=world_model_snapshot or {},
                    feedback=feedback,
                    goal_id=resolved_goal_id,
                    plan_version=resolved_plan_version,
                )
            )
        except PlannerProviderError as err:
            requested_plan_decision = self._requested_plan_decision(
                request,
                feedback=feedback,
                goal_id=resolved_goal_id,
                plan_version=resolved_plan_version,
                status=status,
                communication_policy=resolved_policy,
                raw_model_output='planner backend unavailable: %s' % err,
            )
            if requested_plan_decision is not None:
                return requested_plan_decision
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='planner backend unavailable: %s' % err,
                mode='clarify',
                goal_id=resolved_goal_id,
                plan_version=resolved_plan_version,
                status='waiting_user',
                communication_policy=resolved_policy,
            )

        decision = self._decision_from_model_output(
            request,
            raw_model_output,
            feedback=feedback,
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status=status,
            communication_policy=resolved_policy,
        )
        if decision is not None:
            return decision

        requested_plan_decision = self._requested_plan_decision(
            request,
            feedback=feedback,
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status=status,
            communication_policy=resolved_policy,
            raw_model_output=raw_model_output,
        )
        if requested_plan_decision is not None:
            return requested_plan_decision

        return self._clarification_decision(
            request,
            feedback=feedback,
            reason='planner output did not contain a valid executable plan',
            raw_model_output=raw_model_output,
            mode='clarify',
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status='waiting_user',
            communication_policy=resolved_policy,
        )

    def _build_messages(
        self,
        request: PlannerRequest,
        *,
        world_model_text: str,
        world_model_snapshot: dict,
        feedback: ExecutionFeedback | None,
        goal_id: str,
        plan_version: int,
    ) -> list[dict[str, str]]:
        prompt = {
            'request': self._request_payload(request),
            'goal_id': goal_id,
            'plan_version': plan_version,
            'world_model_text': str(world_model_text or '').strip(),
            'world_model_snapshot': world_model_snapshot,
            'execution_feedback': self._feedback_payload(feedback),
            'skill_registry': self._skill_registry.prompt_manifest(),
            'allowed_step_types': list(self._skill_registry.step_types),
            'allowed_skill_names': list(self._skill_registry.allowed_skill_names),
            'allowed_motion_objects': list(_RULE_BASED_MOTIONS.values()),
        }
        return [
            {'role': 'system', 'content': _SYSTEM_PROMPT},
            {'role': 'user', 'content': json.dumps(prompt, sort_keys=True)},
        ]

    def _decision_from_model_output(
        self,
        request: PlannerRequest,
        raw_model_output: str,
        *,
        feedback: ExecutionFeedback | None,
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
    ) -> PlannerDecision | None:
        parsed = extract_json_object(raw_model_output)
        if not parsed:
            return None

        decision_mode = str(parsed.get('decision', parsed.get('mode', 'plan'))).strip().lower()
        if decision_mode in ('clarify', 'clarification'):
            clarify_reason = str(
                parsed.get(
                    'user_facing_reason',
                    parsed.get('clarification_text', parsed.get('failure_reason', '')),
                )
            ).strip()
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason=clarify_reason,
                raw_model_output=raw_model_output,
                mode='clarify',
                goal_id=goal_id,
                plan_version=plan_version,
                status='waiting_user',
                communication_policy=self._resolved_communication_policy(parsed, communication_policy),
            )

        if decision_mode == 'fail':
            fail_reason = str(
                parsed.get(
                    'user_facing_reason',
                    parsed.get('failure_reason', 'Unable to continue safely.'),
                )
            ).strip()
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason=fail_reason,
                raw_model_output=raw_model_output,
                mode='fail',
                goal_id=goal_id,
                plan_version=plan_version,
                status='failed',
                communication_policy=self._resolved_communication_policy(parsed, communication_policy),
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
            user_facing_reason=str(parsed.get('user_facing_reason', '')).strip(),
            replan_hint=str(parsed.get('replan_hint', '')).strip(),
            retry_budget=self._resolved_retry_budget(parsed, feedback),
            scene_targets=self._scene_targets_for_decision(request, feedback, parsed),
            plan_id=str(parsed.get('plan_id', parsed.get('id', ''))).strip(),
            raw_model_output=raw_model_output,
            mode='replan' if feedback is not None else 'plan',
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=self._resolved_communication_policy(parsed, communication_policy),
        )

    def _clarification_decision(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
        reason: str,
        raw_model_output: str = '',
        mode: str = 'clarify',
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
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
            ack_text='',
            ack_mode='',
            validation_status='draft',
            failure_reason=clean_reason if mode == 'fail' else '',
            user_facing_reason=clean_reason,
            replan_hint='clarify_user',
            retry_budget=0,
            scene_targets=self._scene_targets_for_decision(request, feedback, {}),
            raw_model_output=raw_model_output,
            mode=mode,
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=communication_policy,
        )

    def _rule_based_decision(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
    ) -> PlannerDecision | None:
        if str(request.planner_mode or '').strip().lower() in (
            'multi_step',
            'multistep',
            'composite',
            'sequenced',
        ):
            return None
        if len(request.requested_plan) > 1:
            return None
        if len(request.normalized_intents) > 1:
            return None

        retry_budget = self._resolved_retry_budget({}, feedback)
        scene_targets = self._scene_targets_for_decision(request, feedback, {})
        motion_skill_name = self._first_supported_skill_name('perform_motion', 'motion')

        for normalized_intent in request.normalized_intents:
            motion_name = _RULE_BASED_MOTIONS.get(normalized_intent)
            if motion_name and motion_skill_name:
                return self._build_decision(
                    request=request,
                    feedback=feedback,
                    steps=[
                        self._step(
                            step_type='skill',
                            name=motion_skill_name,
                            args={'object': motion_name},
                            on_failure='replan',
                        )
                    ],
                    ack_text='',
                    ack_mode='',
                    validation_status='draft',
                    retry_budget=retry_budget,
                    scene_targets=scene_targets,
                    mode='rule',
                    goal_id=goal_id,
                    plan_version=plan_version,
                    status=status,
                    communication_policy=communication_policy,
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
                ack_text='',
                ack_mode='',
                validation_status='draft',
                retry_budget=retry_budget,
                scene_targets=scene_targets,
                mode='rule',
                goal_id=goal_id,
                plan_version=plan_version,
                status=status,
                communication_policy=communication_policy,
            )
        return None

    def _requested_plan_decision(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
        raw_model_output: str = '',
    ) -> PlannerDecision | None:
        if feedback is not None or not request.requested_plan:
            return None

        steps = self._skill_registry.filter_supported_steps(
            [dict(step) for step in request.requested_plan]
        )
        if not steps:
            return None

        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=steps,
            ack_text=request.ack_text,
            ack_mode=request.ack_mode,
            validation_status='draft',
            retry_budget=self._default_retry_budget,
            scene_targets=self._scene_targets_for_decision(request, feedback, {}),
            raw_model_output=raw_model_output,
            mode='hint',
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=communication_policy,
        )

    @staticmethod
    def _request_payload(request: PlannerRequest) -> dict:
        return {
            'request_id': request.request_id,
            'goal_id': request.goal_id,
            'parent_goal_id': request.parent_goal_id,
            'supersedes_goal_id': request.supersedes_goal_id,
            'request_kind': request.request_kind,
            'goal_text': request.goal_text,
            'normalized_intents': list(request.normalized_intents),
            'ack_text': request.ack_text,
            'ack_mode': request.ack_mode,
            'scene_targets': list(request.scene_targets),
            'dialogue_context': list(request.dialogue_context),
            'requested_plan': list(request.requested_plan),
            'grounded_context': request.grounded_context,
            'planner_mode': request.planner_mode,
            'interaction_mode': request.interaction_mode,
            'dialogue_turn_id': request.dialogue_turn_id,
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
            'replan_hint': feedback.replan_hint,
            'retry_budget': feedback.retry_budget,
            'blocking': feedback.blocking,
            'unmet_preconditions': list(feedback.unmet_preconditions),
            'needs_user_input': feedback.needs_user_input,
            'scene_targets': list(feedback.scene_targets),
            'step_id': feedback.step_id,
            'step_type': feedback.step_type,
            'step_name': feedback.step_name,
        }

    def _extract_plan_steps(self, parsed: dict) -> list[dict]:
        steps = parsed.get('steps')
        if not isinstance(steps, list):
            plan_payload = parsed.get('plan', {})
            if isinstance(plan_payload, dict):
                steps = plan_payload.get('steps', [])
        if not isinstance(steps, list):
            return []
        return self._skill_registry.filter_supported_steps(normalize_plan_steps(steps))

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
        user_facing_reason: str = '',
        replan_hint: str = '',
        retry_budget: int = 0,
        scene_targets: list[str] | None = None,
        plan_id: str = '',
        raw_model_output: str = '',
        mode: str = 'plan',
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
    ) -> PlannerDecision:
        payload = build_plan_payload(
            request=request,
            ack_text=ack_text,
            ack_mode=ack_mode,
            validation_status=validation_status,
            failure_reason=failure_reason,
            user_facing_reason=user_facing_reason,
            replan_hint=replan_hint,
            retry_budget=retry_budget,
            scene_targets=scene_targets or self._scene_targets_for_decision(request, feedback, {}),
            steps=steps,
            plan_id=plan_id,
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=communication_policy,
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

    @staticmethod
    def _resolved_communication_policy(parsed: dict, fallback: dict) -> dict:
        if 'communication_policy' in parsed:
            return normalize_communication_policy(parsed.get('communication_policy', {}))
        plan_payload = parsed.get('plan', {})
        if isinstance(plan_payload, dict) and 'communication_policy' in plan_payload:
            return normalize_communication_policy(plan_payload.get('communication_policy', {}))
        return normalize_communication_policy(fallback)

    @staticmethod
    def _scene_targets_for_decision(
        request: PlannerRequest,
        feedback: ExecutionFeedback | None,
        parsed: dict,
    ) -> list[str]:
        parsed_targets = parsed.get('scene_targets', [])
        if isinstance(parsed_targets, list) and parsed_targets:
            return [str(item).strip() for item in parsed_targets if str(item).strip()]
        plan_payload = parsed.get('plan', {})
        nested_targets = plan_payload.get('scene_targets', []) if isinstance(plan_payload, dict) else []
        if isinstance(nested_targets, list) and nested_targets:
            return [str(item).strip() for item in nested_targets if str(item).strip()]
        if feedback is not None and feedback.scene_targets:
            return list(feedback.scene_targets)
        return list(request.scene_targets)

    def _first_supported_skill_name(self, *names: str) -> str:
        for name in names:
            clean_name = str(name or '').strip().lower()
            if clean_name and clean_name in self._skill_registry.allowed_skill_names:
                return clean_name
        return ''
