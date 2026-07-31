"""Pure planning engine for planner_llm."""

from __future__ import annotations

from dataclasses import dataclass
import json

from planner_common import DEFAULT_PERFORM_MOTION_OBJECT_LABELS
from planner_common import ExecutionFeedback
from planner_common import IntentLabels
from planner_common import PlannerRequest
from planner_common import build_plan_payload
from planner_common import extract_json_object
from planner_common import is_explicit_knowledge_statement
from planner_common import missing_requested_report_error
from planner_common import normalize_communication_policy
from planner_common import normalize_plan_steps
from planner_common import plan_semantic_errors
from planner_common import strip_live_result_report_summary_text
from planner_common import validate_target_selection

from planner_llm.providers import BasePlannerProvider
from planner_llm.providers import PlannerProviderError
from planner_llm.prompt_pack import PlannerPromptPack
from planner_llm.prompt_pack import default_prompt_pack
from planner_llm.skill_registry import SkillRegistry


_DELIVERY_SELECTION_INTENTS = frozenset({'bring_object', 'deliver_object'})
_DELIVERY_DECOMPOSITION_INTENTS = frozenset({'pick_object', 'place_object'})
_NAVIGATION_SELECTION_INTENTS = frozenset({'navigate_to', 'walk_to'})
_MOTION_INTENT_OBJECTS = {
    'posture_stand': 'stand',
    'posture_sit': 'sit',
    'posture_kneel': 'kneel',
    'posture_crouch': 'crouch',
    'head_look_left': 'head_look_left',
    'head_look_right': 'head_look_right',
    'head_look_up': 'head_look_up',
    'head_look_down': 'head_look_down',
    'head_center': 'head_center',
}
_INTENT_CAPABILITIES = {
    'navigate_to': 'navigation',
    'walk_to': 'navigation',
    'bring_object': 'delivery',
    'deliver_object': 'delivery',
    'pick_object': 'pick_object',
    'place_object': 'place_object',
    'look_at': 'look_at',
    'wave_greet': 'wave_greet',
    'find_object': 'find_object',
    'inspect_scene': 'observation',
    'scan': 'observation',
    'report_result': 'report_result',
}


def _scene_targets_from_steps(steps) -> list[str]:
    targets: list[str] = []
    target_keys_by_skill = {
        'bring_object': ('target', 'object_id', 'object', 'recipient', 'recipient_id'),
        'deliver_object': ('target', 'object_id', 'object', 'recipient', 'recipient_id'),
        'find_object': ('target', 'object_id', 'object'),
        'look_at': ('target', 'target_frame'),
        'navigate_to': ('target', 'location'),
        'pick_object': ('target', 'object_id', 'object'),
        'place_object': ('target', 'object_id', 'object', 'destination'),
        'walk_to': ('target', 'location'),
        'wave_greet': ('target', 'target_frame'),
    }
    for step in steps if isinstance(steps, (list, tuple)) else ():
        if not isinstance(step, dict):
            continue
        skill = str(step.get('name', '')).strip().lower()
        args = step.get('args', {}) if isinstance(step.get('args'), dict) else {}
        for key in target_keys_by_skill.get(skill, ()):
            target = str(args.get(key, '')).strip()
            if target and target not in targets:
                targets.append(target)
    return targets


def _request_requires_target_selection(request: PlannerRequest) -> bool:
    intents = {
        str(intent or '').strip().lower()
        for intent in request.normalized_intents
    }
    if intents.intersection(_DELIVERY_SELECTION_INTENTS):
        return not _has_single_grounded_delivery_target(request)
    return bool(
        intents.intersection(_NAVIGATION_SELECTION_INTENTS)
        and len(request.scene_targets) > 1
    )


def _has_single_grounded_delivery_target(request: PlannerRequest) -> bool:
    """Let the planner resolve one unambiguous grounded object."""
    if len(request.scene_targets) != 1:
        return False
    target = str(request.scene_targets[0] or '').strip()
    if not target:
        return False
    matches = []
    entities = request.grounded_context.get('entities', [])
    for entity in entities if isinstance(entities, list) else ():
        if not isinstance(entity, dict):
            continue
        if str(entity.get('kind', '')).strip().lower() != 'object':
            continue
        if _entity_goal_match_score(entity, target) > 0:
            matches.append(entity)
    return len(matches) == 1


def _entity_goal_match_score(entity: dict, goal_text: str) -> int:
    clean_goal = _normalized_text(goal_text)
    candidates = [
        str(entity.get('id', '')).strip(),
        str(entity.get('label', '') or '').strip(),
        str(entity.get('class', '') or '').strip(),
    ]
    aliases = entity.get('aliases', [])
    if isinstance(aliases, list):
        candidates.extend(str(alias).strip() for alias in aliases)
    for relation in entity.get('relations', []):
        if not isinstance(relation, dict):
            continue
        if str(relation.get('predicate', '')).strip() == 'dbp:name':
            candidates.append(str(relation.get('object', '')).strip())
    best_score = 0
    for candidate in candidates:
        clean_candidate = _normalized_text(candidate)
        if not clean_candidate:
            continue
        if clean_candidate in clean_goal:
            token_count = len(clean_candidate.split())
            best_score = max(best_score, 4 if token_count >= 2 else 2)
            continue
        candidate_tokens = [
            token for token in clean_candidate.split()
            if token and token not in {'codex', 'probe', 'arch', 'iiia', 'base', 'lab', 'gold'}
        ]
        if len(candidate_tokens) >= 2:
            suffix = ' %s ' % ' '.join(candidate_tokens[-2:])
            if suffix in clean_goal:
                best_score = max(best_score, 3)
        elif candidate_tokens and (' %s ' % candidate_tokens[0]) in clean_goal:
            best_score = max(best_score, 1)
    return best_score


def _normalized_text(value: str) -> str:
    return ' %s ' % ' '.join(str(value or '').strip().lower().replace('_', ' ').split())


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
        prompt_pack: PlannerPromptPack | None = None,
        *,
        default_intent_name: str = IntentLabels.RAW_USER_INPUT,
        default_retry_budget: int = 1,
    ) -> None:
        self._provider = provider
        self._skill_registry = skill_registry
        self._prompt_pack = prompt_pack or default_prompt_pack()
        self._default_intent_name = str(default_intent_name or IntentLabels.RAW_USER_INPUT).strip()
        self._default_retry_budget = max(0, int(default_retry_budget))

    def plan_request(
        self,
        request: PlannerRequest,
        *,
        state_t0: dict | None = None,
        feedback: ExecutionFeedback | None = None,
        goal_id: str = '',
        plan_version: int = 1,
        status: str = 'planning',
        communication_policy: dict | None = None,
    ) -> PlannerDecision:
        resolved_goal_id = str(goal_id or request.goal_id).strip()
        resolved_plan_version = max(1, int(plan_version or 1))
        resolved_policy = normalize_communication_policy(communication_policy)
        _next_budget, retry_exhausted = self._next_retry_budget({}, feedback)

        if _request_requires_target_selection(request) and not request.target_selection:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='I need a complete grounded target selection before I can plan that task.',
                mode='clarify',
                goal_id=resolved_goal_id,
                plan_version=resolved_plan_version,
                status='waiting_user',
                communication_policy=resolved_policy,
            )

        expected_operation = ''
        request_intents = {
            str(intent or '').strip().lower()
            for intent in request.normalized_intents
        }
        if (
            request_intents.intersection(_DELIVERY_SELECTION_INTENTS)
            or _DELIVERY_DECOMPOSITION_INTENTS.issubset(request_intents)
        ):
            expected_operation = 'deliver'
        elif request_intents.intersection(_NAVIGATION_SELECTION_INTENTS):
            expected_operation = 'visit'
        selection_validation = (
            validate_target_selection(
                request.target_selection,
                request.grounded_context,
                expected_operation=expected_operation,
            )
            if request.target_selection
            else None
        )
        if selection_validation is not None and not selection_validation.valid:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='; '.join(selection_validation.errors),
                mode='clarify',
                goal_id=resolved_goal_id,
                plan_version=resolved_plan_version,
                status='waiting_user',
                communication_policy=resolved_policy,
            )

        if retry_exhausted:
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

        try:
            raw_model_output = self._provider.generate(
                self._build_messages(
                    request,
                    state_t0=state_t0 or {},
                    feedback=feedback,
                    goal_id=resolved_goal_id,
                    plan_version=resolved_plan_version,
                )
            )
        except PlannerProviderError as err:
            return self._backend_unavailable_decision(
                request,
                feedback=feedback,
                raw_model_output='planner backend unavailable: %s' % err,
                goal_id=resolved_goal_id,
                plan_version=resolved_plan_version,
                communication_policy=resolved_policy,
            )

        decision, validation_errors = self._decision_from_model_output_with_errors(
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

        if validation_errors:
            invalid_outputs = [raw_model_output]
            previous_output = raw_model_output
            previous_errors = list(validation_errors)
            for retry_index in range(2):
                try:
                    retry_output = self._provider.generate(
                        self._build_messages(
                            request,
                            state_t0=state_t0 or {},
                            feedback=feedback,
                            goal_id=resolved_goal_id,
                            plan_version=resolved_plan_version,
                            validation_errors=previous_errors,
                            previous_model_output=previous_output,
                        )
                    )
                except PlannerProviderError:
                    break

                invalid_outputs.append(retry_output)
                retry_decision, retry_errors = self._decision_from_model_output_with_errors(
                    request,
                    retry_output,
                    feedback=feedback,
                    goal_id=resolved_goal_id,
                    plan_version=resolved_plan_version,
                    status=status,
                    communication_policy=resolved_policy,
                )
                if retry_decision is not None:
                    return retry_decision

                retry_errors = retry_errors or previous_errors
                validation_errors = retry_errors
                previous_fingerprint = ' '.join(str(previous_output or '').split())
                retry_fingerprint = ' '.join(str(retry_output or '').split())
                if (
                    retry_index == 0
                    and retry_fingerprint
                    and retry_fingerprint != previous_fingerprint
                ):
                    previous_output = retry_output
                    previous_errors = retry_errors
                    continue
                break

            raw_model_output = invalid_outputs[0]
            for retry_index, retry_output in enumerate(invalid_outputs[1:], start=1):
                suffix = '' if retry_index == 1 else ' %d' % retry_index
                raw_model_output += '\n\n--- invalid retry output%s ---\n%s' % (
                    suffix,
                    retry_output,
                )

        target_selection_recovery = self._validated_target_selection_recovery(
            request,
            feedback=feedback,
            raw_model_output=raw_model_output,
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status=status,
            communication_policy=resolved_policy,
        )
        if target_selection_recovery is not None:
            return target_selection_recovery

        motion_sequence_recovery = self._validated_motion_sequence_recovery(
            request,
            feedback=feedback,
            raw_model_output=raw_model_output,
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status=status,
            communication_policy=resolved_policy,
        )
        if motion_sequence_recovery is not None:
            return motion_sequence_recovery

        if request.target_selection:
            return self._invalid_model_output_decision(
                request,
                feedback=feedback,
                reason=(
                    'incomplete requested action coverage after planner validation retries'
                    if self._target_selection_recovery_omits_capabilities(request)
                    else 'authoritative target selection was not executable'
                ),
                raw_model_output=raw_model_output,
                goal_id=resolved_goal_id,
                plan_version=resolved_plan_version,
                communication_policy=resolved_policy,
            )

        return self._invalid_model_output_decision(
            request,
            feedback=feedback,
            reason='planner output did not contain a valid executable plan: %s'
            % '; '.join(validation_errors or ['no valid executable steps']),
            raw_model_output=raw_model_output,
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            communication_policy=resolved_policy,
        )

    def _build_messages(
        self,
        request: PlannerRequest,
        *,
        state_t0: dict,
        feedback: ExecutionFeedback | None,
        goal_id: str,
        plan_version: int,
        validation_errors: list[str] | None = None,
        previous_model_output: str = '',
    ) -> list[dict[str, str]]:
        prompt = {
            'request': self._request_payload(request),
            'goal_id': goal_id,
            'plan_version': plan_version,
            'state_t0': state_t0 if isinstance(state_t0, dict) else {},
            'execution_feedback': self._feedback_payload(feedback),
            'skill_registry': self._skill_registry.prompt_manifest(),
            'allowed_step_types': list(self._skill_registry.step_types),
            'allowed_skill_names': list(self._skill_registry.allowed_skill_names),
            'allowed_motion_objects': sorted(DEFAULT_PERFORM_MOTION_OBJECT_LABELS),
            'output_contract': dict(self._prompt_pack.output_contract),
        }
        if validation_errors:
            validation_retry = dict(self._prompt_pack.validation_retry)
            validation_instruction = str(validation_retry.get('instruction', '')).strip()
            if not validation_instruction:
                validation_instruction = (
                    'Regenerate the full plan as one valid JSON object. Correct only the '
                    'planner contract errors. Do not ask the user for clarification unless '
                    'the original human request is genuinely ambiguous.'
                )
            try:
                max_output_len = max(
                    0,
                    int(validation_retry.get('previous_model_output_max_chars', 4000)),
                )
            except (TypeError, ValueError):
                max_output_len = 4000
            prompt['validation_retry'] = {
                'errors': list(validation_errors),
                'instruction': validation_instruction,
                'previous_model_output': str(previous_model_output or '')[:max_output_len],
            }
        return [
            {'role': 'system', 'content': self._prompt_pack.system_prompt},
            {'role': 'user', 'content': json.dumps(prompt, sort_keys=True)},
        ]

    def _decision_from_model_output_with_errors(
        self,
        request: PlannerRequest,
        raw_model_output: str,
        *,
        feedback: ExecutionFeedback | None,
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
    ) -> tuple[PlannerDecision | None, list[str]]:
        parsed = extract_json_object(raw_model_output)
        if not parsed:
            return None, ['model output did not contain a JSON object']

        decision_mode = str(parsed.get('decision', parsed.get('mode', 'plan'))).strip().lower()
        if decision_mode in ('clarify', 'clarification', 'fail'):
            decision = self._decision_from_model_output(
                request,
                raw_model_output,
                parsed=parsed,
                feedback=feedback,
                goal_id=goal_id,
                plan_version=plan_version,
                status=status,
                communication_policy=communication_policy,
            )
            return decision, []

        steps, validation_errors = self._extract_plan_steps_with_errors(parsed)
        if validation_errors:
            return None, validation_errors
        if not steps:
            return None, ['model output did not contain executable steps']
        semantic_errors = plan_semantic_errors(
            steps,
            request.grounded_context,
            request.target_selection,
        )
        if semantic_errors:
            return None, semantic_errors
        missing_report_error = missing_requested_report_error(request, steps)
        if missing_report_error:
            return None, [missing_report_error]
        capability_errors = self._requested_capability_errors(request, steps)
        if capability_errors:
            return None, capability_errors

        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=steps,
            validation_status=str(parsed.get('validation_status', 'draft')).strip() or 'draft',
            failure_reason=str(parsed.get('failure_reason', '')).strip(),
            user_facing_reason=str(parsed.get('user_facing_reason', '')).strip(),
            replan_hint=str(parsed.get('replan_hint', '')).strip(),
            retry_budget=self._next_retry_budget(parsed, feedback)[0],
            scene_targets=self._scene_targets_for_decision(
                request,
                feedback,
                parsed,
                steps=steps,
            ),
            plan_id=str(parsed.get('plan_id', parsed.get('id', ''))).strip(),
            raw_model_output=raw_model_output,
            mode='replan' if feedback is not None else 'plan',
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=self._resolved_communication_policy(parsed, communication_policy),
        ), []

    def _decision_from_model_output(
        self,
        request: PlannerRequest,
        raw_model_output: str,
        *,
        parsed: dict | None = None,
        feedback: ExecutionFeedback | None,
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
    ) -> PlannerDecision | None:
        if parsed is None:
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
            validation_status=str(parsed.get('validation_status', 'draft')).strip() or 'draft',
            failure_reason=str(parsed.get('failure_reason', '')).strip(),
            user_facing_reason=str(parsed.get('user_facing_reason', '')).strip(),
            replan_hint=str(parsed.get('replan_hint', '')).strip(),
            retry_budget=self._next_retry_budget(parsed, feedback)[0],
            scene_targets=self._scene_targets_for_decision(
                request,
                feedback,
                parsed,
                steps=steps,
            ),
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
            steps=[],
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

    def _backend_unavailable_decision(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
        raw_model_output: str,
        goal_id: str,
        plan_version: int,
        communication_policy: dict,
    ) -> PlannerDecision:
        reason = 'The planning model is not ready yet. Please try again in a moment.'
        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=[],
            validation_status='failed',
            failure_reason=reason,
            user_facing_reason=reason,
            replan_hint='planner_backend_unavailable',
            retry_budget=0,
            scene_targets=self._scene_targets_for_decision(request, feedback, {}),
            raw_model_output=raw_model_output,
            mode='backend_unavailable',
            goal_id=goal_id,
            plan_version=plan_version,
            status='failed',
            communication_policy=communication_policy,
        )

    def _invalid_model_output_decision(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
        reason: str,
        raw_model_output: str,
        goal_id: str,
        plan_version: int,
        communication_policy: dict,
    ) -> PlannerDecision:
        clean_reason = str(reason or '').strip() or 'planner output was invalid'
        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=[],
            validation_status='invalid',
            failure_reason=clean_reason,
            user_facing_reason=clean_reason,
            replan_hint='planner_invalid_output',
            retry_budget=0,
            scene_targets=self._scene_targets_for_decision(request, feedback, {}),
            raw_model_output=raw_model_output,
            mode='fail',
            goal_id=goal_id,
            plan_version=plan_version,
            status='failed',
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
            'scene_targets': list(request.scene_targets),
            'target_selection': request.target_selection,
            'dialogue_context': list(request.dialogue_context),
            'grounded_context': request.grounded_context,
            'planner_mode': request.planner_mode,
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
            'result_summary': feedback.result_summary,
        }

    def _extract_plan_steps(self, parsed: dict) -> list[dict]:
        steps, validation_errors = self._extract_plan_steps_with_errors(parsed)
        if validation_errors:
            return []
        return steps

    def _extract_plan_steps_with_errors(self, parsed: dict) -> tuple[list[dict], list[str]]:
        steps = parsed.get('steps')
        if not isinstance(steps, list):
            plan_payload = parsed.get('plan', {})
            if isinstance(plan_payload, dict):
                steps = plan_payload.get('steps', [])
        if not isinstance(steps, list):
            return [], ['steps must be a list']
        normalized_steps = normalize_plan_steps(steps)
        if not normalized_steps:
            return [], ['steps list did not contain valid step objects']
        supported_steps, rejected_steps = self._skill_registry.filter_supported_steps_with_rejections(
            normalized_steps
        )
        if rejected_steps:
            return [], [self._step_rejection_reason(step) for step in rejected_steps]
        mixed_say_error = self._mixed_say_step_error(supported_steps)
        if mixed_say_error:
            return [], [mixed_say_error]
        argument_errors = self._step_argument_errors(supported_steps)
        if argument_errors:
            return [], argument_errors
        return strip_live_result_report_summary_text(supported_steps), []

    def _step_argument_errors(self, steps: list[dict]) -> list[str]:
        errors: list[str] = []
        for step in steps:
            errors.extend(self._skill_registry.required_argument_errors(step))
            step_name = str(step.get('name', '')).strip().lower()
            if step_name in {'kb_add', 'kb_revise'}:
                args = step.get('args', {})
                statements = (
                    args.get('statements', args.get('statement', []))
                    if isinstance(args, dict)
                    else []
                )
                if isinstance(statements, str):
                    statements = [statements]
                invalid_statements = (
                    [
                        str(statement).strip()
                        for statement in statements
                        if not is_explicit_knowledge_statement(statement)
                    ]
                    if isinstance(statements, list)
                    else [str(statements)]
                )
                if invalid_statements:
                    errors.append(
                        '%s statements must use explicit "subject predicate object" '
                        'KnowledgeCore form with subject first and a namespace-qualified '
                        'predicate second (for example "cup_1 rdf:type Cup" or '
                        '"cup_1 dbp:color red"); invalid=%s'
                        % (step_name, json.dumps(invalid_statements, sort_keys=True))
                    )
            if step_name != 'perform_motion':
                continue
            args = step.get('args', {})
            motion_object = (
                str(args.get('object', '')).strip().lower()
                if isinstance(args, dict)
                else ''
            )
            if motion_object in DEFAULT_PERFORM_MOTION_OBJECT_LABELS:
                continue
            errors.append(
                'unsupported perform_motion args.object "%s"; '
                'allowed_motion_objects=%s. Decompose composite motions into '
                'explicit supported perform_motion steps.'
                % (
                    motion_object or '<empty>',
                    ','.join(sorted(DEFAULT_PERFORM_MOTION_OBJECT_LABELS)),
                )
            )
        return errors

    def _step_rejection_reason(self, step: dict) -> str:
        step_type = str(step.get('type', '')).strip().lower()
        step_name = str(step.get('name', '')).strip().lower()
        if step_type == 'skill' and step_name not in self._skill_registry.allowed_skill_names:
            return (
                'unsupported skill step name "%s"; allowed_skill_names=%s. '
                'Use decision="clarify" or decision="fail" for user-facing speech; '
                'do not mix speech steps into executable plans.'
                % (step_name or '<empty>', ','.join(self._skill_registry.allowed_skill_names))
            )
        return 'unsupported plan step type="%s" name="%s"' % (
            step_type or '<empty>',
            step_name or '<empty>',
        )

    @classmethod
    def _mixed_say_step_error(cls, steps: list[dict]) -> str:
        has_say = any(cls._is_say_step(step) for step in steps)
        has_executable = any(not cls._is_say_step(step) for step in steps)
        if not has_say or not has_executable:
            return ''
        return (
            'say steps cannot be mixed with executable steps; plan only executable '
            'robot actions and leave completion wording to chatbot_llm after execution'
        )

    @staticmethod
    def _is_say_step(step: dict) -> bool:
        if not isinstance(step, dict):
            return False
        step_type = str(step.get('type', '')).strip().lower()
        step_name = str(step.get('name', '')).strip().lower()
        return step_type == 'say' or step_name == 'say'

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

    def _validated_target_selection_recovery(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
        raw_model_output: str,
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
    ) -> PlannerDecision | None:
        """Compile an already validated target selection after model retries fail."""
        selection = request.target_selection
        if not selection:
            return None

        operation = str(selection.get('operation', '')).strip().lower()
        member_ids = [
            str(member_id).strip()
            for member_id in selection.get('member_ids', [])
            if str(member_id).strip()
        ]
        report_policy = str(selection.get('report_policy', 'none')).strip().lower()
        steps: list[dict] = []

        if operation == 'deliver':
            skill_name = self._first_supported_skill_name('bring_object', 'deliver_object')
            recipient_id = str(selection.get('recipient_id', '')).strip()
            source_id = str(selection.get('source_location_id', '')).strip()
            if not skill_name or not recipient_id:
                return None
            for member_id in member_ids:
                args = {'target': member_id, 'recipient': recipient_id}
                if source_id:
                    args['source'] = source_id
                steps.append(self._step(step_type='skill', name=skill_name, args=args))
                if report_policy == 'per_target':
                    report_name = self._first_supported_skill_name('report_result')
                    if not report_name:
                        return None
                    report = self._step(step_type='skill', name=report_name, args={})
                    report['requires'] = ['step_%d' % len(steps)]
                    steps.append(report)
            if report_policy == 'final':
                report_name = self._first_supported_skill_name('report_result')
                if not report_name:
                    return None
                report_step = self._step(step_type='skill', name=report_name, args={})
                report_step['requires'] = [
                    'step_%d' % index
                    for index, step in enumerate(steps, start=1)
                    if step.get('name') != 'report_result'
                ]
                steps.append(report_step)
            scene_targets = member_ids + [recipient_id]
        elif operation == 'visit':
            skill_name = self._first_supported_skill_name('navigate_to', 'walk_to')
            report_name = self._first_supported_skill_name('report_result')
            if not skill_name or (report_policy in {'per_target', 'final'} and not report_name):
                return None
            previous_step_id = ''
            for member_id in member_ids:
                navigation = self._step(
                    step_type='skill',
                    name=skill_name,
                    args={'target': member_id},
                )
                if previous_step_id:
                    navigation['requires'] = [previous_step_id]
                steps.append(navigation)
                previous_step_id = 'step_%d' % len(steps)
                if report_policy == 'per_target':
                    report = self._step(step_type='skill', name=report_name, args={})
                    report['requires'] = [previous_step_id]
                    steps.append(report)
                    previous_step_id = 'step_%d' % len(steps)
            if report_policy == 'final':
                report = self._step(step_type='skill', name=report_name, args={})
                if previous_step_id:
                    report['requires'] = [previous_step_id]
                steps.append(report)
            scene_targets = member_ids
        else:
            return None

        steps = normalize_plan_steps(steps)
        if not steps or any(not self._skill_registry.supports_step(step) for step in steps):
            return None
        if self._step_argument_errors(steps):
            return None
        if plan_semantic_errors(steps, request.grounded_context, selection):
            return None
        if self._requested_capability_errors(request, steps):
            return None

        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=steps,
            validation_status='valid',
            scene_targets=list(dict.fromkeys(scene_targets)),
            raw_model_output=raw_model_output,
            mode='validated_target_selection_recovery',
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=communication_policy,
        )

    def _validated_motion_sequence_recovery(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
        raw_model_output: str,
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
    ) -> PlannerDecision | None:
        """Compile an explicit composite motion sequence after model retries fail."""
        if feedback is not None or request.target_selection:
            return None
        if str(request.planner_mode or '').strip().lower() != 'multi_step':
            return None

        motion_names = [
            str(intent).strip().lower()
            for intent in request.normalized_intents
            if str(intent).strip()
        ]
        if len(motion_names) < 2 or any(
            name not in DEFAULT_PERFORM_MOTION_OBJECT_LABELS for name in motion_names
        ):
            return None

        steps = [
            self._step(
                step_type='skill',
                name='perform_motion',
                args={'object': motion_name},
                on_failure='replan',
                retry_budget=1,
            )
            for motion_name in motion_names
        ]
        report_name = self._first_supported_skill_name('report_result')
        if not report_name:
            return None
        report = self._step(step_type='skill', name=report_name, args={})
        report['requires'] = [
            'step_%d' % index for index in range(1, len(steps) + 1)
        ]
        steps.append(report)
        steps = normalize_plan_steps(steps)
        if not steps or any(not self._skill_registry.supports_step(step) for step in steps):
            return None
        if self._step_argument_errors(steps):
            return None
        if plan_semantic_errors(steps, request.grounded_context, request.target_selection):
            return None

        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=steps,
            validation_status='valid',
            scene_targets=[],
            raw_model_output=raw_model_output,
            mode='validated_motion_sequence_recovery',
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=communication_policy,
        )

    def _first_supported_skill_name(self, *names: str) -> str:
        for name in names:
            canonical_name = self._skill_registry.resolve_skill_name(name)
            if canonical_name:
                return canonical_name
        return ''

    @staticmethod
    def _target_selection_recovery_omits_capabilities(request: PlannerRequest) -> bool:
        operation = str(request.target_selection.get('operation', '')).strip().lower()
        covered = {'report_result'}
        if operation == 'deliver':
            covered.add('delivery')
        elif operation == 'visit':
            covered.add('navigation')
        requested = set(PlannerEngine._requested_capabilities(request))
        return not requested.issubset(covered)

    @staticmethod
    def _requested_capability_errors(
        request: PlannerRequest,
        steps: list[dict],
    ) -> list[str]:
        required = PlannerEngine._requested_capabilities(request)
        observed = [
            capability
            for capability in (
                PlannerEngine._step_capability(step) for step in steps
            )
            if capability
        ]
        cursor = 0
        missing = []
        for capability in required:
            matching_index = next(
                (
                    index
                    for index in range(cursor, len(observed))
                    if PlannerEngine._capability_matches(capability, observed[index])
                ),
                None,
            )
            if matching_index is None:
                missing.append(capability)
            else:
                cursor = matching_index + 1
        if missing:
            return [
                'incomplete requested action coverage; missing=%s observed=%s'
                % (','.join(missing), ','.join(observed) or '<none>')
            ]

        selection = request.target_selection
        if 'look_at' in required and str(selection.get('operation', '')).lower() == 'visit':
            selected = {
                str(member).strip()
                for member in selection.get('member_ids', [])
                if str(member).strip()
            }
            looked_at = {
                str(step.get('args', {}).get('target_frame', step.get('args', {}).get('target', ''))).strip()
                for step in steps
                if isinstance(step, dict)
                and str(step.get('name', '')).strip().lower() == 'look_at'
                and isinstance(step.get('args'), dict)
            }
            missing_look_targets = sorted(selected - looked_at)
            if missing_look_targets:
                return [
                    'look_at coverage is missing selected visit targets: %s'
                    % ','.join(missing_look_targets)
                ]
        return []

    @staticmethod
    def _requested_capabilities(request: PlannerRequest) -> list[str]:
        capabilities = [
            capability
            for capability in (
                PlannerEngine._intent_capability(intent)
                for intent in request.normalized_intents
            )
            if capability
        ]
        operation = str(request.target_selection.get('operation', '')).strip().lower()
        if operation == 'visit' and 'navigation' not in capabilities:
            return PlannerEngine._collapse_capabilities(
                capabilities,
                aliases={'look_at', 'observation'},
                replacement='navigation',
            )
        if operation != 'deliver':
            return capabilities
        return PlannerEngine._collapse_capabilities(
            capabilities,
            aliases={
                'delivery',
                'find_object',
                'pick_object',
                'navigation',
                'place_object',
            },
            replacement='delivery',
        )

    @staticmethod
    def _collapse_capabilities(
        capabilities: list[str],
        *,
        aliases: set[str],
        replacement: str,
    ) -> list[str]:
        collapsed = []
        replacement_added = False
        for capability in capabilities:
            if capability in aliases:
                if not replacement_added:
                    collapsed.append(replacement)
                    replacement_added = True
                continue
            collapsed.append(capability)
        return collapsed

    @staticmethod
    def _intent_capability(intent) -> str:
        clean = str(intent or '').strip().lower()
        if clean in _MOTION_INTENT_OBJECTS:
            return 'motion:%s' % _MOTION_INTENT_OBJECTS[clean]
        return _INTENT_CAPABILITIES.get(clean, '')

    @staticmethod
    def _step_capability(step: dict) -> str:
        if not isinstance(step, dict):
            return ''
        name = str(step.get('name', '')).strip().lower()
        if name == 'perform_motion':
            args = step.get('args', {}) if isinstance(step.get('args'), dict) else {}
            motion = str(args.get('object', '')).strip().lower()
            return 'motion:%s' % motion if motion else ''
        if name in {'navigate_to', 'walk_to'}:
            return 'navigation'
        if name in {'bring_object', 'deliver_object'}:
            return 'delivery'
        if name in {'scan', 'inspect_area'}:
            return 'observation'
        return _INTENT_CAPABILITIES.get(name, '')

    @staticmethod
    def _capability_matches(required: str, observed: str) -> bool:
        if required == observed:
            return True
        if required == 'observation' and observed == 'look_at':
            return True
        return required == 'motion:stand' and observed == 'motion:standinit'

    def _build_decision(
        self,
        *,
        request: PlannerRequest,
        feedback: ExecutionFeedback | None,
        steps: list[dict],
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
            validation_status=validation_status,
            failure_reason=failure_reason,
            user_facing_reason=user_facing_reason,
            replan_hint=replan_hint,
            retry_budget=retry_budget,
            scene_targets=(
                scene_targets
                if scene_targets is not None
                else self._scene_targets_for_decision(request, feedback, {})
            ),
            steps=steps,
            plan_id=plan_id,
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=communication_policy,
            communication_policy_source='planner_engine:%s' % (mode or 'plan'),
        )
        return PlannerDecision(
            intent_name=self._default_intent_name,
            payload=payload,
            plan_id=payload['plan']['plan_id'],
            raw_model_output=raw_model_output,
            mode=mode,
        )

    @staticmethod
    def _parsed_retry_budget(parsed: dict) -> int | None:
        if 'retry_budget' not in parsed:
            return None
        try:
            return max(0, int(parsed.get('retry_budget', 0)))
        except (TypeError, ValueError):
            return 0

    def _next_retry_budget(
        self,
        parsed: dict,
        feedback: ExecutionFeedback | None,
    ) -> tuple[int, bool]:
        if feedback is not None:
            remaining_from_feedback = max(0, int(feedback.retry_budget) - 1)
        else:
            remaining_from_feedback = self._default_retry_budget
        if feedback is not None and feedback.status in ('failed', 'invalid'):
            exhausted = int(feedback.retry_budget) <= 0
        else:
            exhausted = False
        explicit = self._parsed_retry_budget(parsed)
        if explicit is not None:
            if feedback is not None:
                # Do not let model output increase/reset remaining retries on replans.
                return min(explicit, remaining_from_feedback), exhausted
            return explicit, exhausted
        if feedback is not None:
            return remaining_from_feedback, exhausted
        return self._default_retry_budget, exhausted

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
        *,
        steps: list[dict] | None = None,
    ) -> list[str]:
        request_targets = list(request.scene_targets)
        if steps is not None:
            step_targets = _scene_targets_from_steps(steps)
            admitted_targets = [
                target for target in step_targets if target in request_targets
            ]
            if admitted_targets:
                return admitted_targets
            if len(step_targets) == 1 and len(request_targets) == 1:
                return request_targets
            return []

        parsed_targets = parsed.get('scene_targets', [])
        if isinstance(parsed_targets, list) and parsed_targets:
            return [str(item).strip() for item in parsed_targets if str(item).strip()]
        plan_payload = parsed.get('plan', {})
        nested_targets = plan_payload.get('scene_targets', []) if isinstance(plan_payload, dict) else []
        if isinstance(nested_targets, list) and nested_targets:
            return [str(item).strip() for item in nested_targets if str(item).strip()]
        if feedback is not None and feedback.scene_targets:
            return list(feedback.scene_targets)
        return request_targets
