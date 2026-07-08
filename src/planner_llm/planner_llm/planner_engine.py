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
from planner_common import missing_requested_report_error
from planner_common import normalize_communication_policy
from planner_common import normalize_plan_steps
from planner_common import request_requests_report
from planner_common import strip_live_result_report_summary_text

from planner_llm.providers import BasePlannerProvider
from planner_llm.providers import PlannerProviderError
from planner_llm.prompt_pack import PlannerPromptPack
from planner_llm.prompt_pack import default_prompt_pack
from planner_llm.skill_registry import SkillRegistry


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


def _looks_like_simple_rule_request(request: PlannerRequest) -> bool:
    """Allow deterministic fallback only for one obvious primitive motion/posture."""
    if request_requests_report(request):
        return False
    if request.scene_targets:
        return False
    goal_text = ' %s ' % ' '.join(str(request.goal_text or '').lower().split())
    if not goal_text.strip():
        return True
    if any(
        marker in goal_text
        for marker in (
            ' all ',
            ' every ',
            ' each ',
            ' and ',
            ' then ',
            ' after ',
            ' before ',
            ' report ',
            ' tell me ',
            ' let me know ',
        )
    ):
        return False
    return True


def _looks_like_location_group_delivery(goal_text: str) -> bool:
    clean = _normalized_text(goal_text)
    if not clean:
        return False
    if not any(marker in clean for marker in (' all object', ' every object', ' each object')):
        return False
    if not any(marker in clean for marker in (' bring ', ' deliver ', ' take ', ' carry ')):
        return False
    return any(marker in clean for marker in (' from ', ' in ', ' on '))


def _looks_like_ordered_location_walk(goal_text: str) -> bool:
    clean = _normalized_text(goal_text)
    if not clean:
        return False
    if not any(marker in clean for marker in (' all object', ' every object', ' each object')):
        return False
    if not any(marker in clean for marker in (' walk ', ' navigate ', ' go ', ' move ')):
        return False
    if not any(marker in clean for marker in (' tell me ', ' let me know ', ' report ')):
        return False
    return any(marker in clean for marker in (' from ', ' in ', ' on '))


def _looks_like_grounded_look_at(goal_text: str) -> bool:
    clean = _normalized_text(goal_text)
    return any(marker in clean for marker in (' look at ', ' gaze at ', ' face '))


def _goal_text_requests_report(goal_text: str) -> bool:
    clean = _normalized_text(goal_text)
    return any(
        marker in clean
        for marker in (
            ' report ',
            ' tell me ',
            ' let me know ',
            ' what you did ',
            ' what happened ',
        )
    )


def _matched_location_group(grounded_context: dict, goal_text: str) -> dict:
    groups = grounded_context.get('locations', []) if isinstance(grounded_context, dict) else []
    if not isinstance(groups, list):
        return {}
    matches = []
    for group in groups:
        if not isinstance(group, dict):
            continue
        score = _entity_goal_match_score(group, goal_text)
        if score > 0:
            matches.append((score, group))
    if matches:
        best_score = max(score for score, _group in matches)
        best_matches = [group for score, group in matches if score == best_score]
        if len(best_matches) == 1:
            return best_matches[0]
    if not matches and len(groups) == 1 and _mentions_location_collection(goal_text):
        group = groups[0]
        return group if isinstance(group, dict) else {}
    return {}


def _location_group_object_members(location_group: dict) -> list[dict]:
    """Return concrete object members of a matched location group."""
    members = []
    if not isinstance(location_group, dict):
        return members
    for item in location_group.get('contains', []):
        if not isinstance(item, dict):
            continue
        item_id = str(item.get('id', '')).strip()
        kind = str(item.get('kind', 'object')).strip().lower()
        if item_id and kind == 'object':
            members.append(item)
    return members


def _member_ids(members: list[dict]) -> list[str]:
    ids = []
    for member in members:
        member_id = str(member.get('id', '')).strip()
        if member_id and member_id not in ids:
            ids.append(member_id)
    return ids


def _matched_recipient_entity(
    grounded_context: dict,
    goal_text: str,
    *,
    exclude_id: str = '',
    source_location_group: dict | None = None,
) -> str:
    if not isinstance(grounded_context, dict):
        return ''
    if ' to ' not in _normalized_text(goal_text):
        return ''
    excluded = str(exclude_id or '').strip()
    entity_index = _entity_index(grounded_context)
    candidates = []
    for entity in grounded_context.get('entities', []):
        if not isinstance(entity, dict):
            continue
        entity_id = str(entity.get('id', '')).strip()
        if entity_id == excluded:
            continue
        kind = str(entity.get('kind', '')).strip().lower()
        entity_class = str(entity.get('class', '')).strip().lower()
        if kind != 'person' and 'human' not in entity_class and 'person' not in entity_class:
            continue
        if _entity_mentioned(entity, goal_text):
            candidates.append((entity_id, entity))
    if len(candidates) == 1:
        return candidates[0][0]
    if len(candidates) > 1 and excluded:
        source_entity = entity_index.get(excluded, {})
        source_scope = _entity_location_scope(source_entity, entity_index)
        if isinstance(source_location_group, dict):
            source_scope.update(_entity_location_scope(source_location_group, entity_index))
        scoped_matches = [
            entity_id for entity_id, entity in candidates
            if source_scope and source_scope.intersection(_entity_location_scope(entity, entity_index))
        ]
        if len(scoped_matches) == 1:
            return scoped_matches[0]
        namespace_matches = _unique_best_namespace_matches(excluded, candidates)
        if len(namespace_matches) == 1:
            return namespace_matches[0]
        return ''
    for group in grounded_context.get('locations', []):
        if (
            isinstance(group, dict)
            and str(group.get('id', '')).strip() != excluded
            and _entity_mentioned(group, goal_text)
        ):
            return str(group.get('id', '')).strip()
    return ''


def _entity_index(grounded_context: dict) -> dict[str, dict]:
    if not isinstance(grounded_context, dict):
        return {}
    indexed: dict[str, dict] = {}
    for entity in grounded_context.get('entities', []):
        if not isinstance(entity, dict):
            continue
        entity_id = str(entity.get('id', '')).strip()
        if entity_id and entity_id not in indexed:
            indexed[entity_id] = entity
    return indexed


def _entity_location_scope(entity: dict, entity_index: dict[str, dict]) -> set[str]:
    if not isinstance(entity, dict):
        return set()
    scope: set[str] = set()
    for target_id in _entity_location_targets(entity):
        scope.add(target_id)
        parent = entity_index.get(target_id, {})
        scope.update(_entity_location_targets(parent))
    return scope


def _entity_location_targets(entity: dict) -> set[str]:
    targets: set[str] = set()
    if not isinstance(entity, dict):
        return targets
    for relation in entity.get('relations', []):
        if not isinstance(relation, dict):
            continue
        predicate = str(relation.get('predicate', '')).strip()
        if predicate not in {'oro:isAt', 'oro:isIn', 'oro:isOn'}:
            continue
        target_id = str(relation.get('object', '')).strip()
        if target_id:
            targets.add(target_id)
    return targets


def _unique_best_namespace_matches(source_id: str, candidates: list[tuple[str, dict]]) -> list[str]:
    source_tokens = _namespace_tokens(source_id)
    if len(source_tokens) < 2:
        return []
    scored = []
    for entity_id, _entity in candidates:
        candidate_tokens = _namespace_tokens(entity_id)
        score = 0
        for source_token, candidate_token in zip(source_tokens, candidate_tokens):
            if source_token != candidate_token:
                break
            score += 1
        if score >= 2:
            scored.append((score, entity_id))
    if not scored:
        return []
    best_score = max(score for score, _entity_id in scored)
    return sorted(entity_id for score, entity_id in scored if score == best_score)


def _namespace_tokens(value: str) -> list[str]:
    return [
        token for token in str(value or '').strip().lower().split('_')
        if token and token not in {'person', 'recipient'}
    ]


def _matched_action_target_entity(grounded_context: dict, goal_text: str) -> str:
    """Return one grounded entity that is clearly referenced by the goal text."""
    if not isinstance(grounded_context, dict):
        return ''
    matches: list[tuple[int, str]] = []
    for entity in grounded_context.get('entities', []):
        if not isinstance(entity, dict):
            continue
        entity_id = str(entity.get('id', '')).strip()
        if not entity_id:
            continue
        score = _entity_goal_match_score(entity, goal_text)
        if score > 0:
            matches.append((score, entity_id))
    if not matches:
        return ''
    best_score = max(score for score, _entity_id in matches)
    best_matches = sorted(entity_id for score, entity_id in matches if score == best_score)
    if len(best_matches) == 1:
        return best_matches[0]
    return ''


def _entity_mentioned(entity: dict, goal_text: str) -> bool:
    return _entity_goal_match_score(entity, goal_text) > 0


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


def _mentions_location_collection(goal_text: str) -> bool:
    clean = _normalized_text(goal_text)
    return any(marker in clean for marker in (' from ', ' in ', ' on '))


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

        location_group_decision = self._location_group_delivery_decision(
            request,
            feedback=feedback,
            raw_model_output='',
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status=status,
            communication_policy=resolved_policy,
        )
        if location_group_decision is not None:
            return location_group_decision

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
            try:
                retry_raw_model_output = self._provider.generate(
                    self._build_messages(
                        request,
                        state_t0=state_t0 or {},
                        feedback=feedback,
                        goal_id=resolved_goal_id,
                        plan_version=resolved_plan_version,
                        validation_errors=validation_errors,
                        previous_model_output=raw_model_output,
                    )
                )
            except PlannerProviderError as err:
                retry_raw_model_output = 'planner backend unavailable during validation retry: %s' % err
            else:
                retry_decision, retry_validation_errors = self._decision_from_model_output_with_errors(
                    request,
                    retry_raw_model_output,
                    feedback=feedback,
                    goal_id=resolved_goal_id,
                    plan_version=resolved_plan_version,
                    status=status,
                    communication_policy=resolved_policy,
                )
                if retry_decision is not None:
                    return retry_decision
                validation_errors = retry_validation_errors or validation_errors
                raw_model_output = '%s\n\n--- invalid retry output ---\n%s' % (
                    raw_model_output,
                    retry_raw_model_output,
                )

        rule_fallback_decision = self._rule_based_decision(
            request,
            feedback=feedback,
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status=status,
            communication_policy=resolved_policy,
            mode='rule_fallback',
        )
        if rule_fallback_decision is not None:
            return rule_fallback_decision

        ordered_walk_decision = self._ordered_location_walk_decision(
            request,
            feedback=feedback,
            raw_model_output=raw_model_output,
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status=status,
            communication_policy=resolved_policy,
        )
        if ordered_walk_decision is not None:
            return ordered_walk_decision

        grounded_look_decision = self._grounded_look_decision(
            request,
            feedback=feedback,
            raw_model_output=raw_model_output,
            goal_id=resolved_goal_id,
            plan_version=resolved_plan_version,
            status=status,
            communication_policy=resolved_policy,
        )
        if grounded_look_decision is not None:
            return grounded_look_decision

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
        missing_report_error = missing_requested_report_error(request, steps)
        if missing_report_error:
            return None, [missing_report_error]

        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=steps,
            validation_status=str(parsed.get('validation_status', 'draft')).strip() or 'draft',
            failure_reason=str(parsed.get('failure_reason', '')).strip(),
            user_facing_reason=str(parsed.get('user_facing_reason', '')).strip(),
            replan_hint=str(parsed.get('replan_hint', '')).strip(),
            retry_budget=self._next_retry_budget(parsed, feedback)[0],
            scene_targets=self._scene_targets_for_decision(request, feedback, parsed),
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
                )
            ],
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
            steps=[
                self._step(
                    step_type='say',
                    name='say',
                    args={'text': reason},
                )
            ],
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
            steps=[
                self._step(
                    step_type='say',
                    name='say',
                    args={'text': clean_reason},
                )
            ],
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

    def _rule_based_decision(
        self,
        request: PlannerRequest,
        *,
        feedback: ExecutionFeedback | None,
        goal_id: str,
        plan_version: int,
        status: str,
        communication_policy: dict,
        mode: str = 'rule',
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
        if not _looks_like_simple_rule_request(request):
            return None

        retry_budget = self._next_retry_budget({}, feedback)[0]
        scene_targets = self._scene_targets_for_decision(request, feedback, {})
        motion_skill_name = self._first_supported_skill_name('perform_motion', 'motion')

        for normalized_intent in request.normalized_intents:
            motion_name = _RULE_BASED_MOTIONS.get(normalized_intent)
            if motion_name and motion_skill_name:
                if request_requests_report(request):
                    return None
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
                    validation_status='draft',
                    retry_budget=retry_budget,
                    scene_targets=scene_targets,
                    mode=mode,
                    goal_id=goal_id,
                    plan_version=plan_version,
                    status=status,
                    communication_policy=communication_policy,
                )

        return None

    def _location_group_delivery_decision(
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
        """Fallback for grounded "bring every object from X to Y" requests."""
        goal_text = str(request.goal_text or '').strip()
        if not _looks_like_location_group_delivery(goal_text):
            return None
        bring_skill_name = self._first_supported_skill_name('bring_object', 'deliver_object')
        if not bring_skill_name:
            return None
        location_group = _matched_location_group(
            request.grounded_context,
            goal_text,
        )
        if not location_group:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='Which location should I collect the objects from?',
                raw_model_output=raw_model_output,
                mode='clarify',
                goal_id=goal_id,
                plan_version=plan_version,
                status='waiting_user',
                communication_policy=communication_policy,
            )
        recipient = _matched_recipient_entity(
            request.grounded_context,
            goal_text,
            exclude_id=str(location_group.get('id', '')).strip(),
            source_location_group=location_group,
        )
        if not recipient:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='Who or where should I bring those objects to?',
                raw_model_output=raw_model_output,
                mode='clarify',
                goal_id=goal_id,
                plan_version=plan_version,
                status='waiting_user',
                communication_policy=communication_policy,
            )
        members = _location_group_object_members(location_group)
        if not members:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='I do not have any current objects grounded in that location.',
                raw_model_output=raw_model_output,
                mode='clarify',
                goal_id=goal_id,
                plan_version=plan_version,
                status='waiting_user',
                communication_policy=communication_policy,
            )

        steps = []
        for member in members:
            args = {
                'target': str(member.get('id', '')).strip(),
                'recipient': recipient,
                'source': str(location_group.get('id', '')).strip(),
            }
            steps.append(
                self._step(
                    step_type='skill',
                    name=bring_skill_name,
                    args=args,
                    on_failure='replan',
                )
            )
        if request_requests_report(request):
            steps.append(
                self._step(
                    step_type='skill',
                    name='report_result',
                    args={},
                    on_failure='fail',
                )
            )
        steps = normalize_plan_steps(steps)
        supported_steps, rejected_steps = (
            self._skill_registry.filter_supported_steps_with_rejections(steps)
        )
        if rejected_steps:
            return None
        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=supported_steps,
            validation_status='draft',
            retry_budget=self._next_retry_budget({}, feedback)[0],
            scene_targets=_member_ids(members) + [recipient],
            raw_model_output=raw_model_output,
            mode='grounded_location_group_fallback',
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=communication_policy,
        )

    def _ordered_location_walk_decision(
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
        """Fallback for grounded ordered walk/report after invalid model output."""
        goal_text = str(request.goal_text or '').strip()
        if not _looks_like_ordered_location_walk(goal_text):
            return None
        navigate_skill_name = self._first_supported_skill_name('navigate_to', 'walk_to')
        if not navigate_skill_name or 'report_result' not in self._skill_registry.allowed_skill_names:
            return None
        location_group = _matched_location_group(
            request.grounded_context,
            goal_text,
        )
        if not location_group:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='Which location should I use for the object sequence?',
                raw_model_output=raw_model_output,
                mode='clarify',
                goal_id=goal_id,
                plan_version=plan_version,
                status='waiting_user',
                communication_policy=communication_policy,
            )
        members = _location_group_object_members(location_group)
        if not members:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='I do not have any current objects grounded in that location.',
                raw_model_output=raw_model_output,
                mode='clarify',
                goal_id=goal_id,
                plan_version=plan_version,
                status='waiting_user',
                communication_policy=communication_policy,
            )

        steps = []
        previous_report_id = ''
        step_index = 1
        for member in members:
            member_id = str(member.get('id', '')).strip()
            navigate_id = 'step_%d' % step_index
            step_index += 1
            navigate_step = self._step(
                step_type='skill',
                name=navigate_skill_name,
                args={'target': member_id},
                on_failure='replan',
            )
            navigate_step['id'] = navigate_id
            if previous_report_id:
                navigate_step['requires'] = [previous_report_id]
            steps.append(navigate_step)

            report_id = 'step_%d' % step_index
            step_index += 1
            report_step = self._step(
                step_type='skill',
                name='report_result',
                args={},
                on_failure='fail',
            )
            report_step['id'] = report_id
            report_step['requires'] = [navigate_id]
            steps.append(report_step)
            previous_report_id = report_id

        steps = normalize_plan_steps(steps)
        supported_steps, rejected_steps = (
            self._skill_registry.filter_supported_steps_with_rejections(steps)
        )
        if rejected_steps:
            return None
        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=supported_steps,
            validation_status='draft',
            retry_budget=self._next_retry_budget({}, feedback)[0],
            scene_targets=_member_ids(members),
            raw_model_output=raw_model_output,
            mode='grounded_ordered_walk_fallback',
            goal_id=goal_id,
            plan_version=plan_version,
            status=status,
            communication_policy=communication_policy,
        )

    def _grounded_look_decision(
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
        """Fallback for grounded look-at requests after invalid model output."""
        goal_text = str(request.goal_text or '').strip()
        if not _looks_like_grounded_look_at(goal_text):
            return None
        look_skill_name = self._first_supported_skill_name('look_at')
        if not look_skill_name:
            return None
        target = self._first_scene_target(request) or _matched_action_target_entity(
            request.grounded_context,
            goal_text,
        )
        if not target:
            return self._clarification_decision(
                request,
                feedback=feedback,
                reason='Which object or person should I look at?',
                raw_model_output=raw_model_output,
                mode='clarify',
                goal_id=goal_id,
                plan_version=plan_version,
                status='waiting_user',
                communication_policy=communication_policy,
            )

        steps = [
            self._step(
                step_type='skill',
                name=look_skill_name,
                args={'target_frame': target},
                on_failure='replan',
            )
        ]
        if request_requests_report(request) or _goal_text_requests_report(goal_text):
            steps.append(
                self._step(
                    step_type='skill',
                    name='report_result',
                    args={},
                    on_failure='fail',
                )
            )
        steps = normalize_plan_steps(steps)
        supported_steps, rejected_steps = (
            self._skill_registry.filter_supported_steps_with_rejections(steps)
        )
        if rejected_steps:
            return None
        return self._build_decision(
            request=request,
            feedback=feedback,
            steps=supported_steps,
            validation_status='draft',
            retry_budget=self._next_retry_budget({}, feedback)[0],
            scene_targets=[target],
            raw_model_output=raw_model_output,
            mode='grounded_look_fallback',
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
            'scene_targets': list(request.scene_targets),
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

    @staticmethod
    def _step_argument_errors(steps: list[dict]) -> list[str]:
        errors: list[str] = []
        for step in steps:
            if str(step.get('name', '')).strip().lower() != 'perform_motion':
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
            scene_targets=scene_targets or self._scene_targets_for_decision(request, feedback, {}),
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

    @staticmethod
    def _first_scene_target(request: PlannerRequest) -> str:
        for target in request.scene_targets:
            clean = str(target or '').strip()
            if not clean:
                continue
            try:
                parsed = json.loads(clean.replace("'", '"'))
            except json.JSONDecodeError:
                parsed = {}
            if isinstance(parsed, dict):
                candidate = str(parsed.get('id', parsed.get('target', ''))).strip()
                if candidate:
                    return candidate
            return clean
        return ''

    def _first_supported_skill_name(self, *names: str) -> str:
        for name in names:
            clean_name = str(name or '').strip().lower()
            if clean_name and clean_name in self._skill_registry.allowed_skill_names:
                return clean_name
        return ''
