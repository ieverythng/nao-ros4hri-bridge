#!/usr/bin/env python3
"""Lifecycle orchestrator for ROS4HRI intents.

This node is intentionally downstream-only: it receives normalized intents from
the dialogue stack, deduplicates them, and dispatches the corresponding NAO
skill endpoints without taking over prompt or dialogue ownership.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import threading
import time

from chatbot_msgs.msg import DialogueRole, Utterance
from chatbot_msgs.srv import DialogueInteraction
from communication_skills.action import Say
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from hri_actions_msgs.msg import Intent
from kb_skills.intent_labels import KB_QUERY_INTENTS
from kb_skills.mutation_client import KnowledgeCoreMutationClient
from kb_skills.query_client import KnowledgeCoreQueryClient
from planner_common import build_execution_feedback_payload
from planner_common import build_report_outcome
from planner_common import is_explicit_knowledge_statement
from planner_common import make_plan_id
from planner_common import load_shared_skill_manifest
from planner_common import merge_fake_skill_aliases
from planner_common import merge_scan_skill_names
from planner_common import parse_json_object
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID as UUIDMsg

from nao_orchestrator.action_timing import ActionDeadline
from nao_orchestrator.intent_rules import (
    build_scan_result_payload,
    classify_motion_target,
    is_unresolved_report_template,
    make_intent_signature,
    normalize_incoming_intent,
    normalize_legacy_intent,
    parse_intent_data,
    parse_plan_envelope,
    posture_topic_fallback_for_motion,
    resolve_say_text,
    scan_step_should_auto_report,
    validate_execution_plan,
)
from nao_orchestrator.kb_effects import SPATIAL_EFFECT_CLEANUP_PREDICATES
from nao_orchestrator.kb_effects import SPATIAL_EFFECT_PREDICATES
from nao_orchestrator.kb_effects import SPATIAL_EFFECT_SKILLS
from nao_orchestrator.kb_effects import remove_stale_spatial_effect_values
from nao_orchestrator.kb_effects import validate_spatial_effect_statements
from nao_orchestrator.planner_gate import PlannerGate

try:  # pragma: no cover - available once nao_skills interfaces are rebuilt
    from nao_skills.action import DoHeadMotion, ReplayMotion, ScanScene
except ImportError:  # pragma: no cover - forward-compat for stale interface install
    DoHeadMotion = None
    ReplayMotion = None
    ScanScene = None

try:  # pragma: no cover - runtime interface dependency
    from interaction_skills.action import LookAt
except ImportError:  # pragma: no cover - local tests may not have generated actions
    LookAt = None

try:  # pragma: no cover - runtime dependency
    from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
except ImportError:  # pragma: no cover - runtime dependency
    JointAnglesWithSpeed = None

# Posture bridge JSON may report `crouch` where orchestrator expects `kneel`.
_POSTURE_BRIDGE_NAME_ALIASES = {'stand': 'stand', 'sit': 'sit', 'kneel': 'crouch'}
_DEFAULT_SCAN_SKILL_ALIASES = (
    'scan',
    'look_around',
    'inspect_scene',
    'check_visible_entities',
)
_DEFAULT_FAKE_SKILL_ALIASES = {
    'navigate_to': {'navigate_to', 'go_to', 'move_to_location'},
    'find_object': {'find_object', 'find', 'locate_object', 'find_person'},
    'perform_motion': {'perform_motion', 'motion', 'posture', 'head_motion'},
    'wave_greet': {'wave_greet', 'wave', 'greet_wave', 'wave_hello'},
    'inspect_area': {'inspect_area', 'inspect', 'check_area'},
    'walk_to': {'walk_to', 'walk_forward', 'step_to'},
    'pick_object': {'pick_object', 'pick', 'grab', 'grab_object'},
    'place_object': {'place_object', 'place', 'put_down'},
    'bring_object': {'bring_object', 'bring', 'deliver_object'},
}
_ASK_USER_STEP_NAMES = frozenset({'ask_user', 'ask_clarification', 'ask_for_help'})
_MAX_RELAYED_PLANNER_ACTS = 256
_MAX_PLANNER_REQUEST_CONTEXTS = 64
_MAX_EXECUTION_REPORT_STEPS = 8
_KB_MUTATION_OPERATIONS = {
    'kb_add': 'add',
    'kb_remove': 'remove',
    'kb_revise': 'update',
}
_SPATIAL_EFFECT_PREDICATES = SPATIAL_EFFECT_PREDICATES
_SPATIAL_EFFECT_CLEANUP_PREDICATES = SPATIAL_EFFECT_CLEANUP_PREDICATES
_SPATIAL_EFFECT_SKILLS = SPATIAL_EFFECT_SKILLS


def _first_non_empty_text(*values) -> str:
    for value in values:
        text = str(value or '').strip()
        if text:
            return text
    return ''


def _first_non_empty_value(data: dict, *keys: str) -> str:
    if not isinstance(data, dict):
        return ''
    return _first_non_empty_text(*(data.get(key, '') for key in keys))


def _statement_parts(statement: str) -> tuple[str, str, str]:
    tokens = str(statement or '').strip().split()
    if len(tokens) < 3:
        return '', '', ''
    return tokens[0], tokens[1], ' '.join(tokens[2:])


def _single_entity_statement(statement: str) -> str:
    tokens = str(statement or '').strip().split()
    if len(tokens) != 1:
        return ''
    token = tokens[0]
    if any(char in token for char in ('"', "'", '{', '}', '[', ']')):
        return ''
    return token


def _kb_remove_subject_alias(statement: str) -> str:
    tokens = str(statement or '').strip().split()
    if not tokens:
        return ''
    if len(tokens) == 1:
        return _single_entity_statement(statement)
    subject, predicate, obj = _statement_parts(statement)
    if (
        subject
        and predicate == 'rdf:type'
        and obj in {'owl:Thing', 'Thing', 'oro:Entity'}
    ):
        return subject
    if (
        len(tokens) == 5
        and tokens[1:5] == ['is', 'in', 'knowledge', 'base']
        and _single_entity_statement(tokens[0])
    ):
        return tokens[0]
    return ''


def _binding_value(row: dict, key: str) -> str:
    if not isinstance(row, dict):
        return ''
    return str(row.get(key, row.get('?%s' % key, ''))).strip()


def _statement_from_binding(subject: str, predicate: str, row: dict) -> str:
    clean_subject = str(subject or '').strip()
    clean_predicate = str(predicate or '').strip()
    clean_object = _binding_value(row, 'object')
    if not clean_subject or not clean_predicate or not clean_object:
        return ''
    return '%s %s %s' % (clean_subject, clean_predicate, clean_object)


def _statement_from_values(subject: str, predicate: str, obj: str) -> str:
    clean_subject = str(subject or '').strip()
    clean_predicate = str(predicate or '').strip()
    clean_object = str(obj or '').strip()
    if not clean_subject or not clean_predicate or not clean_object:
        return ''
    return '%s %s %s' % (clean_subject, clean_predicate, clean_object)


def _dedupe_statements(statements: list[str]) -> list[str]:
    deduped: list[str] = []
    seen: set[str] = set()
    for statement in statements:
        clean = str(statement or '').strip()
        if not clean or clean in seen:
            continue
        seen.add(clean)
        deduped.append(clean)
    return deduped


def _kb_effects_from_result_payload(result_payload: dict) -> list[dict]:
    """Return structured KB effects reported by a successful skill payload."""
    if not isinstance(result_payload, dict):
        return []
    evidence = result_payload.get('evidence', {})
    if not isinstance(evidence, dict):
        return []
    effects = evidence.get('kb_effects', [])
    if not isinstance(effects, list):
        return []
    return [item for item in effects if isinstance(item, dict)]


def _group_kb_effect_statements(effects: list[dict]) -> dict[str, list[str]]:
    grouped: dict[str, list[str]] = {}
    for effect in effects:
        operation = str(
            effect.get('operation', effect.get('action', ''))
        ).strip().lower()
        if operation == 'revise':
            operation = 'update'
        if operation not in {'add', 'remove', 'update'}:
            continue
        statements = KnowledgeCoreMutationClient.coerce_statements(
            effect.get('statements', effect.get('statement', []))
        )
        if statements:
            grouped.setdefault(operation, []).extend(statements)
    return {
        operation: _dedupe_statements(statements)
        for operation, statements in grouped.items()
        if statements
    }


def _normalize_execution_mode(value) -> str:
    clean = str(value or '').strip().lower()
    return clean if clean in {'real', 'fake'} else 'fake'


def _looks_like_machine_payload(text: str) -> bool:
    clean_text = str(text or '').strip()
    return clean_text.startswith(('{', '[', '```', '"{'))


def _report_text_from_result_payload(result_payload: dict) -> str:
    """Resolve conservative report text from a prior live skill result payload."""
    if not isinstance(result_payload, dict):
        return ''

    report_text = _first_non_empty_value(
        result_payload,
        'summary_text',
        'result_summary',
        'message',
    )
    if report_text and not is_unresolved_report_template(report_text):
        return report_text

    skill_name = str(result_payload.get('skill', '')).strip().lower()
    if skill_name == 'scan' or any(key in result_payload for key in ('objects', 'people')):
        scan_payload = build_scan_result_payload(result_payload)
        report_text = str(scan_payload.get('summary_text', '')).strip()
        if report_text and not is_unresolved_report_template(report_text):
            return report_text

    target = _first_non_empty_value(result_payload, 'target', 'object', 'location')
    status = str(result_payload.get('status', '')).strip().lower()
    if target and status in ('succeeded', 'success', 'completed'):
        return 'I completed the task for %s.' % target
    return ''


def _execution_step_record(
    step: dict,
    *,
    status: str,
    reason: str = '',
    result_summary: str = '',
    result_payload: dict | None = None,
) -> dict:
    """Build compact execution evidence for chatbot-authored reports."""
    return {
        'id': str(step.get('id', '')).strip(),
        'type': str(step.get('type', '')).strip().lower(),
        'name': str(step.get('name', '')).strip().lower(),
        'args': dict(step.get('args', {}))
        if isinstance(step.get('args', {}), dict)
        else {},
        'status': str(status or '').strip().lower(),
        'reason': str(reason or '').strip(),
        'result_summary': str(result_summary or '').strip(),
        'result_payload': dict(result_payload or {}),
    }


def _plan_outcome_summary(
    plan: list[dict],
    execution_results: list[dict],
    *,
    terminal_reason: str = '',
    terminal_step: dict | None = None,
) -> dict:
    """Build structured execution outcome data without user-facing wording."""
    action_steps = [
        step for step in plan
        if isinstance(step, dict) and _is_required_action_step(step)
    ]
    completed_targets: list[str] = []
    failed_targets: list[str] = []
    completed_step_ids: set[str] = set()
    failed_step_ids: set[str] = set()
    last_successful_step_id = ''
    terminal_step_id = ''
    for result in execution_results:
        if not isinstance(result, dict):
            continue
        step_id = str(result.get('id', '')).strip()
        target = _target_from_step_record(result)
        status = str(result.get('status', '')).strip().lower()
        if status == 'succeeded':
            if step_id:
                completed_step_ids.add(step_id)
                last_successful_step_id = step_id
            if target and target not in completed_targets:
                completed_targets.append(target)
        elif status == 'failed':
            if step_id:
                failed_step_ids.add(step_id)
                terminal_step_id = step_id
            if target and target not in failed_targets:
                failed_targets.append(target)

    required_targets = [
        _target_from_plan_step(step) for step in action_steps
    ]
    pending_targets = [
        target for step, target in zip(action_steps, required_targets)
        if target
        and str(step.get('id', '')).strip() not in completed_step_ids
        and str(step.get('id', '')).strip() not in failed_step_ids
    ]
    if terminal_step is not None and isinstance(terminal_step, dict):
        terminal_step_id = str(terminal_step.get('id', '')).strip() or terminal_step_id
    all_required_steps_succeeded = bool(action_steps) and all(
        str(step.get('id', '')).strip() in completed_step_ids
        for step in action_steps
    )
    return {
        'completed_targets': completed_targets,
        'failed_targets': failed_targets,
        'pending_targets': pending_targets,
        'last_successful_step_id': last_successful_step_id,
        'terminal_step_id': terminal_step_id,
        'terminal_reason': str(terminal_reason or '').strip(),
        'all_required_steps_succeeded': all_required_steps_succeeded,
    }


def _report_context_plan_outcome_summary(summary: dict) -> dict:
    """Return the non-target status fields safe for chatbot wording turns."""
    if not isinstance(summary, dict):
        return {}
    safe_keys = (
        'last_successful_step_id',
        'terminal_step_id',
        'terminal_reason',
        'all_required_steps_succeeded',
    )
    return {
        key: summary[key]
        for key in safe_keys
        if key in summary
    }


def _is_required_action_step(step: dict) -> bool:
    step_name = str(step.get('name', '')).strip().lower()
    step_type = str(step.get('type', '')).strip().lower()
    return step_type in ('skill', 'look_at') and step_name not in (
        'report_result',
        'say',
        *_ASK_USER_STEP_NAMES,
    )


def _target_from_step_record(result: dict) -> str:
    payload = result.get('result_payload', {})
    if not isinstance(payload, dict):
        payload = {}
    return _first_non_empty_value(
        payload,
        'target',
        'object',
        'object_id',
        'target_frame',
        'location',
    ) or _target_from_plan_step(result)


def _target_from_plan_step(step: dict) -> str:
    args = step.get('args', {})
    if not isinstance(args, dict):
        args = {}
    return _first_non_empty_value(
        args,
        'target',
        'object',
        'object_id',
        'target_frame',
        'location',
    )


def _execution_report_dialogue_context(dialogue_context: object) -> list[str]:
    """Keep user request context without re-feeding assistant wording."""
    if not isinstance(dialogue_context, list):
        return []
    bounded: list[str] = []
    for item in dialogue_context:
        text = str(item).strip()
        if not text:
            continue
        if text.lower().startswith('assistant:'):
            continue
        bounded.append(text)
    return bounded[-_MAX_EXECUTION_REPORT_STEPS:]


def _report_text_from_execution_results(execution_results: list) -> str:
    if not isinstance(execution_results, list):
        return ''
    summaries = []
    for step in execution_results[-_MAX_EXECUTION_REPORT_STEPS:]:
        if not isinstance(step, dict):
            continue
        if str(step.get('status', '')).strip().lower() != 'succeeded':
            continue
        summary = str(step.get('result_summary', '')).strip()
        if summary and not is_unresolved_report_template(summary) and summary not in summaries:
            summaries.append(summary)
    return ' '.join(summaries)


def _motion_result_payload(route: str, step_args: dict, resolved_payload: dict) -> dict:
    """Build non-spoken execution evidence for successful motion steps."""
    motion_label = _first_non_empty_value(
        resolved_payload,
        'motion_name',
        'motion',
        'name',
        'target',
        'policy',
    )
    if not motion_label:
        motion_label = _first_non_empty_value(
            step_args,
            'motion',
            'name',
            'target',
            'object',
            'policy',
        )
    clean_label = str(motion_label or route or 'motion').strip()
    summary_text = _motion_summary_text(route, clean_label)
    return {
        'skill': 'perform_motion',
        'route': str(route or '').strip(),
        'motion': clean_label,
        'status': 'succeeded',
        'summary_text': summary_text,
        'metadata': {
            'speech_produced': False,
        },
    }


def _motion_summary_text(route: str, motion_label: str) -> str:
    clean_label = str(motion_label or '').strip().lower().replace('_', ' ')
    if clean_label.startswith('head look '):
        direction = clean_label.removeprefix('head look ').strip()
        return 'I moved my head %s.' % direction
    if clean_label == 'head center':
        return 'I centered my head.'
    if str(route or '').strip().lower() == 'look_at_reset':
        return 'I reset my gaze.'
    if clean_label:
        return 'I performed %s.' % clean_label
    return 'I performed the requested motion.'


def _planner_dialogue_act_signature(payload: str) -> str:
    """Canonicalize one semantic planner event for relay deduplication."""
    try:
        parsed = json.loads(str(payload or '').strip())
    except (TypeError, ValueError, json.JSONDecodeError):
        return str(payload or '').strip()
    if not isinstance(parsed, dict):
        return str(payload or '').strip()
    try:
        plan_version = max(0, int(parsed.get('plan_version', 0) or 0))
    except (TypeError, ValueError):
        plan_version = 0
    signature = {
        'goal_id': str(parsed.get('goal_id', '')).strip(),
        'plan_id': str(parsed.get('plan_id', '')).strip(),
        'plan_version': plan_version,
        'act': str(parsed.get('act', '')).strip().lower(),
    }
    if signature['act'] == 'progress_update':
        signature.update(
            {
                'reason': str(parsed.get('reason', '')).strip(),
                'text_hint': str(parsed.get('text_hint', '')).strip(),
                'context': parsed.get('context', {})
                if isinstance(parsed.get('context', {}), dict)
                else {},
            }
        )
    elif signature['act'] == 'ask_clarification':
        slots_needed = parsed.get('slots_needed', [])
        signature['slots_needed'] = sorted(
            str(slot).strip()
            for slot in slots_needed
            if str(slot).strip()
        ) if isinstance(slots_needed, list) else []
    return json.dumps(signature, sort_keys=True, separators=(',', ':'), ensure_ascii=True)


@dataclass(slots=True)
class _RuntimeStats:
    intents_received: int = 0
    duplicates_ignored: int = 0
    plans_started: int = 0
    plans_succeeded: int = 0
    plans_failed: int = 0
    dispatched_say: int = 0
    dispatched_replay_motion: int = 0
    dispatched_head_motion: int = 0
    dispatched_look_at: int = 0
    dispatched_fake_skill: int = 0
    dispatched_kb_mutation: int = 0
    dispatch_failures: int = 0
    last_intent: str = ''
    last_route: str = ''
    last_plan_id: str = ''
    last_plan_status: str = ''


@dataclass(slots=True)
class _ActionExecutionResult:
    accepted: bool
    success: bool
    reason: str = ''
    raw_result: object | None = None


@dataclass(slots=True, frozen=True)
class _ExecutionReportResult:
    text: str = ''
    source: str = ''


@dataclass(slots=True, frozen=True)
class _ExecutionJoinDecision:
    start_index: int
    join_strategy: str
    mapped_step_id: str = ''


class NaoOrchestrator(Node):
    """Dispatch ROS4HRI intents to NAO-specific skill endpoints."""

    def __init__(self) -> None:
        super().__init__('nao_orchestrator')
        self.get_logger().info('nao_orchestrator created; waiting for lifecycle configure')

        self.declare_parameter('intent_topic', '/intents')
        self.declare_parameter('enable_legacy_intent_bridge', False)
        self.declare_parameter('legacy_intent_topic', '/chatbot/intent')
        self.declare_parameter('nao_say_action', '/nao/say')
        self.declare_parameter('dispatch_speech_intents', False)
        self.declare_parameter('nao_say_wait_sec', 0.2)
        self.declare_parameter('nao_say_result_timeout_sec', 8.0)
        self.declare_parameter('replay_motion_action', '/skill/replay_motion')
        self.declare_parameter('replay_motion_speed', 0.8)
        self.declare_parameter('replay_motion_wait_sec', 0.2)
        self.declare_parameter('replay_motion_result_timeout_sec', 20.0)
        self.declare_parameter('head_motion_action', '/skill/do_head_motion')
        self.declare_parameter('head_motion_speed', 0.25)
        self.declare_parameter('head_motion_wait_sec', 0.2)
        self.declare_parameter('head_motion_result_timeout_sec', 6.0)
        self.declare_parameter('look_at_action', '/skill/look_at')
        self.declare_parameter('look_at_wait_sec', 0.2)
        self.declare_parameter('look_at_result_timeout_sec', 8.0)
        self.declare_parameter('posture_command_topic', '/chatbot/posture_command')
        self.declare_parameter(
            'posture_command_result_topic',
            '/chatbot/posture_command_result',
        )
        self.declare_parameter('posture_command_result_timeout_sec', 20.0)
        self.declare_parameter('fallback_to_posture_topic', True)
        self.declare_parameter('head_motion_joint_angles_topic', '/joint_angles')
        self.declare_parameter('fallback_to_joint_angles_topic', True)
        self.declare_parameter('planner_feedback_topic', '/planner/execution_feedback')
        self.declare_parameter('planner_dialogue_act_topic', '/planner/dialogue_act')
        self.declare_parameter(
            'planner_dialogue_relay_topic',
            '/nao_orchestrator/planner_dialogue_act',
        )
        self.declare_parameter('enable_planner_gate', False)
        self.declare_parameter('planner_gate_request_topic', '/nao_orchestrator/planner_request')
        self.declare_parameter('planner_request_topic', '/planner/request')
        self.declare_parameter('dedupe_window_sec', 0.8)
        self.declare_parameter('default_greeting', 'Hello! Nice to meet you.')
        self.declare_parameter('scan_result_mode', 'success')
        self.declare_parameter('scan_action', '/skill/scan')
        self.declare_parameter('scan_action_wait_sec', 0.2)
        self.declare_parameter('scan_action_result_timeout_sec', 20.0)
        self.declare_parameter('scan_report_after_success', True)
        self.declare_parameter('fake_skill_wait_sec', 0.2)
        self.declare_parameter('fake_skill_result_timeout_sec', 20.0)
        self.declare_parameter('perform_motion_execution_mode', 'real')
        self.declare_parameter('look_at_execution_mode', 'fake')
        self.declare_parameter('fake_skill_navigate_to_action', '/skill/fake/navigate_to')
        self.declare_parameter('fake_skill_find_object_action', '/skill/fake/find_object')
        self.declare_parameter('fake_skill_perform_motion_action', '/skill/fake/perform_motion')
        self.declare_parameter('fake_skill_look_at_action', '/skill/fake/look_at')
        self.declare_parameter('fake_skill_wave_greet_action', '/skill/fake/wave_greet')
        self.declare_parameter('fake_skill_inspect_area_action', '/skill/fake/inspect_area')
        self.declare_parameter('fake_skill_walk_to_action', '/skill/fake/walk_to')
        self.declare_parameter('fake_skill_pick_object_action', '/skill/fake/pick_object')
        self.declare_parameter('fake_skill_place_object_action', '/skill/fake/place_object')
        self.declare_parameter('fake_skill_bring_object_action', '/skill/fake/bring_object')
        self.declare_parameter('report_result_action', '/skill/report_result')
        self.declare_parameter('report_result_action_wait_sec', 0.2)
        self.declare_parameter('report_result_action_result_timeout_sec', 8.0)
        self.declare_parameter('execution_report_chatbot_enabled', True)
        self.declare_parameter(
            'execution_report_chatbot_service',
            '/chatbot_llm/dialogue_interaction',
        )
        self.declare_parameter('execution_report_chatbot_wait_sec', 0.2)
        self.declare_parameter('execution_report_chatbot_timeout_sec', 12.0)
        self.declare_parameter('kb_revise_service_name', '/kb/revise')
        self.declare_parameter('kb_mutation_timeout_sec', 1.0)
        self.declare_parameter('kb_query_service_name', '/kb/query')
        self.declare_parameter('kb_query_timeout_sec', 1.0)
        self.declare_parameter('apply_success_kb_effects', True)

        self.intent_topic = str(self.get_parameter('intent_topic').value)
        self.enable_legacy_intent_bridge = bool(
            self.get_parameter('enable_legacy_intent_bridge').value
        )
        self.legacy_intent_topic = str(self.get_parameter('legacy_intent_topic').value)
        self.nao_say_action = str(self.get_parameter('nao_say_action').value)
        self.dispatch_speech_intents = bool(
            self.get_parameter('dispatch_speech_intents').value
        )
        self.nao_say_wait_sec = max(
            0.0,
            float(self.get_parameter('nao_say_wait_sec').value),
        )
        self.nao_say_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('nao_say_result_timeout_sec').value),
        )
        self.replay_motion_action = str(self.get_parameter('replay_motion_action').value)
        self.replay_motion_speed = float(
            self.get_parameter('replay_motion_speed').value
        )
        self.replay_motion_wait_sec = max(
            0.0,
            float(self.get_parameter('replay_motion_wait_sec').value),
        )
        self.replay_motion_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('replay_motion_result_timeout_sec').value),
        )
        self.head_motion_action = str(self.get_parameter('head_motion_action').value)
        self.head_motion_speed = float(
            self.get_parameter('head_motion_speed').value
        )
        self.head_motion_wait_sec = max(
            0.0,
            float(self.get_parameter('head_motion_wait_sec').value),
        )
        self.head_motion_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('head_motion_result_timeout_sec').value),
        )
        self.look_at_action = str(self.get_parameter('look_at_action').value)
        self.look_at_wait_sec = max(
            0.0,
            float(self.get_parameter('look_at_wait_sec').value),
        )
        self.look_at_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('look_at_result_timeout_sec').value),
        )
        self.posture_command_topic = str(
            self.get_parameter('posture_command_topic').value
        )
        self.posture_command_result_topic = str(
            self.get_parameter('posture_command_result_topic').value
        )
        self.posture_command_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('posture_command_result_timeout_sec').value),
        )
        self.fallback_to_posture_topic = bool(
            self.get_parameter('fallback_to_posture_topic').value
        )
        self.head_motion_joint_angles_topic = str(
            self.get_parameter('head_motion_joint_angles_topic').value
        )
        self.fallback_to_joint_angles_topic = bool(
            self.get_parameter('fallback_to_joint_angles_topic').value
        )
        self.planner_feedback_topic = str(
            self.get_parameter('planner_feedback_topic').value
        )
        self.planner_dialogue_act_topic = str(
            self.get_parameter('planner_dialogue_act_topic').value
        )
        self.planner_dialogue_relay_topic = str(
            self.get_parameter('planner_dialogue_relay_topic').value
        )
        self.enable_planner_gate = bool(self.get_parameter('enable_planner_gate').value)
        self.planner_gate_request_topic = str(
            self.get_parameter('planner_gate_request_topic').value
        )
        self.planner_request_topic = str(self.get_parameter('planner_request_topic').value)
        self.dedupe_window_sec = max(
            0.0,
            float(self.get_parameter('dedupe_window_sec').value),
        )
        self.default_greeting = str(self.get_parameter('default_greeting').value)
        self.scan_result_mode = str(
            self.get_parameter('scan_result_mode').value
        ).strip().lower()
        self.scan_action = str(self.get_parameter('scan_action').value).strip()
        self.scan_action_wait_sec = max(
            0.0,
            float(self.get_parameter('scan_action_wait_sec').value),
        )
        self.scan_action_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('scan_action_result_timeout_sec').value),
        )
        self.scan_report_after_success = bool(
            self.get_parameter('scan_report_after_success').value
        )
        self.fake_skill_wait_sec = max(
            0.0,
            float(self.get_parameter('fake_skill_wait_sec').value),
        )
        self.fake_skill_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('fake_skill_result_timeout_sec').value),
        )
        self.perform_motion_execution_mode = _normalize_execution_mode(
            self.get_parameter('perform_motion_execution_mode').value
        )
        self.look_at_execution_mode = _normalize_execution_mode(
            self.get_parameter('look_at_execution_mode').value
        )
        self._fake_skill_action_names = {
            'navigate_to': str(self.get_parameter('fake_skill_navigate_to_action').value).strip(),
            'find_object': str(self.get_parameter('fake_skill_find_object_action').value).strip(),
            'perform_motion': str(self.get_parameter('fake_skill_perform_motion_action').value).strip(),
            'look_at': str(self.get_parameter('fake_skill_look_at_action').value).strip(),
            'wave_greet': str(self.get_parameter('fake_skill_wave_greet_action').value).strip(),
            'inspect_area': str(self.get_parameter('fake_skill_inspect_area_action').value).strip(),
            'walk_to': str(self.get_parameter('fake_skill_walk_to_action').value).strip(),
            'pick_object': str(self.get_parameter('fake_skill_pick_object_action').value).strip(),
            'place_object': str(self.get_parameter('fake_skill_place_object_action').value).strip(),
            'bring_object': str(self.get_parameter('fake_skill_bring_object_action').value).strip(),
        }
        self.report_result_action = str(
            self.get_parameter('report_result_action').value
        ).strip()
        self.report_result_action_wait_sec = max(
            0.0,
            float(self.get_parameter('report_result_action_wait_sec').value),
        )
        self.report_result_action_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('report_result_action_result_timeout_sec').value),
        )
        self.execution_report_chatbot_enabled = bool(
            self.get_parameter('execution_report_chatbot_enabled').value
        )
        self.execution_report_chatbot_service = str(
            self.get_parameter('execution_report_chatbot_service').value
        ).strip()
        self.execution_report_chatbot_wait_sec = max(
            0.0,
            float(self.get_parameter('execution_report_chatbot_wait_sec').value),
        )
        self.execution_report_chatbot_timeout_sec = max(
            0.1,
            float(self.get_parameter('execution_report_chatbot_timeout_sec').value),
        )
        self.kb_revise_service_name = str(
            self.get_parameter('kb_revise_service_name').value
        ).strip()
        self.kb_mutation_timeout_sec = max(
            0.05,
            float(self.get_parameter('kb_mutation_timeout_sec').value),
        )
        self.kb_query_service_name = str(
            self.get_parameter('kb_query_service_name').value
        ).strip()
        self.kb_query_timeout_sec = max(
            0.05,
            float(self.get_parameter('kb_query_timeout_sec').value),
        )
        self.apply_success_kb_effects = bool(
            self.get_parameter('apply_success_kb_effects').value
        )

        self._intent_sub = None
        self._legacy_intent_sub = None
        self._posture_result_sub = None
        self._diag_pub = None
        self._diag_timer = None
        self._posture_command_pub = None
        self._joint_angles_pub = None
        self._planner_feedback_pub = None
        self._planner_gate_sub = None
        self._planner_dialogue_act_sub = None
        self._planner_dialogue_act_pub = None
        self._planner_request_pub = None
        self._is_active = False
        self._stats = _RuntimeStats()
        self._last_intent_signature = ''
        self._last_intent_ts = 0.0
        self._relayed_planner_act_signatures: list[str] = []
        self._relayed_planner_act_signature_set: set[str] = set()

        self._say_client = None
        self._replay_motion_client = None
        self._head_motion_client = None
        self._look_at_client = None
        self._scan_client = None
        self._fake_skill_clients: dict[str, ActionClient] = {}
        self._report_result_client = None
        self._execution_report_chatbot_client = None
        self._execution_report_callback_group = ReentrantCallbackGroup()
        self._kb_mutation_client = None
        self._kb_query_client = None
        self._posture_result_lock = threading.Lock()
        self._posture_result_event = threading.Event()
        self._latest_posture_result: dict | None = None
        self._planner_gate = PlannerGate()
        self._planner_request_context_by_goal: dict[str, dict] = {}
        self._planner_request_context_order: list[str] = []
        self._scan_skill_names = self._load_scan_skill_names()
        self._fake_skill_aliases = self._load_fake_skill_aliases()
        self._active_execution_goal_id = ''
        self._active_execution_plan_id = ''
        self._active_execution_plan_version = 0
        self._active_execution_step_id = ''
        self._execution_lock = threading.Lock()

    # -------------------------------------------------------------------------
    # Lifecycle configuration
    # -------------------------------------------------------------------------

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Create action clients, diagnostics, and topic fallbacks."""
        self._destroy_runtime_interfaces()
        self._relayed_planner_act_signatures.clear()
        self._relayed_planner_act_signature_set.clear()
        self._say_client = ActionClient(self, Say, self.nao_say_action)
        if ReplayMotion is not None:
            self._replay_motion_client = ActionClient(
                self,
                ReplayMotion,
                self.replay_motion_action,
            )
        else:
            self._replay_motion_client = None
            self.get_logger().warn(
                'nao_skills ReplayMotion action is unavailable; replay_motion dispatch disabled'
            )
        if DoHeadMotion is not None:
            self._head_motion_client = ActionClient(
                self,
                DoHeadMotion,
                self.head_motion_action,
            )
        else:
            self._head_motion_client = None
            self.get_logger().warn(
                'nao_skills DoHeadMotion action is unavailable; head_motion dispatch disabled'
            )
        if LookAt is not None:
            self._look_at_client = ActionClient(self, LookAt, self.look_at_action)
        else:
            self._look_at_client = None
            self.get_logger().warn(
                'interaction_skills LookAt action is unavailable; real look_at dispatch disabled'
            )
        if ScanScene is not None:
            self._scan_client = ActionClient(self, ScanScene, self.scan_action)
            fake_skill_names = set(self._fake_skill_aliases.values())
            if self.perform_motion_execution_mode == 'fake':
                fake_skill_names.add('perform_motion')
            if self.look_at_execution_mode == 'fake':
                fake_skill_names.add('look_at')
            for skill_name in sorted(fake_skill_names):
                action_name = self._fake_skill_action_names.get(
                    skill_name,
                    '/skill/fake/%s' % skill_name,
                )
                if not action_name:
                    continue
                self._fake_skill_clients[skill_name] = ActionClient(
                    self,
                    ScanScene,
                    action_name,
                )
        else:
            self.get_logger().warn(
                'ScanScene interface unavailable; scan dispatch will fail until interfaces are rebuilt'
            )
        self._report_result_client = ActionClient(
            self,
            Say,
            self.report_result_action,
        )
        if self.execution_report_chatbot_enabled and self.execution_report_chatbot_service:
            self._execution_report_chatbot_client = self.create_client(
                DialogueInteraction,
                self.execution_report_chatbot_service,
                callback_group=self._execution_report_callback_group,
            )
        self._kb_mutation_client = KnowledgeCoreMutationClient(
            node=self,
            service_name=self.kb_revise_service_name,
            timeout_sec=self.kb_mutation_timeout_sec,
        )
        self._kb_query_client = KnowledgeCoreQueryClient(
            node=self,
            callback_group=self._execution_report_callback_group,
            service_name=self.kb_query_service_name,
            timeout_sec=self.kb_query_timeout_sec,
        )
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._posture_command_pub = self.create_publisher(
            String,
            self.posture_command_topic,
            10,
        )
        self._posture_result_sub = self.create_subscription(
            String,
            self.posture_command_result_topic,
            self._on_posture_command_result,
            10,
        )
        self._planner_feedback_pub = self.create_publisher(
            String,
            self.planner_feedback_topic,
            10,
        )
        self._planner_dialogue_act_pub = self.create_publisher(
            String,
            self.planner_dialogue_relay_topic,
            10,
        )
        if self.enable_planner_gate:
            self._planner_request_pub = self.create_publisher(
                Intent,
                self.planner_request_topic,
                10,
            )
        if JointAnglesWithSpeed is not None:
            self._joint_angles_pub = self.create_publisher(
                JointAnglesWithSpeed,
                self.head_motion_joint_angles_topic,
                10,
            )
        else:
            self.get_logger().warn(
                'JointAnglesWithSpeed unavailable; joint-topic fallback is disabled'
            )
        self.get_logger().info(
            'nao_orchestrator configured | intents:%s legacy:%s say:%s replay:%s head:%s look:%s scan:%s report:%s fake:%s planner_gate:%s->%s planner_dialogue:%s->%s'
            % (
                self.intent_topic,
                self.legacy_intent_topic,
                self.nao_say_action,
                self.replay_motion_action,
                self.head_motion_action,
                self.look_at_action,
                self.scan_action if self._scan_client is not None else 'unavailable',
                self.report_result_action,
                ','.join(sorted(self._fake_skill_clients.keys())) or 'none',
                self.planner_gate_request_topic if self.enable_planner_gate else 'disabled',
                self.planner_request_topic,
                self.planner_dialogue_act_topic,
                self.planner_dialogue_relay_topic,
            )
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start intent subscriptions once the lifecycle node becomes active."""
        self._is_active = True
        self._intent_sub = self.create_subscription(
            Intent,
            self.intent_topic,
            self._on_intent,
            10,
        )
        if self.enable_legacy_intent_bridge:
            self._legacy_intent_sub = self.create_subscription(
                String,
                self.legacy_intent_topic,
                self._on_legacy_intent,
                10,
            )
        if self.enable_planner_gate:
            self._planner_gate_sub = self.create_subscription(
                Intent,
                self.planner_gate_request_topic,
                self._on_planner_gate_request,
                10,
            )
            self._planner_dialogue_act_sub = self.create_subscription(
                String,
                self.planner_dialogue_act_topic,
                self._on_planner_dialogue_act,
                10,
            )
        self.get_logger().info('nao_orchestrator active')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop subscriptions while keeping the configured action clients alive."""
        self._is_active = False
        if self._intent_sub is not None:
            self.destroy_subscription(self._intent_sub)
            self._intent_sub = None
        if self._legacy_intent_sub is not None:
            self.destroy_subscription(self._legacy_intent_sub)
            self._legacy_intent_sub = None
        if self._planner_gate_sub is not None:
            self.destroy_subscription(self._planner_gate_sub)
            self._planner_gate_sub = None
        if self._planner_dialogue_act_sub is not None:
            self.destroy_subscription(self._planner_dialogue_act_sub)
            self._planner_dialogue_act_sub = None
        self.get_logger().info('nao_orchestrator inactive')
        return super().on_deactivate(state)

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self._destroy_runtime_interfaces()
        self.get_logger().info('nao_orchestrator cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self._destroy_runtime_interfaces()
        self.get_logger().info('nao_orchestrator shutdown complete')
        return TransitionCallbackReturn.SUCCESS

    def _destroy_runtime_interfaces(self) -> None:
        """Tear down publishers, subscriptions, timers, and action clients."""
        if self._intent_sub is not None:
            self.destroy_subscription(self._intent_sub)
            self._intent_sub = None
        if self._legacy_intent_sub is not None:
            self.destroy_subscription(self._legacy_intent_sub)
            self._legacy_intent_sub = None
        if self._planner_gate_sub is not None:
            self.destroy_subscription(self._planner_gate_sub)
            self._planner_gate_sub = None
        if self._planner_dialogue_act_sub is not None:
            self.destroy_subscription(self._planner_dialogue_act_sub)
            self._planner_dialogue_act_sub = None
        if self._posture_result_sub is not None:
            self.destroy_subscription(self._posture_result_sub)
            self._posture_result_sub = None
        if self._diag_timer is not None:
            self.destroy_timer(self._diag_timer)
            self._diag_timer = None
        if self._diag_pub is not None:
            self.destroy_publisher(self._diag_pub)
            self._diag_pub = None
        if self._posture_command_pub is not None:
            self.destroy_publisher(self._posture_command_pub)
            self._posture_command_pub = None
        if self._joint_angles_pub is not None:
            self.destroy_publisher(self._joint_angles_pub)
            self._joint_angles_pub = None
        if self._planner_feedback_pub is not None:
            self.destroy_publisher(self._planner_feedback_pub)
            self._planner_feedback_pub = None
        if self._planner_dialogue_act_pub is not None:
            self.destroy_publisher(self._planner_dialogue_act_pub)
            self._planner_dialogue_act_pub = None
        if self._planner_request_pub is not None:
            self.destroy_publisher(self._planner_request_pub)
            self._planner_request_pub = None
        if self._execution_report_chatbot_client is not None:
            self.destroy_client(self._execution_report_chatbot_client)
            self._execution_report_chatbot_client = None
        for client in (
            self._say_client,
            self._replay_motion_client,
            self._head_motion_client,
            self._look_at_client,
            self._scan_client,
            self._report_result_client,
        ):
            if client is not None:
                client.destroy()
        for client in self._fake_skill_clients.values():
            if client is not None:
                client.destroy()
        self._say_client = None
        self._replay_motion_client = None
        self._head_motion_client = None
        self._look_at_client = None
        self._scan_client = None
        self._fake_skill_clients = {}
        self._report_result_client = None
        if self._kb_mutation_client is not None:
            self._kb_mutation_client.close()
            self._kb_mutation_client = None
        if self._kb_query_client is not None:
            self._kb_query_client.close()
            self._kb_query_client = None

    # -------------------------------------------------------------------------
    # Intent ingestion
    # -------------------------------------------------------------------------

    def _on_legacy_intent(self, msg: String) -> None:
        """Normalize the old `/chatbot/intent` bridge onto the new routing path."""
        intent_name, data = normalize_legacy_intent(
            msg.data,
            default_greeting=self.default_greeting,
        )
        self._handle_intent(
            intent_name=intent_name,
            data=data,
            source='legacy_intent_bridge',
        )

    def _on_intent(self, msg: Intent) -> None:
        """Normalize canonical `hri_actions_msgs/Intent` messages for dispatch."""
        data = parse_intent_data(msg.data)
        intent_name, normalized_data = normalize_incoming_intent(
            intent_name=msg.intent,
            data=data,
            default_greeting=self.default_greeting,
        )
        self._handle_intent(
            intent_name=intent_name,
            data=normalized_data,
            source=str(msg.source or msg.modality or Intent.UNKNOWN),
        )

    def _on_planner_gate_request(self, msg: Intent) -> None:
        """Admit chatbot-originated planner requests before planner_llm sees them."""
        if not self._is_active or self._planner_request_pub is None:
            return

        decision = self._planner_gate.decide(msg.data)
        if not decision.accepted:
            self._stats.last_route = 'planner_gate:rejected'
            self._publish_planner_gate_feedback(decision=decision, status='rejected')
            self.get_logger().warn(
                'Planner gate rejected request | goal_id=%s kind=%s reason=%s'
                % (
                    decision.request.goal_id,
                    decision.request.request_kind,
                    decision.reason,
                )
            )
            return

        forward_msg = msg
        if isinstance(decision.forward_payload, dict):
            forward_msg = Intent()
            forward_msg.intent = msg.intent
            forward_msg.source = msg.source
            forward_msg.modality = msg.modality
            forward_msg.confidence = msg.confidence
            forward_msg.priority = msg.priority
            if hasattr(forward_msg, 'person_id') and hasattr(msg, 'person_id'):
                forward_msg.person_id = msg.person_id
            if hasattr(forward_msg, 'intent_type') and hasattr(msg, 'intent_type'):
                forward_msg.intent_type = msg.intent_type
            forward_msg.data = json.dumps(
                decision.forward_payload,
                separators=(',', ':'),
                ensure_ascii=True,
            )

        self._planner_request_pub.publish(forward_msg)
        self._remember_planner_request_context(
            decision.request.goal_id,
            decision.forward_payload if isinstance(decision.forward_payload, dict) else msg.data,
        )
        self._stats.last_route = 'planner_gate:forwarded'
        if decision.reason:
            self._publish_planner_gate_feedback(decision=decision, status='accepted')
        self.get_logger().info(
            'Planner gate forwarded request | goal_id=%s kind=%s reason=%s active_goal=%s active_plan=%s topic=%s'
            % (
                decision.request.goal_id,
                decision.request.request_kind,
                decision.reason or '-',
                self._planner_gate.active_goal_id or '-',
                self._planner_gate.active_plan_id or '-',
                self.planner_request_topic,
            )
        )

    def _publish_planner_gate_feedback(self, *, decision, status: str) -> None:
        if self._planner_feedback_pub is None:
            return
        request = decision.request
        payload = {
            'goal_id': request.goal_id,
            'plan_id': 'planner_gate',
            'plan_version': 0,
            'event_type': 'planner_gate_%s' % str(status).strip().lower(),
            'status': str(status).strip().lower(),
            'intent': 'planner_request',
            'source': 'nao_orchestrator',
            'reason': str(decision.reason or '').strip(),
            'request_kind': request.request_kind,
            'supersedes_goal_id': request.supersedes_goal_id,
            'goal_text': request.goal_text,
            'timestamp_sec': round(time.time(), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(',', ':'))
        self._planner_feedback_pub.publish(msg)

    def _on_planner_dialogue_act(self, msg: String) -> None:
        """Observe and relay planner acts through orchestrator-owned topic seam."""
        payload = parse_json_object(msg.data)
        act = str(payload.get('act', '')).strip().lower()
        if act == 'acknowledge':
            self._stats.duplicates_ignored += 1
            self.get_logger().info(
                'Suppressed contract-violating planner acknowledge; '
                'chatbot owns immediate acknowledgement'
            )
            return

        signature = _planner_dialogue_act_signature(msg.data)
        if signature in self._relayed_planner_act_signature_set:
            self._stats.duplicates_ignored += 1
            self.get_logger().warn('Ignored duplicate planner dialogue act')
            return
        self._remember_relayed_planner_act(signature)

        active_goal_before = self._planner_gate.active_goal_id
        self._planner_gate.observe_dialogue_act(msg.data)
        active_goal_after = self._planner_gate.active_goal_id
        if active_goal_before and not active_goal_after:
            self.get_logger().info(
                'Planner gate cleared by planner dialogue act | goal_id=%s'
                % active_goal_before
            )
        self._relay_planner_dialogue_act(msg.data)

    def _relay_planner_dialogue_act(self, payload: str) -> None:
        """Publish planner dialogue acts on the orchestrator-owned relay topic."""
        if self._planner_dialogue_act_pub is None:
            return
        relay_msg = String()
        relay_msg.data = payload
        self._planner_dialogue_act_pub.publish(relay_msg)

    def _remember_relayed_planner_act(self, signature: str) -> None:
        """Bound the exact planner-act relay ledger while preserving recent history."""
        if not signature:
            return
        self._relayed_planner_act_signatures.append(signature)
        self._relayed_planner_act_signature_set.add(signature)
        if len(self._relayed_planner_act_signatures) <= _MAX_RELAYED_PLANNER_ACTS:
            return
        expired = self._relayed_planner_act_signatures.pop(0)
        self._relayed_planner_act_signature_set.discard(expired)

    def _handle_intent(self, intent_name: str, data: dict, source: str) -> None:
        """Route one normalized intent through planned or legacy dispatch paths."""
        if not self._is_active:
            return

        self._stats.intents_received += 1
        self._stats.last_intent = str(intent_name).strip()
        signature = make_intent_signature(intent_name, data)
        if self._is_duplicate(signature):
            self._stats.duplicates_ignored += 1
            self._stats.last_route = 'ignored:duplicate'
            self.get_logger().warn(
                'Ignored duplicate intent within %.2fs | intent=%s source=%s'
                % (
                    self.dedupe_window_sec,
                    intent_name,
                    source,
                )
            )
            return

        plan_context = self._validated_plan_context(intent_name, data)
        if plan_context and self._handle_planned_intent(
            intent_name=intent_name,
            data=data,
            plan=plan_context['steps'],
            plan_context=plan_context,
            source=source,
        ):
            return

        if intent_name in (Intent.GREET, Intent.SAY):
            if not self.dispatch_speech_intents:
                self._stats.last_route = 'ignored:speech_owned_by_dialogue_manager'
                self.get_logger().info(
                    'Ignored conversational speech intent: %s source=%s'
                    % (intent_name, source)
                )
                return
            text = resolve_say_text(
                intent_name=intent_name,
                data=data,
                default_greeting=self.default_greeting,
            )
            if self._dispatch_say(text, data):
                self._stats.dispatched_say += 1
                self._stats.last_route = 'say'
            return

        if intent_name == Intent.PERFORM_MOTION:
            self._start_direct_motion_dispatch(data)
            return

        if intent_name in KB_QUERY_INTENTS:
            self._stats.last_route = 'ignored:kb_query'
            self.get_logger().info(
                'Observed KB query intent for future routing: %s source=%s data=%s'
                % (intent_name, source, data)
            )
            return

        self._stats.last_route = 'ignored:unhandled'
        self.get_logger().warn(
            'Unhandled intent: %s source=%s data=%s'
            % (intent_name, source, data)
        )

    # -------------------------------------------------------------------------
    # Structured plan execution
    # -------------------------------------------------------------------------

    def _validated_plan_context(self, intent_name: str, data: dict) -> dict | None:
        if not isinstance(data, dict) or 'plan' not in data:
            return None
        validation_data = dict(data)
        goal_id = str(parse_plan_envelope(data).get('goal_id', '')).strip()
        request_context = self._planner_request_context_by_goal.get(goal_id, {})
        grounded_context = request_context.get('grounded_context', {})
        if isinstance(grounded_context, dict) and grounded_context:
            validation_data['grounded_context'] = grounded_context
        return validate_execution_plan(intent_name, validation_data)

    def _handle_planned_intent(
        self,
        *,
        intent_name: str,
        data: dict,
        plan: list[dict],
        plan_context: dict,
        source: str,
    ) -> bool:
        plan_id = self._resolve_plan_id(plan_context)
        goal_id = str(plan_context.get('goal_id', '')).strip()
        plan_version = self._plan_version_from_context(plan_context)
        if self._is_exact_execution_plan_active(goal_id, plan_id, plan_version):
            self._stats.duplicates_ignored += 1
            self._stats.last_route = 'ignored:duplicate_plan'
            self.get_logger().warn(
                'Ignored duplicate planned intent | intent=%s source=%s plan_id=%s version=%s'
                % (intent_name, source, plan_id, plan_version)
            )
            return True
        join_decision = self._claim_execution_plan(
            goal_id=goal_id,
            plan_id=plan_id,
            plan_version=plan_version,
            plan=plan,
        )
        execution_plan = list(plan[join_decision.start_index :])
        self._stats.plans_started += 1
        self._stats.last_plan_id = plan_id

        if plan_context.get('errors'):
            self._stats.plans_failed += 1
            self._stats.last_plan_status = 'invalid'
            self._stats.last_route = 'planned:invalid'
            self._publish_plan_feedback(
                intent_name=intent_name,
                source=source,
                plan_context=plan_context,
                status='invalid',
                event_type='plan_invalid',
                reason='; '.join(plan_context['errors']),
                blocking=True,
                unmet_preconditions=plan_context['errors'],
                needs_user_input=False,
                validation_errors=plan_context['errors'],
            )
            self.get_logger().warn(
                'Planned intent validation failed | intent=%s source=%s plan_id=%s errors=%s'
                % (intent_name, source, plan_id, plan_context['errors'])
            )
            self._finalize_execution_plan(
                goal_id=goal_id,
                plan_id=plan_id,
                plan_version=plan_version,
            )
            return False

        self._publish_plan_feedback(
            intent_name=intent_name,
            source=source,
            plan_context=plan_context,
            status='accepted',
            event_type='plan_accepted',
        )
        self._maybe_dispatch_acknowledgement(
            intent_name=intent_name,
            data=data,
            plan=plan,
            plan_context=plan_context,
        )
        self._stats.last_route = 'planned:running'
        self._stats.last_plan_status = 'running'
        worker = threading.Thread(
            target=self._execute_planned_intent,
            kwargs={
                'intent_name': intent_name,
                'data': self._execution_context_for_goal(goal_id, data),
                'plan': execution_plan,
                'plan_context': dict(plan_context),
                'source': source,
                'goal_id': goal_id,
                'plan_id': plan_id,
                'plan_version': plan_version,
                'join_strategy': join_decision.join_strategy,
                'mapped_step_id': join_decision.mapped_step_id,
            },
            daemon=True,
        )
        worker.start()
        return True

    def _remember_planner_request_context(self, goal_id: str, payload) -> None:
        """Retain admitted request evidence for execution reports and replans."""
        clean_goal_id = str(goal_id or '').strip()
        request_context = parse_json_object(payload)
        if not clean_goal_id or not request_context:
            return
        if clean_goal_id in self._planner_request_context_by_goal:
            self._planner_request_context_order.remove(clean_goal_id)
        self._planner_request_context_by_goal[clean_goal_id] = request_context
        self._planner_request_context_order.append(clean_goal_id)
        while len(self._planner_request_context_order) > _MAX_PLANNER_REQUEST_CONTEXTS:
            expired_goal_id = self._planner_request_context_order.pop(0)
            self._planner_request_context_by_goal.pop(expired_goal_id, None)

    def _execution_context_for_goal(self, goal_id: str, plan_data: dict) -> dict:
        """Merge the admitted request context with the validated planner output."""
        context = dict(self._planner_request_context_by_goal.get(str(goal_id).strip(), {}))
        context.update(dict(plan_data or {}))
        return context

    def _maybe_dispatch_acknowledgement(
        self,
        *,
        intent_name: str,
        data: dict,
        plan: list[dict],
        plan_context: dict,
    ) -> None:
        _ = (intent_name, data, plan, plan_context)
        # Planner acknowledgement speech is realized through planner dialogue acts so
        # the executor stays focused on deterministic skill dispatch only.
        return

    def _execute_planned_intent(
        self,
        *,
        intent_name: str,
        data: dict,
        plan: list[dict],
        plan_context: dict,
        source: str,
        goal_id: str,
        plan_id: str,
        plan_version: int,
        join_strategy: str,
        mapped_step_id: str,
    ) -> None:
        """Execute a validated plan in a background worker so action results can be awaited."""
        executed_any = False
        latest_result_summary = ''
        latest_result_payload: dict = {}
        execution_results: list[dict] = []
        self.get_logger().info(
            'Executing plan worker | goal_id=%s plan_id=%s version=%s join=%s mapped_step=%s steps=%d'
            % (
                goal_id or '-',
                plan_id or '-',
                plan_version,
                join_strategy,
                mapped_step_id or '-',
                len(plan),
            )
        )

        for step_index, step in enumerate(plan):
            self._set_active_execution_step(step)
            if not self._is_execution_plan_active(goal_id, plan_id, plan_version):
                self._publish_plan_feedback(
                    intent_name=intent_name,
                    source=source,
                    plan_context=plan_context,
                    status='cancelled',
                    event_type='plan_cancelled',
                    reason='superseded by a newer planner goal',
                )
                self._finalize_execution_plan(
                    goal_id=goal_id,
                    plan_id=plan_id,
                    plan_version=plan_version,
                )
                self.get_logger().info(
                    'Stopped stale plan worker | plan_id=%s version=%s'
                    % (plan_id, plan_version)
                )
                return
            step_started = False

            def _mark_step_started() -> None:
                nonlocal step_started
                if step_started:
                    return
                step_started = True
                self._publish_plan_feedback(
                    intent_name=intent_name,
                    source=source,
                    plan_context=plan_context,
                    status='running',
                    event_type='step_started',
                    step=step,
                )

            dispatch_fallback_data = dict(data)
            dispatch_fallback_data['plan_context'] = dict(plan_context)
            dispatch_fallback_data['current_step_index'] = step_index
            dispatch_fallback_data['plan_steps'] = list(plan)
            if latest_result_summary:
                dispatch_fallback_data['last_result_summary'] = latest_result_summary
            if latest_result_payload:
                dispatch_fallback_data['last_result_payload'] = dict(latest_result_payload)
            if execution_results:
                dispatch_fallback_data['execution_results'] = list(execution_results)
            dispatch_fallback_data['plan_outcome_summary'] = _plan_outcome_summary(
                plan,
                execution_results,
            )

            step_ok, reason, result_payload = self._dispatch_plan_step(
                step,
                fallback_data=dispatch_fallback_data,
                plan=plan,
                step_index=step_index,
                on_started=_mark_step_started,
            )
            if step_ok:
                executed_any = True
                latest_result_payload = dict(result_payload or {})
                latest_result_summary = _first_non_empty_value(
                    latest_result_payload,
                    'summary_text',
                    'result_summary',
                    'message',
                ) or str(reason or '').strip()
                execution_results.append(
                    _execution_step_record(
                        step,
                        status='succeeded',
                        reason=reason,
                        result_summary=latest_result_summary,
                        result_payload=latest_result_payload,
                    )
                )
                if not step_started:
                    _mark_step_started()
                self._publish_plan_feedback(
                    intent_name=intent_name,
                    source=source,
                    plan_context=plan_context,
                    status='succeeded',
                    event_type='step_succeeded',
                    step=step,
                    result_summary=latest_result_summary,
                    result_payload=latest_result_payload,
                    plan_outcome_summary=_plan_outcome_summary(plan, execution_results),
                )
                continue

            failure_policy = str(step.get('on_failure', 'fail')).strip().lower()
            step_name = str(step.get('name', '')).strip().lower()
            failure_result_payload = dict(result_payload or {})
            execution_results.append(
                _execution_step_record(
                    step,
                    status='failed',
                    reason=reason,
                    result_summary=_first_non_empty_value(
                        failure_result_payload,
                        'summary_text',
                        'result_summary',
                        'message',
                    ) or str(reason or '').strip(),
                    result_payload=failure_result_payload,
                )
            )
            if step_name in _ASK_USER_STEP_NAMES and failure_policy not in (
                'ask_user',
                'clarify',
            ):
                failure_policy = 'ask_user'
            if failure_policy == 'continue':
                self._publish_plan_feedback(
                    intent_name=intent_name,
                    source=source,
                    plan_context=plan_context,
                    status='failed',
                    event_type='step_failed',
                    reason=reason,
                    step=step,
                    blocking=False,
                    unmet_preconditions=[],
                    needs_user_input=False,
                    plan_outcome_summary=_plan_outcome_summary(
                        plan,
                        execution_results,
                        terminal_reason=reason,
                        terminal_step=step,
                    ),
                )
                self.get_logger().warn(
                    'Planned intent step failed; continuing plan | intent=%s source=%s plan_id=%s step=%s reason=%s'
                    % (intent_name, source, plan_id, step, reason)
                )
                continue
            if failure_policy == 'ignore':
                self.get_logger().info(
                    'Planned intent step failed; on_failure=ignore | intent=%s plan_id=%s step=%s reason=%s'
                    % (intent_name, plan_id, step, reason)
                )
                continue

            self._stats.last_route = 'planned:failed'
            self._stats.plans_failed += 1
            self._stats.last_plan_status = 'failed'
            self._publish_plan_feedback(
                intent_name=intent_name,
                source=source,
                plan_context=plan_context,
                status='failed',
                event_type='step_failed',
                reason=reason,
                step=step,
                blocking=True,
                # `requires` enumerates declared dependencies, not unmet runtime
                # preconditions. Publishing it here makes supervisor treat any
                # retryable step failure as blocked/clarification.
                unmet_preconditions=[],
                needs_user_input=(
                    failure_policy in ('ask_user', 'clarify')
                    or step_name in _ASK_USER_STEP_NAMES
                ),
                result_summary=_first_non_empty_value(
                    failure_result_payload,
                    'summary_text',
                    'result_summary',
                    'message',
                ) or str(reason or '').strip(),
                result_payload=failure_result_payload,
                plan_outcome_summary=_plan_outcome_summary(
                    plan,
                    execution_results,
                    terminal_reason=reason,
                    terminal_step=step,
                ),
            )
            self.get_logger().warn(
                'Planned intent step failed | intent=%s source=%s plan_id=%s step=%s reason=%s'
                % (intent_name, source, plan_id, step, reason)
            )
            self._finalize_execution_plan(
                goal_id=goal_id,
                plan_id=plan_id,
                plan_version=plan_version,
            )
            return

        if executed_any:
            self._stats.last_route = 'planned'
            self._stats.plans_succeeded += 1
            self._stats.last_plan_status = 'completed'
            self._publish_plan_feedback(
                intent_name=intent_name,
                source=source,
                plan_context=plan_context,
                status='completed',
                event_type='plan_completed',
                result_summary=latest_result_summary,
                result_payload=latest_result_payload,
                plan_outcome_summary=_plan_outcome_summary(
                    plan,
                    execution_results,
                    terminal_reason='completed',
                ),
            )
            self._finalize_execution_plan(
                goal_id=goal_id,
                plan_id=plan_id,
                plan_version=plan_version,
            )
            return

        self._stats.plans_failed += 1
        self._stats.last_plan_status = 'empty'
        self._publish_plan_feedback(
            intent_name=intent_name,
            source=source,
            plan_context=plan_context,
            status='failed',
            event_type='plan_invalid',
            reason='plan contained no executable steps',
            blocking=True,
            plan_outcome_summary=_plan_outcome_summary(
                plan,
                execution_results,
                terminal_reason='plan contained no executable steps',
            ),
        )
        self._finalize_execution_plan(
            goal_id=goal_id,
            plan_id=plan_id,
            plan_version=plan_version,
        )

    def _dispatch_plan_step(
        self,
        step: dict,
        fallback_data: dict,
        *,
        plan: list[dict] | None = None,
        step_index: int = 0,
        on_started=None,
    ) -> tuple[bool, str, dict]:
        """Execute one step from the optional structured `Intent.data.plan`."""
        step_type = str(step.get('type', '')).strip().lower()
        step_name = str(step.get('name', '')).strip().lower()
        step_args = dict(step.get('args', {}))

        if step_type == 'noop':
            if on_started is not None:
                on_started()
            return True, '', {}

        if step_type == 'say':
            success, reason = self._execute_say_plan_step(
                step_args,
                fallback_data,
                on_started=on_started,
            )
            return success, reason, {}

        if step_type == 'look_at':
            success, reason = self._dispatch_planned_look_at(
                step_name,
                step_args,
                on_started=on_started,
            )
            return success, reason, {}

        if step_type == 'skill':
            if step_name in ('perform_motion', 'motion', ''):
                success, reason, payload = self._execute_motion_plan_step(
                    step_args,
                    on_started=on_started,
                )
                return success, reason, payload
            if step_name == 'look_at':
                success, reason = self._dispatch_planned_look_at(
                    step_name,
                    step_args,
                    on_started=on_started,
                )
                return success, reason, {}
            if step_name in _KB_MUTATION_OPERATIONS:
                return self._execute_kb_mutation_step(
                    step_name,
                    step_args,
                    on_started=on_started,
                )
            if step_name == 'report_result':
                return self._execute_report_result_step(
                    step_args,
                    fallback_data,
                    on_started=on_started,
                )
            if step_name in _ASK_USER_STEP_NAMES:
                return self._execute_ask_user_step(
                    step_args,
                    fallback_data,
                    on_started=on_started,
                )
            if step_name in self._scan_skill_names:
                return self._execute_scan_step(
                    step_args,
                    should_auto_report=scan_step_should_auto_report(
                        plan=plan,
                        step_index=step_index,
                    ),
                    on_started=on_started,
                )
            fake_skill_name = self._resolve_fake_skill_name(step_name)
            if fake_skill_name:
                return self._execute_fake_skill_step(
                    fake_skill_name,
                    step_args,
                    on_started=on_started,
                )

        self._stats.dispatch_failures += 1
        self.get_logger().warn('Unsupported planned step: %s' % step)
        return False, 'unsupported planned step', {}

    def _execute_kb_mutation_step(
        self,
        step_name: str,
        step_args: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str, dict]:
        """Delegate one explicit planner mutation to the KnowledgeCore seam."""
        statements = step_args.get('statements', step_args.get('statement', []))
        statements = KnowledgeCoreMutationClient.coerce_statements(statements)
        models = step_args.get('models', [])
        if isinstance(models, str):
            models = [models]
        if on_started is not None:
            on_started()
        if self._kb_mutation_client is None:
            self._stats.dispatch_failures += 1
            return False, 'KnowledgeCore mutation client is unavailable', {}
        if step_name in {'kb_add', 'kb_revise'} and (
            not statements
            or any(not is_explicit_knowledge_statement(statement) for statement in statements)
        ):
            self._stats.dispatch_failures += 1
            return False, 'KnowledgeCore mutation requires explicit RDF-style statements', {
                'skill': step_name,
                'operation': _KB_MUTATION_OPERATIONS[step_name],
                'statement_count': len(statements),
                'dispatched': False,
                'success': False,
                'requires_clarification': True,
                'slots_needed': ['subject', 'predicate', 'object'],
            }

        if step_name == 'kb_revise':
            removed, reason = self._remove_previous_kb_values(statements, models)
            if reason:
                self._stats.dispatch_failures += 1
                return False, reason, {
                    'skill': step_name,
                    'operation': 'remove_previous_values',
                    'statement_count': len(removed),
                    'dispatched': bool(removed),
                    'success': False,
                }
        elif step_name == 'kb_remove':
            statements = self._expand_kb_remove_statements(statements, models)
            if not statements:
                self._stats.dispatch_failures += 1
                return False, 'KnowledgeCore remove found no matching facts', {
                    'skill': step_name,
                    'operation': 'remove',
                    'statement_count': 0,
                    'dispatched': False,
                    'success': False,
                }

        result = self._kb_mutation_client.mutate(
            operation=_KB_MUTATION_OPERATIONS[step_name],
            statements=statements,
            models=models if isinstance(models, list) else [],
            lifespan_sec=step_args.get('lifespan_sec', 0.0),
            wait_for_result=True,
        )
        payload = {
            'skill': step_name,
            'operation': result.operation,
            'statement_count': result.statement_count,
            'dispatched': result.dispatched,
            'success': result.success,
        }
        if result.success:
            if step_name == 'kb_remove':
                remaining = self._query_remaining_kb_remove_facts(statements, models)
                if remaining:
                    self._stats.dispatch_failures += 1
                    payload['success'] = False
                    payload['remaining_statements'] = remaining
                    return (
                        False,
                        'KnowledgeCore remove post-condition failed',
                        payload,
                    )
            self._stats.dispatched_kb_mutation += 1
            return True, 'KnowledgeCore mutation completed', payload
        self._stats.dispatch_failures += 1
        payload['error_msg'] = result.error_msg
        return False, result.error_msg or 'KnowledgeCore mutation failed', payload

    def _remove_previous_kb_values(
        self,
        statements: list[str],
        models: list[str],
    ) -> tuple[list[str], str]:
        """Retract existing subject/predicate values before a KB revise update."""
        if self._kb_query_client is None or self._kb_mutation_client is None:
            return [], ''
        removals: list[str] = []
        query_models = models if isinstance(models, list) and models else ['default']
        for statement in statements:
            subject, predicate, new_object = _statement_parts(statement)
            if not subject or not predicate or not new_object:
                continue
            rows = self._kb_query_client.query_rows(
                patterns=['%s %s ?object' % (subject, predicate)],
                query_vars=['?object'],
                models=query_models,
            )
            removals.extend(
                _statement_from_binding(subject, predicate, row)
                for row in rows
                if _binding_value(row, 'object') != new_object
            )
        removals = _dedupe_statements(removals)
        if not removals:
            return [], ''
        result = self._kb_mutation_client.mutate(
            operation='remove',
            statements=removals,
            models=models if isinstance(models, list) else [],
            wait_for_result=True,
        )
        if result.success:
            return removals, ''
        return removals, result.error_msg or 'KnowledgeCore previous-value removal failed'

    def _remove_stale_spatial_effect_values(
        self,
        statements: list[str],
        models: list[str],
    ) -> tuple[list[str], str]:
        """Retract stale spatial facts when a skill reports a new object location."""
        return remove_stale_spatial_effect_values(
            statements=statements,
            models=models,
            query_client=self._kb_query_client,
            mutation_client=self._kb_mutation_client,
        )

    def _validate_spatial_effect_statements(
        self,
        statements: list[str],
        models: list[str],
    ) -> str:
        return validate_spatial_effect_statements(
            statements=statements,
            models=models,
            query_client=self._kb_query_client,
        )

    def _expand_kb_remove_statements(
        self,
        statements: list[str],
        models: list[str],
    ) -> list[str]:
        """Expand entity-only remove requests into current concrete KB facts."""
        if self._kb_query_client is None:
            return statements
        expanded: list[str] = []
        query_models = models if isinstance(models, list) and models else ['default']
        for statement in statements:
            subject = _kb_remove_subject_alias(statement)
            if not subject:
                subject, predicate, obj = _statement_parts(statement)
                if subject and predicate and obj:
                    expanded.append(statement)
                    continue
                expanded.append(statement)
                continue
            expanded.extend(self._concrete_kb_facts_for_subject(subject, query_models))
        return _dedupe_statements(expanded)

    def _query_remaining_kb_remove_facts(
        self,
        statements: list[str],
        models: list[str],
    ) -> list[str]:
        """Return removed facts that still resolve after a successful retract."""
        if self._kb_query_client is None:
            return []
        remaining: list[str] = []
        query_models = models if isinstance(models, list) and models else ['default']
        for statement in statements:
            subject, predicate, obj = _statement_parts(statement)
            if not subject or not predicate or not obj:
                continue
            rows = self._kb_query_client.query_rows(
                patterns=['%s %s ?object' % (subject, predicate)],
                query_vars=['?object'],
                models=query_models,
            )
            remaining.extend(
                _statement_from_binding(subject, predicate, row)
                for row in rows
                if _binding_value(row, 'object') == obj
            )
        return _dedupe_statements(remaining)

    def _concrete_kb_facts_for_subject(
        self,
        subject: str,
        query_models: list[str],
    ) -> list[str]:
        rows = self._kb_query_client.query_rows(
            patterns=['%s ?predicate ?object' % subject],
            query_vars=['?predicate', '?object'],
            models=query_models,
        )
        return [
            _statement_from_binding(subject, _binding_value(row, 'predicate'), row)
            for row in rows
        ]

    def _dispatch_planned_look_at(
        self,
        step_name: str,
        step_args: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        """Map a planned look-at step onto reset or target-frame dispatch."""
        if self.look_at_execution_mode == 'fake':
            success, reason, _payload = self._execute_fake_skill_step(
                'look_at',
                dict(step_args or {}),
                on_started=on_started,
            )
            if success:
                self._stats.dispatched_look_at += 1
                return True, ''
            return False, reason or 'fake look_at dispatch failed'

        policy = str(
            step_args.get('policy', step_args.get('object', step_name))
        ).strip().lower()
        if policy in ('reset', 'look_at_reset'):
            success, reason = self._execute_look_at_reset_step(on_started=on_started)
            if success:
                self._stats.dispatched_look_at += 1
                return True, ''
            return False, reason or 'look_at reset dispatch failed'
        if policy in ('random', 'social', 'auto'):
            success, reason = self._execute_look_at_policy_step(
                policy=policy,
                on_started=on_started,
            )
            if success:
                self._stats.dispatched_look_at += 1
                return True, ''
            return False, reason or 'look_at policy dispatch failed'
        target_frame = str(
            step_args.get(
                'target_frame',
                step_args.get(
                    'frame_id',
                    step_args.get(
                        'target',
                        step_args.get('entity_id', ''),
                    ),
                ),
            )
        ).strip()
        if not target_frame:
            self._stats.dispatch_failures += 1
            self.get_logger().warn(
                'Planned look_at step is missing a target frame or supported policy: %s'
                % step_args
            )
            return False, 'look_at step missing target frame or supported policy'

        success, reason = self._execute_look_at_target_step(
            frame_id=target_frame,
            x_value=step_args.get('x', 0.0),
            y_value=step_args.get('y', 0.0),
            z_value=step_args.get('z', 0.0),
            policy=policy,
            on_started=on_started,
        )
        if success:
            self._stats.dispatched_look_at += 1
            return True, ''
        return False, reason or 'look_at target dispatch failed'

    def _execute_say_plan_step(
        self,
        step_args: dict,
        fallback_data: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        step_text = _first_non_empty_value(
            step_args,
            'summary_text',
            'result_summary',
            'text',
            'message',
            'utterance',
            'content',
            'suggested_response',
            'text_hint',
            'object',
        )
        text = resolve_say_text(
            intent_name=Intent.SAY,
            data={
                'object': step_text,
                'suggested_response': _first_non_empty_value(
                    step_args,
                    'suggested_response',
                    'text_hint',
                    'summary_text',
                    'result_summary',
                    'text',
                ),
                'recipient': step_args.get(
                    'recipient',
                    fallback_data.get('recipient', ''),
                ),
            },
            default_greeting=self.default_greeting,
        )
        clean_text = _first_non_empty_text(
            text,
            fallback_data.get('ack_text', ''),
            fallback_data.get('suggested_response', ''),
            fallback_data.get('goal_text', ''),
        )
        if not clean_text:
            self._stats.dispatch_failures += 1
            return False, 'say dispatch failed: empty text'

        goal = Say.Goal()
        goal.input = clean_text
        goal.person_id = str(
            step_args.get('recipient', fallback_data.get('recipient', ''))
        ).strip()
        result = self._execute_action_step(
            client=self._say_client,
            goal=goal,
            wait_sec=self.nao_say_wait_sec,
            result_timeout_sec=self.nao_say_result_timeout_sec,
            description='nao_say',
            on_started=on_started,
        )
        if result.success:
            self._stats.dispatched_say += 1
            self.get_logger().info('ORCH SAY_DISPATCH | %s' % clean_text)
            return True, ''
        return False, result.reason or 'say dispatch failed'

    def _execute_report_result_step(
        self,
        step_args: dict,
        fallback_data: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str, dict]:
        report_text = self._resolve_report_result_text(step_args, fallback_data)
        if not report_text:
            self._stats.dispatch_failures += 1
            return False, 'report_result step missing summary text', {
                'skill': 'report_result',
                'status': 'failed',
                'summary_text': '',
            }

        goal = Say.Goal()
        goal.input = report_text
        goal.person_id = _first_non_empty_value(
            step_args,
            'recipient',
            'person_id',
        )
        result = self._execute_action_step(
            client=self._report_result_client,
            goal=goal,
            wait_sec=self.report_result_action_wait_sec,
            result_timeout_sec=self.report_result_action_result_timeout_sec,
            description='report_result_skill',
            on_started=on_started,
        )
        success = bool(result.success)
        reason = str(result.reason or '').strip()
        payload = {
            'skill': 'report_result',
            'status': 'completed' if success else 'failed',
            'summary_text': report_text,
        }
        if success:
            return True, report_text, payload
        return False, reason or 'report_result action failed', payload

    def _resolve_report_result_text(self, step_args: dict, fallback_data: dict) -> str:
        explicit_text = _first_non_empty_value(
            step_args,
            'summary_text',
            'result_summary',
            'text',
            'message',
            'utterance',
            'content',
            'suggested_response',
            'text_hint',
            'object',
        )
        report_context = self._execution_report_context(fallback_data)
        if explicit_text and not is_unresolved_report_template(explicit_text):
            report_context['requested_summary'] = explicit_text
        chatbot_result = self._request_execution_report_text(report_context)
        if chatbot_result.text:
            self.get_logger().info(
                'Resolved report_result text via %s' % chatbot_result.source
            )
            return chatbot_result.text

        if explicit_text and not is_unresolved_report_template(explicit_text):
            return explicit_text

        chain_text = _report_text_from_execution_results(report_context.get('steps', []))
        if chain_text:
            return chain_text

        fallback_text = _first_non_empty_value(
            fallback_data,
            'last_result_summary',
            'result_summary',
            'summary_text',
        )
        if fallback_text and not is_unresolved_report_template(fallback_text):
            return fallback_text
        result_payload_text = _report_text_from_result_payload(
            fallback_data.get('last_result_payload', {})
        )
        if result_payload_text:
            return result_payload_text
        return ''

    def _execution_report_context(self, fallback_data: dict) -> dict:
        plan_context = fallback_data.get('plan_context', {})
        if not isinstance(plan_context, dict):
            plan_context = {}
        execution_results = fallback_data.get('execution_results', [])
        if not isinstance(execution_results, list):
            execution_results = []
        plan_steps = fallback_data.get('plan_steps', [])
        if not isinstance(plan_steps, list):
            plan_steps = []
        current_step_index = int(fallback_data.get('current_step_index', -1) or -1)
        bounded_steps = [
            dict(step) for step in execution_results[-_MAX_EXECUTION_REPORT_STEPS:]
            if isinstance(step, dict)
        ]
        future_steps = [
            dict(step)
            for step in plan_steps[current_step_index + 1:]
            if isinstance(step, dict)
        ] if current_step_index >= 0 else []
        future_action_steps = [
            step for step in future_steps
            if str(step.get('name', '')).strip().lower() != 'report_result'
        ]
        report_role = 'intermediate' if future_action_steps else 'final'
        plan_outcome_summary = fallback_data.get('plan_outcome_summary', {})
        if not isinstance(plan_outcome_summary, dict):
            plan_outcome_summary = {}
        scene_targets = list(plan_context.get('scene_targets', [])) \
            if isinstance(plan_context.get('scene_targets', []), list) else []
        grounded_context = dict(
            fallback_data.get('grounded_context', {})
            if isinstance(fallback_data.get('grounded_context', {}), dict)
            else {}
        )
        report_outcome = build_report_outcome(
            plan_steps=plan_steps,
            execution_results=execution_results,
            plan_outcome_summary=plan_outcome_summary,
            grounded_context=grounded_context,
            scene_targets=scene_targets,
            target_selection=plan_context.get('target_selection', {}),
            report_role=report_role,
        )
        return {
            'goal_text': _first_non_empty_value(
                fallback_data,
                'goal_text',
                'goal',
                'task',
                'raw_input',
                'text',
            ),
            'requested_intents': [
                str(item).strip()
                for item in fallback_data.get('normalized_intents', [])
                if str(item).strip()
            ] if isinstance(fallback_data.get('normalized_intents', []), list) else [],
            'dialogue_context': _execution_report_dialogue_context(
                fallback_data.get('dialogue_context', [])
            ),
            'scene_targets': scene_targets,
            'grounded_context': grounded_context,
            'plan_id': str(plan_context.get('plan_id', '')).strip(),
            'plan_version': int(plan_context.get('plan_version', 0) or 0),
            'report_role': report_role,
            'future_steps': future_steps[-_MAX_EXECUTION_REPORT_STEPS:],
            'steps': bounded_steps,
            'latest_result_summary': str(fallback_data.get('last_result_summary', '')).strip(),
            'latest_result_payload': dict(
                fallback_data.get('last_result_payload', {})
                if isinstance(fallback_data.get('last_result_payload', {}), dict)
                else {}
            ),
            'plan_outcome_summary': _report_context_plan_outcome_summary(
                plan_outcome_summary
            ),
            'report_outcome': report_outcome,
        }

    def _request_execution_report_text(self, report_context: dict) -> _ExecutionReportResult:
        if not self.execution_report_chatbot_enabled:
            return _ExecutionReportResult(source='disabled')
        if self._execution_report_chatbot_client is None:
            return _ExecutionReportResult(source='unavailable')
        if not self._execution_report_chatbot_client.wait_for_service(
            timeout_sec=self.execution_report_chatbot_wait_sec
        ):
            return _ExecutionReportResult(source='unavailable')

        request = DialogueInteraction.Request()
        request.dialogue_id = UUIDMsg(uuid=[0] * 16)
        request.role = DialogueRole(name='__default__')
        request.summary = ''
        request.history = [
            Utterance(
                speaker=Utterance.SYSTEM,
                text=json.dumps(
                    {'execution_report': report_context},
                    sort_keys=True,
                    separators=(',', ':'),
                ),
                timestamp=round(time.time(), 3),
            )
        ]

        done_event = threading.Event()
        result = {'response': None, 'error': ''}

        def _on_done(future) -> None:
            try:
                result['response'] = future.result()
            except Exception as err:  # pragma: no cover - ROS transport failure
                result['error'] = str(err)
            finally:
                done_event.set()

        future = self._execution_report_chatbot_client.call_async(request)
        future.add_done_callback(_on_done)
        if not done_event.wait(timeout=self.execution_report_chatbot_timeout_sec):
            return _ExecutionReportResult(source='timeout')
        response = result.get('response')
        if response is None:
            if result.get('error'):
                self.get_logger().warn(
                    'Execution report chatbot request failed: %s' % result['error']
                )
            return _ExecutionReportResult(source='failed')
        error_msg = str(getattr(response, 'error_msg', '') or '').strip()
        if error_msg:
            self.get_logger().warn('Execution report chatbot returned error: %s' % error_msg)
            return _ExecutionReportResult(source='failed')
        text = str(getattr(response, 'response', '') or '').strip()
        if _looks_like_machine_payload(text):
            return _ExecutionReportResult(source='unsafe')
        return _ExecutionReportResult(text=text, source='chatbot')

    def _execute_ask_user_step(
        self,
        step_args: dict,
        fallback_data: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str, dict]:
        _ = fallback_data
        if on_started is not None:
            on_started()
        prompt_text = _first_non_empty_value(
            step_args,
            'question',
            'text',
            'summary_text',
            'text_hint',
            'message',
            'utterance',
            'object',
            'reason',
        )
        slots_needed = [
            str(item).strip()
            for item in list(step_args.get('slots_needed', []))
            if str(item).strip()
        ]
        if not prompt_text and slots_needed:
            prompt_text = 'I need a bit more detail about %s before I continue.' % ', '.join(
                slots_needed
            )
        if not prompt_text:
            prompt_text = 'I need a bit more detail before I continue.'

        payload = {
            'skill': 'ask_user',
            'status': 'awaiting_user',
            'await_user_response': True,
            'prompt_text': prompt_text,
            'slots_needed': slots_needed,
        }
        return False, prompt_text, payload

    def _execute_motion_plan_step(
        self,
        step_args: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str, dict]:
        if self.perform_motion_execution_mode == 'fake':
            success, reason, payload = self._execute_fake_skill_step(
                'perform_motion',
                dict(step_args or {}),
                on_started=on_started,
            )
            if success:
                return True, reason, payload
            return False, reason or 'fake perform_motion dispatch failed', payload

        route, resolved_payload = classify_motion_target(Intent.PERFORM_MOTION, step_args)
        if route == 'replay_motion':
            motion_name = resolved_payload['motion_name']
            success, reason = self._execute_replay_motion_step(
                motion_name,
                on_started=on_started,
            )
            if success:
                self._stats.dispatched_replay_motion += 1
                payload = _motion_result_payload(route, step_args, resolved_payload)
                return True, payload['summary_text'], payload
            return False, reason or 'motion dispatch failed', {}

        if route == 'head_motion':
            success, reason = self._execute_head_motion_step(
                resolved_payload,
                on_started=on_started,
            )
            if success:
                self._stats.dispatched_head_motion += 1
                payload = _motion_result_payload(route, step_args, resolved_payload)
                return True, payload['summary_text'], payload
            return False, reason or 'motion dispatch failed', {}

        if route == 'look_at_reset':
            success, reason = self._execute_look_at_reset_step(
                on_started=on_started,
            )
            if success:
                self._stats.dispatched_look_at += 1
                payload = _motion_result_payload(route, step_args, resolved_payload)
                return True, payload['summary_text'], payload
            return False, reason or 'look_at reset dispatch failed', {}

        self._stats.dispatch_failures += 1
        self.get_logger().warn('Unsupported motion payload: %s' % step_args)
        return False, 'unsupported motion payload', {}

    def _execute_scan_step(
        self,
        step_args: dict,
        *,
        should_auto_report: bool = True,
        on_started=None,
    ) -> tuple[bool, str, dict]:
        if on_started is not None:
            on_started()

        if self._scan_client is None or ScanScene is None:
            self._stats.dispatch_failures += 1
            return False, 'scan action client unavailable', {}
        return self._execute_scan_action_step(
            step_args,
            should_auto_report=should_auto_report,
        )

    def _execute_scan_action_step(
        self,
        step_args: dict,
        *,
        should_auto_report: bool = True,
    ) -> tuple[bool, str, dict]:
        scan_args = self._scan_args_from_step(step_args)
        goal = ScanScene.Goal()
        goal.target = str(scan_args.get('target', '')).strip()
        goal.target_kind = str(scan_args.get('target_kind', goal.target or 'scene')).strip().lower()
        goal.max_sweeps = int(scan_args.get('max_sweeps', 0))
        goal.evidence_policy = str(scan_args.get('evidence_policy', '')).strip()
        goal.result_mode = str(scan_args.get('result_mode', '')).strip().lower()

        result = self._execute_action_step(
            client=self._scan_client,
            goal=goal,
            wait_sec=self.scan_action_wait_sec,
            result_timeout_sec=self.scan_action_result_timeout_sec,
            description='scan_skill',
        )

        payload = self._scan_payload_from_action_result(result.raw_result, fallback_scan_args=scan_args)
        summary_text = str(payload.get('summary_text', result.reason or '')).strip()
        if not result.success:
            return False, result.reason or summary_text or 'scan action failed', payload

        if self.scan_report_after_success and should_auto_report and summary_text:
            speech_ok, speech_reason = self._execute_say_plan_step({'text': summary_text}, {})
            if not speech_ok:
                return False, speech_reason, payload
        return True, summary_text, payload

    def _execute_fake_skill_step(
        self,
        skill_name: str,
        step_args: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str, dict]:
        if on_started is not None:
            on_started()

        if ScanScene is None:
            self._stats.dispatch_failures += 1
            return False, 'ScanScene interface unavailable for fake skill dispatch', {}

        client = self._fake_skill_clients.get(skill_name)
        if client is None:
            self._stats.dispatch_failures += 1
            return False, 'fake skill action client unavailable for %s' % skill_name, {}

        goal = ScanScene.Goal()
        goal.target = str(step_args.get('target', step_args.get('location', ''))).strip()
        goal.target_kind = str(step_args.get('target_kind', '')).strip()
        goal.max_sweeps = int(step_args.get('max_sweeps', 0) or 0)
        goal.result_mode = str(step_args.get('result_mode', '')).strip().lower()

        args_payload = dict(step_args or {})
        scenario_id = str(args_payload.pop('scenario_id', '')).strip()
        scenario_override = args_payload.pop('scenario', args_payload.pop('scenario_override', {}))
        for field_name in ('target', 'location', 'target_kind', 'max_sweeps', 'result_mode'):
            args_payload.pop(field_name, None)

        evidence_payload = {'skill': skill_name}
        if args_payload:
            evidence_payload['args'] = args_payload
        if scenario_id:
            evidence_payload['scenario_id'] = scenario_id
        if isinstance(scenario_override, dict) and scenario_override:
            evidence_payload['scenario'] = scenario_override
        goal.evidence_policy = json.dumps(
            evidence_payload,
            sort_keys=True,
            separators=(',', ':'),
        )

        result = self._execute_action_step(
            client=client,
            goal=goal,
            wait_sec=self.fake_skill_wait_sec,
            result_timeout_sec=self.fake_skill_result_timeout_sec,
            description='fake_skill:%s' % skill_name,
        )
        payload = self._fake_payload_from_action_result(
            skill_name,
            result.raw_result,
            fallback_step_args=step_args,
        )
        summary_text = str(payload.get('summary_text', result.reason or '')).strip()
        if not result.success:
            return False, result.reason or summary_text or ('%s action failed' % skill_name), payload

        effect_summary = self._apply_success_kb_effects(payload)
        if effect_summary:
            payload['kb_effect_application'] = effect_summary
            if not effect_summary.get('applied', False):
                return (
                    False,
                    'successful skill KB post-condition failed',
                    payload,
                )
        self._stats.dispatched_fake_skill += 1
        return True, summary_text, payload

    def _apply_success_kb_effects(self, result_payload: dict) -> dict:
        """Apply successful skill-reported KB effects through the KB boundary."""
        if not self.apply_success_kb_effects:
            return {}
        if self._kb_mutation_client is None:
            return {}
        effects = _kb_effects_from_result_payload(result_payload)
        if not effects:
            return {}

        grouped_effects = _group_kb_effect_statements(effects)
        calls: list[dict] = []
        remaining_statements: list[str] = []
        missing_statements: list[str] = []
        invalid_statements: list[str] = []
        verification_attempted = self._kb_query_client is not None
        effect_models = ['default']
        should_clean_spatial_effects = (
            str(result_payload.get('skill', '')).strip() in _SPATIAL_EFFECT_SKILLS
        )

        for operation, statements in grouped_effects.items():
            if not statements:
                continue
            should_validate_spatial_effects = (
                operation in {'add', 'update'} and should_clean_spatial_effects
            )
            if should_validate_spatial_effects:
                validation_error = self._validate_spatial_effect_statements(
                    statements,
                    effect_models,
                )
                if validation_error:
                    calls.append(
                        {
                            'operation': 'validate_spatial_effects',
                            'statement_count': len(statements),
                            'success': False,
                            'dispatched': False,
                            'error_msg': validation_error,
                        }
                    )
                    self._stats.dispatch_failures += 1
                    invalid_statements.extend(statements)
                    continue
                removals, error_msg = self._remove_stale_spatial_effect_values(
                    statements,
                    effect_models,
                )
                if removals or error_msg:
                    calls.append(
                        {
                            'operation': 'remove_stale_spatial_values',
                            'statement_count': len(removals),
                            'success': not error_msg,
                            'dispatched': bool(removals),
                            'error_msg': error_msg,
                        }
                    )
                    if error_msg:
                        self._stats.dispatch_failures += 1
                        remaining_statements.extend(removals)
                        continue
                    self._stats.dispatched_kb_mutation += 1
            result = self._kb_mutation_client.mutate(
                operation=operation,
                statements=statements,
                models=effect_models,
                wait_for_result=True,
            )
            calls.append(
                {
                    'operation': operation,
                    'statement_count': result.statement_count,
                    'success': result.success,
                    'dispatched': result.dispatched,
                    'error_msg': result.error_msg,
                }
            )
            if result.success:
                self._stats.dispatched_kb_mutation += 1
                if operation == 'remove':
                    remaining_statements.extend(
                        self._query_remaining_kb_remove_facts(
                            statements,
                            effect_models,
                        )
                    )
                elif operation in {'add', 'update'}:
                    missing_statements.extend(
                        self._query_missing_kb_present_facts(
                            statements,
                            effect_models,
                        )
                    )
            else:
                self._stats.dispatch_failures += 1
        if not calls:
            return {}
        remaining_statements = _dedupe_statements(remaining_statements)
        missing_statements = _dedupe_statements(missing_statements)
        postcondition_failed = bool(remaining_statements or missing_statements)
        if postcondition_failed:
            self._stats.dispatch_failures += 1
        all_success = all(call.get('success') for call in calls) and not postcondition_failed
        if all_success:
            self.get_logger().info(
                'Applied successful skill KB effects | calls=%d verified=%s'
                % (len(calls), verification_attempted)
            )
        else:
            self.get_logger().warn(
                'Some successful skill KB effects failed | calls=%s' % calls
            )
        summary = {
            'applied': all_success,
            'verified': verification_attempted,
            'calls': calls,
        }
        if remaining_statements:
            summary['remaining_statements'] = remaining_statements
        if missing_statements:
            summary['missing_statements'] = missing_statements
        if invalid_statements:
            summary['invalid_statements'] = _dedupe_statements(invalid_statements)
        return summary

    def _query_missing_kb_present_facts(
        self,
        statements: list[str],
        models: list[str],
    ) -> list[str]:
        """Return added facts that are not visible through KnowledgeCore query."""
        if self._kb_query_client is None:
            return []
        missing: list[str] = []
        query_models = models if isinstance(models, list) and models else ['default']
        for statement in statements:
            subject, predicate, obj = _statement_parts(statement)
            if not subject or not predicate or not obj:
                continue
            rows = self._kb_query_client.query_rows(
                patterns=['%s %s ?object' % (subject, predicate)],
                query_vars=['?object'],
                models=query_models,
            )
            if not any(_binding_value(row, 'object') == obj for row in rows):
                missing.append(statement)
        return _dedupe_statements(missing)

    def _scan_args_from_step(self, step_args: dict) -> dict:
        scan_args = dict(step_args or {})
        target = str(scan_args.get('target', '')).strip()
        target_kind = str(scan_args.get('target_kind', target or 'scene')).strip().lower() or 'scene'
        scan_args['target'] = target
        scan_args['target_kind'] = target_kind
        scan_args['result_mode'] = (
            str(scan_args.get('result_mode', self.scan_result_mode)).strip().lower()
            or self.scan_result_mode
        )
        return scan_args

    def _scan_payload_from_action_result(self, action_result, *, fallback_scan_args: dict) -> dict:
        target = str(fallback_scan_args.get('target', '')).strip()
        target_kind = str(fallback_scan_args.get('target_kind', target or 'scene')).strip().lower() or 'scene'
        if action_result is None:
            return {
                'skill': 'scan',
                'target': target,
                'target_kind': target_kind,
                'target_found': False,
                'people': [],
                'objects': [],
                'summary_text': '',
                'confidence_policy': 'grounded_current_observation',
            }
        payload_json = str(getattr(action_result, 'result_payload_json', '')).strip()
        if payload_json:
            try:
                payload = json.loads(payload_json)
            except json.JSONDecodeError:
                payload = {}
            if isinstance(payload, dict):
                return payload
        summary_text = _first_non_empty_text(
            getattr(action_result, 'summary_text', ''),
            getattr(action_result, 'message', ''),
        )
        return {
            'skill': 'scan',
            'target': target,
            'target_kind': target_kind,
            'target_found': False,
            'people': [],
            'objects': [],
            'summary_text': summary_text,
            'confidence_policy': 'grounded_current_observation',
        }

    def _fake_payload_from_action_result(
        self,
        skill_name: str,
        action_result,
        *,
        fallback_step_args: dict,
    ) -> dict:
        target = str(fallback_step_args.get('target', fallback_step_args.get('location', ''))).strip()
        target_kind = str(fallback_step_args.get('target_kind', '')).strip()
        if action_result is None:
            return {
                'skill': skill_name,
                'status': 'failed',
                'target': target,
                'target_kind': target_kind,
                'target_found': False,
                'summary_text': '',
                'evidence': {},
                'failure': {'code': 'missing_result', 'recoverable': False},
                'metadata': {'fake': True},
            }
        payload_json = str(getattr(action_result, 'result_payload_json', '')).strip()
        if payload_json:
            try:
                payload = json.loads(payload_json)
            except json.JSONDecodeError:
                payload = {}
            if isinstance(payload, dict):
                return payload
        summary_text = _first_non_empty_text(
            getattr(action_result, 'summary_text', ''),
            getattr(action_result, 'message', ''),
        )
        return {
            'skill': skill_name,
            'status': 'succeeded' if bool(getattr(action_result, 'success', False)) else 'failed',
            'target': target,
            'target_kind': target_kind,
            'target_found': None,
            'summary_text': summary_text,
            'evidence': {},
            'failure': {},
            'metadata': {'fake': True},
        }

    def _execute_replay_motion_step(
        self,
        motion_name: str,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        clean_motion = str(motion_name).strip()
        if not clean_motion:
            self._stats.dispatch_failures += 1
            return False, 'replay motion dispatch failed: empty motion name'
        if ReplayMotion is None:
            return False, 'replay motion dispatch failed: nao_skills ReplayMotion action unavailable'
        goal = ReplayMotion.Goal()
        goal.motion_name = clean_motion
        goal.speed = float(self.replay_motion_speed)
        result = self._execute_action_step(
            client=self._replay_motion_client,
            goal=goal,
            wait_sec=self.replay_motion_wait_sec,
            result_timeout_sec=self.replay_motion_result_timeout_sec,
            description='replay_motion',
            on_started=on_started,
        )
        if result.success:
            self.get_logger().info('ORCH REPLAY_DISPATCH | %s' % clean_motion)
            return True, ''

        if (
            not result.accepted
            and self.fallback_to_posture_topic
            and self._posture_command_pub is not None
        ):
            fallback_ok, fallback_reason = self._execute_posture_topic_fallback(
                clean_motion,
                on_started=on_started,
            )
            if fallback_ok:
                return True, ''
            return False, fallback_reason or 'posture fallback failed'
        return False, result.reason or 'replay motion dispatch failed'

    def _execute_head_motion_step(
        self,
        payload: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        yaw = float(payload.get('yaw', 0.0))
        pitch = float(payload.get('pitch', 0.0))
        relative = bool(payload.get('relative', False))
        if DoHeadMotion is None:
            return False, 'head motion dispatch failed: nao_skills DoHeadMotion action unavailable'
        goal = DoHeadMotion.Goal()
        goal.yaw = yaw
        goal.pitch = pitch
        goal.speed = float(self.head_motion_speed)
        goal.relative = relative
        result = self._execute_action_step(
            client=self._head_motion_client,
            goal=goal,
            wait_sec=self.head_motion_wait_sec,
            result_timeout_sec=self.head_motion_result_timeout_sec,
            description='head_motion',
            on_started=on_started,
        )
        if result.success:
            self.get_logger().info(
                'ORCH HEAD_DISPATCH | yaw=%.3f pitch=%.3f relative=%s'
                % (yaw, pitch, relative)
            )
            return True, ''

        if (
            not result.accepted
            and self.fallback_to_joint_angles_topic
            and self._joint_angles_pub is not None
        ):
            if on_started is not None:
                on_started()
            msg = JointAnglesWithSpeed()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = ['HeadYaw', 'HeadPitch']
            msg.joint_angles = [yaw, pitch]
            msg.speed = float(self.head_motion_speed)
            msg.relative = 1 if relative else 0
            self._joint_angles_pub.publish(msg)
            self.get_logger().warn(
                'ORCH HEAD_TOPIC_FALLBACK | yaw=%.3f pitch=%.3f topic=%s'
                % (yaw, pitch, self.head_motion_joint_angles_topic)
            )
            return True, ''
        return False, result.reason or 'head motion dispatch failed'

    def _execute_look_at_reset_step(self, *, on_started=None) -> tuple[bool, str]:
        if LookAt is None:
            return False, 'look_at dispatch failed: interaction_skills LookAt action unavailable'
        goal = LookAt.Goal()
        goal.policy = LookAt.Goal.RESET
        result = self._execute_action_step(
            client=self._look_at_client,
            goal=goal,
            wait_sec=self.look_at_wait_sec,
            result_timeout_sec=self.look_at_result_timeout_sec,
            description='look_at_reset',
            on_started=on_started,
        )
        if result.success:
            self.get_logger().info('ORCH LOOK_AT_DISPATCH | policy=reset')
            return True, ''

        if not result.accepted:
            self.get_logger().warn('look_at action server unavailable; falling back to head reset')
            return self._execute_head_motion_step(
                {
                    'yaw': 0.0,
                    'pitch': 0.0,
                    'relative': False,
                },
                on_started=on_started,
            )
        return False, result.reason or 'look_at reset dispatch failed'

    def _execute_look_at_target_step(
        self,
        *,
        frame_id: str,
        x_value,
        y_value,
        z_value,
        policy: str = '',
        on_started=None,
    ) -> tuple[bool, str]:
        clean_frame = str(frame_id).strip()
        if not clean_frame:
            self._stats.dispatch_failures += 1
            self.get_logger().warn('No target frame resolved for look_at dispatch')
            return False, 'look_at target dispatch failed: missing target frame'
        if LookAt is None:
            return False, 'look_at dispatch failed: interaction_skills LookAt action unavailable'

        goal = LookAt.Goal()
        goal.policy = str(policy).strip().lower()
        target = PointStamped()
        target.header.frame_id = clean_frame
        target.point.x = float(x_value)
        target.point.y = float(y_value)
        target.point.z = float(z_value)
        goal.target = target
        result = self._execute_action_step(
            client=self._look_at_client,
            goal=goal,
            wait_sec=self.look_at_wait_sec,
            result_timeout_sec=self.look_at_result_timeout_sec,
            description='look_at_target',
            on_started=on_started,
        )
        if result.success:
            self.get_logger().info(
                'ORCH LOOK_AT_DISPATCH | policy=%s frame=%s x=%.3f y=%.3f z=%.3f'
                % (
                    goal.policy or 'track',
                    clean_frame,
                    float(target.point.x),
                    float(target.point.y),
                    float(target.point.z),
                )
            )
            return True, ''
        return False, result.reason or 'look_at target dispatch failed'

    def _execute_look_at_policy_step(
        self,
        *,
        policy: str,
        on_started=None,
    ) -> tuple[bool, str]:
        if LookAt is None:
            return False, 'look_at dispatch failed: interaction_skills LookAt action unavailable'
        goal = LookAt.Goal()
        goal.policy = str(policy).strip().lower()
        result = self._execute_action_step(
            client=self._look_at_client,
            goal=goal,
            wait_sec=self.look_at_wait_sec,
            result_timeout_sec=self.look_at_result_timeout_sec,
            description='look_at_policy',
            on_started=on_started,
        )
        if result.success:
            self.get_logger().info(
                'ORCH LOOK_AT_DISPATCH | policy=%s' % (goal.policy or 'auto')
            )
            return True, ''
        return False, result.reason or 'look_at policy dispatch failed'

    def _execute_action_step(
        self,
        *,
        client,
        goal,
        wait_sec: float,
        result_timeout_sec: float,
        description: str,
        on_started=None,
    ) -> _ActionExecutionResult:
        if client is None:
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=False,
                success=False,
                reason='%s action client unavailable' % description,
            )
        if not client.wait_for_server(timeout_sec=wait_sec):
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=False,
                success=False,
                reason='%s action server unavailable' % description,
            )

        acceptance_event = threading.Event()
        result_event = threading.Event()
        active_goal_handle = {'value': None}
        outcome = {
            'accepted': False,
            'success': False,
            'reason': '%s result timed out' % description,
            'raw_result': None,
        }

        def _goal_response_callback(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as err:  # pragma: no cover - ROS action transport failure
                outcome['reason'] = '%s goal response failed: %s' % (description, err)
                acceptance_event.set()
                result_event.set()
                return

            if goal_handle is None or not goal_handle.accepted:
                outcome['reason'] = '%s goal rejected' % description
                acceptance_event.set()
                result_event.set()
                return

            active_goal_handle['value'] = goal_handle
            outcome['accepted'] = True
            acceptance_event.set()
            if on_started is not None:
                try:
                    on_started()
                except Exception as err:
                    self.get_logger().debug(
                        '%s start callback failed: %s' % (description, err)
                    )
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_callback)

        def _result_callback(future) -> None:
            try:
                wrapped_result = future.result()
                action_result = getattr(wrapped_result, 'result', None)
                success, reason = self._action_result_status(action_result)
                outcome['success'] = success
                outcome['reason'] = reason
                outcome['raw_result'] = action_result
            except Exception as err:  # pragma: no cover - ROS action transport failure
                outcome['success'] = False
                outcome['reason'] = '%s result retrieval failed: %s' % (description, err)
            finally:
                result_event.set()

        deadline = ActionDeadline.start(result_timeout_sec)
        goal_future = client.send_goal_async(goal)
        goal_future.add_done_callback(_goal_response_callback)

        if not acceptance_event.wait(timeout=deadline.remaining()):
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=False,
                success=False,
                reason='%s goal response timed out' % description,
                raw_result=None,
            )
        if not outcome['accepted']:
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=False,
                success=False,
                reason=str(outcome['reason']).strip(),
                raw_result=None,
            )
        if not result_event.wait(timeout=deadline.remaining()):
            goal_handle = active_goal_handle.get('value')
            if goal_handle is not None:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as err:
                    self.get_logger().warn(
                        '%s timed out and cancel request failed: %s'
                        % (description, err)
                    )
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=True,
                success=False,
                reason='%s result timed out' % description,
                raw_result=None,
            )
        if not outcome['success']:
            self._stats.dispatch_failures += 1
        return _ActionExecutionResult(
            accepted=True,
            success=bool(outcome['success']),
            reason=str(outcome['reason']).strip(),
            raw_result=outcome.get('raw_result'),
        )

    @staticmethod
    def _action_result_status(action_result) -> tuple[bool, str]:
        if action_result is None:
            return False, 'action returned no result'
        if hasattr(action_result, 'success'):
            success = bool(getattr(action_result, 'success', False))
            message = str(getattr(action_result, 'message', '')).strip()
            if success:
                return True, message
            return False, message or 'action reported failure'
        return True, ''

    def _load_scan_skill_names(self) -> set[str]:
        return merge_scan_skill_names(
            fallback_names=_DEFAULT_SCAN_SKILL_ALIASES,
            manifest=load_shared_skill_manifest(),
        )

    def _load_fake_skill_aliases(self) -> dict[str, str]:
        fallback_aliases: dict[str, str] = {}
        for canonical, aliases in _DEFAULT_FAKE_SKILL_ALIASES.items():
            for alias in aliases:
                fallback_aliases[str(alias).strip().lower()] = canonical

        return merge_fake_skill_aliases(
            fallback_aliases=fallback_aliases,
            manifest=load_shared_skill_manifest(),
        )

    def _resolve_fake_skill_name(self, step_name: str) -> str:
        clean_name = str(step_name or '').strip().lower()
        if not clean_name:
            return ''
        return self._fake_skill_aliases.get(clean_name, '')

    @staticmethod
    def _parse_posture_result_message(payload: str) -> dict:
        try:
            parsed = json.loads(str(payload or '').strip())
        except json.JSONDecodeError:
            return {}
        return parsed if isinstance(parsed, dict) else {}

    @staticmethod
    def _posture_result_matches(payload: dict, fallback_command: str) -> bool:
        if not isinstance(payload, dict):
            return False
        expected = str(fallback_command or '').strip().lower()
        if not expected:
            return False
        candidates = (
            payload.get('normalized_command', ''),
            payload.get('command', ''),
            payload.get('posture_name', ''),
        )
        alias = _POSTURE_BRIDGE_NAME_ALIASES.get(expected, expected)
        return any(
            str(candidate or '').strip().lower() == expected
            or str(candidate or '').strip().lower() == alias
            for candidate in candidates
            if str(candidate or '').strip()
        )

    def _resolve_plan_id(self, plan_context: dict) -> str:
        plan_id = str(plan_context.get('plan_id', '')).strip()
        if plan_id:
            return plan_id
        return make_plan_id()

    @staticmethod
    def _plan_version_from_context(plan_context: dict) -> int:
        try:
            return max(0, int(plan_context.get('plan_version', 0) or 0))
        except (TypeError, ValueError):
            return 0

    @staticmethod
    def _find_plan_step_index(plan: list[dict], step_id: str) -> int:
        clean_step_id = str(step_id or '').strip()
        if not clean_step_id:
            return -1
        for index, step in enumerate(plan):
            if not isinstance(step, dict):
                continue
            if str(step.get('id', '')).strip() == clean_step_id:
                return index
        return -1

    def _claim_execution_plan(
        self,
        *,
        goal_id: str,
        plan_id: str,
        plan_version: int,
        plan: list[dict],
    ) -> _ExecutionJoinDecision:
        clean_goal_id = str(goal_id or '').strip()
        clean_plan_id = str(plan_id or '').strip()
        resolved_version = max(0, int(plan_version or 0))

        with self._execution_lock:
            previous_goal_id = self._active_execution_goal_id
            previous_plan_version = self._active_execution_plan_version
            previous_step_id = self._active_execution_step_id

            start_index = 0
            join_strategy = 'front_join'
            mapped_step_id = ''
            if (
                clean_goal_id
                and clean_goal_id == previous_goal_id
                and resolved_version > previous_plan_version
                and previous_step_id
            ):
                mapped_index = self._find_plan_step_index(plan, previous_step_id)
                if mapped_index >= 0:
                    start_index = mapped_index
                    join_strategy = 'mid_join'
                    mapped_step_id = previous_step_id

            self._active_execution_goal_id = clean_goal_id
            self._active_execution_plan_id = clean_plan_id
            self._active_execution_plan_version = resolved_version
            if 0 <= start_index < len(plan) and isinstance(plan[start_index], dict):
                self._active_execution_step_id = str(plan[start_index].get('id', '')).strip()
            else:
                self._active_execution_step_id = ''

        return _ExecutionJoinDecision(
            start_index=start_index,
            join_strategy=join_strategy,
            mapped_step_id=mapped_step_id,
        )

    def _set_active_execution_step(self, step: dict) -> None:
        if not isinstance(step, dict):
            return
        step_id = str(step.get('id', '')).strip()
        with self._execution_lock:
            self._active_execution_step_id = step_id

    def _is_execution_plan_active(self, goal_id: str, plan_id: str, plan_version: int) -> bool:
        with self._execution_lock:
            active_goal_id = self._active_execution_goal_id
            active_plan_id = self._active_execution_plan_id
            active_version = self._active_execution_plan_version
        if not active_goal_id:
            return False
        if str(goal_id or '').strip() != active_goal_id:
            return False
        clean_plan_id = str(plan_id or '').strip()
        if active_plan_id and clean_plan_id != active_plan_id:
            return False
        return max(0, int(plan_version or 0)) >= active_version

    def _is_exact_execution_plan_active(self, goal_id: str, plan_id: str, plan_version: int) -> bool:
        with self._execution_lock:
            active_goal_id = self._active_execution_goal_id
            active_plan_id = self._active_execution_plan_id
            active_version = self._active_execution_plan_version
        if not active_goal_id:
            return False
        return (
            str(goal_id or '').strip() == active_goal_id
            and str(plan_id or '').strip() == active_plan_id
            and max(0, int(plan_version or 0)) == active_version
        )

    def _finalize_execution_plan(self, *, goal_id: str, plan_id: str, plan_version: int) -> None:
        with self._execution_lock:
            if str(goal_id or '').strip() != self._active_execution_goal_id:
                return
            clean_plan_id = str(plan_id or '').strip()
            if clean_plan_id and self._active_execution_plan_id and clean_plan_id != self._active_execution_plan_id:
                return
            if max(0, int(plan_version or 0)) < self._active_execution_plan_version:
                return
            self._active_execution_goal_id = ''
            self._active_execution_plan_id = ''
            self._active_execution_plan_version = 0
            self._active_execution_step_id = ''

    def _publish_plan_feedback(
        self,
        *,
        intent_name: str,
        source: str,
        plan_context: dict,
        status: str,
        event_type: str = '',
        reason: str = '',
        step: dict | None = None,
        blocking: bool = False,
        unmet_preconditions: list[str] | None = None,
        needs_user_input: bool = False,
        validation_errors: list[str] | None = None,
        result_summary: str = '',
        result_payload: dict | None = None,
        plan_outcome_summary: dict | None = None,
    ) -> None:
        if self._planner_feedback_pub is None:
            return
        normalized_plan_context = dict(plan_context)
        normalized_plan_context['plan_id'] = self._resolve_plan_id(plan_context)
        payload = build_execution_feedback_payload(
            intent=str(intent_name).strip(),
            source=str(source).strip(),
            plan_context=normalized_plan_context,
            status=str(status).strip().lower(),
            event_type=event_type,
            reason=str(reason).strip(),
            step=step,
            blocking=blocking,
            unmet_preconditions=list(unmet_preconditions or []),
            needs_user_input=needs_user_input,
            validation_errors=list(validation_errors or []),
            timestamp_sec=round(time.time(), 3),
            result_summary=str(result_summary or '').strip(),
            result_payload=dict(result_payload or {}),
            plan_outcome_summary=dict(plan_outcome_summary or {}),
        )
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(',', ':'))
        self._planner_feedback_pub.publish(msg)
        self._planner_gate.observe_feedback(payload)

    def _on_posture_command_result(self, msg: String) -> None:
        payload = self._parse_posture_result_message(msg.data)
        if not payload:
            return
        with self._posture_result_lock:
            self._latest_posture_result = payload
        self._posture_result_event.set()

    def _execute_posture_topic_fallback(
        self,
        motion_name: str,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        if self._posture_command_pub is None:
            self._stats.dispatch_failures += 1
            return False, 'posture fallback publisher unavailable'

        fallback_command = posture_topic_fallback_for_motion(motion_name)
        if not fallback_command:
            self._stats.dispatch_failures += 1
            return False, 'no posture fallback is defined for %s' % motion_name

        with self._posture_result_lock:
            self._latest_posture_result = None
        self._posture_result_event.clear()

        if on_started is not None:
            on_started()

        msg = String()
        msg.data = fallback_command
        self._posture_command_pub.publish(msg)
        self.get_logger().warn(
            'ORCH REPLAY_TOPIC_FALLBACK | motion=%s command=%s topic=%s result_topic=%s'
            % (
                motion_name,
                fallback_command,
                self.posture_command_topic,
                self.posture_command_result_topic,
            )
        )
        return self._wait_for_posture_fallback_result(fallback_command)

    def _wait_for_posture_fallback_result(self, fallback_command: str) -> tuple[bool, str]:
        deadline = time.monotonic() + self.posture_command_result_timeout_sec
        while time.monotonic() <= deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self._posture_result_event.wait(timeout=min(0.1, remaining))
            self._posture_result_event.clear()
            with self._posture_result_lock:
                payload = dict(self._latest_posture_result or {})
            if not self._posture_result_matches(payload, fallback_command):
                continue
            success = bool(payload.get('success', False))
            reason = str(payload.get('message', '')).strip()
            if success:
                return True, reason or 'posture bridge reported success'
            self._stats.dispatch_failures += 1
            return False, reason or 'posture bridge reported failure'

        self._stats.dispatch_failures += 1
        return (
            False,
            'Timed out waiting for posture fallback result on %s'
            % self.posture_command_result_topic,
        )

    def _dispatch_motion_payload(self, payload: dict) -> tuple[bool, str]:
        route, resolved_payload = classify_motion_target(Intent.PERFORM_MOTION, payload)
        if route == 'replay_motion':
            motion_name = resolved_payload['motion_name']
            success, reason = self._execute_replay_motion_step(motion_name)
            return success, 'replay_motion:%s' % motion_name if success else reason

        if route == 'head_motion':
            success, reason = self._execute_head_motion_step(resolved_payload)
            return success, 'head_motion' if success else reason

        if route == 'look_at_reset':
            success, reason = self._execute_look_at_reset_step()
            return success, 'look_at_reset' if success else reason

        return False, 'unsupported'

    def _start_direct_motion_dispatch(self, payload: dict) -> None:
        route, _resolved_payload = classify_motion_target(Intent.PERFORM_MOTION, payload)
        if route == 'unsupported':
            self._stats.dispatch_failures += 1
            self._stats.last_route = 'ignored:unsupported_motion'
            self.get_logger().warn('Unsupported motion payload: %s' % payload)
            return

        worker = threading.Thread(
            target=self._execute_direct_motion_dispatch,
            kwargs={'payload': dict(payload)},
            daemon=True,
        )
        worker.start()

    def _execute_direct_motion_dispatch(self, *, payload: dict) -> None:
        dispatched, route_name = self._dispatch_motion_payload(payload)
        if dispatched:
            self._stats.last_route = route_name
            return
        self._stats.last_route = 'failed:%s' % (route_name or 'perform_motion')
        self.get_logger().warn(
            'Direct motion dispatch failed | payload=%s reason=%s'
            % (payload, route_name)
        )

    # -------------------------------------------------------------------------
    # Skill dispatch helpers
    # -------------------------------------------------------------------------

    def _dispatch_say(self, text: str, data: dict) -> bool:
        """Send one text payload to the canonical `/nao/say` action."""
        clean_text = str(text).strip()
        if not clean_text:
            self._stats.dispatch_failures += 1
            self.get_logger().warn('No text resolved for say/greet dispatch')
            return False
        if self._say_client is None:
            self._stats.dispatch_failures += 1
            return False
        if not self._say_client.wait_for_server(timeout_sec=self.nao_say_wait_sec):
            self._stats.dispatch_failures += 1
            self.get_logger().warn('nao_say action server unavailable')
            return False

        goal = Say.Goal()
        goal.input = clean_text
        goal.person_id = str(data.get('recipient', '')).strip()
        self._say_client.send_goal_async(goal)
        self.get_logger().info('ORCH SAY_DISPATCH | %s' % clean_text)
        return True

    def _is_duplicate(self, signature: str) -> bool:
        """Drop repeated intents that arrive inside the configured dedupe window."""
        now = time.monotonic()
        if (
            signature
            and signature == self._last_intent_signature
            and now - self._last_intent_ts <= self.dedupe_window_sec
        ):
            return True

        self._last_intent_signature = signature
        self._last_intent_ts = now
        return False

    # -------------------------------------------------------------------------
    # Diagnostics
    # -------------------------------------------------------------------------

    def _publish_diagnostics(self) -> None:
        if self._diag_pub is None:
            return
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name='/nao_orchestrator',
            message='nao_orchestrator running',
            values=[
                KeyValue(
                    key='state',
                    value='active' if self._is_active else 'inactive',
                ),
                KeyValue(key='intent_topic', value=self.intent_topic),
                KeyValue(
                    key='legacy_intent_bridge',
                    value=str(self.enable_legacy_intent_bridge),
                ),
                KeyValue(
                    key='dispatch_speech_intents',
                    value=str(self.dispatch_speech_intents),
                ),
                KeyValue(key='planner_gate_enabled', value=str(self.enable_planner_gate)),
                KeyValue(key='planner_gate_active_goal', value=self._planner_gate.active_goal_id),
                KeyValue(key='planner_gate_active_plan', value=self._planner_gate.active_plan_id),
                KeyValue(key='active_execution_goal', value=self._active_execution_goal_id),
                KeyValue(key='active_execution_plan', value=self._active_execution_plan_id),
                KeyValue(
                    key='active_execution_plan_version',
                    value=str(self._active_execution_plan_version),
                ),
                KeyValue(key='active_execution_step', value=self._active_execution_step_id),
                KeyValue(
                    key='intents_received',
                    value=str(self._stats.intents_received),
                ),
                KeyValue(
                    key='duplicates_ignored',
                    value=str(self._stats.duplicates_ignored),
                ),
                KeyValue(
                    key='dispatch_failures',
                    value=str(self._stats.dispatch_failures),
                ),
                KeyValue(
                    key='dispatched_kb_mutation',
                    value=str(self._stats.dispatched_kb_mutation),
                ),
                KeyValue(key='plans_started', value=str(self._stats.plans_started)),
                KeyValue(
                    key='plans_succeeded',
                    value=str(self._stats.plans_succeeded),
                ),
                KeyValue(key='plans_failed', value=str(self._stats.plans_failed)),
                KeyValue(key='last_intent', value=self._stats.last_intent),
                KeyValue(key='last_route', value=self._stats.last_route),
                KeyValue(key='last_plan_id', value=self._stats.last_plan_id),
                KeyValue(
                    key='last_plan_status',
                    value=self._stats.last_plan_status,
                ),
            ],
        )
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)
