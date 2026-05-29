"""Prompt-pack loading and defaults for planner_llm."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:  # pragma: no cover - runtime dependency
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - runtime dependency

    class PackageNotFoundError(Exception):
        pass

    def get_package_share_directory(_package_name: str) -> str:
        raise PackageNotFoundError('ament_index_python is unavailable')

try:  # pragma: no cover - runtime dependency
    import yaml
except ImportError:  # pragma: no cover - runtime dependency
    yaml = None


DEFAULT_PROMPT_PACK_VERSION = 'planner_llm_prompt_pack_v1'
DEFAULT_SYSTEM_PROMPT = (
    'You are planner_llm for a ROS4HRI robot. Reply with one JSON object only. '
    'Return fields ack_text, ack_mode, decision, validation_status, failure_reason, '
    'replan_hint, retry_budget, scene_targets, communication_policy, and steps. '
    'Each step must contain type, name, args, requires, on_failure, and retry_budget. '
    'Plan only over the supplied abstract skill registry and allowed step types. '
    'Important contract: type="skill" may only use names from allowed_skill_names; '
    'do not use skill name "say" or direct speech actions inside executable plans. '
    'Do not reference robot-specific topics, NAOqi APIs, or direct hardware calls. '
    'normalized_intents may be incomplete, so infer the executable request from goal_text, '
    'grounded context, and execution feedback. Treat requested_plan as a compatibility '
    'fallback only, never as higher priority than goal_text or allowed skills. '
    'For perception reports, choose skills from intent and available context instead '
    'of keyword-matching "what do you see" to scan every time. Use scan when the user '
    'asks for a fresh look-around/search or when a report truly needs new observation; '
    'for head-motion plus report, perform the requested motion first and add scan only '
    'if fresh perception is needed. Include target/target_kind when relevant; clarify '
    'if the target is ambiguous. '
    'For greeting-only or social check-in requests (for example "hi", "hello", "hey"), '
    'do not infer physical actions (including wave_greet) unless explicitly requested; '
    'return decision=clarify with a short clarification_text asking what action is wanted. '
    'For report_result after a prior skill, either omit summary_text so the executor '
    'reuses the latest result, or provide plain natural text; never emit unresolved '
    'placeholders such as [evidence.objects] or [evidence.people]. '
    'Do not add say steps to executable plans; chatbot_llm owns user-facing completion '
    'wording after execution. '
    'If the task is ambiguous or blocked, set '
    'decision to clarify and include clarification_text. If no safe continuation exists, '
    'set decision to fail and explain why.'
)

DEFAULT_OUTPUT_CONTRACT: dict[str, Any] = {
    'step_type_skill': {
        'type': 'skill',
        'name': 'must be one allowed_skill_names entry',
    },
    'step_type_say': {
        'type': 'say',
        'name': 'say',
        'usage': (
            'only for clarify/fail/pure dialogue decisions, never mixed with executable '
            'steps'
        ),
        'args': {'text': 'clarification or failure text'},
    },
    'invalid_examples': [
        {'type': 'skill', 'name': 'say'},
        {'type': 'skill', 'name': 'nao_say'},
        {
            'steps': [
                {'type': 'skill', 'name': 'scan'},
                {'type': 'say', 'name': 'say'},
            ],
        },
    ],
}

DEFAULT_VALIDATION_RETRY: dict[str, Any] = {
    'instruction': (
        'Regenerate the full plan as one valid JSON object. Correct only the '
        'planner contract errors. Do not ask the user for clarification unless '
        'the original human request is genuinely ambiguous.'
    ),
    'previous_model_output_max_chars': 4000,
}


@dataclass(frozen=True)
class PlannerPromptPack:
    """Immutable planner prompt pack data."""

    prompt_pack_version: str
    system_prompt: str
    output_contract: dict[str, Any]
    validation_retry: dict[str, Any]
    source_path: str = ''


def default_prompt_pack() -> PlannerPromptPack:
    """Return resilient built-in defaults used when pack loading fails."""
    return PlannerPromptPack(
        prompt_pack_version=DEFAULT_PROMPT_PACK_VERSION,
        system_prompt=DEFAULT_SYSTEM_PROMPT,
        output_contract=dict(DEFAULT_OUTPUT_CONTRACT),
        validation_retry=dict(DEFAULT_VALIDATION_RETRY),
    )


def load_prompt_pack(path: str, logger=None) -> PlannerPromptPack:
    """Load prompt pack YAML and merge into defaults."""
    defaults = default_prompt_pack()
    pack_path = str(path or '').strip()
    source = Path(pack_path) if pack_path else _default_prompt_pack_path()
    if source is None:
        return defaults
    if not source.exists():
        _warn(logger, f'Prompt pack path does not exist: "{source}"')
        return _fallback_pack(defaults, source)

    if yaml is None:
        _warn(logger, 'PyYAML unavailable; prompt pack ignored')
        return _fallback_pack(defaults, source)

    try:
        raw = source.read_text(encoding='utf-8')
    except Exception as err:  # pragma: no cover - filesystem dependent
        _warn(logger, f'Could not read prompt pack: {err}')
        return _fallback_pack(defaults, source)

    try:
        parsed = yaml.safe_load(raw)
    except Exception as err:
        _warn(logger, f'Prompt pack parse failed: {err}')
        return _fallback_pack(defaults, source)

    if not isinstance(parsed, dict):
        _warn(logger, 'Prompt pack root must be a mapping')
        return _fallback_pack(defaults, source)

    merged = _merge_dicts(
        {
            'prompt_pack_version': defaults.prompt_pack_version,
            'system_prompt': defaults.system_prompt,
            'output_contract': defaults.output_contract,
            'validation_retry': defaults.validation_retry,
        },
        parsed,
    )

    output_contract = merged.get('output_contract', defaults.output_contract)
    if not isinstance(output_contract, dict):
        _warn(logger, 'output_contract must be a mapping; using defaults')
        output_contract = defaults.output_contract

    validation_retry = merged.get('validation_retry', defaults.validation_retry)
    if not isinstance(validation_retry, dict):
        _warn(logger, 'validation_retry must be a mapping; using defaults')
        validation_retry = defaults.validation_retry

    return PlannerPromptPack(
        prompt_pack_version=_as_text(merged.get('prompt_pack_version', defaults.prompt_pack_version)),
        system_prompt=_as_text(merged.get('system_prompt', defaults.system_prompt)),
        output_contract=_coerce_output_contract(output_contract, defaults.output_contract),
        validation_retry=_coerce_validation_retry(validation_retry, defaults.validation_retry),
        source_path=str(source),
    )


def _coerce_output_contract(value, fallback: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(value, dict):
        return dict(fallback)
    merged = _merge_dicts(dict(fallback), value)
    return merged


def _coerce_validation_retry(value, fallback: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(value, dict):
        return dict(fallback)
    instruction = _as_text(value.get('instruction', fallback.get('instruction', '')))
    if not instruction:
        instruction = _as_text(fallback.get('instruction', ''))

    max_chars_raw = value.get(
        'previous_model_output_max_chars',
        fallback.get('previous_model_output_max_chars', 4000),
    )
    try:
        max_chars = max(0, int(max_chars_raw))
    except (TypeError, ValueError):
        max_chars = int(fallback.get('previous_model_output_max_chars', 4000))
    return {
        'instruction': instruction,
        'previous_model_output_max_chars': max_chars,
    }


def _merge_dicts(base: dict[str, Any], updates: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, raw_value in updates.items():
        value = raw_value
        if key == '__proto__':
            continue
        existing = merged.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            merged[key] = _merge_dicts(existing, value)
            continue
        if isinstance(existing, list) and isinstance(value, list):
            merged[key] = list(value)
            continue
        merged[key] = value
    return merged


def _as_text(value) -> str:
    if value is None:
        return ''
    return str(value).strip()


def _fallback_pack(defaults: PlannerPromptPack, source: Path) -> PlannerPromptPack:
    return PlannerPromptPack(
        prompt_pack_version=defaults.prompt_pack_version,
        system_prompt=defaults.system_prompt,
        output_contract=defaults.output_contract,
        validation_retry=defaults.validation_retry,
        source_path=str(source),
    )


def _default_prompt_pack_path() -> Path | None:
    try:
        return Path(get_package_share_directory('planner_llm')) / 'config' / 'planner_prompt_pack.yaml'
    except PackageNotFoundError:
        source_candidate = Path(__file__).resolve().parents[1] / 'config' / 'planner_prompt_pack.yaml'
        return source_candidate if source_candidate.exists() else None


def _warn(logger, message: str) -> None:
    if logger is not None:
        logger.warn(message)
        return
    # Avoid raising from import-time logger usage; keep behavior lightweight in unit tests.
    return
