"""Prompt-pack loading for planner_llm."""

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


# Structural defaults are safe because they are not prompt wording.
DEFAULT_PROMPT_PACK_VERSION = 'planner_llm_prompt_pack_v1'
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
    """Load the canonical packaged planner prompt pack."""
    return load_prompt_pack('')


def load_prompt_pack(path: str, logger=None) -> PlannerPromptPack:
    """Load planner prompt pack YAML; fail loudly on prompt defects."""
    pack_path = str(path or '').strip()
    source = Path(pack_path) if pack_path else _default_prompt_pack_path()
    if source is None:
        raise FileNotFoundError('Could not resolve planner prompt pack path')
    if not source.exists():
        raise FileNotFoundError(f'Planner prompt pack path does not exist: "{source}"')

    if yaml is None:
        raise RuntimeError('PyYAML is required to load planner prompt packs')

    try:
        raw = source.read_text(encoding='utf-8')
    except Exception as err:  # pragma: no cover - filesystem dependent
        raise RuntimeError(f'Could not read planner prompt pack "{source}": {err}') from err

    try:
        parsed = yaml.safe_load(raw)
    except Exception as err:
        raise ValueError(f'Planner prompt pack parse failed for "{source}": {err}') from err

    if not isinstance(parsed, dict):
        raise ValueError(f'Planner prompt pack root must be a mapping: "{source}"')
    if not _as_text(parsed.get('system_prompt')):
        raise ValueError(f'Planner prompt pack "{source}" must define non-empty system_prompt')

    merged = _merge_dicts(
        {
            'prompt_pack_version': DEFAULT_PROMPT_PACK_VERSION,
            'system_prompt': '',
            'output_contract': DEFAULT_OUTPUT_CONTRACT,
            'validation_retry': DEFAULT_VALIDATION_RETRY,
        },
        parsed,
    )

    output_contract = merged.get('output_contract', DEFAULT_OUTPUT_CONTRACT)
    if not isinstance(output_contract, dict):
        _warn(logger, 'output_contract must be a mapping; using structural defaults')
        output_contract = DEFAULT_OUTPUT_CONTRACT

    validation_retry = merged.get('validation_retry', DEFAULT_VALIDATION_RETRY)
    if not isinstance(validation_retry, dict):
        _warn(logger, 'validation_retry must be a mapping; using structural defaults')
        validation_retry = DEFAULT_VALIDATION_RETRY

    return PlannerPromptPack(
        prompt_pack_version=_as_text(merged.get('prompt_pack_version', DEFAULT_PROMPT_PACK_VERSION)),
        system_prompt=_as_text(merged.get('system_prompt')),
        output_contract=_coerce_output_contract(output_contract, DEFAULT_OUTPUT_CONTRACT),
        validation_retry=_coerce_validation_retry(validation_retry, DEFAULT_VALIDATION_RETRY),
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
