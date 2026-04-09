"""Planner-owned abstract skill registry."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

try:  # pragma: no cover - runtime dependency
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - import-light unit tests
    class PackageNotFoundError(Exception):
        pass

    def get_package_share_directory(_package_name: str) -> str:
        raise PackageNotFoundError('ament_index_python is unavailable')


_DEFAULT_SKILL_REGISTRY_FILE = 'skill_registry.json'
_DEFAULT_STEP_TYPES = ('noop', 'say', 'skill', 'look_at')


@dataclass(frozen=True)
class PlannerSkill:
    """One abstract skill exposed to the supervisor."""

    name: str
    category: str
    params: tuple[str, ...]
    required_params: tuple[str, ...]
    preconditions: tuple[str, ...]
    expected_effects: tuple[str, ...]
    observable_success: tuple[str, ...]
    failure_modes: tuple[str, ...]
    retryable: bool
    can_request_user_help: bool
    can_request_clarification: bool
    timeout_hint: float
    safety_flags: tuple[str, ...]
    robot_adapter_mapping: str
    aliases: tuple[str, ...] = ()

    @classmethod
    def from_dict(cls, payload: dict) -> 'PlannerSkill':
        return cls(
            name=str(payload.get('name', '')).strip().lower(),
            category=str(payload.get('category', '')).strip(),
            params=_coerce_tuple(payload.get('params', [])),
            required_params=_coerce_tuple(payload.get('required_params', [])),
            preconditions=_coerce_tuple(payload.get('preconditions', [])),
            expected_effects=_coerce_tuple(payload.get('expected_effects', [])),
            observable_success=_coerce_tuple(payload.get('observable_success', [])),
            failure_modes=_coerce_tuple(payload.get('failure_modes', [])),
            retryable=bool(payload.get('retryable', False)),
            can_request_user_help=bool(payload.get('can_request_user_help', False)),
            can_request_clarification=bool(payload.get('can_request_clarification', False)),
            timeout_hint=_coerce_float(payload.get('timeout_hint', 0.0)),
            safety_flags=_coerce_tuple(payload.get('safety_flags', [])),
            robot_adapter_mapping=str(payload.get('robot_adapter_mapping', '')).strip(),
            aliases=_coerce_tuple(payload.get('aliases', [])),
        )

    def matches_name(self, name: str) -> bool:
        clean_name = str(name or '').strip().lower()
        return clean_name == self.name or clean_name in self.aliases

    def prompt_summary(self) -> dict:
        return {
            'name': self.name,
            'category': self.category,
            'required_params': list(self.required_params),
            'preconditions': list(self.preconditions),
            'expected_effects': list(self.expected_effects),
            'failure_modes': list(self.failure_modes),
            'retryable': self.retryable,
            'can_request_user_help': self.can_request_user_help,
            'can_request_clarification': self.can_request_clarification,
        }


class SkillRegistry:
    """Load and validate planner-facing skill metadata."""

    def __init__(
        self,
        *,
        step_types: tuple[str, ...],
        skills: tuple[PlannerSkill, ...],
    ) -> None:
        self._step_types = tuple(str(item).strip().lower() for item in step_types if str(item).strip())
        self._skills = tuple(skill for skill in skills if skill.name)

    @classmethod
    def load(cls, path: str = '') -> 'SkillRegistry':
        registry_path = _resolve_registry_path(path)
        if not registry_path.exists():
            raise FileNotFoundError('planner skill registry not found: %s' % registry_path)

        payload = json.loads(registry_path.read_text(encoding='utf-8'))
        raw_skills = payload.get('skills', [])
        if not isinstance(raw_skills, list):
            raw_skills = []

        return cls(
            step_types=tuple(payload.get('step_types', _DEFAULT_STEP_TYPES)),
            skills=tuple(
                PlannerSkill.from_dict(item)
                for item in raw_skills
                if isinstance(item, dict)
            ),
        )

    @property
    def step_types(self) -> tuple[str, ...]:
        return self._step_types

    @property
    def skills(self) -> tuple[PlannerSkill, ...]:
        return self._skills

    @property
    def allowed_skill_names(self) -> tuple[str, ...]:
        names: list[str] = []
        for skill in self._skills:
            names.append(skill.name)
            names.extend(skill.aliases)
        seen: set[str] = set()
        ordered_names: list[str] = []
        for name in names:
            clean_name = str(name).strip().lower()
            if not clean_name or clean_name in seen:
                continue
            seen.add(clean_name)
            ordered_names.append(clean_name)
        return tuple(ordered_names)

    def supports_step(self, step: dict) -> bool:
        step_type = str(step.get('type', '')).strip().lower()
        if step_type not in self._step_types:
            return False
        if step_type != 'skill':
            return True
        step_name = str(step.get('name', '')).strip().lower()
        if not step_name:
            return False
        return any(skill.matches_name(step_name) for skill in self._skills)

    def filter_supported_steps(self, steps: list[dict]) -> list[dict]:
        return [step for step in steps if self.supports_step(step)]

    def prompt_manifest(self) -> list[dict]:
        return [skill.prompt_summary() for skill in self._skills]


def _resolve_registry_path(path: str) -> Path:
    clean_path = str(path or '').strip()
    if clean_path:
        return Path(clean_path)
    try:
        share_dir = Path(get_package_share_directory('planner_llm'))
    except PackageNotFoundError:
        return Path(__file__).resolve().parent.parent / 'config' / _DEFAULT_SKILL_REGISTRY_FILE
    return share_dir / 'config' / _DEFAULT_SKILL_REGISTRY_FILE


def _coerce_tuple(value) -> tuple[str, ...]:
    if isinstance(value, str):
        clean_value = value.strip()
        return (clean_value,) if clean_value else ()
    if not isinstance(value, (list, tuple)):
        return ()
    return tuple(clean for clean in (str(item).strip() for item in value) if clean)


def _coerce_float(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0
