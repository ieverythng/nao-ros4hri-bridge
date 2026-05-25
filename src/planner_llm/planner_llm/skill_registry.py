"""Planner-facing skill registry derived from exported package metadata."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import warnings

from planner_common import ExportedSkillManifest
from planner_common import load_exported_skill_manifests
from planner_common import load_shared_skill_manifest

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
_DEFAULT_SKILL_PACKAGES = (
    'nao_skills',
    'interaction_skills',
    'communication_skills',
)
_DERIVED_SKILL_SPECS = {
    'perform_motion': {
        'source_skill_ids': ('replay_motion', 'do_posture', 'do_head_motion'),
        'default_payload': {
            'name': 'perform_motion',
            'aliases': ['motion'],
            'category': 'embodiment',
            'params': ['object', 'speed', 'relative', 'yaw', 'pitch'],
            'required_params': ['object'],
            'preconditions': [],
            'expected_effects': ['robot posture or head orientation changes'],
            'observable_success': ['planner feedback accepted', 'planner feedback completed'],
            'failure_modes': ['unsupported motion payload', 'motion dispatch failed'],
            'retryable': True,
            'can_request_user_help': False,
            'can_request_clarification': True,
            'timeout_hint': 10.0,
            'safety_flags': ['motion'],
            'robot_adapter_mapping': 'nao_orchestrator.perform_motion',
        },
    },
    'look_at': {
        'source_skill_ids': ('look_at',),
        'default_payload': {
            'name': 'look_at',
            'aliases': [],
            'category': 'attention',
            'params': ['target_frame', 'frame_id', 'policy', 'x', 'y', 'z'],
            'required_params': [],
            'preconditions': [],
            'expected_effects': ['robot gaze is redirected or reset'],
            'observable_success': ['planner feedback step_started', 'planner feedback completed'],
            'failure_modes': [
                'look_at step missing target frame or reset policy',
                'look_at target dispatch failed',
            ],
            'retryable': True,
            'can_request_user_help': False,
            'can_request_clarification': True,
            'timeout_hint': 8.0,
            'safety_flags': ['attention'],
            'robot_adapter_mapping': 'nao_orchestrator.look_at',
        },
    },
    'scan': {
        'source_skill_ids': ('scan',),
        'default_payload': {
            'name': 'scan',
            'aliases': [],
            'category': 'perception',
            'params': ['target', 'target_kind', 'max_sweeps', 'kb_state'],
            'required_params': [],
            'preconditions': [
                'current KB state may indicate whether the target is already visible',
            ],
            'expected_effects': [
                'scene is scanned and KB-relevant perception output becomes available',
            ],
            'observable_success': [
                'target_detected',
                'kb_revise',
                'planner feedback completed',
            ],
            'failure_modes': ['scan requested failure', 'scan backend unavailable'],
            'retryable': True,
            'can_request_user_help': False,
            'can_request_clarification': True,
            'timeout_hint': 3.0,
            'safety_flags': ['perception'],
            'robot_adapter_mapping': 'nao_orchestrator.scan',
        },
    },
}


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
    planner_guidance: tuple[str, ...] = ()

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
            planner_guidance=_coerce_tuple(payload.get('planner_guidance', [])),
        )

    def matches_name(self, name: str) -> bool:
        clean_name = str(name or '').strip().lower()
        return clean_name == self.name or clean_name in self.aliases

    def prompt_summary(self) -> dict:
        return {
            'name': self.name,
            'aliases': list(self.aliases),
            'category': self.category,
            'params': list(self.params),
            'required_params': list(self.required_params),
            'preconditions': list(self.preconditions),
            'expected_effects': list(self.expected_effects),
            'observable_success': list(self.observable_success),
            'failure_modes': list(self.failure_modes),
            'retryable': self.retryable,
            'can_request_user_help': self.can_request_user_help,
            'can_request_clarification': self.can_request_clarification,
            'timeout_hint': self.timeout_hint,
            'safety_flags': list(self.safety_flags),
            'robot_adapter_mapping': self.robot_adapter_mapping,
            'planner_guidance': list(self.planner_guidance),
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
    def load(cls, path: str = '', *, logger=None) -> 'SkillRegistry':
        shared_registry = _load_shared_registry(path, logger=logger)
        if shared_registry is not None:
            shared_skills = _planner_skills_from_shared_registry(shared_registry)
            return cls(
                step_types=_DEFAULT_STEP_TYPES,
                skills=tuple(shared_skills),
            )

        registry_path, used_fallback = _resolve_registry_path(path)
        overlay_payload = _load_registry_overlay(registry_path)
        if used_fallback and not str(path or '').strip():
            _warn(
                logger,
                'planner_llm skill registry overlay not found in install space; '
                'falling back to source-tree metadata',
            )

        exported_skills = {
            manifest.skill_id: manifest
            for manifest in load_exported_skill_manifests(
                _DEFAULT_SKILL_PACKAGES,
                logger=logger,
            )
        }
        overlay_skills = _planner_skill_overlays(overlay_payload)

        step_types = tuple(overlay_payload.get('step_types', _DEFAULT_STEP_TYPES))
        derived_skills = _build_derived_skills(exported_skills, overlay_skills)
        if not derived_skills:
            derived_skills = tuple(
                PlannerSkill.from_dict(item)
                for item in overlay_skills
                if isinstance(item, dict)
            )

        return cls(
            step_types=step_types,
            skills=derived_skills,
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
        return bool(self.resolve_skill_name(step.get('name', '')))

    def filter_supported_steps(self, steps: list[dict]) -> list[dict]:
        supported: list[dict] = []
        for step in steps:
            normalized_step = self.normalize_step(step)
            if self.supports_step(normalized_step):
                supported.append(normalized_step)
        return supported

    def filter_supported_steps_with_rejections(
        self,
        steps: list[dict],
    ) -> tuple[list[dict], list[dict]]:
        supported: list[dict] = []
        rejected: list[dict] = []
        for step in steps:
            normalized_step = self.normalize_step(step)
            if self.supports_step(normalized_step):
                supported.append(normalized_step)
            else:
                rejected.append(normalized_step)
        return supported, rejected

    def prompt_manifest(self) -> list[dict]:
        return [skill.prompt_summary() for skill in self._skills]

    def resolve_skill_name(self, name: str) -> str:
        """Return canonical skill name for a direct name or alias, else empty."""
        clean_name = str(name or '').strip().lower()
        if not clean_name:
            return ''
        for skill in self._skills:
            if skill.matches_name(clean_name):
                return skill.name
        return ''

    def normalize_step(self, step: dict) -> dict:
        """Normalize one step and canonicalize skill aliases to canonical names."""
        if not isinstance(step, dict):
            return {}
        normalized = dict(step)
        step_type = str(normalized.get('type', '')).strip().lower()
        if step_type == 'skill':
            canonical_name = self.resolve_skill_name(normalized.get('name', ''))
            if canonical_name:
                normalized['name'] = canonical_name
        return normalized


def _build_derived_skills(
    exported_skills: dict[str, ExportedSkillManifest],
    overlay_skills: list[dict],
) -> tuple[PlannerSkill, ...]:
    overlay_by_name = {
        str(item.get('name', '')).strip().lower(): dict(item)
        for item in overlay_skills
        if isinstance(item, dict) and str(item.get('name', '')).strip()
    }

    skills: list[PlannerSkill] = []
    used_overlay_names: set[str] = set()

    for skill_name, spec in _DERIVED_SKILL_SPECS.items():
        source_manifests = [
            exported_skills[skill_id]
            for skill_id in spec['source_skill_ids']
            if skill_id in exported_skills
        ]
        overlay_payload = overlay_by_name.get(skill_name, {})
        if not source_manifests and not overlay_payload:
            continue
        if overlay_payload:
            used_overlay_names.add(skill_name)

        merged_payload = dict(spec['default_payload'])
        merged_payload.update(_derived_payload_from_manifests(skill_name, source_manifests))
        merged_payload.update(overlay_payload)
        skills.append(PlannerSkill.from_dict(merged_payload))

    for overlay_name, overlay_payload in overlay_by_name.items():
        if overlay_name in used_overlay_names:
            continue
        skills.append(PlannerSkill.from_dict(overlay_payload))

    return tuple(skill for skill in skills if skill.name)


def _derived_payload_from_manifests(
    skill_name: str,
    manifests: list[ExportedSkillManifest],
) -> dict:
    if not manifests:
        return {}

    description = ' '.join(
        manifest.description
        for manifest in manifests
        if manifest.description
    ).strip()
    params = _ordered_unique(
        item
        for manifest in manifests
        for item in manifest.input_names
    )
    observable_success = _ordered_unique(
        item
        for manifest in manifests
        for item in manifest.output_names + manifest.feedback_names
    )
    expected_effects = _ordered_unique(
        manifest.description
        for manifest in manifests
        if manifest.description
    )
    safety_flags = _ordered_unique(
        item
        for manifest in manifests
        for item in manifest.functional_domains
    )
    category = manifests[0].functional_domains[0] if manifests[0].functional_domains else ''

    payload = {
        'name': skill_name,
        'category': category,
        'params': params,
        'expected_effects': expected_effects,
        'observable_success': observable_success,
        'safety_flags': safety_flags,
    }
    if description:
        payload['expected_effects'] = [description]
    return payload
def _load_registry_overlay(registry_path: Path | None) -> dict:
    if registry_path is None or not registry_path.exists():
        return {}
    try:
        return json.loads(registry_path.read_text(encoding='utf-8'))
    except Exception:
        return {}


def _planner_skill_overlays(payload: dict) -> list[dict]:
    """Accept either the legacy planner registry or the canonical AB registry."""
    skills = payload.get('skills')
    if isinstance(skills, list):
        return [dict(item) for item in skills if isinstance(item, dict)]

    objects = payload.get('objects', [])
    if not isinstance(objects, list):
        return []
    return [
        _planner_skill_from_ab_object(item)
        for item in objects
        if _is_planner_skill_ab_object(item)
    ]


def _is_planner_skill_ab_object(payload) -> bool:
    if not isinstance(payload, dict):
        return False
    kind = str(payload.get('kind', '')).strip().lower()
    if kind != 'skill':
        return False
    try:
        ab_level = int(payload.get('ab_level', payload.get('abstraction_level', 0)) or 0)
    except (TypeError, ValueError):
        ab_level = 0
    return ab_level >= 1 and bool(str(payload.get('object_id', payload.get('name', ''))).strip())


def _planner_skill_from_ab_object(payload: dict) -> dict:
    skill_payload = dict(payload)
    skill_payload['name'] = str(payload.get('object_id', payload.get('name', ''))).strip()
    skill_payload['abstraction_level'] = int(
        payload.get('ab_level', payload.get('abstraction_level', 1)) or 1
    )
    status = str(payload.get('implementation_status', '')).strip().lower()
    skill_payload.setdefault('retryable', 'navigation' in _coerce_tuple(payload.get('safety_flags', ())))
    skill_payload.setdefault('can_request_user_help', 'navigation' in _coerce_tuple(payload.get('safety_flags', ())))
    skill_payload.setdefault('can_request_clarification', True)
    skill_payload.setdefault('timeout_hint', _timeout_hint_from_ab_object(payload))
    skill_payload.setdefault('is_fake', status == 'fake')
    return skill_payload


def _timeout_hint_from_ab_object(payload: dict) -> float:
    category = str(payload.get('category', '')).strip().lower()
    if category == 'navigation':
        return 20.0
    if category == 'perception':
        return 3.0
    if category == 'attention':
        return 8.0
    if category == 'embodiment':
        return 10.0
    return 5.0


def _resolve_registry_path(path: str) -> tuple[Path | None, bool]:
    clean_path = str(path or '').strip()
    if clean_path:
        return Path(clean_path), False

    install_candidate: Path | None = None
    try:
        share_dir = Path(get_package_share_directory('planner_llm'))
        install_candidate = share_dir / 'config' / _DEFAULT_SKILL_REGISTRY_FILE
        if install_candidate.exists():
            return install_candidate, False
    except PackageNotFoundError:
        install_candidate = None

    source_candidate = Path(__file__).resolve().parent.parent / 'config' / _DEFAULT_SKILL_REGISTRY_FILE
    if source_candidate.exists():
        return source_candidate, install_candidate is not None

    return install_candidate or source_candidate, install_candidate is not None
def _ordered_unique(values) -> list[str]:
    seen: set[str] = set()
    ordered: list[str] = []
    for value in values:
        clean_value = str(value).strip()
        if not clean_value:
            continue
        if clean_value in seen:
            continue
        seen.add(clean_value)
        ordered.append(clean_value)
    return ordered


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


def _warn(logger, message: str) -> None:
    if logger is not None:
        logger.warn(message)
        return
    warnings.warn(message, stacklevel=2)


def _load_shared_registry(path: str, *, logger=None):
    clean_path = str(path or '').strip()
    try:
        shared_registry = load_shared_skill_manifest(clean_path)
        return tuple(shared_registry) if shared_registry else None
    except Exception as err:  # pragma: no cover - runtime dependency/errors
        _warn(
            logger,
            'planner_llm shared skill_common registry unavailable, using legacy overlay: %s'
            % err,
        )
        return None


def _planner_skill_payload_from_shared(payload: dict) -> dict:
    data = dict(payload or {})
    metadata = data.get('metadata', {}) if isinstance(data.get('metadata', {}), dict) else {}
    return {
        'name': str(data.get('name', '')).strip().lower(),
        'aliases': list(data.get('aliases', []) or []),
        'category': str(data.get('category', '')).strip(),
        'params': list(data.get('params', []) or []),
        'required_params': list(data.get('required_params', []) or []),
        'preconditions': list(data.get('preconditions', []) or []),
        'expected_effects': list(data.get('expected_effects', []) or []),
        'observable_success': list(data.get('observable_success', []) or []),
        'failure_modes': list(data.get('failure_modes', []) or []),
        'planner_guidance': list(data.get('planner_guidance', []) or []),
        'safety_flags': list(data.get('safety_flags', []) or []),
        'robot_adapter_mapping': str(data.get('robot_adapter_mapping', '')).strip(),
        'retryable': True,
        'can_request_user_help': False,
        'can_request_clarification': True,
        'timeout_hint': _shared_timeout_hint(data, metadata),
    }


def _planner_skills_from_shared_registry(shared_registry) -> list[PlannerSkill]:
    skills = []
    for item in shared_registry:
        normalized = _planner_skill_payload_from_shared(item)
        if not normalized.get('name', ''):
            continue
        skills.append(PlannerSkill.from_dict(normalized))
    return skills


def _shared_timeout_hint(data: dict, metadata: dict) -> float:
    if 'timeout_hint_sec' in metadata:
        return _coerce_float(metadata.get('timeout_hint_sec'))
    return _timeout_hint_from_ab_object(data)
