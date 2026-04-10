"""Planner-facing skill registry derived from exported package metadata."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import textwrap
import warnings
import xml.etree.ElementTree as ET

try:  # pragma: no cover - runtime dependency
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - import-light unit tests
    class PackageNotFoundError(Exception):
        pass

    def get_package_share_directory(_package_name: str) -> str:
        raise PackageNotFoundError('ament_index_python is unavailable')

try:  # pragma: no cover - optional dependency
    import yaml
except ImportError:  # pragma: no cover - import-light unit tests
    yaml = None


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


@dataclass(frozen=True)
class ExportedSkillManifest:
    """One skill manifest exported by a package.xml file."""

    package: str
    skill_id: str
    interface_path: str
    datatype: str
    description: str
    input_names: tuple[str, ...]
    output_names: tuple[str, ...]
    feedback_names: tuple[str, ...]
    functional_domains: tuple[str, ...]


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
        registry_path, used_fallback = _resolve_registry_path(path)
        overlay_payload = _load_registry_overlay(registry_path)
        if used_fallback and not str(path or '').strip():
            _warn(
                logger,
                'planner_llm skill registry overlay not found in install space; '
                'falling back to source-tree metadata',
            )

        exported_skills = _load_exported_skill_manifests(
            _DEFAULT_SKILL_PACKAGES,
            logger=logger,
        )
        overlay_skills = overlay_payload.get('skills', [])
        if not isinstance(overlay_skills, list):
            overlay_skills = []

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
        step_name = str(step.get('name', '')).strip().lower()
        if not step_name:
            return False
        return any(skill.matches_name(step_name) for skill in self._skills)

    def filter_supported_steps(self, steps: list[dict]) -> list[dict]:
        return [step for step in steps if self.supports_step(step)]

    def prompt_manifest(self) -> list[dict]:
        return [skill.prompt_summary() for skill in self._skills]


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


def _load_exported_skill_manifests(
    package_names: tuple[str, ...],
    *,
    logger=None,
) -> dict[str, ExportedSkillManifest]:
    manifests: dict[str, ExportedSkillManifest] = {}
    for package_name in package_names:
        package_xml = _resolve_package_xml(package_name)
        if package_xml is None:
            continue
        for manifest in _package_skill_manifests(package_name, package_xml, logger=logger):
            manifests[manifest.skill_id] = manifest
    return manifests


def _package_skill_manifests(
    package_name: str,
    package_xml: Path,
    *,
    logger=None,
) -> list[ExportedSkillManifest]:
    try:
        root = ET.parse(package_xml).getroot()
    except Exception as err:
        _warn(logger, 'Could not parse %s: %s' % (package_xml, err))
        return []

    manifests: list[ExportedSkillManifest] = []
    for skill_elem in root.findall('.//skill'):
        if str(skill_elem.attrib.get('content-type', '')).strip().lower() != 'yaml':
            continue

        manifest_text = textwrap.dedent(skill_elem.text or '').strip()
        if not manifest_text:
            continue
        manifest_payload = _parse_yaml_manifest(manifest_text, logger=logger)
        if not manifest_payload:
            continue

        skill_id = str(manifest_payload.get('id', '')).strip().lower()
        if not skill_id:
            continue
        manifests.append(
            ExportedSkillManifest(
                package=package_name,
                skill_id=skill_id,
                interface_path=str(manifest_payload.get('default_interface_path', '')).strip(),
                datatype=str(manifest_payload.get('datatype', '')).strip(),
                description=' '.join(
                    str(manifest_payload.get('description', '')).split()
                ).strip(),
                input_names=_parameter_names(manifest_payload, 'in'),
                output_names=_parameter_names(manifest_payload, 'out'),
                feedback_names=_parameter_names(manifest_payload, 'feedback'),
                functional_domains=_coerce_tuple(manifest_payload.get('functional_domains', [])),
            )
        )
    return manifests


def _parse_yaml_manifest(payload: str, *, logger=None) -> dict:
    if yaml is None:
        _warn(logger, 'PyYAML is unavailable; planner skill metadata parsing is degraded')
        return {}
    try:
        parsed = yaml.safe_load(payload)
    except Exception as err:
        _warn(logger, 'Skill manifest YAML parse failed: %s' % err)
        return {}
    return dict(parsed) if isinstance(parsed, dict) else {}


def _parameter_names(payload: dict, section: str) -> tuple[str, ...]:
    parameters = payload.get('parameters', {})
    if not isinstance(parameters, dict):
        return ()
    raw_section = parameters.get(section, [])
    if not isinstance(raw_section, list):
        return ()
    return tuple(
        str(item.get('name', '')).strip()
        for item in raw_section
        if isinstance(item, dict) and str(item.get('name', '')).strip()
    )


def _load_registry_overlay(registry_path: Path | None) -> dict:
    if registry_path is None or not registry_path.exists():
        return {}
    try:
        return json.loads(registry_path.read_text(encoding='utf-8'))
    except Exception:
        return {}


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


def _resolve_package_xml(package_name: str) -> Path | None:
    candidates: list[Path] = []
    try:
        share_dir = Path(get_package_share_directory(package_name))
        candidates.append(share_dir / 'package.xml')
    except PackageNotFoundError:
        pass

    repo_root = Path(__file__).resolve().parents[3]
    candidates.extend(
        [
            repo_root / 'src' / package_name / 'package.xml',
            Path.cwd() / 'src' / package_name / 'package.xml',
        ]
    )

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


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
