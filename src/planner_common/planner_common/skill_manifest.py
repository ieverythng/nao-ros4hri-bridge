"""Shared package.xml skill-manifest loading helpers."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import re
import textwrap
import warnings
import xml.etree.ElementTree as ET

try:  # pragma: no cover - runtime dependency
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - import-light unit tests
    class PackageNotFoundError(Exception):
        """Fallback error when ament_index_python is unavailable."""

    def get_package_share_directory(_package_name: str) -> str:
        raise PackageNotFoundError('ament_index_python is unavailable')

try:  # pragma: no cover - optional dependency
    import yaml
except ImportError:  # pragma: no cover - import-light unit tests
    yaml = None


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
    content_type: str


def load_exported_skill_manifests(
    package_names,
    *,
    logger=None,
) -> tuple[ExportedSkillManifest, ...]:
    """Load exported skill manifests from one ordered package list."""
    manifests: list[ExportedSkillManifest] = []
    for package_name in tuple(package_names or ()):
        package_xml = resolve_package_xml(str(package_name).strip())
        if package_xml is None:
            _warn(logger, f'Skill manifest package not found: {package_name}')
            continue
        manifests.extend(
            _package_skill_manifests(
                package_name=str(package_name).strip(),
                package_xml=package_xml,
                logger=logger,
            )
        )
    return tuple(manifests)


def resolve_package_xml(package_name: str) -> Path | None:
    """Resolve one package.xml from install-space or source-tree layouts."""
    clean_name = str(package_name or '').strip()
    if not clean_name:
        return None

    candidates: list[Path] = []
    try:
        share_dir = Path(get_package_share_directory(clean_name))
        candidates.append(share_dir / 'package.xml')
    except PackageNotFoundError:
        pass

    repo_root = Path(__file__).resolve().parents[3]
    candidates.extend(
        [
            repo_root / 'src' / clean_name / 'package.xml',
            Path.cwd() / 'src' / clean_name / 'package.xml',
        ]
    )

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def _package_skill_manifests(
    package_name: str,
    package_xml: Path,
    *,
    logger=None,
) -> list[ExportedSkillManifest]:
    try:
        root = ET.parse(package_xml).getroot()
    except Exception as err:
        _warn(logger, f'Could not parse {package_xml}: {err}')
        return []

    manifests: list[ExportedSkillManifest] = []
    for skill_elem in root.findall('.//skill'):
        content_type = str(skill_elem.attrib.get('content-type', 'yaml')).strip().lower() or 'yaml'
        if content_type not in ('yaml', 'json'):
            continue

        manifest_text = textwrap.dedent(skill_elem.text or '').strip()
        if not manifest_text:
            continue

        payload = _parse_skill_manifest_text(
            manifest_text,
            content_type=content_type,
            logger=logger,
        )
        if not payload:
            continue

        skill_id = str(payload.get('id', '')).strip().lower()
        if not skill_id:
            continue

        manifests.append(
            ExportedSkillManifest(
                package=package_name,
                skill_id=skill_id,
                interface_path=str(payload.get('default_interface_path', '')).strip(),
                datatype=str(payload.get('datatype', '')).strip(),
                description=_normalize_spaces(str(payload.get('description', '')).strip()),
                input_names=_parameter_names(payload, 'in'),
                output_names=_parameter_names(payload, 'out'),
                feedback_names=_parameter_names(payload, 'feedback'),
                functional_domains=_coerce_tuple(payload.get('functional_domains', [])),
                content_type=content_type,
            )
        )
    return manifests


def _parse_skill_manifest_text(payload: str, *, content_type: str, logger=None) -> dict:
    if content_type == 'json':
        return _parse_json_manifest(payload, logger=logger)
    return _parse_yaml_manifest(payload, logger=logger)


def _parse_json_manifest(payload: str, *, logger=None) -> dict:
    try:
        parsed = json.loads(payload)
    except Exception as err:
        _warn(logger, f'Skill manifest JSON parse failed: {err}')
        return {}
    return dict(parsed) if isinstance(parsed, dict) else {}


def _parse_yaml_manifest(payload: str, *, logger=None) -> dict:
    if yaml is not None:
        try:
            parsed = yaml.safe_load(payload)
        except Exception as err:
            _warn(logger, f'Skill manifest YAML parse failed: {err}')
            return {}
        return dict(parsed) if isinstance(parsed, dict) else {}
    return _fallback_parse_yaml_manifest(payload)


def _fallback_parse_yaml_manifest(payload: str) -> dict:
    manifest: dict = {}

    for key in ('id', 'default_interface_path', 'datatype'):
        value = _extract_yaml_scalar(payload, key)
        if value:
            manifest[key] = value

    description = _extract_yaml_block(payload, 'description')
    if description:
        manifest['description'] = description

    functional_domains = _extract_yaml_list(payload, 'functional_domains')
    if functional_domains:
        manifest['functional_domains'] = functional_domains

    parameters = {}
    for section_name in ('in', 'out', 'feedback'):
        names = _extract_parameter_names(payload, section_name)
        if names:
            parameters[section_name] = [{'name': name} for name in names]
    if parameters:
        manifest['parameters'] = parameters

    return manifest


def _extract_yaml_scalar(payload: str, key: str) -> str:
    match = re.search(
        r'^\s*%s:\s*(.+?)\s*$' % re.escape(key),
        payload,
        flags=re.MULTILINE,
    )
    if not match:
        return ''
    value = match.group(1).strip()
    if value == '|':
        return ''
    return value.strip('"\'')


def _extract_yaml_block(payload: str, key: str) -> str:
    lines = payload.splitlines()
    capture = False
    block_indent = 0
    collected: list[str] = []
    block_header = f'{key}: |'

    for line in lines:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if not capture:
            if stripped == block_header:
                capture = True
                block_indent = indent
            continue
        if stripped and indent <= block_indent:
            break
        collected.append(line[block_indent + 2 :] if len(line) > block_indent + 2 else '')

    return _normalize_spaces('\n'.join(collected).strip())


def _extract_yaml_list(payload: str, key: str) -> list[str]:
    lines = payload.splitlines()
    capture = False
    key_indent = 0
    values: list[str] = []

    for line in lines:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if not capture:
            if stripped == f'{key}:':
                capture = True
                key_indent = indent
            continue
        if stripped and indent <= key_indent:
            break
        item_match = re.match(r'^\s*-\s*(.+?)\s*$', line)
        if item_match:
            values.append(item_match.group(1).strip())
    return values


def _extract_parameter_names(payload: str, section_name: str) -> list[str]:
    lines = payload.splitlines()
    inside_parameters = False
    parameters_indent = 0
    inside_section = False
    section_indent = 0
    names: list[str] = []

    for line in lines:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())

        if not inside_parameters:
            if stripped == 'parameters:':
                inside_parameters = True
                parameters_indent = indent
            continue

        if stripped and indent <= parameters_indent and not inside_section:
            break

        if not inside_section:
            if stripped == f'{section_name}:':
                inside_section = True
                section_indent = indent
            continue

        if stripped and indent <= section_indent:
            break

        name_match = re.match(r'^\s*-\s*name:\s*(.+?)\s*$', line)
        if name_match:
            names.append(name_match.group(1).strip())

    return names


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


def _coerce_tuple(value) -> tuple[str, ...]:
    if isinstance(value, str):
        clean_value = value.strip()
        return (clean_value,) if clean_value else ()
    if not isinstance(value, (list, tuple)):
        return ()
    return tuple(clean for clean in (str(item).strip() for item in value) if clean)


def _normalize_spaces(value: str) -> str:
    return re.sub(r'\s+', ' ', str(value or '')).strip()


def _warn(logger, message: str) -> None:
    if logger is not None:
        logger.warn(message)
        return
    warnings.warn(message, stacklevel=2)
