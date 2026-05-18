"""Shared skill registry bridge helpers for planner and orchestrator nodes."""

from __future__ import annotations

from collections.abc import Iterable

try:  # pragma: no cover - optional nested dependency
    from skill_common import load_default_registry as _load_default_registry
    from skill_common import load_registry_file as _load_registry_file
except ImportError:  # pragma: no cover - keep deterministic fallbacks
    _load_default_registry = None
    _load_registry_file = None


def load_shared_skill_manifest(path: str = '') -> list[dict]:
    """Load normalized skill records from `skill_common` if available."""
    if _load_default_registry is None:
        return []

    clean_path = str(path or '').strip()
    try:
        registry = _load_registry_file(clean_path) if clean_path else _load_default_registry()
    except Exception:
        return []

    payload = getattr(registry, 'prompt_manifest', lambda: [])()
    return [dict(item) for item in payload if isinstance(item, dict)]


def names_from_manifest(skill_payload: dict) -> set[str]:
    """Extract canonical+alias lowercase names from one skill payload."""
    names = {str(skill_payload.get('name', '')).strip().lower()}
    names.update(
        str(alias).strip().lower()
        for alias in skill_payload.get('aliases', [])
        if str(alias).strip()
    )
    names.discard('')
    return names


def merge_supported_skill_names(
    *,
    fallback_names: Iterable[str],
    manifest: Iterable[dict],
) -> set[str]:
    """Merge fallback names with `skill_common` manifest-derived names."""
    names = {
        str(item).strip().lower()
        for item in fallback_names
        if str(item).strip()
    }
    for skill_payload in manifest:
        names.update(names_from_manifest(skill_payload))
    return names


def merge_scan_skill_names(
    *,
    fallback_names: Iterable[str],
    manifest: Iterable[dict],
) -> set[str]:
    """Merge fallback scan aliases with registry entries mapped to scan."""
    scan_names = {
        str(item).strip().lower()
        for item in fallback_names
        if str(item).strip()
    }
    for skill_payload in manifest:
        mapping = str(skill_payload.get('robot_adapter_mapping', '')).strip().lower()
        canonical = str(skill_payload.get('name', '')).strip().lower()
        if canonical == 'scan' or mapping == 'nao_orchestrator.scan':
            scan_names.update(names_from_manifest(skill_payload))
    return scan_names


def merge_fake_skill_aliases(
    *,
    fallback_aliases: dict[str, str],
    manifest: Iterable[dict],
) -> dict[str, str]:
    """Merge fake skill aliases with registry-provided aliases."""
    aliases = dict(fallback_aliases)
    for skill_payload in manifest:
        mapping = str(skill_payload.get('robot_adapter_mapping', '')).strip().lower()
        if not mapping.startswith('fake_skills.'):
            continue
        canonical = mapping.split('.', 1)[1].strip().lower()
        if not canonical:
            continue
        for name in names_from_manifest(skill_payload):
            aliases[name] = canonical
    return aliases
