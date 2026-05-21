"""Scenario loading and merge helpers for fake skills."""

from __future__ import annotations

from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - runtime dependency
    yaml = None


class ScenarioStore:
    """Load deterministic default/scenario overrides from YAML."""

    def __init__(self, payload: dict | None = None) -> None:
        data = dict(payload or {})
        self._default = data.get('default', {}) if isinstance(data.get('default', {}), dict) else {}
        self._scenarios = data.get('scenarios', {}) if isinstance(data.get('scenarios', {}), dict) else {}

    @classmethod
    def load_file(cls, path: str | Path) -> 'ScenarioStore':
        scenario_path = Path(path)
        if not scenario_path.exists():
            return cls({})
        text = scenario_path.read_text(encoding='utf-8')
        if not text.strip():
            return cls({})
        if yaml is None:
            raise RuntimeError('python3-yaml is required for fake_skills scenario loading')
        payload = yaml.safe_load(text)
        if not isinstance(payload, dict):
            payload = {}
        return cls(payload)

    def resolve_skill_config(
        self,
        *,
        skill: str,
        scenario_id: str = '',
        scenario_override: dict | None = None,
    ) -> dict:
        """Return merged default + named scenario + request override config."""
        clean_skill = str(skill or '').strip()
        merged: dict = {}

        default_item = self._default.get(clean_skill, {})
        if isinstance(default_item, dict):
            merged.update(default_item)

        clean_scenario = str(scenario_id or '').strip()
        if clean_scenario:
            scenario_item = self._scenarios.get(clean_scenario, {})
            if isinstance(scenario_item, dict):
                scoped_item = scenario_item.get(clean_skill, {})
                if isinstance(scoped_item, dict):
                    merged.update(scoped_item)

        if isinstance(scenario_override, dict):
            merged.update(scenario_override)

        return merged
