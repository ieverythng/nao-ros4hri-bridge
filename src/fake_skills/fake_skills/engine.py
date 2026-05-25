"""Deterministic execution engine for fake skills."""

from __future__ import annotations

import hashlib

from fake_skills.scenario_store import ScenarioStore
from fake_skills.skills import SKILL_EXECUTORS


class FakeSkillEngine:
    """Run fake skills with scenario-controlled outcomes."""

    def __init__(
        self,
        *,
        scenario_store: ScenarioStore,
        default_delay_sec: float = 0.75,
        deterministic_seed: int = 42,
    ) -> None:
        self._scenario_store = scenario_store
        self._default_delay_sec = max(0.0, float(default_delay_sec))
        self._deterministic_seed = int(deterministic_seed)
        self._call_counters: dict[str, int] = {}

    @property
    def supported_skills(self) -> tuple[str, ...]:
        return tuple(sorted(SKILL_EXECUTORS.keys()))

    def execute(
        self,
        *,
        skill: str,
        args: dict,
        scenario_id: str = '',
        scenario_override: dict | None = None,
    ) -> tuple[dict, float]:
        """Execute one fake skill and return payload + simulated delay."""
        clean_skill = str(skill or '').strip().lower()
        executor = SKILL_EXECUTORS.get(clean_skill)
        if executor is None:
            return self._unsupported_skill_result(clean_skill), self._default_delay_sec

        merged_config = self._scenario_store.resolve_skill_config(
            skill=clean_skill,
            scenario_id=scenario_id,
            scenario_override=scenario_override,
        )

        merged_args = dict(merged_config)
        merged_args.update(dict(args or {}))
        result_mode = str(merged_args.get('result_mode', 'success')).strip().lower() or 'success'
        delay_sec = self._resolve_delay(merged_args)

        call_key = self._call_key(skill=clean_skill, args=merged_args, mode=result_mode)
        call_count = self._call_counters.get(call_key, 0)
        self._call_counters[call_key] = call_count + 1

        metadata = {
            'ab_object_id': clean_skill,
            'fake': True,
            'result_mode': result_mode,
            'scenario_id': str(scenario_id or '').strip(),
            'duration_sec': delay_sec,
            'deterministic_seed': self._deterministic_seed,
        }

        payload = executor(
            args=merged_args,
            mode=result_mode,
            metadata=metadata,
            fail_once_active=call_count > 0,
        )
        payload['metadata'] = dict(metadata, **dict(payload.get('metadata', {})))

        return payload, delay_sec

    def _resolve_delay(self, merged_args: dict) -> float:
        if 'delay_sec' in merged_args:
            try:
                return max(0.0, float(merged_args.get('delay_sec', self._default_delay_sec)))
            except (TypeError, ValueError):
                return self._default_delay_sec
        return self._default_delay_sec

    def _call_key(self, *, skill: str, args: dict, mode: str) -> str:
        signature = {
            'skill': skill,
            'mode': mode,
            'target': args.get('target', args.get('location', '')),
            'target_kind': args.get('target_kind', ''),
        }
        text = repr(signature).encode('utf-8')
        digest = hashlib.sha1(text).hexdigest()[:16]
        return '%s:%s' % (self._deterministic_seed, digest)

    @staticmethod
    def _unsupported_skill_result(skill: str) -> dict:
        return {
            'skill': skill,
            'status': 'failed',
            'target': '',
            'target_kind': '',
            'target_found': False,
            'summary_text': 'Unknown fake skill: %s' % (skill or '<empty>'),
            'evidence': {},
            'failure': {
                'code': 'unsupported_skill',
                'message': 'No fake executor exists for %s.' % (skill or '<empty>'),
                'recoverable': False,
                'suggested_recovery': 'replan',
            },
            'metadata': {
                'ab_object_id': skill,
                'fake': True,
                'result_mode': 'unsupported_skill',
            },
        }
