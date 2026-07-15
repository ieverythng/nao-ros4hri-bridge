"""Deterministic execution engine for fake skills."""

from __future__ import annotations

import hashlib

from fake_skills.scenario_store import ScenarioStore
from fake_skills.skills import SKILL_EXECUTORS

GLOBAL_MODES = {
    'scenario',
    'always_success',
    'always_fail',
    'every_other',
    'random_seeded',
}

SUCCESS_MODE_BY_SKILL = {
    'navigate_to': 'success',
    'find_object': 'found',
    'perform_motion': 'success',
    'wave_greet': 'success',
    'inspect_area': 'clear',
    'look_at': 'success',
    'walk_to': 'success',
    'pick_object': 'success',
    'place_object': 'success',
    'bring_object': 'success',
}

FAILURE_MODE_BY_SKILL = {
    'navigate_to': 'path_blocked',
    'find_object': 'not_found',
    'perform_motion': 'motion_unavailable',
    'wave_greet': 'motion_unavailable',
    'inspect_area': 'backend_unavailable',
    'look_at': 'target_unavailable',
    'walk_to': 'path_blocked',
    'pick_object': 'object_unavailable',
    'place_object': 'no_held_object',
    'bring_object': 'acquisition_failure',
}

_POSTURE_MOTIONS = {'stand', 'standinit', 'sit', 'kneel', 'crouch'}


class FakeSkillEngine:
    """Run fake skills with scenario-controlled outcomes."""

    def __init__(
        self,
        *,
        scenario_store: ScenarioStore,
        default_delay_sec: float = 0.75,
        deterministic_seed: int = 42,
        global_mode: str = 'scenario',
        random_failure_prob: float = 0.5,
        mode_overrides: dict[str, str] | None = None,
    ) -> None:
        self._scenario_store = scenario_store
        self._default_delay_sec = max(0.0, float(default_delay_sec))
        self._deterministic_seed = int(deterministic_seed)
        self._call_counters: dict[str, int] = {}
        self._posture_state = 'unknown'
        self.update_policy(
            global_mode=global_mode,
            random_failure_prob=random_failure_prob,
            mode_overrides=mode_overrides,
        )

    @property
    def supported_skills(self) -> tuple[str, ...]:
        return tuple(sorted(SKILL_EXECUTORS.keys()))

    @property
    def posture_state(self) -> str:
        """Return the last successfully simulated body posture."""
        return self._posture_state

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

        request_args = dict(args or {})
        request_scenario_override = (
            dict(scenario_override or {})
            if isinstance(scenario_override, dict)
            else {}
        )

        merged_config = self._scenario_store.resolve_skill_config(
            skill=clean_skill,
            scenario_id=scenario_id,
            scenario_override=request_scenario_override,
        )

        merged_args = dict(merged_config)
        merged_args.update(request_args)
        posture_motion = self._posture_motion(clean_skill, merged_args)
        if posture_motion:
            merged_args['previous_posture_state'] = self._posture_state

        policy_fail_once = (
            not _normalize_mode_value(request_scenario_override.get('result_mode', ''))
            and not _normalize_mode_value(request_args.get('result_mode', ''))
            and _normalize_mode_value(self._mode_overrides.get(clean_skill, ''))
            == 'fail_once'
        )
        call_key = self._call_key(
            skill=clean_skill,
            args={} if policy_fail_once else merged_args,
            mode='policy',
        )
        call_count = self._call_counters.get(call_key, 0)
        self._call_counters[call_key] = call_count + 1

        result_mode, mode_source = self._resolve_result_mode(
            skill=clean_skill,
            merged_config=merged_config,
            request_args=request_args,
            request_scenario_override=request_scenario_override,
            call_count=call_count,
        )
        merged_args['result_mode'] = result_mode
        delay_sec = self._resolve_delay(merged_args)

        metadata = {
            'ab_object_id': clean_skill,
            'fake': True,
            'result_mode': result_mode,
            'mode_source': mode_source,
            'global_mode': self._global_mode,
            'random_failure_prob': self._random_failure_prob,
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
        if posture_motion and payload.get('status') == 'succeeded':
            self._posture_state = posture_motion
        payload['metadata'] = dict(metadata, **dict(payload.get('metadata', {})))

        return payload, delay_sec

    @staticmethod
    def _posture_motion(skill: str, args: dict) -> str:
        if skill != 'perform_motion':
            return ''
        motion = str(
            args.get('object', args.get('motion', args.get('target', '')))
        ).strip().lower()
        return motion if motion in _POSTURE_MOTIONS else ''

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
            'target': args.get(
                'object_id',
                args.get('target', args.get('object', args.get('location', ''))),
            ),
            'target_kind': args.get('target_kind', ''),
        }
        text = repr(signature).encode('utf-8')
        digest = hashlib.sha1(text).hexdigest()[:16]
        return '%s:%s' % (self._deterministic_seed, digest)

    def update_policy(
        self,
        *,
        global_mode: str,
        random_failure_prob: float,
        mode_overrides: dict[str, str] | None,
    ) -> None:
        next_global_mode = self._normalize_global_mode(global_mode)
        next_random_failure_prob = self._coerce_probability(random_failure_prob)
        next_mode_overrides = self._normalize_mode_overrides(mode_overrides)
        previous_policy = (
            getattr(self, '_global_mode', None),
            getattr(self, '_random_failure_prob', None),
            getattr(self, '_mode_overrides', None),
        )
        next_policy = (
            next_global_mode,
            next_random_failure_prob,
            next_mode_overrides,
        )
        if previous_policy != next_policy:
            self._call_counters.clear()
        self._global_mode = next_global_mode
        self._random_failure_prob = next_random_failure_prob
        self._mode_overrides = next_mode_overrides

    def _resolve_result_mode(
        self,
        *,
        skill: str,
        merged_config: dict,
        request_args: dict,
        request_scenario_override: dict,
        call_count: int,
    ) -> tuple[str, str]:
        scenario_override_mode = _normalize_mode_value(request_scenario_override.get('result_mode', ''))
        if scenario_override_mode:
            return scenario_override_mode, 'scenario_override'

        request_mode = _normalize_mode_value(request_args.get('result_mode', ''))
        if request_mode:
            return request_mode, 'request_args'

        scenario_mode = _normalize_mode_value(merged_config.get('result_mode', ''))
        if not scenario_mode:
            scenario_mode = _default_success_mode(skill)

        skill_override_mode = _normalize_mode_value(self._mode_overrides.get(skill, ''))
        if skill_override_mode:
            return (
                self._apply_policy_mode(
                    skill=skill,
                    policy_mode=skill_override_mode,
                    call_count=call_count,
                    fallback_mode=scenario_mode,
                ),
                'skill_override',
            )

        if self._global_mode != 'scenario':
            return (
                self._apply_policy_mode(
                    skill=skill,
                    policy_mode=self._global_mode,
                    call_count=call_count,
                    fallback_mode=scenario_mode,
                ),
                'global_mode',
            )

        return scenario_mode, 'scenario_default'

    def _apply_policy_mode(
        self,
        *,
        skill: str,
        policy_mode: str,
        call_count: int,
        fallback_mode: str,
    ) -> str:
        clean_mode = _normalize_mode_value(policy_mode)
        if not clean_mode:
            return fallback_mode
        if clean_mode == 'scenario':
            return fallback_mode
        if clean_mode == 'always_success':
            return _default_success_mode(skill)
        if clean_mode == 'always_fail':
            return _default_failure_mode(skill)
        if clean_mode == 'every_other':
            if call_count % 2 == 0:
                return _default_success_mode(skill)
            return _default_failure_mode(skill)
        if clean_mode == 'random_seeded':
            ratio = self._deterministic_ratio(
                skill=skill,
                call_count=call_count,
                policy_mode=clean_mode,
            )
            if ratio < self._random_failure_prob:
                return _default_failure_mode(skill)
            return _default_success_mode(skill)
        return clean_mode

    def _deterministic_ratio(self, *, skill: str, call_count: int, policy_mode: str) -> float:
        token = '%s|%s|%s|%d' % (
            self._deterministic_seed,
            skill,
            policy_mode,
            call_count,
        )
        digest = hashlib.sha1(token.encode('utf-8')).hexdigest()[:8]
        raw = int(digest, 16)
        return raw / 0xFFFFFFFF

    @staticmethod
    def _normalize_global_mode(value: str) -> str:
        clean = _normalize_mode_value(value)
        if clean in GLOBAL_MODES:
            return clean
        return 'scenario'

    @staticmethod
    def _coerce_probability(value) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return 0.5
        if parsed < 0.0:
            return 0.0
        if parsed > 1.0:
            return 1.0
        return parsed

    @staticmethod
    def _normalize_mode_overrides(value: dict[str, str] | None) -> dict[str, str]:
        if not isinstance(value, dict):
            return {}
        normalized: dict[str, str] = {}
        for key, raw_mode in value.items():
            clean_skill = str(key or '').strip().lower()
            clean_mode = _normalize_mode_value(raw_mode)
            if not clean_skill or not clean_mode:
                continue
            normalized[clean_skill] = clean_mode
        return normalized

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


def _normalize_mode_value(value) -> str:
    return str(value or '').strip().lower()


def _default_success_mode(skill: str) -> str:
    clean_skill = str(skill or '').strip().lower()
    return SUCCESS_MODE_BY_SKILL.get(clean_skill, 'success')


def _default_failure_mode(skill: str) -> str:
    clean_skill = str(skill or '').strip().lower()
    return FAILURE_MODE_BY_SKILL.get(clean_skill, 'always_fail')
