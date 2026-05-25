"""ROS action server surface for fake skill execution."""

from __future__ import annotations

import json
import time

from nao_skills.action import ScanScene
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
try:  # pragma: no cover - runtime dependency
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - unit-test fallback
    get_package_share_directory = None

from fake_skills.contracts import FakeSkillRequest
from fake_skills.engine import FakeSkillEngine
from fake_skills.scenario_store import ScenarioStore


class FakeSkillActionServer(Node):
    """Expose deterministic fake skill endpoints for planner validation."""

    def __init__(self) -> None:
        super().__init__('fake_skill_server')

        self.declare_parameter('execute_action_name', '/skill/fake/execute')
        self.declare_parameter('navigate_to_action_name', '/skill/fake/navigate_to')
        self.declare_parameter('find_object_action_name', '/skill/fake/find_object')
        self.declare_parameter('wave_greet_action_name', '/skill/fake/wave_greet')
        self.declare_parameter('inspect_area_action_name', '/skill/fake/inspect_area')
        self.declare_parameter('walk_to_action_name', '/skill/fake/walk_to')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('default_delay_sec', 0.75)
        self.declare_parameter('deterministic_seed', 42)
        self.declare_parameter('publish_events', True)
        self.declare_parameter('event_topic', '/fake_skills/events')
        self.declare_parameter('active_scenario_id', '')

        scenario_path = str(self.get_parameter('scenario_file').value).strip()
        if not scenario_path and get_package_share_directory is not None:
            try:
                scenario_path = (
                    get_package_share_directory('fake_skills')
                    + '/config/fake_skill_scenarios.yaml'
                )
            except Exception:
                scenario_path = ''
        if scenario_path:
            self._scenario_store = ScenarioStore.load_file(scenario_path)
        else:
            self._scenario_store = ScenarioStore({})

        self._engine = FakeSkillEngine(
            scenario_store=self._scenario_store,
            default_delay_sec=float(self.get_parameter('default_delay_sec').value),
            deterministic_seed=int(self.get_parameter('deterministic_seed').value),
        )
        self._active_scenario_id = str(self.get_parameter('active_scenario_id').value).strip()
        self.declare_parameter('available_scenario_ids', list(self._scenario_store.scenario_ids()))
        self._set_parameters_callback = self.add_on_set_parameters_callback(self._on_set_parameters)
        self._active_scenario_id = self._normalize_active_scenario_id(self._active_scenario_id)

        self._publish_events = bool(self.get_parameter('publish_events').value)
        event_topic = str(self.get_parameter('event_topic').value).strip() or '/fake_skills/events'
        self._event_pub = self.create_publisher(String, event_topic, 10)

        callback_group = ReentrantCallbackGroup()
        self._servers = []
        self._servers.append(
            ActionServer(
                self,
                ScanScene,
                str(self.get_parameter('execute_action_name').value).strip(),
                execute_callback=self._execute_generic,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=callback_group,
            )
        )
        for action_param, skill_name in (
            ('navigate_to_action_name', 'navigate_to'),
            ('find_object_action_name', 'find_object'),
            ('wave_greet_action_name', 'wave_greet'),
            ('inspect_area_action_name', 'inspect_area'),
            ('walk_to_action_name', 'walk_to'),
        ):
            action_name = str(self.get_parameter(action_param).value).strip()
            self._servers.append(
                ActionServer(
                    self,
                    ScanScene,
                    action_name,
                    execute_callback=lambda gh, skill=skill_name: self._execute_fixed(gh, skill),
                    goal_callback=self._goal_callback,
                    cancel_callback=self._cancel_callback,
                    callback_group=callback_group,
                )
            )

        self.get_logger().info(
            'fake_skill_server ready | execute=%s skills=%s active_scenario=%s available_scenarios=%s'
            % (
                str(self.get_parameter('execute_action_name').value).strip(),
                ','.join(self._engine.supported_skills),
                (self._active_scenario_id or '<none>'),
                ','.join(self._scenario_store.scenario_ids()) or '<none>',
            )
        )

    def _goal_callback(self, _goal_request: ScanScene.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_generic(self, goal_handle):
        request = self._request_from_goal(goal_handle.request, fixed_skill='')
        return self._execute_request(goal_handle, request)

    def _execute_fixed(self, goal_handle, skill: str):
        request = self._request_from_goal(goal_handle.request, fixed_skill=skill)
        return self._execute_request(goal_handle, request)

    def _execute_request(self, goal_handle, request: FakeSkillRequest):
        started = time.time()
        self._emit_event('fake_skill_started', request.skill, {'args': request.args, 'scenario_id': request.scenario_id})
        goal_handle.publish_feedback(self._feedback('preparing', 0.1))

        payload, delay_sec = self._engine.execute(
            skill=request.skill,
            args=request.args,
            scenario_id=request.scenario_id,
            scenario_override=request.scenario_override,
        )

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(
                ok=False,
                message='fake skill request canceled',
                payload=payload,
                duration=time.time() - started,
            )

        if delay_sec > 0.0:
            goal_handle.publish_feedback(self._feedback('executing', 0.6))
            time.sleep(delay_sec)

        status = str(payload.get('status', '')).strip().lower()
        success = status == 'succeeded'
        summary_text = str(payload.get('summary_text', '')).strip()

        goal_handle.publish_feedback(self._feedback('completing', 1.0))
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self._emit_event(
            'fake_skill_completed',
            request.skill,
            {
                'status': status,
                'result_mode': payload.get('metadata', {}).get('result_mode', ''),
                'summary_text': summary_text,
                'failure': payload.get('failure', {}),
            },
        )

        return self._result(
            ok=success,
            message=summary_text or ('%s completed' % request.skill),
            payload=payload,
            duration=time.time() - started,
        )

    def _request_from_goal(self, goal: ScanScene.Goal, *, fixed_skill: str) -> FakeSkillRequest:
        payload = self._parse_evidence_policy(goal.evidence_policy)

        skill = str(fixed_skill or payload.get('skill', '')).strip().lower()
        args = {
            'target': str(goal.target or '').strip(),
            'target_kind': str(goal.target_kind or '').strip(),
            'max_sweeps': int(goal.max_sweeps),
        }
        result_mode = str(goal.result_mode or '').strip().lower()
        if result_mode:
            args['result_mode'] = result_mode

        if isinstance(payload.get('args', {}), dict):
            args.update(payload.get('args', {}))

        passthrough = payload.get('params', {})
        if isinstance(passthrough, dict):
            args.update(passthrough)

        for key, value in payload.items():
            if key in ('skill', 'args', 'scenario', 'scenario_id', 'params'):
                continue
            if key not in args:
                args[key] = value

        scenario_override = payload.get('scenario', {})
        if not isinstance(scenario_override, dict):
            scenario_override = {}

        scenario_id = str(payload.get('scenario_id', '')).strip()
        if not scenario_id:
            scenario_id = self._active_scenario_id

        return FakeSkillRequest(
            skill=skill,
            args=args,
            scenario_id=scenario_id,
            scenario_override=scenario_override,
        )

    def _normalize_active_scenario_id(self, value: str) -> str:
        clean = str(value or '').strip()
        if not clean:
            return ''
        if self._scenario_store.has_scenario(clean):
            return clean
        self.get_logger().warn(
            'Unknown active_scenario_id "%s"; using defaults (no named scenario).' % clean
        )
        return ''

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != 'active_scenario_id':
                continue
            requested = str(parameter.value or '').strip()
            if requested and not self._scenario_store.has_scenario(requested):
                available = ','.join(self._scenario_store.scenario_ids()) or '<none>'
                return SetParametersResult(
                    successful=False,
                    reason=(
                        'Unknown active_scenario_id "%s". Available: %s'
                        % (requested, available)
                    ),
                )
            self._active_scenario_id = requested
            self.get_logger().info(
                'active_scenario_id updated to %s'
                % (self._active_scenario_id or '<none>')
            )
        return SetParametersResult(successful=True)

    @staticmethod
    def _parse_evidence_policy(value: str) -> dict:
        clean = str(value or '').strip()
        if not clean:
            return {}
        try:
            parsed = json.loads(clean)
        except json.JSONDecodeError:
            return {}
        return parsed if isinstance(parsed, dict) else {}

    @staticmethod
    def _feedback(status: str, progress: float) -> ScanScene.Feedback:
        feedback = ScanScene.Feedback()
        feedback.status = str(status)
        feedback.progress = float(progress)
        return feedback

    @staticmethod
    def _result(*, ok: bool, message: str, payload: dict, duration: float) -> ScanScene.Result:
        result = ScanScene.Result()
        result.success = bool(ok)
        result.message = str(message or '').strip()
        result.summary_text = str(payload.get('summary_text', result.message)).strip()
        result.result_payload_json = json.dumps(payload, sort_keys=True, separators=(',', ':'))
        result.duration = float(max(0.0, duration))
        return result

    def _emit_event(self, event_type: str, skill: str, payload: dict) -> None:
        if not self._publish_events:
            return
        msg = String()
        msg.data = json.dumps(
            {
                'event_type': str(event_type),
                'skill': str(skill),
                'payload': dict(payload or {}),
            },
            sort_keys=True,
            separators=(',', ':'),
        )
        self._event_pub.publish(msg)
