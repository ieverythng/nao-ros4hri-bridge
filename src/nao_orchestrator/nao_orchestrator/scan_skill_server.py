#!/usr/bin/env python3
"""Lifecycle scan skill action server.

This server owns scan internals (head sweep + evidence shaping) so the
orchestrator can stay a deterministic dispatcher.
"""

from __future__ import annotations

import json
import threading
import time

from nao_skills.action import DoHeadMotion, ScanScene
from planner_common.contracts import coerce_optional_float
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String

from nao_orchestrator.intent_rules import (
    build_scan_result_payload,
    is_people_scan_target,
    resolve_scan_result,
    summarize_people_detection,
)

try:  # pragma: no cover - runtime dependency
    from hri_msgs.msg import IdsList
except ImportError:  # pragma: no cover - runtime dependency
    IdsList = None


class ScanSkillServer(Node):
    """Implement `/skill/scan` as a composite action server."""

    def __init__(self) -> None:
        super().__init__('scan_skill_server')

        self.declare_parameter('action_name', '/skill/scan')
        self.declare_parameter('head_motion_action', '/skill/do_head_motion')
        self.declare_parameter('head_motion_speed', 0.25)
        self.declare_parameter('head_motion_wait_sec', 1.0)
        self.declare_parameter('head_motion_result_timeout_sec', 6.0)
        self.declare_parameter('scan_result_mode', 'success')
        self.declare_parameter('scan_summary', '')
        self.declare_parameter('scan_summary_topic', '/scene/summary')
        self.declare_parameter('scan_people_topic', '/humans/persons/tracked')
        self.declare_parameter('scan_people_max_age_sec', 2.0)
        self.declare_parameter('scan_min_sweeps', 1)
        self.declare_parameter('scan_max_sweeps', 2)
        self.declare_parameter('scan_sweep_yaw_rad', 0.45)
        self.declare_parameter('scan_sweep_pitch_rad', 0.0)
        self.declare_parameter('scan_sweep_settle_sec', 0.2)
        self.declare_parameter('scan_require_motion', True)

        self.action_name = str(self.get_parameter('action_name').value).strip() or '/skill/scan'
        self.head_motion_action = str(self.get_parameter('head_motion_action').value).strip()
        self.head_motion_speed = float(self.get_parameter('head_motion_speed').value)
        self.head_motion_wait_sec = max(0.0, float(self.get_parameter('head_motion_wait_sec').value))
        self.head_motion_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('head_motion_result_timeout_sec').value),
        )
        self.scan_result_mode = str(self.get_parameter('scan_result_mode').value).strip().lower()
        self.scan_summary = str(self.get_parameter('scan_summary').value).strip()
        self.scan_summary_topic = str(self.get_parameter('scan_summary_topic').value).strip()
        self.scan_people_topic = str(self.get_parameter('scan_people_topic').value).strip()
        self.scan_people_max_age_sec = max(
            0.0,
            float(self.get_parameter('scan_people_max_age_sec').value),
        )
        self.scan_min_sweeps = max(1, int(self.get_parameter('scan_min_sweeps').value))
        self.scan_max_sweeps = max(
            self.scan_min_sweeps,
            int(self.get_parameter('scan_max_sweeps').value),
        )
        self.scan_sweep_yaw_rad = max(
            0.0,
            float(self.get_parameter('scan_sweep_yaw_rad').value),
        )
        self.scan_sweep_pitch_rad = float(self.get_parameter('scan_sweep_pitch_rad').value)
        self.scan_sweep_settle_sec = max(
            0.0,
            float(self.get_parameter('scan_sweep_settle_sec').value),
        )
        self.scan_require_motion = bool(self.get_parameter('scan_require_motion').value)

        self._callback_group = ReentrantCallbackGroup()
        self._execution_lock = threading.Lock()
        self._is_active = False

        self._action_server = None
        self._head_motion_client = None
        self._scan_summary_sub = None
        self._scan_people_sub = None
        self._latest_scan_summary = ''
        self._latest_scene_objects: tuple[dict, ...] = ()
        self._latest_tracked_person_ids: tuple[str, ...] = ()
        self._latest_tracked_people_ts = 0.0

        self.get_logger().info('scan_skill_server created; waiting for lifecycle configure')

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        self._teardown_runtime_interfaces()
        self._head_motion_client = ActionClient(
            self,
            DoHeadMotion,
            self.head_motion_action,
        )
        self._action_server = ActionServer(
            self,
            ScanScene,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        if self.scan_summary_topic:
            self._scan_summary_sub = self.create_subscription(
                String,
                self.scan_summary_topic,
                self._on_scan_summary,
                10,
            )
        if self.scan_people_topic and IdsList is not None:
            self._scan_people_sub = self.create_subscription(
                IdsList,
                self.scan_people_topic,
                self._on_scan_people,
                10,
            )
        elif self.scan_people_topic:
            self.get_logger().warn('IdsList unavailable; people-aware scan evidence is disabled')
        self.get_logger().info(
            'scan_skill_server configured | action:%s head:%s'
            % (self.action_name, self.head_motion_action)
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return super().on_deactivate(state)

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self._teardown_runtime_interfaces()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self._teardown_runtime_interfaces()
        return TransitionCallbackReturn.SUCCESS

    def _teardown_runtime_interfaces(self) -> None:
        if self._scan_summary_sub is not None:
            self.destroy_subscription(self._scan_summary_sub)
            self._scan_summary_sub = None
        if self._scan_people_sub is not None:
            self.destroy_subscription(self._scan_people_sub)
            self._scan_people_sub = None
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._head_motion_client is not None:
            self._head_motion_client.destroy()
            self._head_motion_client = None

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def goal_callback(self, _goal_request: ScanScene.Goal) -> GoalResponse:
        if not self._is_active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        started_at = time.time()
        result = ScanScene.Result()

        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = 'Another scan goal is already executing'
            result.summary_text = result.message
            result.result_payload_json = json.dumps(
                {'skill': 'scan', 'summary_text': result.message},
                sort_keys=True,
                separators=(',', ':'),
            )
            result.duration = 0.0
            return result

        try:
            request = goal_handle.request
            scan_args = self._scan_args(request)
            goal_handle.publish_feedback(self._feedback('preparing', 0.1))

            sweep_ok, sweep_reason = self._execute_scan_head_sweep(goal_handle, scan_args)
            if not sweep_ok and goal_handle.is_cancel_requested:
                goal_handle.canceled()
                payload = build_scan_result_payload(
                    scan_args,
                    default_summary=sweep_reason or 'scan goal canceled',
                )
                return self._result_from_payload(
                    success=False,
                    reason=sweep_reason or 'scan goal canceled',
                    payload=payload,
                    duration=time.time() - started_at,
                )
            if not sweep_ok and self.scan_require_motion:
                goal_handle.abort()
                payload = build_scan_result_payload(
                    scan_args,
                    default_summary=sweep_reason or 'scan head sweep failed',
                )
                return self._result_from_payload(
                    success=False,
                    reason=sweep_reason or 'scan head sweep failed',
                    payload=payload,
                    duration=time.time() - started_at,
                )

            result_mode = str(request.result_mode or '').strip().lower() or self.scan_result_mode
            success, reason, _metadata = resolve_scan_result(
                scan_args,
                default_result_mode=result_mode,
                default_summary=self.scan_summary,
            )
            payload = build_scan_result_payload(
                scan_args,
                default_summary=str(reason or '').strip(),
            )
            payload['skill_server'] = self.get_name()
            payload['result_mode'] = result_mode

            goal_handle.publish_feedback(self._feedback('completing', 1.0))
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return self._result_from_payload(
                success=success,
                reason=reason,
                payload=payload,
                duration=time.time() - started_at,
            )
        finally:
            self._execution_lock.release()

    # ------------------------------------------------------------------
    # Internal scan flow
    # ------------------------------------------------------------------

    def _execute_scan_head_sweep(self, goal_handle, scan_args: dict) -> tuple[bool, str]:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return False, 'scan goal canceled'

        sweep_count = self._scan_sweep_count(scan_args)
        if sweep_count <= 0:
            return True, ''
        if self.scan_sweep_yaw_rad <= 0.0:
            self.get_logger().warn('scan_sweep_yaw_rad<=0.0, skipping scan head sweep')
            return True, ''
        if self._head_motion_client is None:
            return False, 'head motion client unavailable'

        sweep_positions = (
            self.scan_sweep_yaw_rad,
            -self.scan_sweep_yaw_rad,
            0.0,
        )
        total_moves = max(1, sweep_count * len(sweep_positions))
        move_index = 0

        for _ in range(sweep_count):
            for yaw in sweep_positions:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return False, 'scan goal canceled'
                move_index += 1
                progress = min(0.9, 0.1 + 0.8 * (move_index / float(total_moves)))
                goal_handle.publish_feedback(self._feedback('sweeping', progress))

                motion_ok, motion_reason = self._dispatch_head_motion(yaw=yaw)
                if not motion_ok:
                    return False, motion_reason or 'scan head sweep dispatch failed'
                if self.scan_sweep_settle_sec > 0.0:
                    time.sleep(self.scan_sweep_settle_sec)

        return True, ''

    def _dispatch_head_motion(self, *, yaw: float) -> tuple[bool, str]:
        if self._head_motion_client is None:
            return False, 'head motion client unavailable'
        wait_timeout = max(self.head_motion_wait_sec, 1.0)
        if not self._head_motion_client.wait_for_server(timeout_sec=wait_timeout):
            return False, 'head motion action server unavailable'

        goal = DoHeadMotion.Goal()
        goal.yaw = float(yaw)
        goal.pitch = float(self.scan_sweep_pitch_rad)
        goal.speed = float(self.head_motion_speed)
        goal.relative = False

        acceptance_event = threading.Event()
        result_event = threading.Event()
        outcome = {'accepted': False, 'success': False, 'reason': 'head motion result timed out'}

        def _goal_response_callback(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as err:  # pragma: no cover - ROS transport errors
                outcome['reason'] = 'head motion goal response failed: %s' % err
                acceptance_event.set()
                result_event.set()
                return
            if goal_handle is None or not goal_handle.accepted:
                outcome['reason'] = 'head motion goal rejected'
                acceptance_event.set()
                result_event.set()
                return
            outcome['accepted'] = True
            acceptance_event.set()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_callback)

        def _result_callback(future) -> None:
            try:
                wrapped_result = future.result()
                action_result = getattr(wrapped_result, 'result', None)
                success = bool(getattr(action_result, 'success', False))
                message = str(getattr(action_result, 'message', '')).strip()
                outcome['success'] = success
                outcome['reason'] = message
            except Exception as err:  # pragma: no cover - ROS transport errors
                outcome['success'] = False
                outcome['reason'] = 'head motion result retrieval failed: %s' % err
            finally:
                result_event.set()

        goal_future = self._head_motion_client.send_goal_async(goal)
        goal_future.add_done_callback(_goal_response_callback)

        if not acceptance_event.wait(timeout=max(self.head_motion_wait_sec, 1.0)):
            return False, 'head motion goal response timed out'
        if not outcome['accepted']:
            return False, str(outcome['reason']).strip() or 'head motion goal rejected'
        if not result_event.wait(timeout=self.head_motion_result_timeout_sec):
            return False, 'head motion result timed out'
        if not outcome['success']:
            return False, str(outcome['reason']).strip() or 'head motion failed'
        return True, ''

    def _scan_args(self, request: ScanScene.Goal) -> dict:
        target = str(request.target or '').strip()
        target_kind = str(request.target_kind or target or 'scene').strip().lower() or 'scene'
        scan_args = {
            'target': target,
            'target_kind': target_kind,
            'max_sweeps': int(request.max_sweeps),
            'evidence_policy': str(request.evidence_policy or '').strip(),
        }

        scene_objects = [dict(item) for item in self._latest_scene_objects if isinstance(item, dict)]
        if scene_objects:
            scan_args['objects'] = scene_objects

        if is_people_scan_target(target_kind, target):
            people_payload = self._tracked_people_scan_people_payload()
            if people_payload:
                scan_args['people'] = people_payload
                people_summary = summarize_people_detection(
                    [str(item.get('id', '')).strip() for item in people_payload]
                )
                if people_summary:
                    scan_args['summary'] = people_summary
            return scan_args

        if self._latest_scan_summary:
            scan_args['summary'] = self._latest_scan_summary
        return scan_args

    def _scan_sweep_count(self, scan_args: dict) -> int:
        raw_max_sweeps = scan_args.get('max_sweeps', self.scan_min_sweeps)
        try:
            requested = int(raw_max_sweeps)
        except (TypeError, ValueError):
            requested = self.scan_min_sweeps
        requested = max(self.scan_min_sweeps, requested)
        return min(self.scan_max_sweeps, requested)

    def _tracked_people_scan_people_payload(self) -> list[dict]:
        if not self._latest_tracked_person_ids:
            return []
        age_sec = max(0.0, time.time() - float(self._latest_tracked_people_ts or 0.0))
        if self.scan_people_max_age_sec > 0.0 and age_sec > self.scan_people_max_age_sec:
            return []
        return [
            {
                'id': person_id,
                'source': 'hri_tracked_persons',
                'last_seen_age_sec': round(age_sec, 3),
            }
            for person_id in self._latest_tracked_person_ids
            if str(person_id).strip()
        ]

    # ------------------------------------------------------------------
    # Topic ingestion
    # ------------------------------------------------------------------

    def _on_scan_summary(self, msg: String) -> None:
        raw_payload = str(msg.data or '').strip()
        parsed_payload = self._parse_scan_summary_payload(raw_payload)
        self._latest_scene_objects = tuple(self._extract_scene_objects(parsed_payload))
        summary_text = str(parsed_payload.get('summary_text', '')).strip()
        self._latest_scan_summary = summary_text or raw_payload

    def _on_scan_people(self, msg: IdsList) -> None:
        self._latest_tracked_person_ids = tuple(
            str(person_id).strip()
            for person_id in msg.ids
            if str(person_id).strip()
        )
        self._latest_tracked_people_ts = time.time()

    @staticmethod
    def _parse_scan_summary_payload(raw_payload: str) -> dict:
        if not raw_payload:
            return {}
        try:
            parsed = json.loads(raw_payload)
        except json.JSONDecodeError:
            return {}
        return parsed if isinstance(parsed, dict) else {}

    @staticmethod
    def _extract_scene_objects(payload: dict) -> list[dict]:
        raw_objects = payload.get('objects', [])
        if not isinstance(raw_objects, list):
            return []
        objects: list[dict] = []
        for item in raw_objects:
            if not isinstance(item, dict):
                continue
            entity_id = str(item.get('entity_id', item.get('id', ''))).strip()
            label = str(item.get('label', item.get('kb_class', entity_id))).strip()
            normalized = {
                'id': entity_id,
                'entity_id': entity_id,
                'label': label,
                'kb_class': str(item.get('kb_class', '')).strip(),
                'source': str(item.get('source', 'scene_summary')).strip() or 'scene_summary',
            }
            center_x = coerce_optional_float(item.get('center_x'))
            center_y = coerce_optional_float(item.get('center_y'))
            confidence = coerce_optional_float(item.get('confidence'))
            last_seen_sec = coerce_optional_float(item.get('last_seen_sec'))
            distance_m = coerce_optional_float(item.get('distance_m'))
            if center_x is not None:
                normalized['center_x'] = center_x
            if center_y is not None:
                normalized['center_y'] = center_y
            if confidence is not None:
                normalized['confidence'] = confidence
            if last_seen_sec is not None:
                normalized['last_seen_sec'] = last_seen_sec
            if distance_m is not None:
                normalized['distance_m'] = distance_m
            objects.append(normalized)
        return objects

    @staticmethod
    def _feedback(status: str, progress: float) -> ScanScene.Feedback:
        feedback = ScanScene.Feedback()
        feedback.status = str(status).strip()
        feedback.progress = max(0.0, min(1.0, float(progress)))
        return feedback

    @staticmethod
    def _result_from_payload(
        *,
        success: bool,
        reason: str,
        payload: dict,
        duration: float,
    ) -> ScanScene.Result:
        result = ScanScene.Result()
        result.success = bool(success)
        result.message = str(reason or '').strip()
        result.summary_text = str(payload.get('summary_text', '')).strip()
        result.result_payload_json = json.dumps(
            payload,
            sort_keys=True,
            separators=(',', ':'),
        )
        result.duration = float(max(0.0, duration))
        return result
