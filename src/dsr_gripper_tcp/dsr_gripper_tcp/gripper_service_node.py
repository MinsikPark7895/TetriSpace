from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState

from dsr_gripper_tcp.example_gripper_tcp import set_robot_mode_autonomous
from dsr_gripper_tcp.gripper_tcp_bridge import BridgeConfig, DoosanGripperTcpBridge
from dsr_gripper_tcp.gripper_tcp_protocol import GripperState as BridgeState
from dsr_gripper_tcp_interfaces.action import SafeGrasp
from dsr_gripper_tcp_interfaces.msg import GripperState
from dsr_gripper_tcp_interfaces.srv import (
    GetMotionProfile,
    GetPosition,
    GetState,
    SetMotionProfile,
    SetPosition,
    SetTorque,
)


class GripperServiceNode(Node):
    """Single owner for the gripper TCP bridge, exposed through ROS services."""

    def __init__(self) -> None:
        super().__init__('gripper_service')

        # Controller / bridge parameters.
        self.declare_parameter('controller_host', '110.120.1.56')
        self.declare_parameter('tcp_port', 20002)
        self.declare_parameter('namespace', 'dsr01')
        self.declare_parameter('service_prefix', '')
        self.declare_parameter('skip_set_autonomous', False)
        self.declare_parameter('initialize_on_start', True)
        self.declare_parameter('goal_current', 400)
        self.declare_parameter('profile_velocity', 1500)
        self.declare_parameter('profile_acceleration', 1000)
        self.declare_parameter('connect_timeout_sec', 20.0)
        self.declare_parameter('post_drl_start_sleep_sec', 0.5)
        self.declare_parameter('stop_existing_drl', True)
        self.declare_parameter('drl_stop_mode', 1)
        self.declare_parameter('drl_stop_settle_sec', 5.0)
        self.declare_parameter('drl_start_retry_count', 3)
        self.declare_parameter('drl_start_retry_delay_sec', 1.0)
        self.declare_parameter('init_attempts', 5)
        self.declare_parameter('init_timeout_sec', 30.0)
        self.declare_parameter('init_retry_delay_sec', 1.0)

        # Service node behavior.
        self.declare_parameter('poll_rate_hz', 20.0)
        self.declare_parameter('joint_name', 'rh_p12_rn')
        self.declare_parameter('position_max', 1150)
        self.declare_parameter('default_move_timeout_sec', 5.0)
        self.declare_parameter('default_safe_grasp_timeout_sec', 10.0)
        self.declare_parameter('grasp_current_threshold', 300)
        self.declare_parameter('object_lost_current_threshold', 80)
        self.declare_parameter('object_lost_position_delta', 80)

        gp = self.get_parameter
        self.robot_namespace = gp('namespace').get_parameter_value().string_value
        self.service_prefix = gp('service_prefix').get_parameter_value().string_value
        self.skip_set_autonomous = gp('skip_set_autonomous').get_parameter_value().bool_value
        self.initialize_on_start = gp('initialize_on_start').get_parameter_value().bool_value
        self._joint_name = gp('joint_name').get_parameter_value().string_value
        self._position_max = gp('position_max').get_parameter_value().integer_value
        self._poll_rate_hz = max(gp('poll_rate_hz').get_parameter_value().double_value, 1.0)
        self._default_move_timeout = gp('default_move_timeout_sec').get_parameter_value().double_value
        self._default_safe_grasp_timeout = gp(
            'default_safe_grasp_timeout_sec'
        ).get_parameter_value().double_value
        self._grasp_current_threshold = gp('grasp_current_threshold').get_parameter_value().integer_value
        self._object_lost_current_threshold = gp(
            'object_lost_current_threshold'
        ).get_parameter_value().integer_value
        self._object_lost_position_delta = gp(
            'object_lost_position_delta'
        ).get_parameter_value().integer_value

        self._goal_current = gp('goal_current').get_parameter_value().integer_value
        self._profile_velocity = gp('profile_velocity').get_parameter_value().integer_value
        self._profile_acceleration = gp('profile_acceleration').get_parameter_value().integer_value
        self._last_goal_position = 0
        self._last_state: GripperState | None = None
        self._had_grasp = False
        self._last_grasp_position: int | None = None

        cfg = BridgeConfig(
            controller_host=gp('controller_host').get_parameter_value().string_value,
            tcp_port=gp('tcp_port').get_parameter_value().integer_value,
            namespace=self.robot_namespace,
            service_prefix=self.service_prefix,
            goal_current=self._goal_current,
            profile_velocity=self._profile_velocity,
            profile_acceleration=self._profile_acceleration,
            connect_timeout_sec=gp('connect_timeout_sec').get_parameter_value().double_value,
            post_drl_start_sleep_sec=gp(
                'post_drl_start_sleep_sec'
            ).get_parameter_value().double_value,
            stop_existing_drl=gp('stop_existing_drl').get_parameter_value().bool_value,
            drl_stop_mode=gp('drl_stop_mode').get_parameter_value().integer_value,
            drl_stop_settle_sec=gp('drl_stop_settle_sec').get_parameter_value().double_value,
            drl_start_retry_count=gp('drl_start_retry_count').get_parameter_value().integer_value,
            drl_start_retry_delay_sec=gp(
                'drl_start_retry_delay_sec'
            ).get_parameter_value().double_value,
        )

        self._bridge_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._bridge = DoosanGripperTcpBridge(node=self, config=cfg)
        self._callback_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._state_pub = self.create_publisher(GripperState, '~/state', qos)
        self._joint_state_pub = self.create_publisher(JointState, '~/joint_state', qos)

        self.create_service(
            GetState,
            '~/get_state',
            self._handle_get_state,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetPosition,
            '~/get_position',
            self._handle_get_position,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetPosition,
            '~/set_position',
            self._handle_set_position,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetMotionProfile,
            '~/set_motion_profile',
            self._handle_set_motion_profile,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetMotionProfile,
            '~/get_motion_profile',
            self._handle_get_motion_profile,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetTorque,
            '~/set_torque',
            self._handle_set_torque,
            callback_group=self._callback_group,
        )

        self._safe_grasp_action = ActionServer(
            self,
            SafeGrasp,
            '~/safe_grasp',
            execute_callback=self._execute_safe_grasp,
            goal_callback=self._handle_safe_grasp_goal,
            cancel_callback=self._handle_safe_grasp_cancel,
            callback_group=self._callback_group,
        )

        self._poll_timer = None

    def boot_bridge(self) -> None:
        if not self.skip_set_autonomous:
            self.get_logger().info('Setting robot mode to autonomous...')
            set_robot_mode_autonomous(self, self.robot_namespace, self.service_prefix)

        self.get_logger().info('Starting DRL TCP gripper server...')
        self._bridge.start()

        if self.initialize_on_start:
            attempts = self.get_parameter('init_attempts').get_parameter_value().integer_value
            timeout_sec = self.get_parameter('init_timeout_sec').get_parameter_value().double_value
            retry_delay = self.get_parameter('init_retry_delay_sec').get_parameter_value().double_value
            with self._bridge_lock:
                state = self._bridge.initialize_with_retry(
                    attempts=attempts,
                    timeout_sec=timeout_sec,
                    retry_delay_sec=retry_delay,
                )
            self._update_cached_state(state, 'initialized')

        self._poll_timer = self.create_timer(1.0 / self._poll_rate_hz, self._poll_state)
        self.get_logger().info(f'Gripper service node ready at {self._poll_rate_hz:.1f} Hz')

    def shutdown(self) -> None:
        self._safe_grasp_action.destroy()
        try:
            self._bridge.close(shutdown_remote=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Bridge close failed: {exc}')

    def _poll_state(self) -> None:
        if not self._bridge_lock.acquire(timeout=0.005):
            return
        try:
            bridge_state = self._bridge.read_state()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Gripper state polling failed: {exc}', throttle_duration_sec=2.0)
            return
        finally:
            self._bridge_lock.release()

        state_msg = self._update_cached_state(bridge_state, 'ok')
        self._state_pub.publish(state_msg)
        self._publish_joint_state(state_msg)

    def _handle_get_state(self, request, response):
        try:
            state_msg = self._get_state(force_read=bool(request.force_read))
            response.success = True
            response.message = 'ok'
            response.state = state_msg
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            response.state = self._last_state_or_empty(str(exc))
        return response

    def _handle_get_position(self, request, response):
        try:
            state_msg = self._get_state(force_read=bool(request.force_read))
            response.success = True
            response.message = 'ok'
            response.present_position = state_msg.present_position
            response.goal_position = state_msg.goal_position
            response.present_current = state_msg.present_current
            response.present_velocity = state_msg.present_velocity
            response.moving = state_msg.moving
            response.in_position = state_msg.in_position
            response.torque_enabled = state_msg.torque_enabled
            response.grasp_detected = state_msg.grasp_detected
            response.object_lost = state_msg.object_lost
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _handle_set_position(self, request, response):
        timeout = float(request.timeout_sec) if request.timeout_sec > 0 else self._default_move_timeout
        try:
            with self._bridge_lock:
                bridge_state = self._bridge.move_to(int(request.position), timeout_sec=timeout)
            self._last_goal_position = int(request.position)
            state_msg = self._update_cached_state(bridge_state, 'position set')
            response.success = True
            response.message = 'ok'
            response.present_position = state_msg.present_position
            response.goal_position = state_msg.goal_position
            response.present_current = state_msg.present_current
            response.in_position = state_msg.in_position
            response.grasp_detected = state_msg.grasp_detected
            response.object_lost = state_msg.object_lost
            response.state = state_msg
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            response.state = self._last_state_or_empty(str(exc))
        return response

    def _handle_set_motion_profile(self, request, response):
        try:
            with self._bridge_lock:
                bridge_state = self._bridge.set_motion_profile(
                    goal_current=int(request.goal_current),
                    profile_velocity=int(request.profile_velocity),
                    profile_acceleration=int(request.profile_acceleration),
                )
            self._goal_current = int(request.goal_current)
            self._profile_velocity = int(request.profile_velocity)
            self._profile_acceleration = int(request.profile_acceleration)
            state_msg = self._update_cached_state(bridge_state, 'motion profile set')
            response.success = True
            response.message = 'ok'
            response.goal_current = self._goal_current
            response.profile_velocity = self._profile_velocity
            response.profile_acceleration = self._profile_acceleration
            response.state = state_msg
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            response.goal_current = self._goal_current
            response.profile_velocity = self._profile_velocity
            response.profile_acceleration = self._profile_acceleration
            response.state = self._last_state_or_empty(str(exc))
        return response

    def _handle_get_motion_profile(self, request, response):  # noqa: ARG002
        response.success = True
        response.message = 'cached profile'
        response.goal_current = self._goal_current
        response.profile_velocity = self._profile_velocity
        response.profile_acceleration = self._profile_acceleration
        return response

    def _handle_set_torque(self, request, response):
        try:
            with self._bridge_lock:
                bridge_state = self._bridge.set_torque(bool(request.enabled))
            state_msg = self._update_cached_state(bridge_state, 'torque set')
            response.success = True
            response.message = 'ok'
            response.torque_enabled = state_msg.torque_enabled
            response.state = state_msg
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            response.torque_enabled = False
            response.state = self._last_state_or_empty(str(exc))
        return response

    def _handle_safe_grasp_goal(self, goal_request):
        if goal_request.target_position < 0 or goal_request.target_position > self._position_max:
            self.get_logger().warning(f'Rejecting safe_grasp target={goal_request.target_position}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_safe_grasp_cancel(self, goal_handle):  # noqa: ARG002
        return CancelResponse.ACCEPT

    def _execute_safe_grasp(self, goal_handle):
        goal = goal_handle.request
        result = SafeGrasp.Result()
        timeout_sec = float(goal.timeout_sec) if goal.timeout_sec > 0 else self._default_safe_grasp_timeout
        max_current = abs(int(goal.max_current)) if goal.max_current > 0 else self._goal_current
        delta_threshold = abs(int(goal.current_delta_threshold))

        try:
            with self._bridge_lock:
                self._bridge.set_motion_profile(
                    goal_current=max_current,
                    profile_velocity=self._profile_velocity,
                    profile_acceleration=self._profile_acceleration,
                )
                start_state = self._bridge.read_state()
            self._goal_current = max_current
            baseline_current = abs(int(start_state.present_current))
            target_position = int(goal.target_position)

            feedback = SafeGrasp.Feedback()
            feedback.present_position = int(start_state.present_position)
            feedback.present_current = int(start_state.present_current)
            feedback.current_delta = 0
            feedback.grasp_detected = False
            feedback.object_lost = False
            feedback.state = self._update_cached_state(start_state, 'safe grasp starting')
            goal_handle.publish_feedback(feedback)

            if goal_handle.is_cancel_requested:
                state_msg = self._hold_current_position()
                result.success = False
                result.message = 'safe_grasp canceled'
                result.final_position = state_msg.present_position
                result.final_current = state_msg.present_current
                result.grasp_detected = state_msg.grasp_detected
                result.object_lost = state_msg.object_lost
                result.state = state_msg
                goal_handle.canceled()
                return result

            with self._bridge_lock:
                bridge_state = self._bridge.move_to(target_position, timeout_sec=timeout_sec)
                bridge_state = self._bridge.read_state()

            self._last_goal_position = target_position
            state_msg = self._update_cached_state(bridge_state, 'safe grasp complete')
            current_abs = abs(int(bridge_state.present_current))
            current_delta = abs(current_abs - baseline_current)
            grasp_detected = current_abs >= max_current or (
                delta_threshold > 0 and current_delta >= delta_threshold
            )

            feedback = SafeGrasp.Feedback()
            feedback.present_position = state_msg.present_position
            feedback.present_current = state_msg.present_current
            feedback.current_delta = current_delta
            feedback.grasp_detected = grasp_detected
            feedback.object_lost = state_msg.object_lost
            feedback.state = state_msg
            goal_handle.publish_feedback(feedback)

            result.success = grasp_detected
            result.message = 'grasp detected' if grasp_detected else 'target reached without grasp'
            result.final_position = state_msg.present_position
            result.final_current = state_msg.present_current
            result.grasp_detected = grasp_detected
            result.object_lost = state_msg.object_lost
            result.state = state_msg

            if grasp_detected:
                self._had_grasp = True
                self._last_grasp_position = state_msg.present_position
                state_msg.grasp_detected = True
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:  # noqa: BLE001
            result.success = False
            result.message = str(exc)
            result.state = self._last_state_or_empty(str(exc))
            result.final_position = result.state.present_position
            result.final_current = result.state.present_current
            result.grasp_detected = result.state.grasp_detected
            result.object_lost = result.state.object_lost
            goal_handle.abort()
            return result

    def _hold_current_position(self) -> GripperState:
        with self._bridge_lock:
            bridge_state = self._bridge.read_state()
            bridge_state = self._bridge.move_to(int(bridge_state.present_position), timeout_sec=1.0)
        return self._update_cached_state(bridge_state, 'holding current position')

    def _get_state(self, force_read: bool = False) -> GripperState:
        if force_read:
            with self._bridge_lock:
                bridge_state = self._bridge.read_state()
            return self._update_cached_state(bridge_state, 'ok')

        with self._state_lock:
            if self._last_state is not None:
                return self._last_state

        with self._bridge_lock:
            bridge_state = self._bridge.read_state()
        return self._update_cached_state(bridge_state, 'ok')

    def _update_cached_state(self, bridge_state: BridgeState, status_text: str) -> GripperState:
        msg = self._state_msg_from_bridge(bridge_state, status_text)
        with self._state_lock:
            self._last_state = msg
        return msg

    def _state_msg_from_bridge(self, bridge_state: BridgeState, status_text: str) -> GripperState:
        msg = GripperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.ready = bool(bridge_state.torque_enabled)
        msg.torque_enabled = bool(bridge_state.torque_enabled)
        msg.moving = bool(bridge_state.moving)
        msg.in_position = bool(bridge_state.in_position)
        msg.status = int(bridge_state.status)
        msg.moving_status = int(bridge_state.moving_status)
        msg.present_position = int(bridge_state.present_position)
        msg.goal_position = int(self._last_goal_position)
        msg.present_current = int(bridge_state.present_current)
        msg.current_limit = int(self._goal_current)
        msg.present_velocity = int(bridge_state.present_velocity)
        msg.present_temperature = int(bridge_state.present_temperature)
        msg.grasp_detected = self._is_grasp_detected(bridge_state)
        msg.object_lost = self._is_object_lost(bridge_state, msg.grasp_detected)
        msg.status_text = status_text
        return msg

    def _is_grasp_detected(self, bridge_state: BridgeState) -> bool:
        if not bridge_state.torque_enabled or bridge_state.moving:
            return False
        current_abs = abs(int(bridge_state.present_current))
        return current_abs >= self._grasp_current_threshold or current_abs >= int(self._goal_current * 0.9)

    def _is_object_lost(self, bridge_state: BridgeState, grasp_detected: bool) -> bool:
        if grasp_detected:
            self._had_grasp = True
            self._last_grasp_position = int(bridge_state.present_position)
            return False
        if not self._had_grasp or not bridge_state.torque_enabled:
            return False
        current_abs = abs(int(bridge_state.present_current))
        if current_abs > self._object_lost_current_threshold:
            return False
        if self._last_grasp_position is None:
            return True
        position_delta = abs(int(bridge_state.present_position) - self._last_grasp_position)
        return position_delta >= self._object_lost_position_delta

    def _last_state_or_empty(self, status_text: str) -> GripperState:
        with self._state_lock:
            if self._last_state is not None:
                state = self._last_state
                state.status_text = status_text
                return state
        msg = GripperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.status_text = status_text
        return msg

    def _publish_joint_state(self, state_msg: GripperState) -> None:
        msg = JointState()
        msg.header.stamp = state_msg.stamp
        msg.name = [self._joint_name]
        msg.position = [float(state_msg.present_position) / float(max(self._position_max, 1))]
        msg.velocity = [float(state_msg.present_velocity)]
        msg.effort = [float(state_msg.present_current)]
        self._joint_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node.boot_bridge()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
