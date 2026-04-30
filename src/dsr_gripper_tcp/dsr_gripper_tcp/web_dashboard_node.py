"""ROS2 node version of the Doosan gripper web dashboard.

Same Flask/SocketIO frontend as :mod:`web_dashboard`, but wrapped in a proper
``rclpy.Node`` so you can:

* configure everything via ROS2 parameters (controller IP, namespace, ports,
  poll rate, ...),
* publish gripper state on ROS topics (sensor_msgs/JointState +
  std_msgs/Float32MultiArray for raw counts),
* command the gripper from ROS subscribers (goal position, torque enable,
  motion profile, e-stop),
* launch it through ``ros2 run`` / ``ros2 launch`` next to other nodes.

The Flask/SocketIO web UI keeps working exactly the same way as the original
threaded dashboard - only the lifecycle moves under ROS2.
"""

from __future__ import annotations

import threading
import socket as pysocket

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float32MultiArray, Int32
from sensor_msgs.msg import JointState

from flask import Flask, render_template_string
from flask_socketio import SocketIO

from dsr_gripper_tcp.gripper_tcp_bridge import BridgeConfig, DoosanGripperTcpBridge
from dsr_gripper_tcp.example_gripper_tcp import set_robot_mode_autonomous
from dsr_gripper_tcp.web_dashboard import (
    HTML_TEMPLATE,
    POSITION_MAX,
    GOAL_CURRENT_MIN,
    GOAL_CURRENT_MAX,
    PROFILE_VEL_MIN,
    PROFILE_VEL_MAX,
    PROFILE_ACC_MIN,
    PROFILE_ACC_MAX,
)


class GripperWebDashboardNode(Node):
    """ROS2 node that hosts the Flask/SocketIO dashboard for the gripper."""

    def __init__(self) -> None:
        super().__init__('gripper_web_dashboard')

        # ---------- Parameters ----------
        self.declare_parameter('controller_host', '110.120.1.56')
        self.declare_parameter('tcp_port', 20002)
        self.declare_parameter('namespace', 'dsr01')
        self.declare_parameter('service_prefix', '')
        self.declare_parameter('skip_set_autonomous', False)
        self.declare_parameter('initialize_on_start', True)
        self.declare_parameter('goal_current', 400)
        self.declare_parameter('profile_velocity', 1500)
        self.declare_parameter('profile_acceleration', 1000)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5000)
        self.declare_parameter('poll_rate_hz', 20.0)
        self.declare_parameter('joint_name', 'rh_p12_rn')
        self.declare_parameter('move_timeout_sec', 5.0)
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

        gp = self.get_parameter
        self.web_host: str = gp('web_host').get_parameter_value().string_value
        self.web_port: int = gp('web_port').get_parameter_value().integer_value
        self.skip_set_autonomous: bool = gp('skip_set_autonomous').get_parameter_value().bool_value
        self.initialize_on_start: bool = gp('initialize_on_start').get_parameter_value().bool_value
        self.namespace: str = gp('namespace').get_parameter_value().string_value
        self.service_prefix: str = gp('service_prefix').get_parameter_value().string_value
        self._joint_name: str = gp('joint_name').get_parameter_value().string_value
        self._move_timeout: float = gp('move_timeout_sec').get_parameter_value().double_value

        cfg = BridgeConfig(
            controller_host=gp('controller_host').get_parameter_value().string_value,
            tcp_port=gp('tcp_port').get_parameter_value().integer_value,
            namespace=self.namespace,
            service_prefix=self.service_prefix,
            goal_current=gp('goal_current').get_parameter_value().integer_value,
            profile_velocity=gp('profile_velocity').get_parameter_value().integer_value,
            profile_acceleration=gp('profile_acceleration').get_parameter_value().integer_value,
            connect_timeout_sec=gp('connect_timeout_sec').get_parameter_value().double_value,
            post_drl_start_sleep_sec=gp('post_drl_start_sleep_sec').get_parameter_value().double_value,
            stop_existing_drl=gp('stop_existing_drl').get_parameter_value().bool_value,
            drl_stop_mode=gp('drl_stop_mode').get_parameter_value().integer_value,
            drl_stop_settle_sec=gp('drl_stop_settle_sec').get_parameter_value().double_value,
            drl_start_retry_count=gp('drl_start_retry_count').get_parameter_value().integer_value,
            drl_start_retry_delay_sec=gp('drl_start_retry_delay_sec').get_parameter_value().double_value,
        )

        # ---------- Bridge ----------
        self.tcp_lock = threading.Lock()
        self.bridge = DoosanGripperTcpBridge(node=self, config=cfg)

        # ---------- ROS pub/sub ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.joint_state_pub = self.create_publisher(JointState, '~/joint_state', qos)
        self.raw_state_pub = self.create_publisher(Float32MultiArray, '~/raw_state', qos)

        self.create_subscription(Int32, '~/goal_position', self._on_goal_position, 10)
        self.create_subscription(Bool, '~/torque_enable', self._on_torque_enable, 10)
        self.create_subscription(Float32MultiArray, '~/motion_profile', self._on_motion_profile, 10)
        self.create_subscription(Bool, '~/emergency_stop', self._on_estop_topic, 10)

        # ---------- Flask / SocketIO ----------
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, async_mode='threading', cors_allowed_origins='*')
        self._register_http_routes()
        self._register_socketio_handlers()

        # ---------- Polling timer (created later, after bridge is up) ----------
        self._poll_rate_hz: float = max(gp('poll_rate_hz').get_parameter_value().double_value, 1.0)
        self._poll_timer = None

        self.get_logger().info(
            f"GripperWebDashboardNode constructed. "
            f"Controller: {cfg.controller_host}:{cfg.tcp_port}, "
            f"namespace=/{self.namespace}, web=http://{self.web_host}:{self.web_port}"
        )

    # ------------------------------------------------------------------
    # Lifecycle helpers (called by main() before executor.spin())
    # ------------------------------------------------------------------
    def boot_bridge(self) -> None:
        if not self.skip_set_autonomous:
            self.get_logger().info('Setting robot mode to autonomous...')
            set_robot_mode_autonomous(self, self.namespace, self.service_prefix)

        self.get_logger().info('Starting DRL TCP gripper server...')
        self.bridge.start()

        if self.initialize_on_start:
            attempts = self.get_parameter('init_attempts').get_parameter_value().integer_value
            timeout_sec = self.get_parameter('init_timeout_sec').get_parameter_value().double_value
            retry_delay = self.get_parameter('init_retry_delay_sec').get_parameter_value().double_value
            with self.tcp_lock:
                state = self.bridge.initialize_with_retry(
                    attempts=attempts,
                    timeout_sec=timeout_sec,
                    retry_delay_sec=retry_delay,
                )
            self.get_logger().info(
                f"Gripper initialized: pos={state.present_position}, "
                f"torque_enabled={state.torque_enabled}"
            )

        self._poll_timer = self.create_timer(1.0 / self._poll_rate_hz, self._poll_callback)
        self.get_logger().info(f"Polling timer started at {self._poll_rate_hz:.1f} Hz")

    def shutdown(self) -> None:
        try:
            self.bridge.close(shutdown_remote=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Bridge close failed: {exc}")

    def run_web_server(self) -> None:
        self.get_logger().info(f"Web server starting on http://{self.web_host}:{self.web_port}")
        try:
            self.socketio.run(
                self.app,
                host=self.web_host,
                port=self.web_port,
                debug=False,
                use_reloader=False,
                allow_unsafe_werkzeug=True,
            )
        except TypeError:
            self.socketio.run(
                self.app,
                host=self.web_host,
                port=self.web_port,
                debug=False,
                use_reloader=False,
            )

    # ------------------------------------------------------------------
    # Polling: TCP read -> SocketIO emit + ROS publish
    # ------------------------------------------------------------------
    def _poll_callback(self) -> None:
        if not self.tcp_lock.acquire(timeout=0.005):
            return
        try:
            try:
                state = self.bridge.read_state()
            except (BrokenPipeError, ConnectionError, OSError, pysocket.error) as exc:
                self.get_logger().warning(
                    f"Bridge socket error: {exc}", throttle_duration_sec=2.0
                )
                self._reset_socket()
                self.socketio.emit('state_update', {'status': 'error'})
                return
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"Bridge read failed: {exc}", throttle_duration_sec=2.0
                )
                return
        finally:
            self.tcp_lock.release()

        self.socketio.emit('state_update', {
            'status': 'ok',
            'present_position': state.present_position,
            'present_current': state.present_current,
            'present_temperature': state.present_temperature,
            'present_velocity': state.present_velocity,
            'moving': state.moving,
            'moving_status': state.moving_status,
            'torque_enabled': state.torque_enabled,
        })

        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = [self._joint_name]
        js.position = [float(state.present_position) / float(POSITION_MAX)]
        js.velocity = [float(state.present_velocity)]
        js.effort = [float(state.present_current)]
        self.joint_state_pub.publish(js)

        raw = Float32MultiArray()
        raw.data = [
            float(state.present_position),
            float(state.present_current),
            float(state.present_temperature),
            float(state.present_velocity),
            float(state.moving),
            float(state.moving_status),
            1.0 if state.torque_enabled else 0.0,
        ]
        self.raw_state_pub.publish(raw)

    # ------------------------------------------------------------------
    # ROS topic callbacks
    # ------------------------------------------------------------------
    def _on_goal_position(self, msg: Int32) -> None:
        pos = int(msg.data)
        self._run_in_bridge(lambda: self.bridge.move_to(pos, self._move_timeout))

    def _on_torque_enable(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        self._run_in_bridge(lambda: self.bridge.set_torque(enabled))

    def _on_motion_profile(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warning(
                'motion_profile expects [goal_current, profile_velocity, profile_acceleration]'
            )
            return
        gc, pv, pa = int(msg.data[0]), int(msg.data[1]), int(msg.data[2])
        self._run_in_bridge(lambda: self.bridge.set_motion_profile(gc, pv, pa))

    def _on_estop_topic(self, msg: Bool) -> None:
        if msg.data:
            self._do_estop()

    # ------------------------------------------------------------------
    # SocketIO handlers (mirror of web_dashboard.py)
    # ------------------------------------------------------------------
    def _register_http_routes(self) -> None:
        @self.app.route('/')
        def _index():
            return render_template_string(
                HTML_TEMPLATE,
                pos_max=POSITION_MAX,
                cur_min=GOAL_CURRENT_MIN, cur_max=GOAL_CURRENT_MAX,
                vel_min=PROFILE_VEL_MIN, vel_max=PROFILE_VEL_MAX,
                acc_min=PROFILE_ACC_MIN, acc_max=PROFILE_ACC_MAX,
            )

    def _register_socketio_handlers(self) -> None:
        @self.socketio.on('move_cmd')
        def _on_move(data):
            if 'goal_position' not in data:
                return
            pos = int(data['goal_position'])
            self._run_in_bridge(lambda: self.bridge.move_to(pos, self._move_timeout))

        @self.socketio.on('torque_cmd')
        def _on_torque(data):
            if 'enabled' not in data:
                return
            enabled = bool(data['enabled'])
            self._run_in_bridge(lambda: self.bridge.set_torque(enabled))

        @self.socketio.on('profile_cmd')
        def _on_profile(data):
            gc = int(data.get('goal_current', 400))
            pv = int(data.get('profile_velocity', 1500))
            pa = int(data.get('profile_acceleration', 1000))
            self._run_in_bridge(lambda: self.bridge.set_motion_profile(gc, pv, pa))

        @self.socketio.on('estop_cmd')
        def _on_estop():
            self._do_estop()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _run_in_bridge(self, fn) -> None:
        """Run a bridge call on a worker thread under the TCP lock."""
        def runner():
            try:
                with self.tcp_lock:
                    fn()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Bridge call failed: {exc}")
                self._reset_socket()
        threading.Thread(target=runner, daemon=True).start()

    def _do_estop(self) -> None:
        """Hold current position - usable from ROS topic or SocketIO."""
        def runner():
            try:
                with self.tcp_lock:
                    state = self.bridge.read_state()
                    self.bridge.move_to(int(state.present_position), 1.0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"E-stop failed: {exc}")
                self._reset_socket()
        threading.Thread(target=runner, daemon=True).start()

    def _reset_socket(self) -> None:
        try:
            sock = getattr(self.bridge, '_socket', None)
            if sock is not None:
                sock.close()
        except Exception:
            pass
        self.bridge._socket = None  # noqa: SLF001


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperWebDashboardNode()

    try:
        node.boot_bridge()

        web_thread = threading.Thread(target=node.run_web_server, daemon=True)
        web_thread.start()

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
