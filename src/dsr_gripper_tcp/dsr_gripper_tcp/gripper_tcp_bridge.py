from __future__ import annotations

from dataclasses import dataclass
import socket
import textwrap
import time

import rclpy
from rclpy.node import Node

from dsr_msgs2.srv import DrlStart, DrlStop, GetDrlState

from dsr_gripper_tcp.gripper_tcp_protocol import (
    Command,
    GripperState,
    StatusCode,
    build_packet,
    pack_config_payload,
    pack_initialize_payload,
    pack_move_payload,
    pack_torque_payload,
    recv_packet,
    unpack_state_payload,
)


DRL_PROGRAM_STATE_PLAY = 0


@dataclass(slots=True)
class BridgeConfig:
    controller_host: str
    tcp_port: int = 20002
    namespace: str = "dsr01"
    service_prefix: str = "dsr_controller2"
    robot_system: int = 0
    slave_id: int = 1
    baudrate: int = 57600
    goal_current: int = 400
    profile_velocity: int = 1500
    profile_acceleration: int = 1000
    connect_timeout_sec: float = 20.0
    socket_timeout_sec: float = 10.0
    position_tolerance: int = 5
    post_drl_start_sleep_sec: float = 0.5
    stop_existing_drl: bool = True
    drl_stop_mode: int = 1
    drl_stop_settle_sec: float = 5.0
    drl_start_retry_count: int = 3
    drl_start_retry_delay_sec: float = 1.0


class DoosanGripperTcpBridge:
    def __init__(self, node: Node, config: BridgeConfig) -> None:
        self._node = node
        self._config = config
        self._sequence = 1
        self._socket: socket.socket | None = None
        self._service_root = build_service_root(config.namespace, config.service_prefix)
        self._drl_start = self._node.create_client(
            DrlStart,
            f"{self._service_root}/drl/drl_start",
        )
        self._drl_stop = self._node.create_client(
            DrlStop,
            f"{self._service_root}/drl/drl_stop",
        )
        self._get_drl_state = self._node.create_client(
            GetDrlState,
            f"{self._service_root}/drl/get_drl_state",
        )

        self._wait_for_service(self._drl_start, f"{self._service_root}/drl/drl_start")
        self._wait_for_service(self._drl_stop, f"{self._service_root}/drl/drl_stop")
        self._wait_for_service(self._get_drl_state, f"{self._service_root}/drl/get_drl_state")

    def start(self) -> None:
        current_state = self.get_drl_state()
        if current_state == DRL_PROGRAM_STATE_PLAY:
            if self._config.stop_existing_drl:
                self._node.get_logger().warning(
                    "A DRL program is already running. Stopping it before "
                    "starting the gripper TCP server."
                )
                self.stop_drl(self._config.drl_stop_mode)
                self._wait_for_drl_idle(self._config.drl_stop_settle_sec)
            else:
                self._node.get_logger().warning(
                    "A DRL program is already running and stop_existing_drl=False. "
                    "Starting the gripper TCP server may fail."
                )

        # DrlStart can transiently return success=False if the controller is
        # still tearing down the previous program. Retry a few times.
        retry_count = max(1, int(self._config.drl_start_retry_count))
        retry_delay = max(0.0, float(self._config.drl_start_retry_delay_sec))

        last_response = None
        for attempt in range(1, retry_count + 1):
            req = DrlStart.Request()
            req.robot_system = self._config.robot_system
            req.code = self._build_drl_server_script()
            response = self._call_service(self._drl_start, req, "DrlStart")
            last_response = response
            if response is not None and response.success:
                break

            if attempt < retry_count:
                self._node.get_logger().warning(
                    f"DrlStart attempt {attempt}/{retry_count} returned "
                    f"success=False; retrying in {retry_delay:.1f}s..."
                )
                if retry_delay > 0:
                    time.sleep(retry_delay)
                # Make sure the controller is in a clean state before retrying.
                try:
                    if self.get_drl_state() == DRL_PROGRAM_STATE_PLAY:
                        self.stop_drl(self._config.drl_stop_mode)
                        self._wait_for_drl_idle(self._config.drl_stop_settle_sec)
                except Exception as exc:  # noqa: BLE001
                    self._node.get_logger().warning(
                        f"State check before retry failed: {exc}"
                    )
        else:
            raise RuntimeError(
                "Failed to start the DRL gripper TCP server after "
                f"{retry_count} attempt(s). Last response: {last_response}"
            )

        # Give the controller a brief moment to actually start running the DRL
        # script before we try to connect to its TCP server.
        if self._config.post_drl_start_sleep_sec > 0:
            time.sleep(self._config.post_drl_start_sleep_sec)

        self._connect_tcp_client()
        self._node.get_logger().info(
            f"Connected to gripper TCP bridge at {self._config.controller_host}:{self._config.tcp_port}"
        )

    def _wait_for_drl_idle(self, timeout_sec: float) -> None:
        """Poll get_drl_state until DRL leaves the PLAY state, or timeout."""
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        last_state: int | None = None
        while time.monotonic() < deadline:
            try:
                state = self.get_drl_state()
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().warning(
                    f"get_drl_state failed while waiting for idle: {exc}"
                )
                time.sleep(0.2)
                continue
            if state != DRL_PROGRAM_STATE_PLAY:
                self._node.get_logger().info(
                    f"DRL settled to state={state} (not PLAY)."
                )
                return
            last_state = state
            time.sleep(0.2)

        self._node.get_logger().warning(
            f"Timed out waiting for DRL to leave PLAY state "
            f"(last_state={last_state}, timeout={timeout_sec:.1f}s). "
            "Proceeding anyway; DrlStart will be retried if needed."
        )

    def close(self, shutdown_remote: bool = True) -> None:
        try:
            if shutdown_remote and self._socket is not None:
                try:
                    self._send_request(Command.SHUTDOWN, b"", timeout_sec=2.0)
                except Exception as exc:  # noqa: BLE001
                    self._node.get_logger().warning(f"Failed to send shutdown packet: {exc}")
        finally:
            if self._socket is not None:
                self._socket.close()
                self._socket = None

    def get_drl_state(self) -> int:
        req = GetDrlState.Request()
        response = self._call_service(self._get_drl_state, req, "GetDrlState")
        if response is None:
            raise RuntimeError("GetDrlState returned no response.")
        return int(response.drl_state)

    def stop_drl(self, stop_mode: int = 1) -> bool:
        """Stop any DRL program currently running on the controller."""
        req = DrlStop.Request()
        req.stop_mode = int(stop_mode)
        response = self._call_service(self._drl_stop, req, "DrlStop")
        if response is None:
            return False
        return bool(response.success)

    def ping(self) -> GripperState:
        return self._request_state(Command.PING, b"")

    def initialize(
        self,
        goal_current: int | None = None,
        timeout_sec: float | None = None,
    ) -> GripperState:
        current = self._config.goal_current if goal_current is None else int(goal_current)
        return self._request_state(
            Command.INITIALIZE,
            pack_initialize_payload(current),
            timeout_sec=timeout_sec,
        )

    def initialize_with_retry(
        self,
        goal_current: int | None = None,
        attempts: int = 5,
        timeout_sec: float = 30.0,
        retry_delay_sec: float = 1.0,
    ) -> GripperState:
        """Retry INITIALIZE several times, recovering the TCP socket on timeout.

        Errors are classified into two kinds:

        * **Socket-level errors** (TimeoutError, OSError, ConnectionError):
          the request/response sequence may be out of sync, so we reset the
          socket and reconnect before retrying.
        * **Protocol-level errors** (e.g. ``Controller returned error status 3``):
          the bridge protocol is fine, the gripper just isn't responding to
          modbus. Wait and retry without dropping the socket.
        """
        attempts = max(1, int(attempts))
        last_error: Exception | None = None
        for attempt in range(1, attempts + 1):
            try:
                return self.initialize(goal_current=goal_current, timeout_sec=timeout_sec)
            except (TimeoutError, ConnectionError, OSError) as exc:
                last_error = exc
                self._node.get_logger().warning(
                    f"INITIALIZE attempt {attempt}/{attempts} failed (socket): {exc}"
                )
                self._reset_socket()
                if attempt < attempts and retry_delay_sec > 0:
                    time.sleep(retry_delay_sec)
                if attempt < attempts:
                    try:
                        self._connect_tcp_client()
                    except Exception as connect_exc:  # noqa: BLE001
                        last_error = connect_exc
                        self._node.get_logger().warning(
                            f"Reconnect for retry failed: {connect_exc}"
                        )
            except RuntimeError as exc:
                last_error = exc
                self._node.get_logger().warning(
                    f"INITIALIZE attempt {attempt}/{attempts} failed (gripper): {exc}"
                )
                if attempt < attempts and retry_delay_sec > 0:
                    time.sleep(retry_delay_sec)
        raise RuntimeError(
            f"INITIALIZE failed after {attempts} attempts. Last error: {last_error}. "
            "If this persists, power-cycle the gripper or restart the Doosan controller."
        )

    def _reset_socket(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
            self._sequence = 1

    def set_motion_profile(
        self,
        goal_current: int | None = None,
        profile_velocity: int | None = None,
        profile_acceleration: int | None = None,
    ) -> GripperState:
        current = self._config.goal_current if goal_current is None else int(goal_current)
        velocity = self._config.profile_velocity if profile_velocity is None else int(profile_velocity)
        acceleration = (
            self._config.profile_acceleration
            if profile_acceleration is None
            else int(profile_acceleration)
        )

        self._config.goal_current = current
        self._config.profile_velocity = velocity
        self._config.profile_acceleration = acceleration
        payload = pack_config_payload(current, velocity, acceleration)
        return self._request_state(Command.SET_CONFIG, payload)

    def read_state(self) -> GripperState:
        return self._request_state(Command.READ_STATE, b"")

    def set_torque(self, enabled: bool) -> GripperState:
        return self._request_state(Command.SET_TORQUE, pack_torque_payload(bool(enabled)))

    def move_to(self, goal_position: int, timeout_sec: float = 10.0) -> GripperState:
        timeout_ms = max(0, int(timeout_sec * 1000.0))
        payload = pack_move_payload(int(goal_position), timeout_ms)
        return self._request_state(Command.MOVE, payload, timeout_sec=max(timeout_sec + 2.0, 5.0))

    def _request_state(
        self,
        command: Command,
        payload: bytes,
        timeout_sec: float | None = None,
    ) -> GripperState:
        response = self._send_request(command, payload, timeout_sec=timeout_sec)
        state = unpack_state_payload(response)
        if state.status != StatusCode.OK:
            raise RuntimeError(
                "Controller returned error status "
                f"{state.status} for command {Command(command).name}."
            )
        return state

    def _send_request(
        self,
        command: Command,
        payload: bytes,
        timeout_sec: float | None = None,
    ) -> bytes:
        self._ensure_socket()
        assert self._socket is not None

        packet = build_packet(int(command), self._sequence, payload)
        self._socket.settimeout(self._config.socket_timeout_sec if timeout_sec is None else timeout_sec)
        self._socket.sendall(packet)

        response_command, response_sequence, response_payload = recv_packet(self._socket)
        expected_sequence = self._sequence
        self._sequence = (self._sequence + 1) % 65536
        if self._sequence == 0:
            self._sequence = 1

        if response_command != int(command):
            raise RuntimeError(
                f"Unexpected response command {response_command}, expected {int(command)}."
            )
        if response_sequence != expected_sequence:
            raise RuntimeError(
                f"Unexpected response sequence {response_sequence}, expected {expected_sequence}."
            )
        return response_payload

    def _ensure_socket(self) -> None:
        if self._socket is None:
            self._connect_tcp_client()

    def _connect_tcp_client(self) -> None:
        deadline = time.monotonic() + self._config.connect_timeout_sec
        last_error: Exception | None = None
        attempt = 0
        last_log = 0.0

        while time.monotonic() < deadline:
            try:
                sock = socket.create_connection(
                    (self._config.controller_host, self._config.tcp_port),
                    timeout=1.0,
                )
                sock.settimeout(self._config.socket_timeout_sec)
                self._socket = sock
                return
            except OSError as exc:
                last_error = exc
                attempt += 1
                now = time.monotonic()
                if now - last_log >= 2.0:
                    self._node.get_logger().info(
                        f"Waiting for controller TCP server "
                        f"{self._config.controller_host}:{self._config.tcp_port} "
                        f"(attempt {attempt}, {exc})..."
                    )
                    last_log = now
                time.sleep(0.25)

        raise RuntimeError(
            f"Failed to connect to controller TCP server "
            f"{self._config.controller_host}:{self._config.tcp_port} "
            f"after {self._config.connect_timeout_sec:.1f}s: {last_error}"
        )

    def _wait_for_service(self, client, name: str) -> None:
        while not client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info(f"Waiting for {name}...")

    def _call_service(self, client, request, name: str):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=None)
        if future.result() is None:
            raise RuntimeError(f"{name} returned no response.")
        return future.result()

    def _build_drl_server_script(self) -> str:
        cfg = self._config
        return textwrap.dedent(
            f"""
            CMD_PING = 1
            CMD_INITIALIZE = 2
            CMD_SET_CONFIG = 3
            CMD_MOVE = 4
            CMD_READ_STATE = 5
            CMD_SHUTDOWN = 6
            CMD_SET_TORQUE = 7

            STATUS_OK = 0
            STATUS_BAD_PACKET = 1
            STATUS_BAD_COMMAND = 2
            STATUS_IO_ERROR = 3
            STATUS_TIMEOUT = 4
            STATUS_RANGE_ERROR = 5
            STATUS_NOT_READY = 6

            HEADER_SIZE = 8
            POLL_WAIT_SEC = 0.05
            POSITION_TOLERANCE = {cfg.position_tolerance}

            ADDR_TORQUE_ENABLE = 256
            ADDR_GOAL_CURRENT = 275
            ADDR_PROFILE_ACCELERATION = 278
            ADDR_PROFILE_VELOCITY = 280
            ADDR_GOAL_POSITION = 282
            ADDR_MOVING_STATUS = 285
            ADDR_PRESENT_CURRENT = 287
            ADDR_PRESENT_VELOCITY = 288
            ADDR_PRESENT_POSITION = 290
            ADDR_PRESENT_TEMPERATURE = 297

            g_slaveid = {cfg.slave_id}
            g_goal_current = {cfg.goal_current}
            g_profile_velocity = {cfg.profile_velocity}
            g_profile_acceleration = {cfg.profile_acceleration}
            g_sock = None
            g_ready = False

            def modbus_set_slaveid(slaveid):
                global g_slaveid
                g_slaveid = slaveid

            def modbus_fc03(startaddress, cnt):
                global g_slaveid
                data = (g_slaveid).to_bytes(1, byteorder='big')
                data += (3).to_bytes(1, byteorder='big')
                data += (startaddress).to_bytes(2, byteorder='big')
                data += (cnt).to_bytes(2, byteorder='big')
                return modbus_send_make(data)

            def modbus_fc06(address, value):
                global g_slaveid
                data = (g_slaveid).to_bytes(1, byteorder='big')
                data += (6).to_bytes(1, byteorder='big')
                data += (address).to_bytes(2, byteorder='big')
                data += (value).to_bytes(2, byteorder='big')
                return modbus_send_make(data)

            def modbus_fc16(startaddress, cnt, valuelist):
                global g_slaveid
                data = (g_slaveid).to_bytes(1, byteorder='big')
                data += (16).to_bytes(1, byteorder='big')
                data += (startaddress).to_bytes(2, byteorder='big')
                data += (cnt).to_bytes(2, byteorder='big')
                data += (2 * cnt).to_bytes(1, byteorder='big')
                for i in range(0, cnt):
                    data += (valuelist[i]).to_bytes(2, byteorder='big')
                return modbus_send_make(data)

            def u32_to_words(value):
                low_word = value & 0xFFFF
                high_word = (value >> 16) & 0xFFFF
                return [low_word, high_word]

            def words_to_i32(low_word, high_word):
                value = low_word + (high_word << 16)
                if value >= 2147483648:
                    value = value - 4294967296
                return value

            def recv_modbus_response(timeout, expected_length=0):
                deadline_ms = int(timeout * 1000)
                elapsed_ms = 0
                buffer = b""

                while elapsed_ms <= deadline_ms:
                    size, val = flange_serial_read(0.05)
                    if size > 0 and val is not None:
                        buffer += val

                        if expected_length > 0 and len(buffer) >= expected_length:
                            return True, buffer[0:expected_length]

                        if len(buffer) >= 3:
                            function_code = buffer[1]
                            if function_code == 3:
                                frame_length = 5 + buffer[2]
                                if len(buffer) >= frame_length:
                                    return True, buffer[0:frame_length]
                            elif function_code == 6 or function_code == 16:
                                if len(buffer) >= 8:
                                    return True, buffer[0:8]

                    wait(0.01)
                    elapsed_ms = elapsed_ms + 60

                return False, buffer

            def tcp_read_exact(size):
                global g_sock
                data = b""
                while len(data) < size:
                    res, chunk = server_socket_read(g_sock, size - len(data), 1.0)
                    if res < 0:
                        return res, None
                    data += chunk
                return len(data), data

            def send_response(command, seq, payload):
                global g_sock
                tx_data = b"GP"
                tx_data += (1).to_bytes(1, byteorder='big')
                tx_data += (command).to_bytes(1, byteorder='big')
                tx_data += (seq).to_bytes(2, byteorder='big')
                tx_data += (len(payload)).to_bytes(2, byteorder='big')
                tx_data += payload
                server_socket_write(g_sock, tx_data)

            def encode_state_payload(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position):
                global g_ready
                torque_flag = 1 if g_ready else 0
                payload = (status).to_bytes(1, byteorder='big')
                payload += (moving & 0xFF).to_bytes(1, byteorder='big')
                payload += (moving_status & 0xFF).to_bytes(1, byteorder='big')
                payload += (torque_flag).to_bytes(1, byteorder='big')
                payload += (present_current & 0xFFFF).to_bytes(2, byteorder='big')
                payload += (present_temperature & 0xFFFF).to_bytes(2, byteorder='big')
                payload += (present_velocity & 0xFFFFFFFF).to_bytes(4, byteorder='big')
                payload += (present_position & 0xFFFFFFFF).to_bytes(4, byteorder='big')
                return payload

            def read_state():
                flange_serial_write(modbus_fc03(ADDR_MOVING_STATUS, 13))
                ok, val = recv_modbus_response(0.5)
                if ok is False or val is None:
                    return False, 0, 0, 0, 0, 0, 0

                if len(val) < 31:
                    return False, 0, 0, 0, 0, 0, 0

                moving_reg = val[3:5]
                current_reg = val[7:9]
                velocity_reg = val[9:13]
                position_reg = val[13:17]
                temperature_reg = val[27:29]

                moving = moving_reg[1]
                moving_status = moving_reg[0]
                present_current = int.from_bytes(current_reg, byteorder='big', signed=True)

                velocity_low = int.from_bytes(velocity_reg[0:2], byteorder='big', signed=False)
                velocity_high = int.from_bytes(velocity_reg[2:4], byteorder='big', signed=False)
                present_velocity = words_to_i32(velocity_low, velocity_high)

                position_low = int.from_bytes(position_reg[0:2], byteorder='big', signed=False)
                position_high = int.from_bytes(position_reg[2:4], byteorder='big', signed=False)
                present_position = words_to_i32(position_low, position_high)

                present_temperature = temperature_reg[1]
                return True, moving, moving_status, present_current, present_temperature, present_velocity, present_position

            def apply_profile_settings():
                flange_serial_write(modbus_fc06(ADDR_GOAL_CURRENT, g_goal_current))
                ok, val = recv_modbus_response(0.3, 8)
                if ok is False:
                    return False

                flange_serial_write(modbus_fc16(ADDR_PROFILE_ACCELERATION, 2, u32_to_words(g_profile_acceleration)))
                ok, val = recv_modbus_response(0.3, 8)
                if ok is False:
                    return False

                flange_serial_write(modbus_fc16(ADDR_PROFILE_VELOCITY, 2, u32_to_words(g_profile_velocity)))
                ok, val = recv_modbus_response(0.3, 8)
                if ok is False:
                    return False

                return True

            def open_serial_port():
                flange_serial_open(
                    baudrate={cfg.baudrate},
                    bytesize=DR_EIGHTBITS,
                    parity=DR_PARITY_NONE,
                    stopbits=DR_STOPBITS_ONE,
                )
                modbus_set_slaveid({cfg.slave_id})

            def reset_serial_port():
                # Recycle the flange serial port to recover from a stuck state
                # left over from a previous DRL session.
                flange_serial_close()
                wait(0.3)
                open_serial_port()

            def initialize_gripper():
                global g_ready
                g_ready = False
                open_serial_port()

                # Some grippers need a moment to wake up after the serial
                # port is reopened. Try modbus torque-enable a few times,
                # recycling the serial port halfway through if needed.
                attempts = 0
                enable_ok = False
                while attempts < 4:
                    flange_serial_write(modbus_fc06(ADDR_TORQUE_ENABLE, 1))
                    ok, val = recv_modbus_response(0.5, 8)
                    if ok is True:
                        enable_ok = True
                        break
                    attempts = attempts + 1
                    if attempts == 2:
                        reset_serial_port()
                    else:
                        wait(0.3)

                if enable_ok is False:
                    close_gripper()
                    return False

                ok = apply_profile_settings()
                if ok is False:
                    close_gripper()
                    return False

                g_ready = True
                return True

            def close_gripper():
                global g_ready
                g_ready = False
                # Best-effort torque off so the next session starts clean.
                flange_serial_write(modbus_fc06(ADDR_TORQUE_ENABLE, 0))
                ok, val = recv_modbus_response(0.2, 8)
                flange_serial_close()

            def wait_until_arrived(goal_position, timeout_ms):
                elapsed_ms = 0
                while True:
                    ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                    if ok is False:
                        return STATUS_IO_ERROR, moving, moving_status, present_current, present_temperature, present_velocity, present_position

                    in_position = (moving_status & 0x01) == 1
                    
                    # 💡 핵심 수정: 단순히 도착했냐 뿐만 아니라, 
                    # 1) 멈췄고 (moving == 0)
                    # 2) 전류(힘)가 세팅한 목표 전류(g_goal_current)의 90% 이상 도달했다면 
                    # 물체를 꽉 잡은 것으로 간주하고 정상(OK) 리턴!
                    is_grasping = (abs(present_current) >= (g_goal_current * 0.9))
                    
                    if moving == 0 and (in_position or abs(goal_position - present_position) <= POSITION_TOLERANCE or is_grasping):
                        return STATUS_OK, moving, moving_status, present_current, present_temperature, present_velocity, present_position

                    if timeout_ms > 0 and elapsed_ms >= timeout_ms:
                        return STATUS_TIMEOUT, moving, moving_status, present_current, present_temperature, present_velocity, present_position

                    wait(POLL_WAIT_SEC)
                    elapsed_ms = elapsed_ms + int(POLL_WAIT_SEC * 1000)

            def reopen_socket():
                global g_sock
                if g_sock is not None:
                    server_socket_close(g_sock)
                g_sock = server_socket_open({cfg.tcp_port})

            def handle_initialize(command, seq, payload):
                global g_goal_current
                global g_ready
                if len(payload) >= 2:
                    g_goal_current = int.from_bytes(payload[0:2], byteorder='big', signed=False)

                if g_ready is False:
                    ok = initialize_gripper()
                else:
                    ok = apply_profile_settings()
                    g_ready = ok

                # If we still couldn't bring the gripper up, return immediately
                # rather than attempting read_state() on a closed serial port.
                if ok is False:
                    send_response(
                        command,
                        seq,
                        encode_state_payload(STATUS_IO_ERROR, 0, 0, 0, 0, 0, 0),
                    )
                    return

                state_ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                status = STATUS_OK if state_ok else STATUS_IO_ERROR
                send_response(
                    command,
                    seq,
                    encode_state_payload(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position),
                )

            def handle_set_config(command, seq, payload):
                global g_goal_current
                global g_profile_velocity
                global g_profile_acceleration

                if len(payload) != 10:
                    send_response(command, seq, encode_state_payload(STATUS_BAD_PACKET, 0, 0, 0, 0, 0, 0))
                    return

                g_goal_current = int.from_bytes(payload[0:2], byteorder='big', signed=False)
                g_profile_velocity = int.from_bytes(payload[2:6], byteorder='big', signed=False)
                g_profile_acceleration = int.from_bytes(payload[6:10], byteorder='big', signed=False)

                # Stash the values regardless of torque state; they will be
                # applied to hardware on the next torque-on or move command.
                if g_ready is False:
                    state_ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                    status = STATUS_OK if state_ok else STATUS_IO_ERROR
                    send_response(
                        command,
                        seq,
                        encode_state_payload(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position),
                    )
                    return

                ok = apply_profile_settings()
                state_ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                status = STATUS_OK if ok and state_ok else STATUS_IO_ERROR
                send_response(
                    command,
                    seq,
                    encode_state_payload(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position),
                )

            def handle_read_like(command, seq):
                if g_ready is False:
                    send_response(command, seq, encode_state_payload(STATUS_NOT_READY, 0, 0, 0, 0, 0, 0))
                    return
                ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                status = STATUS_OK if ok else STATUS_IO_ERROR
                send_response(
                    command,
                    seq,
                    encode_state_payload(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position),
                )

            def handle_read_state(command, seq):
                ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                status = STATUS_OK if ok else STATUS_IO_ERROR
                send_response(
                    command,
                    seq,
                    encode_state_payload(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position),
                )

            def handle_set_torque(command, seq, payload):
                global g_ready
                if len(payload) != 2:
                    send_response(command, seq, encode_state_payload(STATUS_BAD_PACKET, 0, 0, 0, 0, 0, 0))
                    return

                enable_value = int.from_bytes(payload[0:2], byteorder='big', signed=False)
                enable = enable_value != 0

                if enable:
                    flange_serial_write(modbus_fc06(ADDR_TORQUE_ENABLE, 1))
                    ok, val = recv_modbus_response(0.3, 8)
                    if ok is False:
                        send_response(command, seq, encode_state_payload(STATUS_IO_ERROR, 0, 0, 0, 0, 0, 0))
                        return
                    profile_ok = apply_profile_settings()
                    g_ready = profile_ok
                    state_ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                    status = STATUS_OK if profile_ok and state_ok else STATUS_IO_ERROR
                    send_response(
                        command,
                        seq,
                        encode_state_payload(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position),
                    )
                    return

                flange_serial_write(modbus_fc06(ADDR_TORQUE_ENABLE, 0))
                ok, val = recv_modbus_response(0.3, 8)
                g_ready = False
                if ok is False:
                    send_response(command, seq, encode_state_payload(STATUS_IO_ERROR, 0, 0, 0, 0, 0, 0))
                    return
                state_ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                status = STATUS_OK if state_ok else STATUS_IO_ERROR
                send_response(
                    command,
                    seq,
                    encode_state_payload(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position),
                )

            def handle_move(command, seq, payload):
                if g_ready is False:
                    send_response(command, seq, encode_state_payload(STATUS_NOT_READY, 0, 0, 0, 0, 0, 0))
                    return

                if len(payload) != 8:
                    send_response(command, seq, encode_state_payload(STATUS_BAD_PACKET, 0, 0, 0, 0, 0, 0))
                    return

                goal_position = int.from_bytes(payload[0:4], byteorder='big', signed=False)
                timeout_ms = int.from_bytes(payload[4:8], byteorder='big', signed=False)

                if goal_position < 0 or goal_position > 1150:
                    send_response(command, seq, encode_state_payload(STATUS_RANGE_ERROR, 0, 0, 0, 0, 0, 0))
                    return

                if apply_profile_settings() is False:
                    send_response(command, seq, encode_state_payload(STATUS_IO_ERROR, 0, 0, 0, 0, 0, 0))
                    return

                flange_serial_write(modbus_fc16(ADDR_GOAL_POSITION, 2, u32_to_words(goal_position)))
                ok, val = recv_modbus_response(0.3, 8)
                if ok is False:
                    send_response(command, seq, encode_state_payload(STATUS_IO_ERROR, 0, 0, 0, 0, 0, 0))
                    return

                status, moving, moving_status, present_current, present_temperature, present_velocity, present_position = wait_until_arrived(goal_position, timeout_ms)
                send_response(
                    command,
                    seq,
                    encode_state_payload(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position),
                )

            # Open the TCP server and start serving commands IMMEDIATELY.
            # We deliberately do NOT auto-initialize the gripper here:
            # blocking the command loop on a slow/failing serial init would
            # cause the host's INITIALIZE request to time out. The host
            # always sends INITIALIZE explicitly, which calls
            # initialize_gripper() inside handle_initialize() and gets a
            # proper STATUS_OK / STATUS_IO_ERROR response.
            g_sock = server_socket_open({cfg.tcp_port})

            while True:
                res, header = tcp_read_exact(HEADER_SIZE)
                if res < 0:
                    reopen_socket()
                    continue

                if header[0:2] != b"GP":
                    continue

                version = header[2]
                command = header[3]
                seq = int.from_bytes(header[4:6], byteorder='big', signed=False)
                payload_size = int.from_bytes(header[6:8], byteorder='big', signed=False)

                if version != 1:
                    send_response(command, seq, encode_state_payload(STATUS_BAD_PACKET, 0, 0, 0, 0, 0, 0))
                    continue

                payload = b""
                if payload_size > 0:
                    res, payload = tcp_read_exact(payload_size)
                    if res < 0:
                        reopen_socket()
                        continue

                if command == CMD_PING:
                    handle_read_like(command, seq)
                elif command == CMD_INITIALIZE:
                    handle_initialize(command, seq, payload)
                elif command == CMD_SET_CONFIG:
                    handle_set_config(command, seq, payload)
                elif command == CMD_MOVE:
                    handle_move(command, seq, payload)
                elif command == CMD_READ_STATE:
                    handle_read_state(command, seq)
                elif command == CMD_SET_TORQUE:
                    handle_set_torque(command, seq, payload)
                elif command == CMD_SHUTDOWN:
                    handle_read_like(command, seq)
                    break
                else:
                    send_response(command, seq, encode_state_payload(STATUS_BAD_COMMAND, 0, 0, 0, 0, 0, 0))

            if g_sock is not None:
                server_socket_close(g_sock)
            close_gripper()
            """
        ).strip()


def build_service_root(namespace: str, service_prefix: str = "dsr_controller2") -> str:
    normalized_namespace = namespace.strip("/")
    normalized_prefix = service_prefix.strip("/")
    if normalized_prefix:
        return f"/{normalized_namespace}/{normalized_prefix}"
    return f"/{normalized_namespace}"
