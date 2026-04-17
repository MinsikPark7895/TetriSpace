#!/usr/bin/env python3
import textwrap
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dsr_msgs2.srv import DrlStart
#from std_srvs.srv import Trigger  # 값 전달 없이 버튼만 누르는 트리거 서비스
from dakae_interfaces.srv import MoveGripper

if not hasattr(RcutilsLogger, "info"):
    RcutilsLogger.info = RcutilsLogger.dinfo

DRL_FUNCTIONS = """
g_slaveid = 1

def modbus_send_make(data):
    crc = 0xFFFF
    for b in bytearray(data):
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    data += (crc & 0xFF).to_bytes(1, byteorder='big')
    data += ((crc >> 8) & 0xFF).to_bytes(1, byteorder='big')
    return data

def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid

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
"""

class GripperServerNode(Node):
    def __init__(self):
        super().__init__('gripper_server_node', namespace='dsr01')
        self.drl_timeout_sec = 10.0
        
        # 데드락 방지용 멀티스레드 콜백 그룹
        self.cb_group = ReentrantCallbackGroup()
        
        # 두산 로봇 DRL 클라이언트
        self.cli = self.create_client(DrlStart, "drl/drl_start", callback_group=self.cb_group)
        
        # 그리퍼 이동 서비스
        self.srv_move = self.create_service(MoveGripper, 'gripper_move', self.gripper_move_callback, callback_group=self.cb_group)

        # 여기서 무한 대기하면 executor가 돌기 전에 노드 생성이 멈춰서
        # 외부에서는 서비스가 보이는데 실제 응답은 못 하는 상태가 될 수 있다.
        self.get_logger().info("gripper_move 서버 생성 완료. DRL 서비스는 요청 시점에 확인합니다.")
            
        # 서버가 켜질 때 초기화 로직 1회 실행 (데드락 방지를 위해 주석 처리 후 main으로 이동)
        # self.initialize_gripper()
        
    def _send_drl_script_sync(self, code):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("❌ drl/drl_start 서비스가 준비되지 않았습니다.")
            return False

        req = DrlStart.Request()
        req.robot_system = 0
        req.code = code
        future = self.cli.call_async(req)

        self.get_logger().info("DRL 스크립트 제출 후 응답 대기 중...")
        deadline = time.monotonic() + self.drl_timeout_sec

        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            self.cli.remove_pending_request(future)
            self.get_logger().error(
                f"❌ drl/drl_start 응답 타임아웃 ({self.drl_timeout_sec:.1f}초)"
            )
            return False

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ drl/drl_start 호출 예외: {e}")
            return False

        if res is None:
            self.get_logger().error("❌ drl/drl_start 응답이 비어 있습니다.")
            return False

        self.get_logger().info(f"DRL 응답 수신: success={res.success}")
        return res.success

    def initialize_gripper(self):
        drl_body = """
def gripper_init():
    res = -1
    for _ in range(3):
        res = flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
        if res == 0: break
        wait(0.2)
    if res != 0: return
    wait(0.5)
    modbus_set_slaveid(1)
    
    flange_serial_write(modbus_fc06(256, 1))
    wait(0.3)
    flange_serial_write(modbus_fc06(275, 400))
    wait(0.3)
    flange_serial_close()

gripper_init()
"""
        code = textwrap.dedent(f"{DRL_FUNCTIONS}\n{drl_body}")
        if self._send_drl_script_sync(code):
            self.get_logger().info("초기화 명령 전송 성공! (그리퍼가 벌어지며 가장 넓은 너비를 인식합니다)")
            self.get_logger().info("물리적 영점이 완료될 때까지 안전하게 5초 대기합니다...")
            time.sleep(5.0) 
            self.get_logger().info("✅ 서버 부팅 및 영점 잡기 완료! 언제든 명령을 받을 수 있습니다.")
        else:
            self.get_logger().error("❌ 초기화 실패!")

    # ====== 이동 요청이 왔을 때 실행 ======
    def gripper_move_callback(self, request, response):
        self.get_logger().info(f"[오더 수신] 👉 그리퍼 이동 명령 (Stroke: {request.stroke})")
        return self._execute_stroke(stroke=request.stroke, action_str=f"Move({request.stroke})", response=response)

    # 실제 로봇으로 이동 명령을 쏘는 공통 단위
    def _execute_stroke(self, stroke, action_str, response):
        drl_body = f"""
def gripper_action():
    res = -1
    for _ in range(3):
        res = flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
        if res == 0: break
        wait(0.2)
    if res != 0: return
    wait(0.5)
    modbus_set_slaveid(1)
    
    flange_serial_write(modbus_fc16(282, 2, [{int(stroke)}, 0]))
    wait(0.1)
    flange_serial_write(modbus_fc16(282, 2, [{int(stroke)}, 0]))
    wait(0.1)
    flange_serial_write(modbus_fc16(282, 2, [{int(stroke)}, 0]))
    wait(0.1)
    flange_serial_write(modbus_fc16(282, 2, [{int(stroke)}, 0]))

    
    wait(2.5) 
    flange_serial_close()

gripper_action()
"""
        code = textwrap.dedent(f"{DRL_FUNCTIONS}\n{drl_body}")
        if self._send_drl_script_sync(code):
            # [방법 B] DRL은 제출 즉시 리턴되고 물리 동작은 비동기로 진행됨.
            # 그리퍼가 실제로 완료될 때까지 충분히 대기한 후 응답을 돌려줌.
            # DRL 내부 타이밍 합계 ≈ 4.1s → 안전 마진 포함하여 4.5s 대기
            # (추후 방법 A: Modbus Read로 현재 위치 확인 후 응답하는 방식으로 교체 예정)
            self.get_logger().info(f"⏳ [{action_str}] DRL 제출 완료. 물리 동작 완료 대기 중... (4.5초)")
            time.sleep(4.5)
            self.get_logger().info(f"👍 [{action_str}] 물리 동작 완료! 의뢰인에게 완료 회신!")
            response.success = True
            response.message = f"{action_str} Success"
        else:
            self.get_logger().error(f"❌ [{action_str}] 수행 실패!")
            response.success = False
            response.message = f"{action_str} Failed"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperServerNode()
    
    # 1. 수신 엔진(spin)이 안 돌면 응답을 못 받아서 굳어버리므로, 별도의 백그라운드 스레드로 켜둡니다!
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # 2. 엔진이 켜졌으니, 이제 맘 편히 초기화를 실행해서 응답을 받아옵니다!
    node.get_logger().info("통신 수신 엔진 가동 완료! 초기화를 시작합니다...")
    node.initialize_gripper()
    
    try:
        # 서버가 꺼지지 않게 메인 스레드는 살려둡니다.
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
