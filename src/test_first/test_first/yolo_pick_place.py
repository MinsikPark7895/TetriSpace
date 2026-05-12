#!/usr/bin/env python3
"""
yolo_pick_place.py

dakae_helper.py 에서 생성한 /tmp/dakae_task.json 을 읽어
YOLO8 탐지 좌표 기반 픽앤플레이스를 실행합니다.

config 포맷:
{
  "objects": [
    {"label": "toy",    "pick_x": 100.0, "pick_y": 50.0,  "pick_z": 200.0, "dest_type": "home"},
    {"label": "bottle", "pick_x": 200.0, "pick_y": -30.0, "pick_z": 180.0,
     "dest_type": "custom", "dest_x": 300.0, "dest_y": 100.0, "dest_z": 200.0}
  ]
}
"""

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.rcutils_logger import RcutilsLogger
import DR_init

from dsr_gripper_tcp_interfaces.action import SafeGrasp

if not hasattr(RcutilsLogger, "info"):
    RcutilsLogger.info = RcutilsLogger.dinfo

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

CONFIG_PATH = "/tmp/dakae_task.json"

VEL = 20
ACC = 20
Z_SAFE = 400.0
GRIPPER_OFFSET = 85.0
HOME_PLACE_X = 373.0
HOME_PLACE_Y = 0.0
HOME_PLACE_Z = 200.0
GRASP_POSITION = 800


class GripperHelper:
    """별도 스레드에서 dsr_gripper_tcp SafeGrasp 액션만 사용하는 헬퍼."""

    def __init__(self):
        self.node = rclpy.create_node("yolo_gripper_helper")
        self._safe_grasp = ActionClient(self.node, SafeGrasp, "/gripper_service/safe_grasp")

        self.node.get_logger().info("그리퍼 액션 서버 연결 대기 중...")
        if not self._safe_grasp.wait_for_server(timeout_sec=10.0):
            self.node.destroy_node()
            raise RuntimeError("그리퍼 액션 서버에 연결할 수 없습니다. gripper_service_node가 실행 중인지 확인하세요.")
        self.node.get_logger().info("그리퍼 액션 연결 완료")

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def open(self):
        self._send_goal(0)

    def grasp(self, target_position=GRASP_POSITION):
        return self._send_goal(target_position)

    def _send_goal(self, target_position):
        goal = SafeGrasp.Goal()
        goal.target_position = target_position
        goal.max_current = 400
        goal.current_delta_threshold = 0
        goal.timeout_sec = 10.0

        goal_event = threading.Event()
        goal_holder = [None]

        def on_goal_response(f):
            goal_holder[0] = f.result()
            goal_event.set()

        send_future = self._safe_grasp.send_goal_async(goal)
        send_future.add_done_callback(on_goal_response)

        if not goal_event.wait(timeout=15.0):
            raise RuntimeError("SafeGrasp: goal response timeout")

        gh = goal_holder[0]
        if not gh or not gh.accepted:
            raise RuntimeError("SafeGrasp goal rejected")

        result_event = threading.Event()
        result_holder = [None]

        def on_result(f):
            result_holder[0] = f.result()
            result_event.set()

        gh.get_result_async().add_done_callback(on_result)

        if not result_event.wait(timeout=30.0):
            raise RuntimeError("SafeGrasp: result timeout")

        return result_holder[0].result


class YoloPickPlaceNode(Node):
    def __init__(self):
        super().__init__("yolo_pick_place_node", namespace=ROBOT_ID)


def main(args=None):
    try:
        with open(CONFIG_PATH, "r") as f:
            config = json.load(f)
    except Exception as e:
        print(f"[오류] config 파일 읽기 실패 ({CONFIG_PATH}): {e}")
        return

    tasks = config.get("objects", [])
    if not tasks:
        print("[오류] 실행할 작업이 없습니다.")
        return

    rclpy.init(args=args)

    try:
        gripper = GripperHelper()
    except RuntimeError as e:
        print(f"[오류] {e}")
        rclpy.shutdown()
        return

    node = YoloPickPlaceNode()
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import wait, movej, movel, get_current_posx, DR_BASE, DR_MV_MOD_ABS
        from DR_common2 import posj, posx
    except ImportError as e:
        node.get_logger().error(f"DSR import 실패: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    home = posj(0, 0, 90, 0, 90, 0)

    def go_home():
        node.get_logger().info("홈으로 이동")
        movej(home, VEL, ACC)
        wait(0.5)

    go_home()
    home_pos, _ = get_current_posx(ref=DR_BASE)
    HOME_RX, HOME_RY, HOME_RZ = home_pos[3], home_pos[4], home_pos[5]

    def move_to(name, x, y, z):
        pose = posx(x, y, z, HOME_RX, HOME_RY, HOME_RZ)
        node.get_logger().info(f"[이동] {name}  x={x:.1f} y={y:.1f} z={z:.1f}")
        movel(pose, VEL, ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    try:
        for i, task in enumerate(tasks, 1):
            label = task["label"]
            pick_x = task["pick_x"]
            pick_y = task["pick_y"]
            pick_z = task["pick_z"] + GRIPPER_OFFSET
            dest_type = task["dest_type"]

            if dest_type == "home":
                place_x, place_y, place_z = HOME_PLACE_X, HOME_PLACE_Y, HOME_PLACE_Z
            else:
                place_x = task["dest_x"]
                place_y = task["dest_y"]
                place_z = task["dest_z"]

            node.get_logger().info(
                f"\n{'='*50}\n"
                f"  [{i}/{len(tasks)}] {label}\n"
                f"  픽:      x={pick_x:.1f}  y={pick_y:.1f}  z={pick_z:.1f}\n"
                f"  플레이스: x={place_x:.1f}  y={place_y:.1f}  z={place_z:.1f}\n"
                f"{'='*50}"
            )

            move_to("픽 XY 이동", pick_x, pick_y, Z_SAFE)
            gripper.open()
            wait(0.5)
            move_to("Z 하강 (픽)", pick_x, pick_y, pick_z)
            wait(0.3)
            result = gripper.grasp()
            node.get_logger().info(
                f"파지 결과: {'감지됨' if result.grasp_detected else '미감지'}"
            )
            wait(0.5)
            move_to("Z 상승", pick_x, pick_y, Z_SAFE)
            wait(0.3)

            move_to("플레이스 XY 이동", place_x, place_y, Z_SAFE)
            wait(0.3)
            move_to("Z 하강 (플레이스)", place_x, place_y, place_z)
            wait(0.3)
            gripper.open()
            wait(0.5)
            move_to("Z 상승 (플레이스 후)", place_x, place_y, Z_SAFE)
            wait(0.3)

            go_home()

        node.get_logger().info("✅ 모든 작업 완료")

    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C — 중단됨")
    except Exception as e:
        node.get_logger().error(f"오류: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
