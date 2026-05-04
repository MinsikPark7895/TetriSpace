#!/usr/bin/env python3
"""
vision_arm_test.py

동작 순서 (반복):
  물체 인식 → XY 이동 → 그리퍼 열기 → Z 하강 → 그리퍼 닫기
  → Z 상승 → 플레이스 XY 이동 → Z 하강 → 그리퍼 열기 → Z 상승 → 홈

사용:
  ros2 run test_first vision_arm_test
"""

import rclpy
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
import DR_init

from dakae_interfaces.srv import DetectObjects
from dakae_interfaces.srv import MoveGripper

if not hasattr(RcutilsLogger, "info"):
    RcutilsLogger.info = RcutilsLogger.dinfo

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ══════════════════════════════════════════════════════════════
# 조정 파라미터
# ══════════════════════════════════════════════════════════════
VEL = 20
ACC = 20

# Z 높이 (mm) — 실제 로봇에 맞게 조정
Z_SAFE        = 400.0   # XY 이동 시 안전 높이
GRIPPER_OFFSET = 85.0   # 비전Z(75) + 85 = 160mm  # 비전 Z + 이 값 = 실제 잡는 TCP 높이
                        # 로봇이 너무 높으면 줄이고, 너무 낮으면 늘림

# 임시 플레이스 위치 (mm) — 물체를 내려놓을 곳
PLACE_X = 373.0
PLACE_Y = 0.0
PLACE_Z = 200.0         # 물체 놓을 높이, 실제 환경에 맞게 조정

# 탐지 라벨
TARGET_LABELS = ["toy", "bottle", "box", "cube"]
# ══════════════════════════════════════════════════════════════


class VisionArmTestNode(Node):
    def __init__(self):
        super().__init__("vision_arm_test_node", namespace=ROBOT_ID)

        self.vision_client = self.create_client(DetectObjects, "/detect_objects")
        self.gripper_client = self.create_client(MoveGripper, "gripper_move")

        self.get_logger().info("서비스 연결 대기 중...")
        while not self.vision_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/detect_objects 대기 중...")
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("gripper_move 대기 중...")
        self.get_logger().info("서비스 연결 완료")

    def detect(self):
        req = DetectObjects.Request()
        req.target_labels = TARGET_LABELS
        future = self.vision_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def gripper(self, stroke: int):
        req = MoveGripper.Request()
        req.stroke = stroke
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)


def main(args=None):
    rclpy.init(args=args)
    node = VisionArmTestNode()
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

    # 홈 도착 후 rx, ry, rz 고정값 읽기
    go_home()
    home_pos, _ = get_current_posx(ref=DR_BASE)
    HOME_RX = home_pos[3]
    HOME_RY = home_pos[4]
    HOME_RZ = home_pos[5]
    node.get_logger().info(f"홈 자세 고정: rx={HOME_RX:.1f} ry={HOME_RY:.1f} rz={HOME_RZ:.1f}")

    def move_to(name, x, y, z):
        pose = posx(x, y, z, HOME_RX, HOME_RY, HOME_RZ)
        node.get_logger().info(f"[이동] {name}  x={x:.1f} y={y:.1f} z={z:.1f}")
        movel(pose, VEL, ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    def open_gripper():
        node.get_logger().info("그리퍼 열기")
        node.gripper(0)
        wait(0.5)

    def close_gripper():
        node.get_logger().info("그리퍼 닫기 (잡기)")
        node.gripper(500)
        wait(0.8)

    try:
        # ── 1. 물체 인식 (1회) ────────────────────────────────────
        node.get_logger().info("── YOLO8 탐지 ──")
        res = node.detect()

        if res is None or not res.success or len(res.detected_objects) == 0:
            node.get_logger().info("탐지된 물체 없음, 종료")
        else:
            node.get_logger().info(f"탐지 {len(res.detected_objects)}개:")
            for obj in res.detected_objects:
                node.get_logger().info(
                    f"  [{obj.label}] x={obj.position.x*1000:.1f} "
                    f"y={obj.position.y*1000:.1f} z={obj.position.z*1000:.1f} mm"
                )

            # ── 2. 탐지된 물체 순서대로 픽앤플레이스 ──────────────
            for obj in res.detected_objects:
                x       = obj.position.x * 1000.0
                y       = obj.position.y * 1000.0
                z_grasp = obj.position.z * 1000.0 + GRIPPER_OFFSET

                node.get_logger().info(
                    f"\n{'='*45}\n"
                    f"  [{obj.label}]  x={x:.1f} y={y:.1f} z_grasp={z_grasp:.1f}\n"
                    f"{'='*45}"
                )

                # XY 이동 (안전 높이)
                move_to("XY 이동", x, y, Z_SAFE)
                wait(0.3)

                # 그리퍼 열기
                open_gripper()

                # Z 하강 (물체 위치)
                move_to("Z 하강 (픽)", x, y, z_grasp)
                wait(0.3)

                # 그리퍼 닫기
                close_gripper()

                # Z 상승
                move_to("Z 상승", x, y, Z_SAFE)
                wait(0.3)

                # 플레이스 XY 이동
                move_to("플레이스 XY 이동", PLACE_X, PLACE_Y, Z_SAFE)
                wait(0.3)

                # Z 하강 (내려놓기)
                move_to("Z 하강 (플레이스)", PLACE_X, PLACE_Y, PLACE_Z)
                wait(0.3)

                # 그리퍼 열기 (놓기)
                open_gripper()

                # Z 상승
                move_to("Z 상승 (플레이스 후)", PLACE_X, PLACE_Y, Z_SAFE)
                wait(0.3)

                # 홈 복귀
                go_home()

            node.get_logger().info("✅ 모든 물체 완료")

    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C — 종료")
    except Exception as e:
        node.get_logger().error(f"오류: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
