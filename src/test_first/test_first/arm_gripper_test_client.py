#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
import DR_init

from dakae_interfaces.srv import MoveGripper

if not hasattr(RcutilsLogger, "info"):
    RcutilsLogger.info = RcutilsLogger.dinfo

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class ArmGripperTestNode(Node):
    def __init__(self):
        super().__init__("arm_gripper_test_node", namespace=ROBOT_ID)

        self.gripper_client = self.create_client(MoveGripper, "gripper_move")

        self.get_logger().info("그리퍼 서비스 연결 대기 중...")
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("gripper_move 서비스 대기 중...")

        self.get_logger().info("그리퍼 서비스 연결 완료")

    def move_gripper(self, stroke: int):
        req = MoveGripper.Request()
        req.stroke = stroke
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(f"그리퍼 이동 실패 (stroke={stroke})")

        self.get_logger().info(f"그리퍼 이동 완료 (stroke={stroke})")


def main(args=None):
    rclpy.init(args=args)

    node = ArmGripperTestNode()
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            wait,
            movej,
            movel,
            get_current_posx,
            DR_BASE,
            DR_MV_MOD_ABS,
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        node.get_logger().error(f"DSR import 실패: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        vel = 30
        acc = 30

        home = posj(0, 0, 90, 0, 90, 0)
        approach_offset_z = 100.0

        # 실제 셋업에 맞게 x, y, z 값을 조정해서 테스트합니다.
        test_points = [
            {"name": "center", "x": 370.0, "y": 0.0, "z": 320.0, "close_stroke": 500},
            {"name": "left", "x": 370.0, "y": 120.0, "z": 320.0, "close_stroke": 700},
            {"name": "right", "x": 370.0, "y": -120.0, "z": 320.0, "close_stroke": 300},
        ]

        def go_home():
            node.get_logger().info("홈 위치로 이동")
            movej(home, vel, acc)
            wait(0.5)

        def get_current_orientation_base():
            current_pos, _ = get_current_posx(ref=DR_BASE)
            return current_pos[3], current_pos[4], current_pos[5]

        def make_target_pos(x, y, z):
            rx, ry, rz = get_current_orientation_base()
            return posx(x, y, z, rx, ry, rz)

        def make_approach_pos(x, y, z):
            rx, ry, rz = get_current_orientation_base()
            return posx(x, y, z + approach_offset_z, rx, ry, rz)

        def move_linear(name, pose):
            node.get_logger().info(f"{name} 이동")
            movel(pose, vel, acc, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        def run_test_point(name, x, y, z, close_stroke):
            approach = make_approach_pos(x, y, z)
            target = make_target_pos(x, y, z)

            node.get_logger().info(
                f"[{name}] 테스트 시작 x={x:.1f}, y={y:.1f}, z={z:.1f}, close={close_stroke}"
            )

            node.move_gripper(0)
            wait(0.5)

            move_linear(f"[{name}] 접근 위치", approach)
            wait(0.5)

            move_linear(f"[{name}] 작업 위치", target)
            wait(0.5)

            node.move_gripper(500)
            wait(0.5)

            node.move_gripper(0)
            wait(0.5)

            move_linear(f"[{name}] 접근 위치 복귀", approach)
            wait(0.5)

        go_home()

        for point in test_points:
            run_test_point(
                point["name"],
                point["x"],
                point["y"],
                point["z"],
                point["close_stroke"],
            )
            go_home()

        node.get_logger().info("로봇팔/그리퍼 테스트 완료")

    except Exception as e:
        node.get_logger().error(f"테스트 중 오류: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
