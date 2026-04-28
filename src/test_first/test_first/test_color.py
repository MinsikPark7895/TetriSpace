#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
import DR_init

from dakae_interfaces.srv import DetectCubes
from dakae_interfaces.srv import MoveGripper

if not hasattr(RcutilsLogger, "info"):
    RcutilsLogger.info = RcutilsLogger.dinfo

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_place_node", namespace=ROBOT_ID)

        # 비전 서비스 클라이언트
        self.vision_client = self.create_client(DetectCubes, "/detect_cubes")

        # 그리퍼 서비스 클라이언트
        self.gripper_client = self.create_client(MoveGripper, "gripper_move")

        self.get_logger().info("서비스 연결 대기 중...")

        while not self.vision_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("detect_cubes 서비스 대기 중...")

        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("gripper_move 서비스 대기 중...")

        self.get_logger().info("모든 서비스 연결 완료")

    def move_gripper(self, stroke: int):
        req = MoveGripper.Request()
        req.stroke = stroke
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(f"그리퍼 이동 실패 (stroke={stroke})")


def main(args=None):
    rclpy.init(args=args)

    node = PickPlaceNode()
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
        vel = 100
        acc = 100

        home = posj(0, 0, 90, 0, 90, 0)

        APPROACH_OFFSET_Z = 100.0
        # 이 값은 그리퍼 오프셋이 이미 포함된 TCP 기준 절댓값입니다.
        # 따라서 별도의 그리퍼 길이/오프셋 보정을 다시 더하지 않습니다.
        FIXED_TCP_Z = 145.0

        # 색상별 배치 위치
        # 실제 환경에 맞게 좌표 수정 필요
        PLACE_POS_1 = (249.640, -148.480, FIXED_TCP_Z)   # red
        PLACE_POS_2 = (250.980, 269.550, FIXED_TCP_Z)   # blue
        PLACE_POS_3 = (421.830, -148.280, FIXED_TCP_Z)   # white

        def go_home():
            node.get_logger().info("홈 위치로 이동")
            movej(home, vel, acc)

        def get_current_orientation_base():
            current_pos, sol = get_current_posx(ref=DR_BASE)
            return current_pos[3], current_pos[4], current_pos[5]

        def make_target_pos(x, y, z):
            rx, ry, rz = get_current_orientation_base()
            return posx(x, y, z, rx, ry, rz)

        def make_approach_pos(x, y, z):
            rx, ry, rz = get_current_orientation_base()
            return posx(x, y, z + APPROACH_OFFSET_Z, rx, ry, rz)

        def move_linear(name, pose):
            node.get_logger().info(f"{name} 이동")
            movel(pose, vel, acc, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        def pick_cycle(x, y, z):
            approach = make_approach_pos(x, y, z)
            target = make_target_pos(x, y, z)

            node.move_gripper(0)
            wait(0.5)

            move_linear("픽 접근 위치", approach)
            wait(0.5)

            move_linear("픽 위치", target)
            wait(0.5)

            node.move_gripper(500)
            wait(0.5)

            move_linear("픽 접근 위치 복귀", approach)
            wait(0.5)

        def place_cycle(x, y, z):
            approach = make_approach_pos(x, y, z)
            target = make_target_pos(x, y, z)

            move_linear("플레이스 접근 위치", approach)
            wait(0.5)

            move_linear("플레이스 위치", target)
            wait(0.5)

            node.move_gripper(0)
            wait(0.5)

            move_linear("플레이스 접근 위치 복귀", approach)
            wait(0.5)

        def get_place_position_by_color(color):
            color = color.lower()

            if color == "red":
                return PLACE_POS_1
            elif color == "blue":
                return PLACE_POS_2
            elif color == "white":
                return PLACE_POS_3
            else:
                raise RuntimeError(f"지원하지 않는 색상입니다: {color}")

        # 비전 요청
        req = DetectCubes.Request()
        req.target_colors = []

        future = node.vision_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()

        if response is None:
            raise RuntimeError("비전 응답 없음")
        if not response.success:
            raise RuntimeError(f"비전 실패: {response.message}")
        if len(response.detected_cubes) == 0:
            raise RuntimeError("큐브 없음")

        node.get_logger().info(f"검출된 큐브 개수: {len(response.detected_cubes)}")

        go_home()

        for i, cube in enumerate(response.detected_cubes, start=1):
            x = cube.position.x * 1000.0
            y = cube.position.y * 1000.0
            z = FIXED_TCP_Z

            node.get_logger().info(
                f"[{i}] {cube.color} cube -> x={x:.2f}, y={y:.2f}, z={z:.2f} (fixed)"
            )

            place_x, place_y, place_z = get_place_position_by_color(cube.color)

            node.get_logger().info(
                f"{cube.color} cube -> place x={place_x:.2f}, y={place_y:.2f}, z={place_z:.2f}"
            )

            pick_cycle(x, y, z)
            place_cycle(place_x, place_y, place_z)
            go_home()

        go_home()

    except Exception as e:
        node.get_logger().error(f"작업 중 오류: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
