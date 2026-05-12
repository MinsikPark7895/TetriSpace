#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
import DR_init

from dakae_interfaces.srv import DetectMarkers
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
        self.service_call_timeout_sec = 15.0

        # 마커 검출 서비스 클라이언트
        self.marker_client = self.create_client(DetectMarkers, "/detect_markers")

        # 그리퍼 서비스 클라이언트
        self.gripper_client = self.create_client(MoveGripper, "gripper_move")

        self.get_logger().info("서비스 연결 대기 중...")

        while not self.marker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("detect_markers 서비스 대기 중...")

        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("gripper_move 서비스 대기 중...")

        self.get_logger().info("모든 서비스 연결 완료")

    def move_gripper(self, stroke: int):
        req = MoveGripper.Request()
        req.stroke = stroke
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.service_call_timeout_sec,
        )

        if not future.done():
            self.gripper_client.remove_pending_request(future)
            raise TimeoutError(
                f"그리퍼 응답 타임아웃 ({self.service_call_timeout_sec:.1f}초, stroke={stroke})"
            )

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

        # 픽/플레이스 높이
        # 실제 환경에 맞게 조정하세요.
        PICK_TCP_Z = 135.0
        PLACE_TCP_Z = 135.0

        # cube n -> target i
        # 예시: cube 1을 target 4로, cube 2를 target 1로 보냄
        task_pairs = [
            (2, 2),
            (3, 3),
            (4, 4),
        ]

        def go_home():
            node.get_logger().info("홈 위치로 이동")
            movej(home, vel, acc)

        def get_current_orientation_base():
            current_pos, _ = get_current_posx(ref=DR_BASE)
            return current_pos[3], current_pos[4], current_pos[5]

        def normalize_angle_deg(angle):
            normalized = (angle + 180.0) % 360.0 - 180.0
            if normalized == -180.0:
                return 180.0
            return normalized

        def choose_shortest_grasp_rz(marker_rz, current_rz):
            # 180도 대칭 파지가 가능하다고 보고 현재 rz에서 가장 적게 도는 각도를 선택
            candidates = [marker_rz - 180.0, marker_rz, marker_rz + 180.0]
            return min(
                candidates,
                key=lambda candidate: abs(
                    normalize_angle_deg(candidate - current_rz)
                ),
            )

        # 기준 자세를 유지해서 목표 자세 생성
        def make_target_pos(x, y, z, orientation=None):
            rx, ry, rz = orientation or get_current_orientation_base()
            return posx(x, y, z, rx, ry, rz)

        # 기준 자세를 유지해서 접근 자세 생성
        def make_approach_pos(x, y, z, orientation=None):
            rx, ry, rz = orientation or get_current_orientation_base()
            return posx(x, y, z + APPROACH_OFFSET_Z, rx, ry, rz)

        # 기준 자세의 rx, ry를 유지하고 rz만 새 값으로 변경
        def make_target_pos_with_rz(x, y, z, new_rz, orientation=None):
            rx, ry, _ = orientation or get_current_orientation_base()
            return posx(x, y, z, rx, ry, new_rz)

        # 기준 자세의 rx, ry를 유지하고 rz만 새 값으로 변경 + 접근 높이
        def make_approach_pos_with_rz(x, y, z, new_rz, orientation=None):
            rx, ry, _ = orientation or get_current_orientation_base()
            return posx(x, y, z + APPROACH_OFFSET_Z, rx, ry, new_rz)

        def move_linear(name, pose):
            node.get_logger().info(f"{name} 이동")
            movel(pose, vel, acc, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        def pick_cycle(x, y, z, cube_rz):
            current_orientation = get_current_orientation_base()
            current_rz = current_orientation[2]
            # target_rz = choose_shortest_grasp_rz(cube_rz, current_rz)
            target_rz = current_rz

            # 1) 현재 자세(rx, ry, rz)를 그대로 유지한 채 접근
            fixed_approach = make_approach_pos(x, y, z, current_orientation)

            # 2) 접근 높이에서만 rz를 한 번 변경하고 그 자세로 내려감
            rotated_approach = make_approach_pos_with_rz(
                x, y, z, target_rz, current_orientation
            )
            rotated_target = make_target_pos_with_rz(
                x, y, z, target_rz, current_orientation
            )

            node.move_gripper(0)
            wait(0.5)

            move_linear("픽 접근 위치(고정 자세)", fixed_approach)
            wait(0.5)

            node.get_logger().info(
                f"rz 보정: marker={cube_rz:.2f} deg, current={current_rz:.2f} deg, target={target_rz:.2f} deg"
            )

            move_linear(f"픽 직전 rz 정렬 ({target_rz:.2f} deg)", rotated_approach)
            wait(0.5)

            move_linear("픽 위치", rotated_target)
            wait(0.5)

            node.move_gripper(500)
            wait(0.5)

            move_linear("픽 접근 위치 복귀", rotated_approach)
            wait(0.5)

        def place_cycle(x, y, z, target_rz):
            current_orientation = get_current_orientation_base()
            current_rz = current_orientation[2]
            aligned_rz = choose_shortest_grasp_rz(target_rz, current_rz)

            fixed_approach = make_approach_pos(x, y, z, current_orientation)
            rotated_approach = make_approach_pos_with_rz(
                x, y, z, aligned_rz, current_orientation
            )
            rotated_target = make_target_pos_with_rz(
                x, y, z, aligned_rz, current_orientation
            )

            move_linear("플레이스 접근 위치(고정 자세)", fixed_approach)
            wait(0.5)

            node.get_logger().info(
                f"place rz 보정: marker={target_rz:.2f} deg, current={current_rz:.2f} deg, target={aligned_rz:.2f} deg"
            )

            move_linear(f"플레이스 직전 rz 정렬 ({aligned_rz:.2f} deg)", rotated_approach)
            wait(0.5)

            move_linear("플레이스 위치", rotated_target)
            wait(0.5)

            node.move_gripper(0)
            wait(0.5)

            move_linear("플레이스 접근 위치 복귀", rotated_approach)
            wait(0.5)

        # 마커 검출 요청
        req = DetectMarkers.Request()
        future = node.marker_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()

        if response is None:
            raise RuntimeError("마커 검출 응답 없음")
        if not response.success:
            raise RuntimeError(f"마커 검출 실패: {response.message}")
        if len(response.markers) == 0:
            raise RuntimeError("검출된 마커 없음")


        cubes = {}
        targets = {}

        for marker in response.markers:
            if marker.type == "cube":
                cubes[marker.id] = marker
            elif marker.type == "target":
                targets[marker.id] = marker

        node.get_logger().info(f"검출된 cube 수: {len(cubes)}")
        node.get_logger().info(f"검출된 target 수: {len(targets)}")

        missing_cube_ids = [cube_id for cube_id, _ in task_pairs if cube_id not in cubes]
        missing_target_ids = [target_id for _, target_id in task_pairs if target_id not in targets]

        if missing_cube_ids:
            node.get_logger().warning(f"검출되지 않은 cube id: {missing_cube_ids}")
        if missing_target_ids:
            node.get_logger().warning(f"검출되지 않은 target id: {missing_target_ids}")

        go_home()

        for cube_id, target_id in task_pairs:
            if cube_id not in cubes:
                node.get_logger().warning(f"cube {cube_id} 없음 -> 건너뜀")
                continue

            if target_id not in targets:
                node.get_logger().warning(f"target {target_id} 없음 -> 건너뜀")
                continue

            cube = cubes[cube_id]
            target = targets[target_id]

            # detect_markers 결과는 m 단위라고 가정 -> mm 변환
            pick_x = cube.position.x * 1000.0
            pick_y = cube.position.y * 1000.0
            pick_z = PICK_TCP_Z

            place_x = target.position.x * 1000.0
            place_y = target.position.y * 1000.0
            place_z = PLACE_TCP_Z

            node.get_logger().info(
                f"[작업] cube {cube_id} -> target {target_id}"
            )
            node.get_logger().info(
                f"cube {cube_id}: x={pick_x:.2f}, y={pick_y:.2f}, z={pick_z:.2f}, rz={cube.rz:.2f}"
            )
            node.get_logger().info(
                f"target {target_id}: x={place_x:.2f}, y={place_y:.2f}, z={place_z:.2f}, rz={target.rz:.2f}"
            )

            pick_cycle(pick_x, pick_y, pick_z, cube.rz)
            place_cycle(place_x, place_y, place_z, target.rz)
            go_home()

        go_home()

    except Exception as e:
        node.get_logger().error(f"작업 중 오류: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
