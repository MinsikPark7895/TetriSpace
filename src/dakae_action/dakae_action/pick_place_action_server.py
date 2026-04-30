#!/usr/bin/env python3

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.impl.rcutils_logger import RcutilsLogger
import DR_init

from dakae_interfaces.action import PickAndPlace
from dakae_interfaces.srv import DetectMarkers
from dakae_interfaces.srv import MoveGripper

if not hasattr(RcutilsLogger, "info"):
    RcutilsLogger.info = RcutilsLogger.dinfo

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

# [핵심] 더 이상 여기서 초기화하지 않습니다. main() 함수 안으로 이동합니다.
# DR_init.__dsr__id = ROBOT_ID
# DR_init.__dsr__model = ROBOT_MODEL

# =================================================================
# [1] 헬퍼 노드 (통신 전담)
# =================================================================
class ServiceHelper:
    def __init__(self):
        self.node = rclpy.create_node('pick_place_service_helper')
        self.marker_client = self.node.create_client(DetectMarkers, "/detect_markers")
        self.gripper_client = self.node.create_client(MoveGripper, "/dsr01/gripper_move")
        
        self.node.get_logger().info("헬퍼 노드: 서비스 연결 대기 중...")
        while not self.marker_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("detect_markers 대기 중...")
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("gripper_move 대기 중...")
        self.node.get_logger().info("헬퍼 노드: 통신 라인 확보 완료!")

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def call_detect_markers(self):
        req = DetectMarkers.Request()
        future = self.marker_client.call_async(req)
        
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        
        event.wait(timeout=10.0)
        if not future.done():
            self.node.get_logger().error("마커 검출 응답 타임아웃!")
            self.marker_client.remove_pending_request(future)
            return []
        
        res = future.result()
        return res.markers if (res and res.success) else []

    def move_gripper(self, stroke: int):
        req = MoveGripper.Request()
        req.stroke = stroke
        future = self.gripper_client.call_async(req)
        
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        
        event.wait(timeout=15.0)
        if not future.done():
            self.gripper_client.remove_pending_request(future)
            raise RuntimeError(f"그리퍼 이동 타임아웃 (stroke={stroke})")
        res = future.result()
        if not res or not res.success:
            raise RuntimeError("그리퍼 이동 실패")

# =================================================================
# [2] 메인 액션 서버
# =================================================================
class PickPlaceActionServer(Node):
    def __init__(self):
        super().__init__("pick_place_action_server", namespace=ROBOT_ID)
        
        self.helper = ServiceHelper()

        # 노드가 격리되었으므로, 액션 서버 본연의 ReentrantCallbackGroup을 안전하게 사용 가능합니다!
        self.action_cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            PickAndPlace,
            'pick_and_place',
            self.execute_callback,
            callback_group=self.action_cb_group
        )
        
        self.vel = 100
        self.acc = 100
        self.APPROACH_OFFSET_Z = 100.0
        self.PICK_TCP_Z = 155.0
        self.PLACE_TCP_Z = 155.0
        self.get_logger().info(">>> Pick and Place Action Server Ready.")

    def get_dsr_api(self):
        from DSR_ROBOT2 import wait, movej, movel, get_current_posx, DR_BASE, DR_MV_MOD_ABS
        from DR_common2 import posj, posx
        return wait, movej, movel, get_current_posx, DR_BASE, DR_MV_MOD_ABS, posj, posx

    def find_marker(self, markers, marker_type, marker_id):
        for m in markers:
            if m.type == marker_type and m.id == marker_id:
                return m
        return None

    def execute_callback(self, goal_handle):
        self.get_logger().info('픽앤플레이스 작업 시작!')
        wait, movej, movel, get_current_posx, DR_BASE, DR_MV_MOD_ABS, posj, posx = self.get_dsr_api()
        
        home = posj(0, 0, 90, 0, 90, 0)
        feedback_msg = PickAndPlace.Feedback()
        failed_cubes = []

        cube_ids = goal_handle.request.cube_ids
        target_ids = goal_handle.request.target_ids

        # --- 로봇 모션 제어 로직 ---
        def go_home():
            movej(home, self.vel, self.acc)
            wait(0.5)

        def normalize_angle_deg(angle):
            normalized = (angle + 180.0) % 360.0 - 180.0
            return 180.0 if normalized == -180.0 else normalized

        def choose_shortest_grasp_rz(marker_rz, current_rz):
            candidates = [marker_rz - 180.0, marker_rz, marker_rz + 180.0]
            return min(candidates, key=lambda c: abs(normalize_angle_deg(c - current_rz)))

        def move_linear(pose):
            movel(pose, self.vel, self.acc, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        def execute_pick(x, y, z, cube_rz):
            current_pos, _ = get_current_posx(ref=DR_BASE)
            current_rz = current_pos[5]
            target_rz = choose_shortest_grasp_rz(cube_rz, current_rz)

            approach_pose = posx(x, y, z + self.APPROACH_OFFSET_Z, current_pos[3], current_pos[4], target_rz)
            target_pose = posx(x, y, z, current_pos[3], current_pos[4], target_rz)

            self.helper.move_gripper(0)
            wait(0.5)
            move_linear(approach_pose)
            wait(0.5)
            move_linear(target_pose)
            wait(0.5)
            self.helper.move_gripper(500)
            wait(0.5)
            move_linear(approach_pose)
            wait(0.5)

        def execute_place(x, y, z, target_rz):
            current_pos, _ = get_current_posx(ref=DR_BASE)
            current_rz = current_pos[5]
            aligned_rz = choose_shortest_grasp_rz(target_rz, current_rz)

            approach_pose = posx(x, y, z + self.APPROACH_OFFSET_Z, current_pos[3], current_pos[4], aligned_rz)
            target_pose = posx(x, y, z, current_pos[3], current_pos[4], aligned_rz)

            move_linear(approach_pose)
            wait(0.5)
            move_linear(target_pose)
            wait(0.5)
            self.helper.move_gripper(0)
            wait(0.5)
            move_linear(approach_pose)
            wait(0.5)

        # --- 메인 시퀀스 ---
        for c_id, t_id in zip(cube_ids, target_ids):
            feedback_msg.current_cube_id = c_id
            
            feedback_msg.current_phase = "MOVING_HOME_FOR_DETECT"
            goal_handle.publish_feedback(feedback_msg)
            go_home()
            wait(1.0)
            
            feedback_msg.current_phase = "DETECTING_START"
            goal_handle.publish_feedback(feedback_msg)
            markers = self.helper.call_detect_markers()
            
            cube_m = self.find_marker(markers, 'cube', c_id)
            target_m = self.find_marker(markers, 'target', t_id)
            
            if not cube_m or not target_m:
                self.get_logger().warning(f"마커 {c_id} 또는 {t_id} 찾기 실패")
                failed_cubes.append(c_id)
                continue

            backup_target_pos = target_m.position

            feedback_msg.current_phase = "PICKING_AND_PLACING"
            goal_handle.publish_feedback(feedback_msg)
            
            pick_x = cube_m.position.x * 1000.0
            pick_y = cube_m.position.y * 1000.0
            place_x = backup_target_pos.x * 1000.0
            place_y = backup_target_pos.y * 1000.0

            execute_pick(pick_x, pick_y, self.PICK_TCP_Z, cube_m.rz)
            execute_place(place_x, place_y, self.PLACE_TCP_Z, target_m.rz)

            feedback_msg.current_phase = "MOVING_HOME_FOR_VERIFY"
            goal_handle.publish_feedback(feedback_msg)
            go_home()
            wait(1.5)

            feedback_msg.current_phase = "VERIFYING"
            goal_handle.publish_feedback(feedback_msg)
            v_markers = self.helper.call_detect_markers()
            
            v_cube = self.find_marker(v_markers, 'cube', c_id)
            v_target = self.find_marker(v_markers, 'target', t_id)

            if v_cube:
                ref_x = v_target.position.x if v_target else backup_target_pos.x
                ref_y = v_target.position.y if v_target else backup_target_pos.y

                dx = v_cube.position.x - ref_x
                dy = v_cube.position.y - ref_y
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist < 0.025: 
                    self.get_logger().info(f"✅ 검증 성공 (오차: {dist*1000:.1f}mm)")
                else:
                    self.get_logger().warning(f"❌ 검증 실패 (오차: {dist*1000:.1f}mm)")
                    failed_cubes.append(c_id)
            else:
                self.get_logger().warning(f"검증 실패: 큐브 마커({c_id}) 안 보임")
                failed_cubes.append(c_id)

        result = PickAndPlace.Result()
        result.success = (len(failed_cubes) == 0)
        result.failed_cube_ids = failed_cubes
        result.message = "모든 지시 작업 완료"
        
        if result.success: 
            goal_handle.succeed()
        else: 
            goal_handle.abort()
            
        return result

# =================================================================
# [3] 메인 실행부 (가장 중요한 아키텍처 변화)
# =================================================================
def main(args=None):
    rclpy.init(args=args)

    # 🚨 1. 두산 라이브러리 전용 '격리 노드' 생성 및 할당
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    dsr_dummy_node = rclpy.create_node("dsr_internal_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_dummy_node

    # 🚨 2. 우리의 메인 '액션 서버 노드' 생성
    action_node = PickPlaceActionServer()

    # 🚨 3. 실행기에는 오직 우리의 '액션 서버 노드'만 추가합니다.
    # (더미 노드는 두산 라이브러리가 wait() 함수 안에서 스스로 돌리도록 방치합니다)
    executor = MultiThreadedExecutor()
    executor.add_node(action_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt: 
        pass
    finally:
        action_node.destroy_node()
        dsr_dummy_node.destroy_node() # 종료 시 더미 노드도 함께 정리
        rclpy.shutdown()

if __name__ == "__main__":
    main()