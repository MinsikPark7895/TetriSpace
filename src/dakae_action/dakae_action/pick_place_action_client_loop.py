#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
import time

from dakae_interfaces.action import PickAndPlace

class TaskCommander(Node):
    def __init__(self):
        super().__init__('task_commander')
        self._action_client = ActionClient(self, PickAndPlace, '/dsr01/pick_and_place')
        self.is_busy = False # 로봇이 작업 중인지 확인하는 플래그

    def start_input_thread(self):
        # 사용자 입력을 받는 별도의 백그라운드 쓰레드 실행
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def input_loop(self):
        self.get_logger().info('액션 서버 연결 대기 중...')
        self._action_client.wait_for_server()
        self.get_logger().info('서버 연결 완료! 명령을 입력할 수 있습니다.')

        while rclpy.ok():
            if self.is_busy:
                # 작업 중일 때는 새로운 입력을 막음
                time.sleep(0.5)
                continue

            try:
                print("\n" + "="*60)
                cube_str = input("픽업할 [큐브 ID]를 띄어쓰기로 입력하세요 (종료는 q) : ")
                if cube_str.strip().lower() == 'q':
                    self.get_logger().info('클라이언트를 종료합니다.')
                    rclpy.shutdown()
                    break

                target_str = input("이동시킬 [타겟 ID]를 동일한 개수로 입력하세요 : ")

                # 입력받은 문자열을 숫자 리스트로 변환 (예: "1 2 3" -> [1, 2, 3])
                cube_ids = [int(x.strip()) for x in cube_str.replace(',', ' ').split() if x.strip()]
                target_ids = [int(x.strip()) for x in target_str.replace(',', ' ').split() if x.strip()]

                # 입력 검증
                if not cube_ids or len(cube_ids) != len(target_ids):
                    print("입력 오류: 큐브 ID와 타겟 ID의 개수가 같아야 합니다.")
                    continue

                self.send_goal(cube_ids, target_ids)

            except ValueError:
                print("❌ 입력 오류: 숫자만 입력해주세요.")
            except EOFError:
                break

    def send_goal(self, cube_ids, target_ids):
        self.is_busy = True # 작업 시작 상태로 변경
        goal_msg = PickAndPlace.Goal()
        goal_msg.cube_ids = cube_ids
        goal_msg.target_ids = target_ids

        self.get_logger().info(f'작업 전송: 큐브 {cube_ids} -> 타겟 {target_ids}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # \r 을 사용하여 터미널 창에 줄바꿈 없이 현재 상태를 덮어쓰기로 표시 (깔끔함)
        print(f'\r[작업 중] 큐브: {feedback.current_cube_id} | 상태: {feedback.current_phase:<25}', end='')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('\n작업이 서버에 의해 거부되었습니다.')
            self.is_busy = False
            return

        print('\n작업이 승인되었습니다.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print('\n' + '-'*60)
        if result.success:
            self.get_logger().info('지시한 모든 작업 검증 완료 및 성공!')
        else:
            self.get_logger().warning(f'작업 실패 또는 오차 발생 (큐브 ID): {result.failed_cube_ids}')
        print('-'*60)
        
        self.is_busy = False # 작업이 끝났으므로 다시 입력을 받을 수 있도록 해제

def main(args=None):
    rclpy.init(args=args)
    action_client = TaskCommander()
    action_client.start_input_thread() # 입력 대기 쓰레드 시작

    try:
        # ROS 2 통신 콜백을 처리하기 위해 메인 쓰레드는 스핀 상태 유지
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()