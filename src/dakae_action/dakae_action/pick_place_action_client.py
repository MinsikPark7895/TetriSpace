#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dakae_interfaces.action import PickAndPlace

class TaskCommander(Node):
    def __init__(self):
        super().__init__('task_commander')
        self._action_client = ActionClient(self, PickAndPlace, '/dsr01/pick_and_place')

    def send_goal(self, cube_ids, target_ids):
        self.get_logger().info('액션 서버 대기 중...')
        self._action_client.wait_for_server()

        goal_msg = PickAndPlace.Goal()
        goal_msg.cube_ids = cube_ids
        goal_msg.target_ids = target_ids

        self.get_logger().info(f'작업 지시 전송: 큐브 {cube_ids} -> 타겟 {target_ids}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[진행 상황] 큐브 ID: {feedback.current_cube_id} | 상태: {feedback.current_phase}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('작업이 거부되었습니다.')
            return

        self.get_logger().info('작업이 승인되어 실행 중입니다.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('모든 픽앤플레이스 및 검증 작업 성공!')
        else:
            self.get_logger().warning(f'작업 실패 큐브 목록: {result.failed_cube_ids}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = TaskCommander()
    
    # 예시: 1번 큐브를 1번 타겟으로, 2번 큐브를 2번 타겟으로, 3번 큐브를 3번 타겟으로 이동
    action_client.send_goal(cube_ids=[1, 2, 3], target_ids=[1, 2, 3])
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()