"""
dakae_bringup - 비전 단독 실행 런치파일

용도: 로봇팔/그리퍼 없이 비전 노드만 띄워서 디텍션 테스트
사용: ros2 launch dakae_bringup vision_only.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- 런치 인자 선언 ---
    # 사용자가 'ros2 launch ... debug:=true' 식으로 바꿀 수 있음
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='디버그 모드 활성화 (시각화 창 등)',
    )

    # --- config 파일 경로 ---
    # install/dakae_bringup/share/dakae_bringup/config/vision_params.yaml 로 찾아감
    vision_params = os.path.join(
        get_package_share_directory('dakae_bringup'),
        'config',
        'vision_params.yaml',
    )

    # --- 비전 노드 ---
    cube_service_node = Node(
        package='dakae_vision',
        executable='cube_service_server',
        name='cube_service_server',
        output='screen',
        parameters=[
            vision_params,
            {'debug_mode': LaunchConfiguration('debug')},
        ],
        emulate_tty=True,  # 노드의 logger 색상 출력 유지
    )

    return LaunchDescription([
        debug_arg,
        cube_service_node,
    ])