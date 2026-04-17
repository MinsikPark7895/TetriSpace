"""
dakae_bringup - 전체 시스템 통합 런치파일 (스켈레톤)

용도: 팀 전체 시스템(비전 + 로봇팔 + 그리퍼 + 상위 로직)을 한 번에 띄우는 진입점
사용: ros2 launch dakae_bringup system.launch.py

현재 상태:
    - 비전: ✅ 연결됨 (vision_only.launch.py 재사용)
    - 로봇팔: ⏳ 팀원 B 런치파일 대기
    - 그리퍼: ⏳ 팀원 C 런치파일 대기
    - 오케스트레이터: ⏳ 팀원 D 코드 대기
"""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # ========================================================================
    # 1. 런치 인자
    # ========================================================================
    use_vision_arg = DeclareLaunchArgument(
        'use_vision', default_value='true',
        description='비전 서브시스템 활성화')
    use_arm_arg = DeclareLaunchArgument(
        'use_arm', default_value='false',
        description='로봇팔 서브시스템 활성화 (팀원 B 연결 후 true)')
    use_gripper_arg = DeclareLaunchArgument(
        'use_gripper', default_value='false',
        description='그리퍼 서브시스템 활성화 (팀원 C 연결 후 true)')
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='전체 시스템 디버그 모드')

    # ========================================================================
    # 2. 환영 배너
    # ========================================================================
    banner = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════════════╗\n',
        '║       [dakae] 프로젝트 통합 시스템 시작          ║\n',
        '╠══════════════════════════════════════════════════╣\n',
        '║  이 터미널: 노드 로그                            ║\n',
        '║  조작 터미널: 3초 후 자동으로 열립니다           ║\n',
        '║                                                  ║\n',
        '║  종료: 이 터미널에서 Ctrl+C                      ║\n',
        '╚══════════════════════════════════════════════════╝\n',
    ])

    # ========================================================================
    # 3. 비전 서브시스템
    # ========================================================================
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dakae_vision'),
                'launch',
                'full_vision.launch.py',
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_vision')),
    )
    # ========================================================================
    # 4. 로봇팔 (placeholder)
    # ========================================================================
    arm_placeholder = LogInfo(
        msg='[dakae_bringup] TODO: 로봇팔 런치파일 연결 필요 (팀원 B)',
        condition=IfCondition(LaunchConfiguration('use_arm')),
    )

    # ========================================================================
    # 5. 그리퍼 서버 (test_first 패키지)
    # ========================================================================
    gripper_server = Node(
        package='test_first',
        executable='gripper_server',
        name='gripper_server',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_gripper')),
    )

    # ========================================================================
    # 6. 조작 도우미 터미널 (3초 후 자동 오픈)
    # ========================================================================
    helper_script = os.path.join(
        get_package_prefix('dakae_bringup'),
        'lib', 'dakae_bringup', 'dakae_helper.py',
    )

    helper_terminal = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal',
                    '--title=dakae 조작기',
                    '--',
                    'python3', helper_script,
                ],
                output='log',
            ),
        ],
    )

    return LaunchDescription([
        use_vision_arg,
        use_arm_arg,
        use_gripper_arg,
        debug_arg,
        banner,
        vision_launch,
        arm_placeholder,
        gripper_server,
        helper_terminal,
    ])