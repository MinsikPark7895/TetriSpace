"""
dakae_bringup - 전체 시스템 통합 런치파일

사용: ros2 launch dakae_bringup system.launch.py

현재 상태:
    - 비전:     ✅ YOLO8 (object_contour_service_server)
    - 그리퍼:   ✅ dsr_gripper_tcp (gripper_service_node)
    - 로봇팔:   ⏳ 팀원 B 런치파일 대기
"""

import os
import socket
import struct
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _send_gripper_shutdown(context, *args, **kwargs):
    host = context.perform_substitution(LaunchConfiguration('gripper_host'))
    try:
        header = struct.pack('>2sBBHH', b'GP', 1, 6, 1, 0)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3)
        s.connect((host, 20002))
        s.sendall(header)
        s.close()
        print(f'[dakae_bringup] 그리퍼 SHUTDOWN 전송 완료 ({host}:20002)')
    except Exception as e:
        print(f'[dakae_bringup] 그리퍼 SHUTDOWN 전송 실패 (무시): {e}')
    return []


def generate_launch_description():
    # ========================================================================
    # 1. 런치 인자
    # ========================================================================
    use_vision_arg = DeclareLaunchArgument(
        'use_vision', default_value='true',
        description='YOLO8 비전 서브시스템 활성화')
    use_gripper_arg = DeclareLaunchArgument(
        'use_gripper', default_value='true',
        description='dsr_gripper_tcp 그리퍼 서브시스템 활성화')
    use_arm_arg = DeclareLaunchArgument(
        'use_arm', default_value='false',
        description='로봇팔 서브시스템 활성화 (팀원 B 런치파일 연결 후 true)')
    gripper_host_arg = DeclareLaunchArgument(
        'gripper_host', default_value='110.120.1.56',
        description='그리퍼 컨트롤러 IP 주소')

    # ========================================================================
    # 2. 환영 배너
    # ========================================================================
    banner = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════════════╗\n',
        '║       [dakae] 프로젝트 통합 시스템 시작          ║\n',
        '╠══════════════════════════════════════════════════╣\n',
        '║  비전:   YOLO8 object_contour_service_server     ║\n',
        '║  그리퍼: dsr_gripper_tcp gripper_service_node    ║\n',
        '║  조작기: 3초 후 자동으로 새 터미널 오픈          ║\n',
        '║  종료:   이 터미널에서 Ctrl+C                    ║\n',
        '╚══════════════════════════════════════════════════╝\n',
    ])

    # ========================================================================
    # 3. 비전 서브시스템 (YOLO8)
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
    # 4. 그리퍼 서브시스템 (dsr_gripper_tcp)
    # ========================================================================
    gripper_shutdown = OpaqueFunction(function=_send_gripper_shutdown)

    gripper_node = TimerAction(
        period=1.0,
        actions=[Node(
        package='dsr_gripper_tcp',
        executable='gripper_service_node',
        name='gripper_service',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_gripper')),
        parameters=[{
            'controller_host': LaunchConfiguration('gripper_host'),
            'tcp_port': 20002,
            'namespace': 'dsr01',
            'service_prefix': '',
            'initialize_on_start': True,
            'stop_existing_drl': True,
            'drl_start_retry_count': 5,
            'drl_start_retry_delay_sec': 2.0,
        }],
    )])

    # ========================================================================
    # 5. 로봇팔 (placeholder)
    # ========================================================================
    arm_placeholder = LogInfo(
        msg='[dakae_bringup] TODO: 로봇팔 런치파일 연결 필요 (팀원 B)',
        condition=IfCondition(LaunchConfiguration('use_arm')),
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
        use_gripper_arg,
        use_arm_arg,
        gripper_host_arg,
        banner,
        vision_launch,
        gripper_shutdown,
        gripper_node,
        arm_placeholder,
        helper_terminal,
    ])
