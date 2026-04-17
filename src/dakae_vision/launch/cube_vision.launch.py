from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dakae_vision',
            executable='camera_node',  # setup.py에 등록된 실행 파일 이름
            name='realsense_camera',
            output='screen'
        ),
        Node(
            package='dakae_vision',
            executable='cube_service_server',
            name='cube_detector',
            output='screen'
        )
    ])