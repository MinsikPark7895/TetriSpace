from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dakae_vision',
            executable='object_contour_service_server',
            name='yolo_detector',
            output='screen',
            emulate_tty=True,
        ),
    ])
