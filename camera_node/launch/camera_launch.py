import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('camera_node'),
        'config',
        'camera_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file',
            default_value=params_file,
            description='Path to camera parameters yaml file'
        ),

        Node(
            package='camera_node',
            executable='camera_node_node',
            name='camera_node',
            output='screen',
            emulate_tty=True,
        )
    ])
