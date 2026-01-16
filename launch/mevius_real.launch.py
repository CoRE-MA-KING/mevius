import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mevius')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription(
        [
            Node(
                package='mevius',
                executable='mevius_main',
                name='mevius_main',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='mevius',
                executable='can_communication',
                name='can_communication',
                output='screen',
                parameters=[params_file],
            )
        ]
    )
