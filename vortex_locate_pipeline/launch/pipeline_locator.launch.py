import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('vortex_locate_pipeline'), 
        'config', 
        'pipeline_locator_params.yaml'
        )

    node = Node(
        package='vortex_locate_pipeline',
        executable='pipeline_locator_node',
        name='pipeline_locator',
        parameters=[params_file],
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug'],
    )

    return LaunchDescription([node])
