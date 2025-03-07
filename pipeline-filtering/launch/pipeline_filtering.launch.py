import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pipeline_filtering_node = Node(
        package='pipeline_filtering',
        executable='pipeline_filtering_node',
        name='pipeline_filtering_node',
        parameters=[
            os.path.join(
                get_package_share_directory('pipeline_filtering'),
                'config',
                'pipeline_filtering_params.yaml',
            )
        ],
        output='screen',
    )
    return LaunchDescription([pipeline_filtering_node])
