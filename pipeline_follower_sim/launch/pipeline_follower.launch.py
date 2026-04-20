import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pipeline_follower_sim'),
        'config', 'pipeline_follower_sim.yaml'
    )

    return LaunchDescription([
        Node(
            package='pipeline_follower_sim',
            executable='pipeline_follower_node',
            name='pipeline_follower_node',
            parameters=[config],
        )
    ])
