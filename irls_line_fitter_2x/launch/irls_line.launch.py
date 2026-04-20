from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('irls_line_fitter_2x'),
        'config', 'irls_line.yaml'
    )

    return LaunchDescription([
        Node(
            package='irls_line_fitter_2x',
            executable='irls_line_node',
            name='irls_line_node',
            parameters=[config],
        )
    ])
