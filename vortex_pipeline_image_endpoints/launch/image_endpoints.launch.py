from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('vortex_pipeline_image_endpoints'),
        'config', 'image_endpoints_params.yaml'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug visualization'
    )

    node = Node(
        package='vortex_pipeline_image_endpoints',
        executable='image_endpoints_node',
        name='pipeline_image_endpoints',
        parameters=[
            config_path,
            {'debug': LaunchConfiguration('debug')}
        ],
        output='screen'
    )

    return LaunchDescription([
        debug_arg,
        node
    ])
