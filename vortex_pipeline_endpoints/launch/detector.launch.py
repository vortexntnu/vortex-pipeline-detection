from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('vortex_pipeline_endpoints'),
        'config', 'detector_params.yaml'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug visualization'
    )

    detector_node = Node(
        package='vortex_pipeline_endpoints',
        executable='detector_node',
        name='pipeline_detector',
        parameters=[
            config_path,
            {'debug': LaunchConfiguration('debug')}
        ],
        output='screen'
    )

    return LaunchDescription([
        debug_arg,
        detector_node
    ])
