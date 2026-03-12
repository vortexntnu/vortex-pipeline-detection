import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('vortex_pipeline_position_estimator'),
        'config',
        'position_estimator_params.yaml',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'log_level',
                default_value='info',
                description='ROS log level (debug, info, warn, error)',
            ),
            Node(
                package='vortex_pipeline_position_estimator',
                executable='position_estimator_node',
                name='pipeline_position_estimator',
                parameters=[config_path],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    ['pipeline_position_estimator:=', LaunchConfiguration('log_level')],
                ],
                output='screen',
            ),
        ]
    )
