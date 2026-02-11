from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('vortex_pipeline_3d'),
        'config', 'localizer_params.yaml'
    )

    # Launch arguments for visualization
    debug_image_arg = DeclareLaunchArgument(
        'enable_debug_image',
        default_value='false',
        description='Enable 2D image overlay visualization'
    )

    debug_markers_arg = DeclareLaunchArgument(
        'enable_3d_markers',
        default_value='false',
        description='Enable 3D marker visualization'
    )

    return LaunchDescription([
        debug_image_arg,
        debug_markers_arg,
        Node(
            package='vortex_pipeline_3d',
            executable='localizer_node',
            name='pipeline_localizer',
            parameters=[
                config_path,
                {'enable_debug_image': LaunchConfiguration('enable_debug_image')},
                {'enable_3d_markers': LaunchConfiguration('enable_3d_markers')}
            ],
            output='screen'
        )
    ])
