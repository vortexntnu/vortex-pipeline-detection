import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pipeline_line_fitting_node = Node(
            package='pipeline_line_fitting',
            executable='pipeline_line_fitting_node',
            name='pipeline_line_fitting_node',
            parameters=[os.path.join(get_package_share_directory('pipeline_line_fitting'),'params','pipeline_line_fitting_params.yaml')],
            output='screen',
        )
    return LaunchDescription([
        pipeline_line_fitting_node
    ])