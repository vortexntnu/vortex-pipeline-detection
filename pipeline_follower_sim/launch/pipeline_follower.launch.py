from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pipeline_follower_sim',
            executable='pipeline_follower_node',
            name='pipeline_follower_node',
            parameters=[
                # Topics
                {'input_topic': '/filtered_image'},
                {'output_topic': '/irls_line/image'},

                # Preprocessing
                {'binary_threshold': 200},
                {'min_pixels': 200},

            ],
        )
    ])
