from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ransac_line_fitter_cpp',
            executable='ransac_line_node',
            name='ransac_line_node',
            parameters=[
                {'input_topic': '/segmentation/mask'},
                {'output_topic': '/ransac_line/image'},
                {'binary_threshold': 200},
                {'min_pixels': 200},
                {'iterations': 200},
                {'distance_threshold_px': 2.5},
                {'min_inliers': 150},
                {'draw_thickness': 3},
                {'draw_b': 0},
                {'draw_g': 0},
                {'draw_r': 255},
                {'publish_original_if_fail': True},
            ],
        )
    ])
