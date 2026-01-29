from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='irls_line_fitter_2x',
            executable='irls_line_node',
            name='irls_line_node',
            parameters=[
                # Topics
                {'input_topic': '/filtered_image'},
                {'output_topic': '/irls_line/image'},

                # Preprocessing
                {'binary_threshold': 200},
                {'min_pixels': 250},

                # IRLS
                {'max_irls_iters': 30},
                {'eps_change': 1e-4},
                {'loss': 'tukey'},         # 'huber' or 'tukey'
                {'huber_delta': 1.5},
                {'tukey_c': 1.485},
                {'scale_with_mad': True},

                # Drawing
                {'draw_thickness': 3},
                {'draw_b': 0},
                {'draw_g': 0},
                {'draw_r': 255},
                {'publish_original_if_fail': True},
                {'clip_to_object': True},
                {'clip_max_dist_px': 6.0},

                # 2x
                {'find_second_line': True},
                {'removal_band_px': 120.0},
                {'min_pixels_second': 250},
                {'draw2_b': 0}, {'draw2_g': 255}, {'draw2_r': 0},
                {'draw_intersection': True},
                {'intersection_radius': 10},
            ],
        )
    ])
