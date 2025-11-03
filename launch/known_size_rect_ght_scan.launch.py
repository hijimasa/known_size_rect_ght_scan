from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='known_size_rect_ght_scan',
            executable='known_size_rect_ght_scan_node',
            name='known_size_rect_ght_scan_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'known_size_rect_width': 8.0,
                'known_size_rect_height': 12.0,
                'accum_grid_res': 0.10,       # dp
                'theta_bins': 90,
                'angle_bins': 60,
                'samples_per_edge': 24,
                'beam_step': 1,
                'normal_window': 3,
                'jump_threshold': 0.5,
                'max_range': 30.0,
                'nms_xy_cells': 2,
                'nms_theta_bins': 2,
                'topk_candidates': 5,
                'edge_distance_thresh': 0.08,
                'edge_margin': 0.10,
                'min_edge_points': 50,
            }],
        )
    ])

