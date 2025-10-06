import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='asv_controller',
            executable='asv_controller_node',
            name='controller_node',
            output='screen',
            parameters=[
                os.path.join(os.path.dirname(__file__), '..', 'params', 'tuned_circle_3m.yaml'),
                {'path_file': os.path.join(os.path.dirname(__file__), '..', 'circle_radius_3m.yaml')},
                {'ref_lat': -22.986633223025905},  # Rio de Janeiro area
                {'ref_lon': -43.2025620936613},
                {'ref_alt': 0.0}
            ]
        )
    ])
