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
                {'U': 1.0},  # Desired surge speed for LOS guidance (m/s)
                {'Delta': 5.0},  # Lookahead distance for LOS guidance (m)
                {'k': 0.5},  # Path convergence gain for LOS guidance
                {'L': 2.0},  # Pendulum controller parameter (length of 'rod')
                {'epsilon': 0.5},  # Pendulum controller adaptation rate
                {'k_v': 1.0},  # Velocity feedback gain for pendulum controller
                {'k_a': 0.1},  # Adaptation gain for pendulum controller
                {'m_virtual': 1.0},  # Virtual mass for heading controller
                {'path_file': os.path.join(os.path.dirname(__file__), '..', 'path_example.yaml')},  # Path to YAML file with waypoints
                {'ref_lat': -22.986633223025905},  # Rio de Janeiro area
                {'ref_lon': -43.2025620936613},
                {'ref_alt': 0.0}
            ]
        )
    ])
