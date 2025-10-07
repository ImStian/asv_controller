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
                {'path_file': os.path.join(os.path.dirname(__file__), '..', 'circle_radius_3m.yaml')}
            ]
        )
    ])
