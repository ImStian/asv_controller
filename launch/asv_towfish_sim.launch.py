import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    circle_path = os.path.join(os.path.dirname(__file__), '..', 'circle_radius_3m.yaml')

    return LaunchDescription([
        Node(
            package='asv_controller',
            executable='asv_towfish_sim',
            name='asv_towfish_sim',
            output='screen',
            parameters=[
                {'path_file': circle_path},
            ],
        )
    ])