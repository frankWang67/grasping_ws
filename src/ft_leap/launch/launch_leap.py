import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ft_leap',
            executable='leapnode_FT_topics_interp.py',
            # executable='leapnode_FT.py',
            name='leaphand_FT_node',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'kP': 25.0},
                {'kI': 0.0},
                {'kD': 50.0},
                {'curr_lim': 500.0}
            ]
        ),
    #     Node(
    #         package='leap_hand',
    #         executable='ros2_example.py',
    #         name='ros2_example',
    #         emulate_tty=True,
    #         output='screen'
    #     )
    ])
