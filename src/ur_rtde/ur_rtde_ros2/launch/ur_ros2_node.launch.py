from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ur_rtde_ros2",  # replace with your package name
                executable="ur_ros2_node.py",  # the script installed via CMakeLists.txt
                name="ur_ros2_node",  # ROS 2 node name
                output="screen",  # print logs to console
                parameters=[  # optional: pass parameters here
                ],
            )
        ]
    )
