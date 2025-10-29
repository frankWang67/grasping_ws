""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: world -> camera_link """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "world",
                "--child-frame-id",
                "camera_link",
                "--x",
                # "1.26796",
                "1.205",
                "--y",
                "-0.0339003",
                "--z",
                # "0.871984",
                "0.926984",
                "--qx",
                "-0.693306",
                "--qy",
                "0.00593537",
                "--qz",
                "0.72059",
                "--qw",
                "0.00643249",
                # "--roll",
                # "2.71635",
                # "--pitch",
                # "-1.61316",
                # "--yaw",
                # "-0.425941",
            ],
        ),
    ]
    return LaunchDescription(nodes)
