from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ur_rtde_ros2_node = Node(
        package="ur_rtde_ros2",
        executable="ur_ros2_node.py",
        name="ur_ros2_node",
        output="screen",
        parameters=[],
    )

    hand_launch_file = os.path.join(
        get_package_share_directory('ft_leap'),
        'launch',
        'launch_leap.py'
    )
    hand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hand_launch_file),
    )

    kinect_launch_file = os.path.join(
        get_package_share_directory('azure_kinect_ros_driver'),
        'launch',
        'driver.launch.py'
    )
    kinect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kinect_launch_file),
    )

    rviz_launch_file = os.path.join(get_package_share_directory("grasping_control"), "launch", "rviz_vis.launch.py")
    rviz_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz_launch_file))

    # Path to easy_handeye2 launch file
    easy_handeye_share = get_package_share_directory("easy_handeye2")
    publish_launch = os.path.join(easy_handeye_share, "launch", "publish.launch.py")
    calib_tf_pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(publish_launch)
    )

    table_node = Node(
        package="grasping_perception",
        executable="table_frame_publisher",
        name="table_frame_publisher",
        output="screen",
    )

    joint_state_forwarder_node = Node(
        package="grasping_control",
        executable="joint_state_forwarder.py",
        name="joint_state_forwarder",
        output="screen",
    )

    return LaunchDescription(
        [
            ur_rtde_ros2_node,
            hand_launch,
            calib_tf_pub_launch,
            rviz_launch,
            kinect_launch,
            table_node,
            joint_state_forwarder_node,
        ]
    )
