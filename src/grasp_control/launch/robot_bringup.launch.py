# my_bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ur_launch_file = os.path.join(
        get_package_share_directory('grasp_control'),
        'launch',
        'ur_control_new.launch.py'
    )

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch_file),
        launch_arguments={
            'ur_type': 'ur5',
            'robot_ip': '192.168.54.130',
        }.items() 
    )

    hand_launch_file = os.path.join(
        get_package_share_directory('ft_leap'),
        'launch',
        'launch_leap.py'
    )

    hand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hand_launch_file),
    )

    moveit_config_launch_file = os.path.join(
        get_package_share_directory('ur5leap_moveit_config'),
        'launch',
        'move_group.launch.py'
    )

    moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_config_launch_file),
    )

    kinect_launch_file = os.path.join(
        get_package_share_directory('azure_kinect_ros_driver'),
        'launch',
        'driver.launch.py'
    )

    kinect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kinect_launch_file),
    )

    handeye_launch_file = os.path.join(
        get_package_share_directory('easy_handeye2'),
        'launch',
        'publish.launch.py'
    )

    handeye_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(handeye_launch_file),
    )

    joint_state_forwarder_node = Node(
        package='grasp_control',
        executable='joint_state_forwarder',
        name='joint_state_forwarder',
        output='screen',
    )

    table_frame_publisher_node = Node(
        package='grasping_perception',
        executable='table_frame_publisher',
        name='table_frame_publisher',
        output='screen',
    )

    object_pointcloud_publisher_node = Node(
        package='grasping_perception',
        executable='object_pointcloud_publisher',
        name='object_pointcloud_publisher',
        output='screen',
    )

    return LaunchDescription([
        ur_launch,
        hand_launch,
        moveit_config_launch,
        kinect_launch,
        handeye_launch,
        joint_state_forwarder_node,
        table_frame_publisher_node,
        object_pointcloud_publisher_node
    ])
