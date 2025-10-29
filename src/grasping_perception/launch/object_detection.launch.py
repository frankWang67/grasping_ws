from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    object_pointcloud_publisher_node = Node(
        package='grasping_perception',
        executable='object_pointcloud_publisher',
        name='object_pointcloud_publisher',
        output='screen',
    )

    return LaunchDescription([
        object_pointcloud_publisher_node
    ])