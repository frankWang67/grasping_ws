from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the gripper'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Rate at which to publish joint states'
        ),
        DeclareLaunchArgument(
            'hand_side',
            default_value='left',
            description='Which hand to control: left or right'
        ),
        Node(
            package='lz_gripper_ros2',
            executable='lz_gripper_node',
            name='lz_gripper_node',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'hand_side': LaunchConfiguration('hand_side'),
            }],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gripper_frame_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'gripper_frame']
        ),
        Node(
            package='lz_gripper_ros2',
            executable='hand_controller_node.py',
            name='hand_controller',
            parameters=[{
                'hand_side': LaunchConfiguration('hand_side'),
            }],
            output='screen',
            emulate_tty=True,
        ), 
    ]) 
