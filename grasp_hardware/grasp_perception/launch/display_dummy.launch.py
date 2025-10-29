from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    declared_arguments = []
    
    # 获取包路径
    pkg_share = FindPackageShare('grasp_perception')
    
    # 使用通用的URDF文件（关节名称是通用的，可用于左右手）
    # 如果未来需要不同的URDF模型，可以取消下面注释的代码
    urdf_path = PathJoinSubstitution([pkg_share, 'config', 'leap_right.urdf'])
    
    # 设置RViz配置文件路径
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'display_dummy.rviz'])
    
    # 准备robot_description参数
    hand_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    arm_description = ParameterValue(
        Command(['xacro ', '/home/peter/ur10e.urdf']),
        value_type=str
    )

    # 创建robot_state_publisher节点
    hand_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='hand_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': hand_description,
        }],
        remappings=[
            ('/joint_states', '/hand_joint_states'),
            ('/robot_description', '/hand_description'),
        ]
    )

    arm_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': arm_description,
        }],
        remappings=[
            ('/robot_description', '/arm_description'),
        ]
    )
    
    # 创建joint_state_publisher节点
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    # )
    
    # 创建RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )
    
    # 返回LaunchDescription
    return LaunchDescription(
        declared_arguments + [
            hand_state_publisher,
            arm_state_publisher,
            # joint_state_publisher,
            rviz_node,
        ]
    )