from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    declared_arguments = []
    
    # 添加手边选择参数
    declared_arguments.append(
        DeclareLaunchArgument(
            'hand_side',
            default_value='right',
            description='Which hand to display: left or right'
        )
    )
    
    # 获取包路径
    pkg_share = FindPackageShare('lz_gripper_rviz')
    
    # 根据手边参数选择URDF文件
    hand_side = LaunchConfiguration('hand_side')
    
    # 使用通用的URDF文件（关节名称是通用的，可用于左右手）
    # 如果未来需要不同的URDF模型，可以取消下面注释的代码
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'right.urdf'])
    
    # 如果有专门的左右手URDF文件，可以使用下面的代码：
    # urdf_path = PathJoinSubstitution([
    #     pkg_share, 'urdf', 
    #     [hand_side, '.urdf']  # 会生成 left.urdf 或 right.urdf
    # ])
    
    # 设置RViz配置文件路径
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'display.rviz'])
    
    # 准备robot_description参数
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    
    # 创建robot_state_publisher节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'hand_side': hand_side
        }]
    )
    
    # 创建joint_state_publisher节点
    joint_state_publisher = Node(
        package='lz_gripper_rviz',
        executable='joint_state_publisher',
        name='gripper_joint_state_publisher',
        output='screen',
        parameters=[{
            'hand_side': hand_side
        }]
    )
    
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
            robot_state_publisher,
            joint_state_publisher,
            rviz_node,
        ]
    )