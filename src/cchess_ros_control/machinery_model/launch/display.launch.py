from launch import LaunchDescription
import launch
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = get_package_share_directory('machinery_model')+'/urdf/'
    config_path = get_package_share_directory('machinery_model')+'/config/'
    rviz_config_path = config_path+'urdf.rviz'

    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value='machinery_model.urdf.xacro',
        description='URDF 的文件名'
    )

    # 获取文件内容生成新的参数
    robot_description = launch.substitutions.Command(
        ['xacro ', urdf_path, launch.substitutions.LaunchConfiguration('model')]
    )

    # 状态发布节点
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(robot_description)  # 直接传递URDF内容
            }],
            output='screen'
    )
    
    # 关节状态发布节点
    joint_state_publisher_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
    )

    # rviz节点
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d',rviz_config_path]
    )

    return LaunchDescription([
        # 声明参数
        action_declare_arg_mode_path,

        # 执行节点
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])