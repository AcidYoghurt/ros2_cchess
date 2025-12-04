import os
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 该模块用来测试 机械臂是否能够到达棋盘点位

# 测试：ros2 launch machinery_ros2control_bringup machinery_keypoint_debug.launch.py serial_port_name:=/dev/MachineryA namespace:=left/

config_path = LaunchConfiguration("config_path")
urdf_path = LaunchConfiguration("urdf_path")
serial_port_name = LaunchConfiguration("serial_port_name")
namespace = LaunchConfiguration("namespace")

# 声明参数
def declare_parameters():
    config_path_arg = DeclareLaunchArgument(
        name='config_path',
        default_value=str(os.path.join(get_package_share_directory('machinery_ros2control_bringup'),'config')),
        description='config文件夹 的 路径'
    )

    urdf_path_arg = DeclareLaunchArgument(
        name='urdf_path',
        default_value=str(os.path.join(get_package_share_directory('machinery_ros2control_bringup'),'urdf')),
        description='URDF文件夹 的 路径'
    )

    serial_port_name_arg = DeclareLaunchArgument(
        name='serial_port_name',
        default_value="/dev/MachineryA",
        description="机械臂的串口名称"
    )

    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value="right/",
        description="为节点和TF设置命名空间"
    )
    return [config_path_arg, urdf_path_arg, namespace_arg, serial_port_name_arg]

# ros2control节点
def machinery_ros2control(context: launch.LaunchContext):
    # 使用 context.perform_substitution 来解析 LaunchConfiguration 的值
    config_path_str = LaunchConfiguration('config_path').perform(context)
    namespace_str = namespace.perform(context)

    with open(config_path_str+'/'+namespace_str+'machinery.yaml', 'r', encoding='utf-8') as file:
        config_file = yaml.safe_load(file)

    robot_description = ParameterValue(launch.substitutions.Command([
        'xacro ', PathJoinSubstitution([LaunchConfiguration('urdf_path'), 'machinery.urdf.xacro']),
        ' origin_position:=', '"'+str(config_file['/**']['ros__parameters']['origin_position'])+'"',
        ' frame_prefix:=', namespace,
        ' serial_port_name:=', serial_port_name
    ]), value_type=str)
    robot_controllers_config = PathJoinSubstitution([config_path, namespace, 'machinery_controllers.yaml'])
    rviz_config = PathJoinSubstitution([config_path, namespace, 'urdf.rviz'])

    # 启动 ros2_control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        output="both",
        parameters=[
            {"robot_description": robot_description},
            robot_controllers_config
        ]
    )

    # 启动 robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[{
            "robot_description": robot_description,
        }],
    )

    # 加载并启动手臂控制器
    cartesian_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["cartesian_position_controller", "-c", "controller_manager"],
        output="both",
    )

    # 加载并启动吸盘控制器
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["gripper_controller", "-c", "controller_manager"],
        output="both",
    )

    # rviz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        name='rviz',
        arguments=['-d', rviz_config],
        remappings=[
            ("/robot_description", "robot_description"),
        ],
    )

    return [control_node, robot_state_pub_node, cartesian_position_controller_spawner, gripper_controller_spawner]

# 关键点节点
def machinery_keypoint():
    position_csv_path = PathJoinSubstitution([config_path,namespace,'position.csv'])
    points_to_move_config = PathJoinSubstitution([config_path,namespace,'points_to_move.yaml'])
    machinery_config = PathJoinSubstitution([config_path,namespace,'machinery.yaml'])

    artificial_chess_corner_recognition_node = Node(
        package='chess_corner_recognition',
        executable='artificial_chess_corner_recognition_node',
        name='artificial_chess_corner_recognition_node',
        namespace=namespace,
        output='both',
        parameters=[{
            "position_csv_path": position_csv_path,
            "namespace": namespace
        }]
    )
    
    machinery_keypoint_node = Node(
        package="machinery_keyboard_control",
        executable="machinery_keypoint_debug_node",
        namespace=namespace,
        output="both",
        parameters=[
            {"namespace": namespace},
            points_to_move_config,
            machinery_config
        ],
        prefix="xterm -e",
    )
    return [artificial_chess_corner_recognition_node,machinery_keypoint_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    machinery_ros2control_node = [OpaqueFunction(function=machinery_ros2control)]
    machinery_keypoint_node = machinery_keypoint()

    return LaunchDescription(
        declare_parameters_node +
        machinery_ros2control_node +
        machinery_keypoint_node
    )
