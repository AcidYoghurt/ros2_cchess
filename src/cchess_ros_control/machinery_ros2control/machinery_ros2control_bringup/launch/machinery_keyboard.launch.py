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

# 测试：ros2 launch machinery_ros2control_bringup machinery_keyboard.launch.py serial_port_name:=/dev/ttyUSB0 namespace:=left/ baud_rate:=115200

config_path = LaunchConfiguration("config_path")
urdf_path = LaunchConfiguration("urdf_path")
serial_port_name = LaunchConfiguration("serial_port_name")
baud_rate = LaunchConfiguration("baud_rate")
namespace = LaunchConfiguration("namespace")

# 声明参数
def declare_parameters():
    serial_port_name_arg = DeclareLaunchArgument(
        name='serial_port_name',
        default_value="/dev/USB0",
        description="机械臂的串口名称"
    )

    baud_rate_arg = DeclareLaunchArgument(
        name='baud_rate',
        default_value="115200",
        description="机械臂的串口波特率"
    )

    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value="left/",
        description="为节点和TF设置命名空间"
    )

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
    return [config_path_arg, urdf_path_arg, namespace_arg, serial_port_name_arg, baud_rate_arg]

# ros2control节点
def machinery_ros2control(context: launch.LaunchContext):
    # 使用 context.perform_substitution 来解析 LaunchConfiguration 的值
    config_path_str = LaunchConfiguration('config_path').perform(context)
    namespace_str = namespace.perform(context)

    with open(config_path_str+'/machinery/'+namespace_str+'machinery.yaml', 'r', encoding='utf-8') as file:
        config_file = yaml.safe_load(file)

    robot_description = ParameterValue(launch.substitutions.Command([
        'xacro ', PathJoinSubstitution([LaunchConfiguration('urdf_path'), 'machinery.urdf.xacro']),
        ' origin_position:=', '"'+str(config_file['/**']['ros__parameters']['origin_position'])+'"',
        ' custom_origin_position:=', '"'+str(config_file['/**']['ros__parameters']['custom_origin_position'])+'"',
        ' frame_prefix:=', namespace,
        ' serial_port_name:=', serial_port_name,
        ' baud_rate:=',baud_rate
    ]), value_type=str)
    robot_controllers_config = PathJoinSubstitution([config_path, 'machinery', namespace, 'machinery_controllers.yaml'])
    rviz_config = PathJoinSubstitution([config_path, 'machinery', namespace, 'urdf.rviz'])

    # 启动 ros2_control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        output="both",
        parameters=[
            {"robot_description": robot_description},   # 这个参数让ros2_control知道：这个机器人“有哪些硬件 + 接口”
            robot_controllers_config                    # 传入controller的参数
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
        arguments=["gripper_suction_controller", "-c", "controller_manager"],
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

# 键盘节点
def machinery_keypoint():
    machinery_config = PathJoinSubstitution([config_path,'machinery',namespace,'machinery.yaml'])
    
    machinery_keyboard_node = Node(
        package="machinery_keyboard_control",
        executable="machinery_keyboard_control_node",
        namespace=namespace,
        output="both",
        parameters=[
            {"namespace": namespace},
            machinery_config
        ],
        prefix="xterm -e",
    )
    return [machinery_keyboard_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    machinery_ros2control_node = [OpaqueFunction(function=machinery_ros2control)]
    machinery_keypoint_node = machinery_keypoint()

    return LaunchDescription(
        declare_parameters_node +
        machinery_ros2control_node +
        machinery_keypoint_node
    )
