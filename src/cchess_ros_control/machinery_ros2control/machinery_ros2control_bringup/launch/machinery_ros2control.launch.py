import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument , OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 该模块仅用来测试 ros2_control 是否正常运行

# 测试：
# ros2 topic pub --once  /cartesian_position_controller/command geometry_msgs/msg/PointStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, point: {x: 320.0, y: 0.0, z: 100.0}}"

# ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"

config_path = LaunchConfiguration("config_path")
urdf_path = LaunchConfiguration("urdf_path")
namespace = LaunchConfiguration("namespace")
serial_port_name = LaunchConfiguration("serial_port_name")


def declare_parameters():
    serial_port_name_arg = DeclareLaunchArgument(
        name='serial_port_name',
        default_value="/dev/ttyUSB0",
        description="机械臂的串口名称"
    )

    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value="left/",
        description="域名空间"
    )

    urdf_path_arg = DeclareLaunchArgument(
        name='urdf_path',
        default_value=str(get_package_share_directory('machinery_ros2control_bringup')+'/urdf'),
        description='URDF 的绝对路径'
    )

    config_path_arg = DeclareLaunchArgument(
        name='config_path',
        default_value=str(get_package_share_directory('machinery_ros2control_bringup')+'/config'),
        description='config 的绝对路径'
    )

    return [config_path_arg, urdf_path_arg, namespace_arg, serial_port_name_arg]

def machinery_ros2control(context: launch.LaunchContext):
    config_path_str = LaunchConfiguration('config_path').perform(context)
    namespace_str = namespace.perform(context)

    with open(config_path_str+'/'+namespace_str+'machinery.yaml', 'r', encoding='utf-8') as file:
        config_file = yaml.safe_load(file)

    robot_description = ParameterValue(launch.substitutions.Command([
        'xacro ', PathJoinSubstitution([LaunchConfiguration('urdf_path'), 'machinery.urdf.xacro']),
        ' origin_position:=', '"'+str(config_file['/**']['ros__parameters']['origin_position'])+'"',
        ' custom_origin_position:=', '"'+str(config_file['/**']['ros__parameters']['custom_origin_position'])+'"',
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

    return [control_node, robot_state_pub_node, cartesian_position_controller_spawner, gripper_controller_spawner, rviz_node]

def generate_launch_description():

    declare_parameters_node = declare_parameters()
    machinery_ros2control_node = [OpaqueFunction(function=machinery_ros2control)]

    return LaunchDescription(
        declare_parameters_node +
        machinery_ros2control_node
    )
