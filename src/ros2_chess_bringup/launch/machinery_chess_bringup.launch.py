import os
import stat
import grp
import pwd
import yaml
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription, SomeSubstitutionsType
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

## 测试：
# 发送棋盘数据：ros2 topic pub --once /chess/points_json std_msgs/msg/String "{data: '{\"A1\": [0.353, 0.0, 0.242], \"A2\": [0.353, 0.200, 0.0242] }'}"

# 发送目标点：ros2 topic pub --once /ai_move_topic std_msgs/msg/String "{data: 'a1,a2,b1,b2'}"


config_path = LaunchConfiguration("config_path")
urdf_path = LaunchConfiguration("urdf_path")
namespace = LaunchConfiguration("namespace")
serial_port_name = LaunchConfiguration("serial_port_name")

# 声明参数
def declare_parameters():
    config_path_arg = DeclareLaunchArgument(
        name='config_path',
        default_value=str(os.path.join(get_package_share_directory('machinery_chess_bringup'),'config','machinery')),
        description='config文件夹 的 路径'
    )

    urdf_path_arg = DeclareLaunchArgument(
        name='urdf_path',
        default_value=str(os.path.join(get_package_share_directory('machinery_chess_bringup'),'urdf')),
        description='URDF文件夹 的 路径'
    )

    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value="none/",
        description="为节点和TF设置命名空间（记得命名空间后要加 / ）"
    )

    serial_port_name_arg = DeclareLaunchArgument(
        name='serial_port_name',
        default_value="/dev/ttyUSB0",
        description="机械臂的串口名称"
    )

    return [config_path_arg, urdf_path_arg, namespace_arg, serial_port_name_arg, ]

# 检查USB设备权限
def check_usb_permissions(context):
    port_name = LaunchConfiguration('serial_port_name').perform(context)
    try:
        # 检查文件是否存在
        if not os.path.exists(port_name):
            raise RuntimeError(f"设备 {port_name} 不存在")

        # 获取文件状态
        file_stat = os.stat(port_name)

        # 获取权限信息
        permissions = stat.filemode(file_stat.st_mode)
        owner_uid = file_stat.st_uid
        group_gid = file_stat.st_gid

        # 获取用户名和组名
        try:
            owner_name = pwd.getpwuid(owner_uid).pw_name
        except KeyError:
            owner_name = str(owner_uid)

        try:
            group_name = grp.getgrgid(group_gid).gr_name
        except KeyError:
            group_name = str(group_gid)

        print(f"设备: {port_name}")
        print(f"权限: {permissions}")
        print(f"所有者: {owner_name} (UID: {owner_uid})")
        print(f"所属组: {group_name} (GID: {group_gid})")

        # 检查当前用户是否有读写权限
        current_uid = os.getuid()
        current_gid = os.getgid()

        # 检查用户权限
        if current_uid == owner_uid:
            user_readable = file_stat.st_mode & stat.S_IRUSR
            user_writable = file_stat.st_mode & stat.S_IWUSR
        else:
            user_readable = file_stat.st_mode & stat.S_IROTH
            user_writable = file_stat.st_mode & stat.S_IWOTH

        print(f"当前用户ID: {current_uid}")
        print(f"当前用户组ID: {current_gid}")
        print(f"用户可读: {'是' if user_readable else '否'}")
        print(f"用户可写: {'是' if user_writable else '否'}")

        if not user_writable or not user_writable:
            raise RuntimeError(f'无法访问 {port_name}，请检查设备连接和权限设置')
        return []

    except Exception as e:
        raise RuntimeError(f"检查权限时出错: {e}")

# 节点
def start_camera():
    # 返回照片触发器
    pub_camera_msg_node = Node(
        package="pub_camera",
        executable="pub_camera_msg_node",
        namespace=namespace,
        output="both",
        parameters=[{
            "namespace": namespace
        }]
    )

    return [pub_camera_msg_node]

# ros2control节点
def machinery_ros2control(context: launch.LaunchContext):
    # 使用 context.perform_substitution 来解析 LaunchConfiguration 的值
    config_path_str = LaunchConfiguration('config_path').perform(context)
    namespace_str = namespace.perform(context)

    with open(config_path_str+'/'+namespace_str+'machinery.yaml', 'r', encoding='utf-8') as file:
        config_file = yaml.safe_load(file)

    robot_description = ParameterValue(launch.substitutions.Command([
        'xacro ', PathJoinSubstitution([LaunchConfiguration('urdf_path'), 'machinery.urdf.xacro']),
        ' origin_position:=', '"'+str(config_file['/**']['ros__parameters']['custom_origin_position'])+'"',
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

# 象棋位置映射节点
def artificial_chess_corner_recognition():
    artificial_chess_corner_recognition_node = Node(
        package='chess_corner_recognition',
        executable='artificial_chess_corner_recognition_node',
        namespace=namespace,
        name='artificial_chess_corner_recognition_node',
        output='both',
        parameters=[{
            "position_csv_path": PathJoinSubstitution([config_path,namespace,'position.csv']),
            "namespace": namespace
        }]
    )
    return [artificial_chess_corner_recognition_node]

# 机械臂控制节点
def machinery_control():
    machinery_config = PathJoinSubstitution([config_path, namespace, 'machinery.yaml'])
    machinery_control_node = Node(
        package='machinery_control',
        executable='machinery_command_control_topic_node',
        namespace=namespace,
        name='machinery_command_control_topic_node',
        output='both',
        parameters=[
            {"namespace": namespace},
            machinery_config
        ]
    )
    return [machinery_control_node]

# 汇总
def generate_launch_description():
    # 声明参数
    declare_parameter = declare_parameters()

    # 启动节点
    camera_node = start_camera()
    check_usb_permissions_node = [OpaqueFunction(function=check_usb_permissions)]
    machinery_ros2control_node = [OpaqueFunction(function=machinery_ros2control)]
    artificial_chess_corner_recognition_node = artificial_chess_corner_recognition()
    machinery_control_node = machinery_control()

    return LaunchDescription(
        declare_parameter+
        check_usb_permissions_node+
        # camera_node +
        machinery_ros2control_node +
        artificial_chess_corner_recognition_node +
        machinery_control_node
    )
