import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


## 测试：
# 发送棋盘数据：ros2 topic pub --once /chess/points_json std_msgs/msg/String "{data: '{\"A1\": [0.353, 0.0, 0.242], \"A2\": [0.353, 0.200, 0.0242] }'}"

# 发送目标点：ros2 topic pub --once /left/ai_move_topic std_msgs/msg/String "{data: 'a1,a2,b1,b2'}"

namespace = LaunchConfiguration('namespace')
serial_port_name = LaunchConfiguration('serial_port_name')
config_path = LaunchConfiguration('config_path')
urdf_path = LaunchConfiguration('urdf_path')
baud_rate = LaunchConfiguration("baud_rate")

# 声明参数
def declare_parameters():
    config_path_arg = DeclareLaunchArgument(
        name='config_path',
        default_value=str(os.path.join(get_package_share_directory('machinery_chess_ros_control_bringup'),'config','machinery')),
        description='config文件夹 的 路径'
    )

    urdf_path_arg = DeclareLaunchArgument(
        name='urdf_path',
        default_value=str(os.path.join(get_package_share_directory('machinery_chess_ros_control_bringup'),'urdf')),
        description='URDF文件夹 的 路径'
    )

    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value="left/",
        description="为节点和TF设置命名空间（记得命名空间后要加 / ）"
    )

    serial_port_name_arg = DeclareLaunchArgument(
        name='serial_port_name',
        default_value="/dev/ttyUSB0",
        description="机械臂的串口名称"
    )

    baud_rate_arg = DeclareLaunchArgument(
        name='baud_rate',
        default_value="115200",
        description="机械臂的串口波特率"
    )

    return [config_path_arg, urdf_path_arg, namespace_arg, serial_port_name_arg, baud_rate_arg]

# 节点
def cchess_ros_control():
    cchess_ros_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('machinery_chess_ros_control_bringup'),
                'launch',
                'machinery_chess_ros_control_bringup.launch.py'
            )
        ),
        launch_arguments={
            'namespace': namespace,
            'serial_port_name': serial_port_name,
            'config_path': config_path,
            'urdf_path': urdf_path,
            'baud_rate': baud_rate,
        }.items()
    )
    return [cchess_ros_control_launch]

# 汇总
def generate_launch_description():
    # 声明参数
    declare_parameter = declare_parameters()

    # 启动节点
    cchess_ros_control_launch = cchess_ros_control()

    return LaunchDescription(
        declare_parameter+
        cchess_ros_control_launch
    )
