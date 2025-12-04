from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # 启动Gazebo空世界
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={'world': 'empty.world'}.items()
        ),

        # 静态TF变换 (base_link到base_footprint)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),

        # 在Gazebo中生成URDF模型
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=[
                '-file', PathJoinSubstitution([
                    FindPackageShare('machinery_model'),
                    'urdf/machinery_model.urdf.xacro'
                ]),
                '-entity', 'machinery_model'
            ],
            output='screen'
        ),

        # 发布校准消息 (ROS2中使用ros2 topic pub替代rostopic)
        Node(
            package='ros2topic',
            executable='ros2topic',
            name='fake_joint_calibration',
            arguments=['pub', '/calibrated', 'std_msgs/Bool', '{data: true}'],
            output='screen'
        )
    ])