from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    artificial_chess_corner_recognition_node = Node(
        package='chess_corner_recognition',
        executable='artificial_chess_corner_recognition_node',
        name='artificial_chess_corner_recognition_node',
        output='screen',
        parameters=[{
        }]
    )

    return LaunchDescription([
        artificial_chess_corner_recognition_node
    ])
