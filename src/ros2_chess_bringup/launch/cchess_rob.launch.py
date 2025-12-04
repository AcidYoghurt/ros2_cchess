from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 象棋引擎节点
        Node(
            package='cchess_ros',
            executable='chess_engine_node',
            name='chess_engine_node',
            output='screen',
            parameters=[]
        ),
        
        # 机器人引擎节点
        Node(
            package='cchess_ros',
            executable='rob_engine_node',
            name='rob_engine_node',
            output='screen',
            parameters=[]
        )
    ])