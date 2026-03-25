from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 语音节点
        Node(
            package='cchess_decision',
            executable='rob_voice_node',
            name='rob_voice_node',
            output='screen',
            parameters=[]
        ),
        
        # 机器人引擎节点
        Node(
            package='cchess_decision',
            executable='rob_engine_node',
            name='rob_engine_node',
            output='screen',
            parameters=[]
        )
    ])