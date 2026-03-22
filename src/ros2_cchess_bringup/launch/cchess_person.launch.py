from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 棋盘识别节点
        Node(
            package='cchess_ros',
            executable='chessboard_recognizer_node',
            name='chessboard_recognizer_node',
            output='screen',
            parameters=[]  # 可以在此添加参数文件或参数字典
        ),
        
        # 象棋引擎节点
        Node(
            package='cchess_ros',
            executable='chess_engine_node',
            name='chess_engine_node',
            output='screen',
            parameters=[]
        ),
        
        # 相机发布节点
        Node(
            package='cchess_ros',
            executable='chess_camera_publisher',
            name='chess_camera_publisher',
            output='screen',
            parameters=[]
        )
    ])