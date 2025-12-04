import json
import os.path
import csv
import std_msgs
import rclpy
import std_msgs.msg
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class ArtificialChessCornerRecognitionNode(Node):
    def __init__(self):
        super().__init__('artificial_chess_corner_recognition_node')

        # 参数
        self.declare_parameter('position_csv_path', os.path.join(get_package_share_directory('chess_corner_recognition'), 'resource', 'position.csv'))
        self.positon_csv_path = self.get_parameter('position_csv_path').get_parameter_value().string_value

        # 变量
        self.points={}
        self.current_point_index = 0
        self.get_logger().info('棋盘格识别节点已启动。')

        self.qt_debug_msg_pub = self.create_publisher(std_msgs.msg.String,'/status_topic',10)
        self.ChessPointsPub = self.create_publisher(std_msgs.msg.String,'chess/points_json',10)
        self.timer = self.create_timer(timer_period_sec=0.3, callback=self.pub_points_msg)

        # 打开文件
        try:
            with open(self.positon_csv_path, 'r', encoding='utf-8') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    self.points[row['point']] = [float(row['x'])/1000, float(row['y'])/1000, float(row['z'])/1000]  # 单位为m
            self.get_logger().info(f'成功加载 {len(self.points)} 个点位数据。')
            self.qt_debug_msg_pub.publish(std_msgs.msg.String(data=f'[{self.get_name()}]成功加载 {len(self.points)} 个点位数据。'))
        except FileNotFoundError:
            self.get_logger().error(f'找不到点位文件: {self.positon_csv_path}')
            self.qt_debug_msg_pub.publish(std_msgs.msg.String(data=f'[{self.get_name()}]找不到点位文件: {self.positon_csv_path}'))
        except Exception as e:
            self.get_logger().error(f'读取点位文件时发生错误: {e}')
            self.qt_debug_msg_pub.publish(std_msgs.msg.String(data=f'[{self.get_name()}]读取点位文件时发生错误: {e}'))

    def pub_points_msg(self):
        points_json = json.dumps(self.points)
        point_json_string = std_msgs.msg.String()
        point_json_string.data = points_json
        self.ChessPointsPub.publish(point_json_string)


def main(args=None):
    rclpy.init(args=args)
    chess_node = ArtificialChessCornerRecognitionNode()
    try:
        rclpy.spin(chess_node)
        chess_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("程序被打断，节点已关闭")
