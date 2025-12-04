import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import tf2_ros
import sys
import select
import tty
import termios
import json
import re
import ast

class MachineryKeypointDebug(Node):
    def __init__(self):
        super().__init__('machinery_keypoint_debug_node')

        # 参数
        self.declare_parameter('namespace','left/')
        self.namespace_ = self.get_parameter('namespace').get_parameter_value().string_value
        self.declare_parameter('points_to_move',"['a0','a9','i0','i9']")
        self.points_to_move = self.get_parameter('points_to_move').get_parameter_value().string_value
        self.declare_parameter('origin_position','[243.0, 0.0, 444.2]')
        origin_position_str = self.get_parameter('origin_position').get_parameter_value().string_value
        self.origin_position = ast.literal_eval(origin_position_str)  # 转换为列表

        self.points = { # 定义按键映射
            'h': (self.origin_position[0], self.origin_position[1], self.origin_position[2]),
            'q': (0, 0, 10),            # Z+
            'e': (0, 0, -10),           # Z-
        }
        self.step = 1.0 # 移动步长 (mm)
        self.is_chess_points_received = False

        # 发布者和接收者
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(PointStamped,'cartesian_position_controller/command',10)
        self.chess_points_json_subscriber_ = self.create_subscription(String,'chess/points_json',self.chess_points_json_callback,10)
        self.timer = self.create_timer(1,self.run)

        # 从硬件接口获取初始/HOME位置
        self.home_x = self.points['h'][0]
        self.home_y = self.points['h'][1]
        self.home_z = self.points['h'][2]

        # 初始化当前目标位置
        self.target_x = self.home_x
        self.target_y = self.home_y
        self.target_z = self.home_z
        self.current_point = 'h'

        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)

    def chess_points_json_callback(self, msg:String):
        if self.is_chess_points_received == False:
            # 读取需要走的点
            try:
                points_to_move = ast.literal_eval(self.points_to_move)
                is_list=True
            except (ValueError, SyntaxError):
                points_to_move = self.points_to_move
                is_list=False

            # 解析json数据
            json_string = msg.data
            json_data = json.loads(json_string)
            for point in json_data:
                if is_list==True:
                    if point in points_to_move:
                        self.points[point] = tuple([item * 1000.0 for item in json_data[point]])
                elif is_list==False and points_to_move=='all':
                    self.points[point] = tuple([item * 1000.0 for item in json_data[point]])
                else:
                    self.get_logger().error('非法输入')
                    return
            self.get_logger().info('成功接收棋盘点位')
            self.is_chess_points_received = True

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        if self.is_chess_points_received == True:
            self.timer.destroy()
            # 回到原点
            self.publish_target()

            while rclpy.ok():
                key = self.get_key()

                if key == '\x03':  # Ctrl+C
                    break

                current_point_match = re.match(r'([a-zA-Z]+)([0-9]+)', self.current_point)
                if key=='w':
                    if self.current_point!='h':
                        min = 9999
                        for point in self.points.keys():
                            temp_point_match = re.match(r'([a-zA-Z]+)([0-9]+)', point)
                            if temp_point_match is not None and temp_point_match.group(1)!='o' and ord(temp_point_match.group(1))-ord(current_point_match.group(1))<min and ord(temp_point_match.group(1))-ord(current_point_match.group(1))>0:
                                min = ord(temp_point_match.group(1))-ord(current_point_match.group(1))
                                self.current_point = str(temp_point_match.group(1)+current_point_match.group(2))
                        self.target_x = self.points[self.current_point][0]
                        self.target_y = self.points[self.current_point][1]
                        self.target_z = self.points[self.current_point][2]

                        
                        self.publish_target()

                elif key=='s':
                    if self.current_point!='h':
                        min = 9999
                        for point in self.points.keys():
                            temp_point_match = re.match(r'([a-zA-Z]+)([0-9]+)', point)
                            if temp_point_match is not None and -ord(temp_point_match.group(1))+ord(current_point_match.group(1))<min and -ord(temp_point_match.group(1))+ord(current_point_match.group(1))>0:
                                min = -ord(temp_point_match.group(1))+ord(current_point_match.group(1))
                                self.current_point = str(temp_point_match.group(1)+current_point_match.group(2))
                        self.target_x = self.points[self.current_point][0]
                        self.target_y = self.points[self.current_point][1]
                        self.target_z = self.points[self.current_point][2]

                        
                        self.publish_target()

                elif key=='a':
                    if self.current_point=='h':
                        self.target_x = self.points['a9'][0]
                        self.target_y = self.points['a9'][1]
                        self.target_z = self.points['a9'][2]
                        self.current_point='a9'
                    else:
                        min = 9999
                        for point in self.points.keys():
                            temp_point_match = re.match(r'([a-zA-Z]+)([0-9]+)', point)
                            if temp_point_match is not None and ord(temp_point_match.group(2))-ord(current_point_match.group(2))<min and ord(temp_point_match.group(2))-ord(current_point_match.group(2))>0:
                                min = ord(temp_point_match.group(2))-ord(current_point_match.group(2))
                                self.current_point = str(current_point_match.group(1)+temp_point_match.group(2))
                        self.target_x = self.points[self.current_point][0]
                        self.target_y = self.points[self.current_point][1]
                        self.target_z = self.points[self.current_point][2]
                    self.publish_target()

                elif key=='d':
                    if self.current_point=='h':
                        self.target_x = self.points['a0'][0]
                        self.target_y = self.points['a0'][1]
                        self.target_z = self.points['a0'][2]
                        self.current_point='a0'
                    else:
                        min = 9999
                        for point in self.points.keys():
                            temp_point_match = re.match(r'([a-zA-Z]+)([0-9]+)', point)
                            if temp_point_match is not None and -ord(temp_point_match.group(2))+ord(current_point_match.group(2))<min and -ord(temp_point_match.group(2))+ord(current_point_match.group(2))>0:
                                min = -ord(temp_point_match.group(2))+ord(current_point_match.group(2))
                                self.current_point = str(current_point_match.group(1)+temp_point_match.group(2))
                        self.target_x = self.points[self.current_point][0]
                        self.target_y = self.points[self.current_point][1]
                        self.target_z = self.points[self.current_point][2]
                    self.publish_target()

                elif key == 'q':
                    self.target_x += self.points['q'][0]
                    self.target_y += self.points['q'][1]
                    self.target_z += self.points['q'][2]
                    self.publish_target()

                elif key == 'e':
                    self.target_x += self.points['e'][0]
                    self.target_y += self.points['e'][1]
                    self.target_z += self.points['e'][2]
                    self.publish_target()

                elif key == 'h':
                    self.target_x = self.points['h'][0]
                    self.target_y = self.points['h'][1]
                    self.target_z = self.points['h'][2]
                    self.current_point='h'
                    self.get_logger().info("回到起点")
                    self.publish_target()
        else:
            self.get_logger().info('未接收棋盘点位')

    def publish_target(self):
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.namespace_+'base_link'  # 或者您的机器人基座标系
        point_msg.point.x = float(self.target_x)
        point_msg.point.y = float(self.target_y)
        point_msg.point.z = float(self.target_z)
        self.publisher_.publish(point_msg)
        self.get_logger().info(f"发送目标 {self.current_point}: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}")

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = MachineryKeypointDebug()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"键盘控制节点出错: {e}")
    finally:
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()
