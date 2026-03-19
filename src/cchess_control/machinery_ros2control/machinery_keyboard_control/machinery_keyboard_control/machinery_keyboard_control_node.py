import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
import csv
import sys
import select
import tty
import ast
import termios

# 定义按键映射
key_bindings = {
    'w': (1, 0, 0),    # X+
    's': (-1, 0, 0),   # X-
    'a': (0, 1, 0),    # Y+
    'd': (0, -1, 0),   # Y-
    'q': (0, 0, 1),    # Z+
    'e': (0, 0, -1),   # Z-
}

class MachineryKeyboardControl(Node):
    def __init__(self):
        super().__init__('machinery_keyboard_control_node')

        # 获取参数
        self.declare_parameter('namespace','left/')
        self.namespace_ = self.get_parameter('namespace').get_parameter_value().string_value
        self.declare_parameter('origin_position','[180.0, 0.0, 444.2]')
        origin_position_str = self.get_parameter('origin_position').get_parameter_value().string_value
        self.origin_position = ast.literal_eval(origin_position_str)  # 转换为列表
        self.declare_parameter('csv_path','')
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.csv_points = self.load_csv_points()
        self.current_csv_index = 0
        self.pending_confirm_point = False

        # 创建发布者，话题名称必须与控制器订阅的名称一致
        self.publisher_ = self.create_publisher(JointJog,'cartesian_position_controller/reference',10)

        # 初始化当前目标位置
        self.target_x = self.origin_position[0]
        self.target_y = self.origin_position[1]
        self.target_z = self.origin_position[2]

        # 移动步长 (mm)
        self.step = 1.0

        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("DEBUG关键点节点已启动")
        self.log_current_csv_point()
        self.publish_target() # 发布初始位置

    def load_csv_points(self):
        if not self.csv_path:
            self.get_logger().warn("未配置 csv_path，空格确认功能不可用")
            return []

        points = []
        try:
            with open(self.csv_path, 'r', encoding='utf-8', newline='') as csv_file:
                reader = csv.DictReader(csv_file)
                required_fields = {'point', 'x', 'y', 'z'}
                if reader.fieldnames is None or not required_fields.issubset(reader.fieldnames):
                    self.get_logger().error(f"CSV格式错误，必须包含表头: {sorted(required_fields)}，当前文件: {self.csv_path}")
                    return []

                for row in reader:
                    if not row.get('point'):
                        continue
                    points.append({
                        'point': row['point'],
                        'x': row['x'],
                        'y': row['y'],
                        'z': row['z'],
                    })
        except FileNotFoundError:
            self.get_logger().error(f"未找到CSV文件: {self.csv_path}")
            return []
        except Exception as exc:
            self.get_logger().error(f"读取CSV文件失败: {exc}")
            return []

        self.get_logger().info(f"已读取 {len(points)} 个CSV点位: {self.csv_path}")
        return points

    def log_current_csv_point(self):
        if not self.csv_points:
            return

        if self.current_csv_index >= len(self.csv_points):
            self.get_logger().info("CSV中的点位已全部处理完成")
            return

    def request_update_current_point(self):
        if not self.csv_points:
            self.get_logger().warn("没有可修改的CSV点位")
            return

        if self.current_csv_index >= len(self.csv_points):
            self.get_logger().info("CSV中的点位已全部处理完成，无需再修改")
            return

        point_name = self.csv_points[self.current_csv_index]['point']
        self.pending_confirm_point = True
        self.get_logger().info(f"是否要修改 {point_name} 点？再次按空格确认写入，按其他键取消")

    def save_current_point_to_csv(self):
        if not self.csv_points:
            self.get_logger().warn("没有可修改的CSV点位")
            return

        if self.current_csv_index >= len(self.csv_points):
            self.get_logger().info("CSV中的点位已全部处理完成，无需再修改")
            return

        current_point = self.csv_points[self.current_csv_index]
        current_point['x'] = f"{self.target_x:.2f}"
        current_point['y'] = f"{self.target_y:.2f}"
        current_point['z'] = f"{self.target_z:.2f}"

        try:
            with open(self.csv_path, 'w', encoding='utf-8', newline='') as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=['point', 'x', 'y', 'z'])
                writer.writeheader()
                writer.writerows(self.csv_points)
        except Exception as exc:
            self.get_logger().error(f"写入CSV文件失败: {exc}")
            return

        self.get_logger().info(
            f"已更新点 {current_point['point']}: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}"
        )
        self.current_csv_index += 1
        self.log_current_csv_point()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == '\x03':  # Ctrl+C
                break

            if self.pending_confirm_point:
                if key == ' ':
                    self.pending_confirm_point = False
                    self.save_current_point_to_csv()
                    continue

                self.pending_confirm_point = False
                self.get_logger().info("未连续按下两次空格，已取消本次修改")

            if key in key_bindings:
                self.target_x += key_bindings[key][0] * self.step
                self.target_y += key_bindings[key][1] * self.step
                self.target_z += key_bindings[key][2] * self.step
                self.publish_target()
            elif key == '+':
                self.step *= 1.5
                self.get_logger().info(f"增加每次前进距离到: {self.step:.2f} mm")
            elif key == '-':
                self.step /= 1.5
                self.get_logger().info(f"减少每次前进距离到: {self.step:.2f} mm")
            elif key == 'h':
                self.target_x, self.target_y, self.target_z = self.origin_position[0], self.origin_position[1], self.origin_position[2]
                self.get_logger().info("回到起点")
                self.publish_target()
            elif key == ' ':
                self.request_update_current_point()

    def publish_target(self):
        point_msg = JointJog()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.namespace_+'base_link'
        point_msg.joint_names = [self.namespace_+'gripper_position']
        point_msg.displacements = [float(self.target_x),float(self.target_y),float(self.target_z)]
        self.publisher_.publish(point_msg)
        self.get_logger().info(f"发送目标: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}")

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = MachineryKeyboardControl()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"键盘控制节点出错: {e}")
    finally:
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()
