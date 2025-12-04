# ros_interface.py

import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from PySide6.QtCore import QObject, Signal, QThread
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QtRosNode(Node, QObject):
    """
    ROS 2通信节点，作为Qt前端与ROS后端之间的桥梁。
    它继承自 rclpy.node.Node 和 PySide6.QtCore.QObject 以便发送信号。
    """
    ai_move_signal = Signal(str)
    fen_signal = Signal(str)
    status_signal = Signal(str)
    log_signal = Signal(str)
    machinery_trigger_signal = Signal(bool)

    def __init__(self):
        QObject.__init__(self)
        super().__init__('qt_chinese_chess_interface')

        # 定义一个可靠的QoS配置，用于状态和命令
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # === 订阅者 ===
        self.create_subscription(String, 'ai_move_topic', self.ai_move_callback, reliable_qos)
        self.create_subscription(String, 'fen_topic', self.fen_callback, reliable_qos)
        self.create_subscription(String, 'status_topic', self.status_callback, reliable_qos)
        self.create_subscription(Bool, 'machinery_pub_image_trigger', self.machinery_trigger_callback, reliable_qos)

        # === 发布者 ===
        self.uci_publisher = self.create_publisher(String, 'uci_topic', reliable_qos)
        self.rob_uci_publisher = self.create_publisher(String, 'rob_uci_topic', reliable_qos)
        self.move_piece_publisher = self.create_publisher(Bool, 'person_pub_image_trigger', reliable_qos)
        self.machinery_trigger_publisher = self.create_publisher(Bool, 'machinery_pub_image_trigger', reliable_qos)

        self.get_logger().info("QtRosNode 初始化完成，等待消息...")

    # ---------------- ROS → Qt 信号 ----------------
    def ai_move_callback(self, msg: String):
        """接收AI走子消息并发送信号"""
        self.get_logger().info(f"收到AI走子: '{msg.data}'")
        self.ai_move_signal.emit(msg.data)

    def fen_callback(self, msg: String):
        """接收FEN字符串并发送信号"""
        self.get_logger().debug(f"收到FEN: '{msg.data}'")
        self.fen_signal.emit(msg.data)
        self.log_signal.emit("棋盘状态更新")

    def status_callback(self, msg: String):
        """接收状态消息并发送信号"""
        self.get_logger().info(f"收到状态消息: '{msg.data}'")
        self.status_signal.emit(msg.data)
        self.log_signal.emit(f"状态: {msg.data}")

    def machinery_trigger_callback(self, msg: Bool):
        self.machinery_trigger_signal.emit(msg.data)

    # ---------------- Qt → ROS 命令 ----------------
    def publish_uci_command(self, command: str):
        """向引擎发送UCI或配置命令"""
        msg = String()
        msg.data = command
        self.uci_publisher.publish(msg)
        self.log_signal.emit(f"已发送UCI命令: '{command}'")

    def publish_rob_uci_command(self, command: str):
        """向引擎发送UCI或配置命令"""
        msg = String()
        msg.data = command
        self.rob_uci_publisher.publish(msg)
        self.log_signal.emit(f"已发送ROB_UCI命令: '{command}'")

    def publish_move_piece_signal(self, trigger: bool):
        """发布人类移动棋子的信号"""
        msg = Bool()
        msg.data = trigger
        self.move_piece_publisher.publish(msg)
        self.log_signal.emit(f"已发布人类移动信号: {str(trigger)}")


    def shutdown(self):
        """在关闭前销毁节点"""
        self.get_logger().info("Shutting down QtRosNode...")
        self.destroy_node()


class RosSpinThread(QThread):
    """
    在一个独立的线程中运行ROS节点的事件循环(spin)，以防止阻塞Qt主GUI线程。
    """
    def __init__(self, node: QtRosNode):
        super().__init__()
        self.node = node
        self._is_running = True

    def run(self):
        """
        线程主函数。循环调用 rclpy.spin_once() 直到被外部停止。
        """
        try:
            while rclpy.ok() and self._is_running:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            # 在 rclpy.ok() 为 false 时，spin_once 可能会抛出异常
            if rclpy.ok():
                self.node.get_logger().error(f"ROS Spin线程异常: {e}")
        finally:
            self.node.get_logger().info("ROS Spin线程已停止。")

    def quit(self):
        """
        重写 quit 方法，设置标志位以优雅地停止线程循环。
        """
        self._is_running = False
        super().quit() # 调用父类的quit来处理事件循环
