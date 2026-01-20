import os
import rclpy
import subprocess
import signal
import yaml
from PySide6.QtWidgets import QMainWindow, QStackedWidget, QWidget, QVBoxLayout
from PySide6.QtCore import Slot, Qt,QThread, Signal
from PySide6.QtGui import QCursor
from ros_interface import QtRosNode, RosSpinThread
from pages.start_page import StartPage
from pages.mode_choose_page import ModeChoosePage
from pages.settings_page import SettingsPage
from pages.game_page import GamePage
from pages.history_page import HistoryPage

class RosTerminatorThread(QThread):
    finished_signal = Signal()

    def __init__(self, processes, ros_node):
        super().__init__()
        self.processes = processes.copy()
        self.ros_node = ros_node

    def run(self):
        if not self.processes:
            self.finished_signal.emit()
            return

        self.ros_node.log_signal.emit("正在关闭 ROS launch 进程... (SIGINT)")
        for process in self.processes:
            if process.poll() is None:
                try:
                    pgid = os.getpgid(process.pid)
                    os.killpg(pgid, signal.SIGINT)
                    self.ros_node.log_signal.emit(f"已向进程组 {pgid} 发送 SIGINT 信号。")
                except (OSError, ProcessLookupError):
                    self.ros_node.log_signal.emit(f"尝试关闭进程 {process.pid} 时失败，可能已退出。")
        graceful_shutdown_timeout = 5.0
        try:
            for process in self.processes:
                process.wait(timeout=graceful_shutdown_timeout)
            self.ros_node.log_signal.emit("所有 ROS launch 进程已正常关闭。")
        except subprocess.TimeoutExpired:
            self.ros_node.log_signal.emit(f"部分进程在 {graceful_shutdown_timeout} 秒内未能关闭，将强制终止 (SIGKILL)...")
            for process in self.processes:
                if process.poll() is None:
                    try:
                        pgid = os.getpgid(process.pid)
                        os.killpg(pgid, signal.SIGKILL)
                        self.ros_node.log_signal.emit(f"已向进程组 {pgid} 发送 SIGKILL 信号。")
                    except (OSError, ProcessLookupError):
                        self.ros_node.log_signal.emit(f"尝试强制关闭进程 {process.pid} 时失败，可能已退出。")
        for process in self.processes:
            if process.poll() is None:
                process.wait()
        self.finished_signal.emit()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("中国象棋 Qt/ROS 2 前端")
        self.resize(1920, 1080)

        # 限制最小比例
        self.setMinimumSize(960, 540)

        # 用于存储ROS launch子进程的句柄
        self.ros_launch_processes = []

        # 初始化ROS2
        self.init_ros()

        # 优雅地处理 Ctrl+C
        signal.signal(signal.SIGINT, lambda sig, frame: self.close())
        # 在非Windows系统上，设置preexec_fn以将子进程放入新的进程组
        # 这可以防止终端的Ctrl+C信号直接影响到子进程
        if os.name != 'nt':
            self.preexec_fn = os.setpgrp
        else:
            self.preexec_fn = None

        # 设置UI
        self.setup_ui()

        # 启动ROS线程
        self.ros_thread.start()

    def init_ros(self):
        """初始化ROS2组件"""
        if not rclpy.ok():
            rclpy.init()

        self.ros_node = QtRosNode()
        self.ros_thread = RosSpinThread(self.ros_node)

    def setup_ui(self):
        """设置主界面和页面堆栈"""
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)

        # 创建页面堆栈
        self.stacked_widget = QStackedWidget()

        # 创建各个页面
        self.start_page = StartPage(self.ros_node)
        self.mode_choose_page = ModeChoosePage(self.ros_node)  # 新增模式选择页
        self.history_page = HistoryPage() # 实例化历史页面
        self.settings_page = None  # 延迟创建，因为需要知道游戏模式
        self.game_page = None

        # 添加页面到堆栈
        self.stacked_widget.addWidget(self.start_page)
        self.stacked_widget.addWidget(self.mode_choose_page)
        self.stacked_widget.addWidget(self.history_page) # 添加到堆栈


        # 连接页面切换信号
        self.history_page.back_signal.connect(self.show_start_page)
        self.start_page.history_signal.connect(self.show_history_page)

        self.start_page.settings_signal.connect(self.show_mode_choose_page)  # 修改：先到模式选择页
        self.start_page.quit_signal.connect(self.close)

        self.mode_choose_page.back_signal.connect(self.show_start_page)      # 模式选择页返回
        self.mode_choose_page.mode_selected_signal.connect(self.show_settings_page)  # 模式选择后到设置页
        main_layout.addWidget(self.stacked_widget)
        self.setCentralWidget(central_widget)

        # 默认显示开始页面
        self.show_start_page()

    @Slot()
    def show_start_page(self):
        self.terminate_ros_nodes()  # 返回主页时，关闭所有ROS launch进程
        self.stacked_widget.setCurrentWidget(self.start_page)

    @Slot()
    def show_mode_choose_page(self):
        """显示模式选择页面"""
        # 当返回到模式选择页时，关闭可能已提前启动的ROS节点
        self.terminate_ros_nodes()
        self.stacked_widget.setCurrentWidget(self.mode_choose_page)

    @Slot(str)
    def show_settings_page(self, game_mode):
        """显示设置页面，现在可以识别 'VOICE_AI'"""
        self.setCursor(QCursor(Qt.WaitCursor))

        # 此时 game_mode 可能是 "HUMAN_AI", "AI_AI" 或 "VOICE_AI"
        self.ros_node.get_logger().info(f"MainWindow 接收到游戏模式信号: {game_mode}")

        # 根据模式启动 ROS 节点 (后续您需要在 config.yaml 中增加 VOICE_AI 对应配置)
        self.launch_ros_nodes(game_mode) 

        if self.settings_page is not None:
            self.stacked_widget.removeWidget(self.settings_page)
            self.settings_page.deleteLater()
            self.settings_page = None

        # 创建 SettingsPage 实例，将 game_mode 传进去
        self.settings_page = SettingsPage(self.ros_node, game_mode=game_mode)
        
        # self.settings_page.setup_ui() 

        self.stacked_widget.addWidget(self.settings_page)
        self.settings_page.back_signal.connect(self.show_mode_choose_page)
        self.settings_page.start_game_signal.connect(self.show_game_page)

        self.stacked_widget.setCurrentWidget(self.settings_page)
        self.unsetCursor()
    
    @Slot()
    def show_history_page(self):
        """切换到历史记录页面"""
        self.history_page.refresh_file_list() # 每次进入时刷新列表
        self.stacked_widget.setCurrentWidget(self.history_page)

    @Slot(dict)  # 接收 {"config":..., "player_color":..., "game_mode":...}
    def show_game_page(self, data):
        config_msg_1 = data["config_1"]
        config_msg_2 = data["config_2"]
        player_color = data["player_color"]
        game_mode = data.get("game_mode")  # 获取游戏模式

        # 如果已有旧的 game_page，先移除
        if self.game_page is not None:
            self.stacked_widget.removeWidget(self.game_page)
            self.game_page.deleteLater()
            self.game_page = None

        # 创建新的 game_page 并传递 player_color 和 game_mode
        self.game_page = GamePage(self.ros_node, player_color=player_color,game_mode=game_mode)
        self.game_page.game_mode = game_mode  # 传递游戏模式到游戏页
        self.stacked_widget.addWidget(self.game_page)

        # 连接信号
        # 1. 游戏结束后返回主页的信号
        self.game_page.back_signal.connect(self.show_start_page)
        # 2. 游戏页面请求结束游戏的信号
        self.game_page.end_game_request_signal.connect(self.handle_game_end_request)

        # 切换显示
        self.stacked_widget.setCurrentWidget(self.game_page)



    @Slot()
    def handle_game_end_request(self):
        """处理来自GamePage的结束游戏请求"""
        self.terminate_ros_nodes() # 立即开始关闭ROS节点
        self.game_page.show_wait_dialog_and_go_back() # 通知GamePage显示等待对话框

    def launch_ros_nodes(self, game_mode):
        """根据游戏模式启动一个或多个 ROS2 launch 进程"""
        # 先确保之前的进程已关闭
        self.terminate_ros_nodes()

        # 定义不同模式下需要启动的launch文件和参数
        # 读取配置文件
        current_file = os.path.abspath(__file__)
        current_dir = os.path.dirname(current_file)
        with open(current_dir+'/'+'config/config.yaml','r',encoding='utf-8') as file:
            launch_config = yaml.safe_load(file)

        if game_mode not in launch_config:
            self.ros_node.get_logger().warn(f"未找到游戏模式 '{game_mode}' 的launch配置，跳过启动。")
            return

        for launch_file in launch_config[game_mode]:
            command = [
                'ros2', 'launch',
                'ros2_chess_bringup',  # 你的功能包名称
                launch_file['name']
            ]
            for param_name, param_value in launch_file['parameters'].items():
                command.append(f"{param_name}:={param_value}")

            self.ros_node.log_signal.emit(f"正在启动: {' '.join(command)}")

            # 启动子进程并保存句柄
            # preexec_fn=os.setpgrp 会在新进程中、执行命令前调用os.setpgrp()
            process = subprocess.Popen(command, preexec_fn=self.preexec_fn)
            self.ros_launch_processes.append(process)

    def terminate_ros_nodes(self):
        """后台线程终止所有由本程序启动的 ROS2 launch 进程"""
        if not self.ros_launch_processes:
            return
        # 避免重复启动线程
        if hasattr(self, "ros_terminator_thread") and self.ros_terminator_thread.isRunning():
            return
        self.ros_terminator_thread = RosTerminatorThread(self.ros_launch_processes, self.ros_node)
        self.ros_terminator_thread.finished_signal.connect(self.on_ros_terminated)
        self.ros_terminator_thread.start()

    def on_ros_terminated(self):
        self.ros_launch_processes.clear()
        self.ros_node.log_signal.emit("ROS launch 进程关闭线程已完成。")


    def closeEvent(self, event):
        """安全退出时关闭ROS线程"""
        self.ros_node.log_signal.emit("正在关闭应用程序...")
        self.terminate_ros_nodes()  # 关闭所有ROS launch进程
        if self.ros_thread.isRunning():
            self.ros_thread.quit()
            self.ros_thread.wait() # 等待ROS自旋线程结束
        super().closeEvent(event)