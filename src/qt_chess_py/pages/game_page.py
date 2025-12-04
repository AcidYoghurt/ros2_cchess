# qt_chess_p/pages/game_page.py

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTextEdit, QSizePolicy, QDialog, QLineEdit, QMessageBox,
    QProgressBar
)
from PySide6.QtCore import Signal, Slot, Qt, QTimer, QEvent
from PySide6.QtGui import QFont, QPixmap, QImage, QPainter,QColor
from PySide6.QtCore import QSize
import os, time


# qt_chess_p/pages/game_page.py

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTextEdit, QSizePolicy, QDialog, QLineEdit, QMessageBox,
    QProgressBar
)
from PySide6.QtCore import Signal, Slot, Qt, QTimer, QEvent, QPropertyAnimation, QEasingCurve
from PySide6.QtGui import QFont, QPixmap, QImage, QPainter, QColor
from PySide6.QtCore import QSize
import os, time


# ================= 自定义消息框 =================
class CustomMessageBox(QDialog):
    def __init__(self, parent=None, title="提示", message="", color="#f39c12", timeout=2000):
        super().__init__(parent)
        self.timeout = timeout
        self.setup_ui(title, message, color)
        self.setup_animation()
        
    def setup_ui(self, title, message, color):
        self.setWindowTitle(title)
        self.setModal(False)  # 非模态，不阻塞其他窗口
        self.setFixedSize(600, 300)  # 固定大小
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 30, 30, 30)
        
        # 消息标签
        self.message_label = QLabel(message)
        self.message_label.setAlignment(Qt.AlignCenter)
        self.message_label.setFont(QFont("Microsoft YaHei", 20, QFont.Bold))
        self.message_label.setWordWrap(True)
        self.message_label.setStyleSheet(f"""
            QLabel {{
                color: white;
                background-color: {color};
                padding: 30px;
                border-radius: 15px;
                font-size: 20px;
            }}
        """)
        
        layout.addWidget(self.message_label)
        
    def setup_animation(self):
        # 设置窗口标志：无边框、置顶
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        
        # 淡入动画
        self.fade_in_animation = QPropertyAnimation(self, b"windowOpacity")
        self.fade_in_animation.setDuration(300)
        self.fade_in_animation.setStartValue(0)
        self.fade_in_animation.setEndValue(1)
        self.fade_in_animation.setEasingCurve(QEasingCurve.OutCubic)
        
        # 淡出动画
        self.fade_out_animation = QPropertyAnimation(self, b"windowOpacity")
        self.fade_out_animation.setDuration(300)
        self.fade_out_animation.setStartValue(1)
        self.fade_out_animation.setEndValue(0)
        self.fade_out_animation.setEasingCurve(QEasingCurve.InCubic)
        self.fade_out_animation.finished.connect(self.close)
        
    def showEvent(self, event):
        super().showEvent(event)
        # 居中显示
        if self.parent():
            parent_rect = self.parent().frameGeometry()
            self.move(parent_rect.center() - self.rect().center())
        
        # 启动淡入动画
        self.fade_in_animation.start()
        
        # 设置定时器自动关闭
        QTimer.singleShot(self.timeout, self.start_fade_out)
        
    def start_fade_out(self):
        self.fade_out_animation.start()


# ================= 等待对话框 =================
class WaitDialog(QDialog):
    """一个模态对话框，用于在关闭节点时等待几秒钟。"""
    def __init__(self, message_template="正在关闭ROS节点，请等待 {seconds} 秒...", wait_seconds=6, parent=None):
        super().__init__(parent)
        self.wait_seconds = wait_seconds
        self.remaining_seconds = wait_seconds
        self.message_template = message_template

        self.setWindowTitle("请稍候...")
        self.setModal(True)
        self.setFixedSize(500, 250)  # 增大对话框尺寸

        layout = QVBoxLayout(self)
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(25)

        self.message_label = QLabel(self.message_template.format(seconds=self.remaining_seconds))
        self.message_label.setAlignment(Qt.AlignCenter)
        self.message_label.setFont(QFont("Microsoft YaHei", 16))  # 增大字体
        self.message_label.setStyleSheet("color: black;")  # 设置字体颜色为黑色
        self.message_label.setWordWrap(True)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, self.wait_seconds)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFormat("%v 秒")
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 10px;
                text-align: center;
                font-size: 14px;
                height: 25px;
                color: black;  /* 设置进度条文字颜色为黑色 */
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
                border-radius: 8px;
            }
        """)

        layout.addWidget(self.message_label)
        layout.addWidget(self.progress_bar)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_countdown)

    def showEvent(self, event: QEvent):
        """对话框显示时启动计时器"""
        self.timer.start(1000)
        super().showEvent(event)

    def update_countdown(self):
        """每秒更新一次倒计时和进度条"""
        self.remaining_seconds -= 1
        self.progress_bar.setValue(self.wait_seconds - self.remaining_seconds)
        
        if self.remaining_seconds > 0:
            self.message_label.setText(self.message_template.format(seconds=self.remaining_seconds))
        else:
            self.timer.stop()
            self.accept()


# ================= 棋盘可视化 =================
class ChessVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.piece_images = {}
        self.load_piece_images()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 添加标题标签
        self.title_label = QLabel("当前局势")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setFont(QFont("Microsoft YaHei", 44, QFont.Bold))  # 稍微减小字体到24
        self.title_label.setFixedHeight(65)  
        self.title_label.setStyleSheet("""
            QLabel {
                color: #1E3A8A;  # 深蓝色文字
                background-color: #87CEEB;  # 浅蓝色背景
                padding: 5px;
                border-radius: 8px;
                margin-bottom: 4px;  # 减小底部外边距
                border: 2px solid #1E3A8A;  # 添加深蓝色边框
            }
        """)
        
        layout.addWidget(self.title_label, alignment=Qt.AlignHCenter)  # 水平居中
        
        
        self.board_label = QLabel()
        self.board_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.board_label)


    def load_piece_images(self):
        print("当前工作目录:", os.getcwd())
        pieces = {
            'r': 'black_r', 'n': 'black_n', 'b': 'black_b', 'a': 'black_a',
            'k': 'black_k', 'c': 'black_c', 'p': 'black_p',
            'R': 'red_R', 'N': 'red_N', 'B': 'red_B', 'A': 'red_A',
            'K': 'red_K', 'C': 'red_C', 'P': 'red_P'
        }
        for fen_char, base_name in pieces.items():
            path = f"src/qt_chess_p/resources/images/{base_name}.png"
            if os.path.exists(path):
                self.piece_images[fen_char] = QPixmap(path)

    def update_board(self, fen_str):
        self.board_background = QPixmap("src/qt_chess_p/resources/images/qipan.jpg")
        if self.board_background.isNull():
            print("棋盘背景图加载失败")
        
        # 增大棋盘尺寸
        board_width = 800  # 进一步增大棋盘宽度
        board_height = 800  # 进一步增大棋盘高度
        self.board_size = QSize(board_width, board_height)
        
        self.border_size = 42  # 增大边框
        self.cell_size = (board_width - 2 * self.border_size) // 9 - 1  # 计算更大的格子大小

        board_part = fen_str.split()[0]
        print(board_part)

        # 创建一个新的棋盘图像
        board_img = QImage(self.board_size, QImage.Format_ARGB32)
        board_img.fill(Qt.transparent)
        
        painter = QPainter(board_img)

        # 绘制背景图，保持缩放比例
        if not self.board_background.isNull():
            painter.drawPixmap(0, 0, self.board_background.scaled(self.board_size, Qt.KeepAspectRatio))
        
        # 绘制棋子
        self.draw_pieces(painter, board_part)

        
        painter.end()

        # 设置棋盘标签
        self.board_label.setPixmap(QPixmap.fromImage(board_img))

    def draw_pieces(self, painter, fen_board):
        start_x = self.border_size
        start_y = self.border_size
        rows = fen_board.split('/')
        
        for row_idx, row in enumerate(rows):
            col_idx = 0
            for char in row:
                if char.isdigit():
                    col_idx += int(char)  # 跳过空格
                elif char in self.piece_images:
                    # 增大棋子的缩放比例
                    pixmap = self.piece_images[char].scaled(self.cell_size * 1.3, self.cell_size * 1.3, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    
                    # 计算每个棋子的绘制位置
                    cross_x = start_x + col_idx * self.cell_size
                    cross_y = start_y + row_idx * self.cell_size
                    
                    # 绘制棋子，居中显示
                    painter.drawPixmap(int(cross_x - pixmap.width() / 2), int(cross_y - pixmap.height() / 2), pixmap)
                    col_idx += 1



# ================= 菜单对话框 =================
class MenuDialog(QDialog):
    end_game_signal = Signal()
    debug_send_signal = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("游戏菜单")
        self.setFixedSize(400, 480)
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(25)
        layout.setContentsMargins(60, 50, 60, 50)

        btn_style = """
            QPushButton {
                font-size: 22px;
                padding: 16px;
                border-radius: 14px;
                font-weight: bold;
                min-height: 60px;
            }
        """

        # 结束游戏按钮
        self.end_game_btn = QPushButton("结束游戏")
        self.end_game_btn.setStyleSheet(btn_style + "background-color: #e74c3c; color: white;")

        # 单按钮控制暂停 / 继续
        self.pause_resume_btn = QPushButton("暂停游戏")
        self.pause_resume_btn.setCheckable(True)
        self.pause_resume_btn.setStyleSheet(btn_style + "background-color: #f39c12; color: white;")

        # 调试命令输入框
        self.debug_input = QLineEdit()
        self.debug_input.setPlaceholderText("请输入 UCI 命令（如 move e2e4）")
        self.debug_input.setStyleSheet("""
            QLineEdit {
                font-size: 20px;
                padding: 12px;
                border: 2px solid #bbb;
                border-radius: 10px;
                background-color: #fff;
            }
        """)

        # 发送调试命令按钮
        self.send_debug_btn = QPushButton("发送调试命令")
        self.send_debug_btn.setStyleSheet(btn_style + "background-color: #3498db; color: white;")

        # 关闭菜单按钮
        self.close_btn = QPushButton("关闭菜单")
        self.close_btn.setStyleSheet(btn_style + "background-color: #7f8c8d; color: white;")

        # 加入布局
        layout.addWidget(self.end_game_btn)
        layout.addWidget(self.pause_resume_btn)
        layout.addWidget(self.debug_input)
        layout.addWidget(self.send_debug_btn)
        layout.addWidget(self.close_btn)

        # 信号连接
        self.end_game_btn.clicked.connect(self.end_game_signal.emit)
        self.send_debug_btn.clicked.connect(self.send_debug_command)
        self.close_btn.clicked.connect(self.close)

        # 暂停/继续逻辑绑定
        self.pause_resume_btn.clicked.connect(self.toggle_pause_resume)

    def toggle_pause_resume(self):
        """暂停 / 继续游戏按钮切换逻辑"""
        if self.pause_resume_btn.isChecked():
            # 切换为暂停状态
            self.pause_resume_btn.setText("继续游戏")
            self.pause_resume_btn.setStyleSheet("""
                QPushButton {
                    font-size: 22px;
                    padding: 16px;
                    border-radius: 14px;
                    font-weight: bold;
                    min-height: 60px;
                    background-color: #27ae60;
                    color: white;
                }
            """)
            self.debug_send_signal.emit("pause")
        else:
            # 切换为继续状态
            self.pause_resume_btn.setText("暂停游戏")
            self.pause_resume_btn.setStyleSheet("""
                QPushButton {
                    font-size: 22px;
                    padding: 16px;
                    border-radius: 14px;
                    font-weight: bold;
                    min-height: 60px;
                    background-color: #f39c12;
                    color: white;
                }
            """)
            self.debug_send_signal.emit("resume")

    def send_debug_command(self):
        """从输入框发送UCI命令"""
        cmd = self.debug_input.text().strip()
        if cmd:
            self.debug_send_signal.emit(cmd)
            self.debug_input.clear()

class GamePage(QWidget):
    back_signal = Signal()
    end_game_request_signal = Signal()

    def __init__(self, ros_node, player_color="RED", game_mode="HUMAN_AI"):
        super().__init__()
        self.ros_node = ros_node
        self.player_color = player_color
        self.game_mode = game_mode  
        self.start_time = None
        self.is_timing = False

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time_display)

        self.setup_ui()
        self.connect_signals()
        self.start_timing()

    def setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(40, 30, 40, 30)
        main_layout.setSpacing(40)

        # 左侧棋盘
        self.chess_visualizer = ChessVisualizer()
        self.chess_visualizer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        main_layout.addWidget(self.chess_visualizer, stretch=3)

        # 右侧控制区
        right_layout = QVBoxLayout()
        right_layout.setSpacing(20)

        # 时间显示框
        self.time_label = QLabel("00:00")
        self.time_label.setAlignment(Qt.AlignCenter)
        self.time_label.setFont(QFont("Arial", 40, QFont.Bold))
        self.time_label.setStyleSheet("""
            QLabel {
                background-color: #2c3e50;
                color: white;
                border-radius: 14px;
                padding: 12px;
            }
        """)
        self.time_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        right_layout.addWidget(self.time_label)

        # 我移动了棋子按钮（仅在人机模式下显示）
        self.move_piece_btn = QPushButton("我移动了棋子")
        self.move_piece_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.move_piece_btn.setMinimumHeight(120)
        self.move_piece_btn.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
        self.move_piece_btn.setStyleSheet("""
            QPushButton {
                border-radius: 60px;
                background-color: #e74c3c;
                color: white;
                font-weight: bold;
                font-size: 22px;
            }
            QPushButton:hover {
                background-color: #ff6150;
            }
        """)

        if self.game_mode == "HUMAN_AI":
            right_layout.addWidget(self.move_piece_btn)
        else:
            self.move_piece_btn.setVisible(False)

        # 日志显示框
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setPlaceholderText("调试 / 状态信息将在此显示...")
        self.log_box.setStyleSheet("""
            QTextEdit {
                border: 2px solid #aaa;
                border-radius: 10px;
                background-color: #fafafa;
                color: #222;
                font-size: 16px;
            }
        """)
        self.log_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_layout.addWidget(self.log_box)

        # 菜单按钮
        self.menu_btn = QPushButton("☰ 菜单")
        self.menu_btn.setFont(QFont("Arial", 20, QFont.Bold))
        self.menu_btn.setStyleSheet("""
            QPushButton {
                background-color: #34495e;
                color: white;
                border-radius: 10px;
                font-weight: bold;
                padding: 12px 20px;
            }
            QPushButton:hover {
                background-color: #3e5870;
            }
        """)
        self.menu_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        right_layout.addWidget(self.menu_btn)

        main_layout.addLayout(right_layout, stretch=2)

        if self.game_mode == "HUMAN_AI":
            self.move_piece_btn.clicked.connect(self.trigger_move_piece_signal)
        self.menu_btn.clicked.connect(self.open_menu)

        if self.game_mode == "AI_AI":
            self.add_ai_ai_description(right_layout)

    def add_ai_ai_description(self, layout):
        """在机机模式下添加说明标签"""
        description = QLabel("机机对战模式\n两个AI正在自动对弈...")
        description.setAlignment(Qt.AlignCenter)
        description.setFont(QFont("Microsoft YaHei", 18, QFont.Bold))
        description.setStyleSheet("""
            QLabel {
                background-color: #3498db;
                color: white;
                border-radius: 12px;
                padding: 20px;
                margin: 10px;
            }
        """)
        description.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.insertWidget(1, description)

    # ---------------- 功能逻辑 ----------------
    def connect_signals(self):
        self.ros_node.fen_signal.connect(self.update_board)
        self.ros_node.log_signal.connect(self.append_log)
        self.ros_node.status_signal.connect(self.handle_status_update)
        if hasattr(self.ros_node, "machinery_trigger_signal"):
            self.ros_node.machinery_trigger_signal.connect(self.handle_machinery_trigger)

    def start_timing(self):
        self.start_time = time.time()
        self.is_timing = True
        self.timer.start(1000)

    def stop_timing(self):
        self.is_timing = False
        self.timer.stop()

    def update_time_display(self):
        if self.is_timing and self.start_time:
            elapsed = int(time.time() - self.start_time)
            self.time_label.setText(f"{elapsed//60:02d}:{elapsed%60:02d}")

    @Slot(str)
    def update_board(self, fen):
        self.chess_visualizer.update_board(fen)

    def trigger_move_piece_signal(self):
        self.ros_node.publish_move_piece_signal(True)
        self.append_log("已发布‘我移动了棋子’信号")

    def open_menu(self):
        dialog = MenuDialog()
        dialog.end_game_signal.connect(self.handle_end_game)
        dialog.end_game_signal.connect(dialog.accept) 
        dialog.debug_send_signal.connect(self.handle_debug_send)
        dialog.exec()

    @Slot()
    def handle_end_game(self):
        self.stop_timing()
        self.ros_node.publish_uci_command("over")
        self.append_log("已发布结束信号：over")
        self.end_game_request_signal.emit()

    @Slot()
    def show_wait_dialog_and_go_back(self):
        wait_dialog = WaitDialog(
            message_template="正在关闭ROS节点，请等待 {seconds} 秒...",
            wait_seconds=6,
            parent=self
        )
        wait_dialog.exec()
        self.back_signal.emit()

    @Slot(str)
    def handle_debug_send(self, cmd):
        self.ros_node.publish_uci_command(cmd)
        if hasattr(self.ros_node, "publish_rob_uci_command"):
            self.ros_node.publish_rob_uci_command(cmd)

        if cmd.lower() == "pause":
            self.stop_timing()
            self.append_log("游戏已暂停")
        elif cmd.lower() == "resume":
            self.start_timing()
            self.append_log("游戏继续")
        else:
            self.append_log(f"已发送 UCI 指令: {cmd}")

    def show_temporary_popup(self, message, color="#f39c12"):
        """显示一个带淡入淡出动画的临时弹窗"""
        popup = CustomMessageBox(
            parent=self,
            title="提示",
            message=message,
            color=color,
            timeout=2000
        )
        popup.show()

    @Slot(bool)
    def handle_machinery_trigger(self, value):
        if self.game_mode == "HUMAN_AI" and self.player_color == "BLACK" and value and not self.is_timing:
            self.start_timing()
            self.append_log("收到机械臂触发信号，黑方计时开始")

    @Slot(str)
    def handle_status_update(self, status_msg: str):
        self.append_log(f"[状态] {status_msg}")
        
        # 检测是否是"将军"提示
        if "check_detected" in status_msg:
            self.show_temporary_popup("将军！", color="#f39c12")
        # 检测是否是"将军"提示
        if "移动无效" in status_msg:
            self.show_temporary_popup("移动无效", color="#d91908")    
        # 检测是否是结束游戏
        if status_msg.startswith("over:"):
            # 使用自定义的消息框来显示游戏结束信息
            game_over_msg ="游戏结束！" + status_msg.replace('over:', '').strip()
            self.show_game_over_dialog(game_over_msg)

    def show_game_over_dialog(self, message):
        """显示游戏结束对话框"""
        # 创建一个自定义的对话框来替代 QMessageBox
        dialog = QDialog(self)
        dialog.setWindowTitle("游戏结束")
        dialog.setModal(True)
        
        # 设置固定大小
        dialog.setFixedSize(800, 600)
        
        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(30)
        
        # 图标标签
        icon_label = QLabel("🎯")
        icon_label.setAlignment(Qt.AlignCenter)
        icon_label.setFont(QFont("Arial", 60))
        layout.addWidget(icon_label)
        
        # 消息标签
        message_label = QLabel(message)
        message_label.setAlignment(Qt.AlignCenter)
        message_label.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
        message_label.setWordWrap(True)  # 重要：允许文本自动换行
        message_label.setStyleSheet("""
            QLabel {
                color: #2c3e50;
                padding: 20px;
                background-color: #f8f9fa;
                border-radius: 15px;
                border: 2px solid #bdc3c7;
            }
        """)
        layout.addWidget(message_label)
        
        # 确定按钮
        ok_button = QPushButton("确定")
        ok_button.setFont(QFont("Microsoft YaHei", 20, QFont.Bold))
        ok_button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                border: none;
                border-radius: 10px;
                padding: 15px 30px;
                min-height: 50px;
            }
            QPushButton:hover {
                background-color: #2ecc71;
            }
        """)
        ok_button.clicked.connect(dialog.accept)
        layout.addWidget(ok_button)
        
        # 显示对话框并等待用户响应
        result = dialog.exec()
        
        # 无论用户点击什么按钮，都结束游戏并返回
        self.stop_timing()
        self.back_signal.emit()
        
    @Slot(str)
    def append_log(self, message):
        self.log_box.append(message)
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())