# qt_chess_p/pages/game_page.py
import time
import json
import os
import re
import cchess
import cchess.svg
import cairosvg
import pyaudio
from vosk import Model, KaldiRecognizer

from PySide6.QtCore import (
    Qt, Signal, Slot, QTimer, QEvent, QThread,
    QPropertyAnimation, QEasingCurve, QSize, QByteArray
)
from PySide6.QtGui import (
    QFont, QPixmap, QImage, QPainter, QColor
)
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTextEdit, QSizePolicy, QDialog, QLineEdit, QMessageBox,
    QProgressBar, QListWidget, QFrame, QGroupBox
)
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtSvgWidgets import QSvgWidget

# ================= 语音识别线程 =================
class VoiceRecognitionThread(QThread):
    recognized_move_signal = Signal(str)    # 最终识别出的 UCI 走法
    partial_result_signal = Signal(str)     # 实时识别的中间文字
    status_signal = Signal(str)             # 状态提示

    def __init__(self, model_path):
        super().__init__()
        # 模型路径检测
        actual_path = model_path if os.path.exists(model_path) else "/home/chess/Desktop/ros2_cchess/src/qt_chess_p/resources/vosk-model-small-cn-0.22"
        self.model = Model(actual_path)
        self.grammar = ["帅", "仕", "相", "马", "车", "炮", "兵", "将", "士", "象", "卒", "进", "退", "平", "一", "二", "三", "四", "五", "六", "七", "八", "九"]
        self.rec = KaldiRecognizer(self.model, 16000, json.dumps(self.grammar, ensure_ascii=False))
        self.rec.SetWords(True)
        self._is_running = False
        self.current_fen = ""

    def run(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
        stream.start_stream()
        self._is_running = True
        
        self.status_signal.emit("请下指令...")
        
        try:
            while self._is_running:
                data = stream.read(4000, exception_on_overflow=False)
                if self.rec.AcceptWaveform(data):
                    # 最终识别结果
                    result = json.loads(self.rec.Result())
                    text = result.get("text", "").replace(" ", "")
                    if text:
                        self.process_text(text)
                else:
                    # 实时中间结果
                    partial = json.loads(self.rec.PartialResult())
                    partial_text = partial.get("partial", "").replace(" ", "")
                    if partial_text:
                        self.partial_result_signal.emit(partial_text)
        finally:
            self._is_running = False
            stream.stop_stream()
            stream.close()
            p.terminate()

    def process_text(self, text):
        board = cchess.Board(self.current_fen)
        try:
            move = board.parse_notation(text)
            uci = move.uci()
            self.recognized_move_signal.emit(uci)
            self._is_running = False 
        except Exception:
            self.status_signal.emit(f"无法识别: {text}")

    def stop(self):
        self._is_running = False


class ChessVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.player_orientation = cchess.RED  # 默认红方在下

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.title_label = QLabel("当前局势")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
        self.title_label.setFixedHeight(65)
        self.title_label.setStyleSheet("""
            QLabel {
                color: #1E3A8A;
                background-color: #87CEEB;
                padding: 5px;
                border-radius: 8px;
                border: 2px solid #1E3A8A;
            }
        """)
        layout.addWidget(self.title_label, alignment=Qt.AlignHCenter)

        self.board_label = QLabel()
        self.board_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.board_label, stretch=1)

    def set_orientation(self, color_str):
        """设置棋盘视角：RED 或 BLACK"""
        if color_str.upper() == "BLACK":
            self.player_orientation = cchess.BLACK
        else:
            self.player_orientation = cchess.RED

    def update_board(self, fen_str):
        """使用 SVG → PNG 渲染棋盘（稳定方案）"""
        try:
            board = cchess.Board(fen_str)

            # 尝试获取上一步（用于高亮）
            last_move = None
            try:
                last_move = board.peek()
            except Exception:
                pass

            svg_content = cchess.svg.board(
                board=board,
                size=1200,                     # 用大尺寸避免细线丢失
                coordinates=True,
                axes_type=1,
                lastmove=last_move,
                checkers=board.checkers(),
                orientation=self.player_orientation,
                style="#board{fill:#f3e5ab; stroke:#5d4037}"
            )

            # SVG → PNG
            png_bytes = cairosvg.svg2png(
                bytestring=svg_content.encode("utf-8")
            )

            pixmap = QPixmap()
            pixmap.loadFromData(png_bytes)

            # 根据 QLabel 大小自适应
            pixmap = pixmap.scaled(
                self.board_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            self.board_label.setPixmap(pixmap)

        except Exception as e:
            print(f"棋盘渲染错误: {e}")
            self.board_label.setText("棋盘渲染失败")


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

# ================= 游戏主页面 =================
class GamePage(QWidget):
    back_signal = Signal()
    end_game_request_signal = Signal()

    def __init__(self, ros_node, player_color="RED", game_mode="HUMAN_AI"):
        super().__init__()
        self.ros_node = ros_node
        self.player_color = player_color
        self.game_mode = game_mode
        self.is_voice_mode = (game_mode == "VOICE_AI")
        
        # 1. 核心状态初始化
        self.board = cchess.Board() # 初始棋盘
        self.is_timing = False
        self.start_time = None
        
        # 2. 计时器初始化
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time_display)

        # 3. 语音线程初始化
        if self.is_voice_mode:
            self.voice_thread = VoiceRecognitionThread("")
            self.voice_thread.recognized_move_signal.connect(self.on_voice_move_detected)
            self.voice_thread.partial_result_signal.connect(self.on_voice_partial_update)
            self.voice_thread.status_signal.connect(self.update_voice_status)

        # 4. 构建 UI
        self.setup_ui()
        
        # 5. 连接 ROS 信号
        self.setup_connections()
        
        # 6. 开局立即渲染初始棋盘 (修复识别节点晚的问题)
        self.visualizer.set_orientation(self.player_color)
        self.visualizer.update_board(self.board.fen())
        self.start_timing()
        
        # 7. 如果是语音模式且红方先走，自动触发一次录音
        if self.is_voice_mode and self.player_color == "RED":
            QTimer.singleShot(1500, lambda: self.handle_voice_trigger(True))

    def setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)
        
        # 左侧：棋盘 (变量名统一为 self.visualizer)
        self.visualizer = ChessVisualizer()
        main_layout.addWidget(self.visualizer, 3)

        # 右侧：控制面板
        right_panel = QVBoxLayout()
        
        # --- 语音交互区 (大字显示，修复对比度) ---
        if self.is_voice_mode:
            self.voice_group = QGroupBox("语音助手")
            self.voice_group.setStyleSheet("""
                QGroupBox {
                    background-color: #ffffff;
                    border: 3px solid #3498db;
                    border-radius: 15px;
                    margin-top: 20px;
                    font-weight: bold;
                    color: #2c3e50;
                }
            """)
            v_layout = QVBoxLayout(self.voice_group)
            
            # 大字状态标签
            self.voice_status_main = QLabel("等待开启")
            self.voice_status_main.setAlignment(Qt.AlignCenter)
            self.voice_status_main.setFont(QFont("Microsoft YaHei", 32, QFont.Bold))
            self.voice_status_main.setStyleSheet("color: #95a5a6; margin-top: 10px;") # 初始灰色
            
            # 实时识别文字内容
            self.voice_partial_label = QLabel("「 实时语音预览 」")
            self.voice_partial_label.setAlignment(Qt.AlignCenter)
            self.voice_partial_label.setWordWrap(True)
            self.voice_partial_label.setFont(QFont("Microsoft YaHei", 18))
            self.voice_partial_label.setStyleSheet("""
                background-color: #f7f9f9; 
                color: #2980b9; 
                padding: 10px; 
                border-radius: 8px;
                border: 1px dashed #bdc3c7;
            """)
            
            v_layout.addWidget(self.voice_status_main)
            v_layout.addWidget(self.voice_partial_label)
            right_panel.addWidget(self.voice_group)

        # B. 机机模式：显示“AI自动对弈中”
        elif self.game_mode == "AI_AI":
            self.ai_status_group = QGroupBox("对弈状态")
            self.ai_status_group.setStyleSheet(self.get_group_box_style())
            ai_layout = QVBoxLayout(self.ai_status_group)
            self.ai_status_label = QLabel("🤖 AI 自动对弈中...")
            self.ai_status_label.setAlignment(Qt.AlignCenter)
            self.ai_status_label.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
            self.ai_status_label.setStyleSheet("color: #27ae60; padding: 20px;")
            ai_layout.addWidget(self.ai_status_label)
            right_panel.addWidget(self.ai_status_group)

        # --- 2. 时间显示 (通用) ---
        self.time_label = QLabel("00:00")
        self.time_label.setAlignment(Qt.AlignCenter)
        self.time_label.setFont(QFont("Arial", 45, QFont.Bold))
        self.time_label.setStyleSheet("""
            background-color: #2c3e50; 
            color: #ecf0f1; 
            border-radius: 15px; 
            padding: 10px;
            margin-bottom: 10px;
        """)
        right_panel.addWidget(self.time_label)

        # --- 3. 手动操作区 (仅普通人机模式可见) ---
        if self.game_mode == "HUMAN_AI" and not self.is_voice_mode:
            self.manual_btn = QPushButton("我已移动棋子")
            self.manual_btn.setMinimumHeight(80)
            self.manual_btn.setFont(QFont("Microsoft YaHei", 20, QFont.Bold))
            self.manual_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2ecc71; color: white; border-radius: 12px;
                }
                QPushButton:hover { background-color: #27ae60; }
            """)
            # 按钮2：重新确认当前状态 (新增)
            # self.reconfirm_btn = QPushButton("重新确认状态")
            # self.reconfirm_btn.setFixedHeight(60)
            # self.reconfirm_btn.setFont(QFont("Microsoft YaHei", 16))
            # self.reconfirm_btn.setStyleSheet("background-color: #f39c12; color: white; border-radius: 12px; margin-top: 5px;")
            # 连接到 move_callback 或对应函数
            # self.reconfirm_btn.clicked.connect(self.on_reconfirm_status_clicked)
            self.manual_btn.clicked.connect(self.trigger_move_piece_signal)
            # right_panel.addWidget(self.reconfirm_btn)
            right_panel.addWidget(self.manual_btn)

        # 日志框 (修复白底看不清的问题)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setPlaceholderText("系统日志...")
        self.log_box.setStyleSheet("""
            QTextEdit {
                border: 2px solid #bdc3c7;
                border-radius: 10px;
                background-color: #ffffff;
                color: #2c3e50;
                font-size: 16px;
                padding: 10px;
            }
        """)
        right_panel.addWidget(self.log_box)

        # 菜单按钮
        self.menu_btn = QPushButton("☰ 游戏菜单")
        self.menu_btn.setMinimumHeight(70)
        self.menu_btn.setFont(QFont("Microsoft YaHei", 20, QFont.Bold))
        self.menu_btn.setStyleSheet("""
            QPushButton {
                background-color: #34495e; color: white; border-radius: 12px;
            }
            QPushButton:hover { background-color: #5d6d7e; }
        """)
        self.menu_btn.clicked.connect(self.open_menu)
        right_panel.addWidget(self.menu_btn)

        main_layout.addLayout(right_panel, 2)
        
    def get_group_box_style(self):
        return """
            QGroupBox {
                background-color: #ffffff;
                border: 2px solid #3498db;
                border-radius: 10px;
                margin-top: 15px;
                font-weight: bold;
                color: #2c3e50;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }
        """

    def setup_connections(self):
        # 连接 ROS 信号
        self.ros_node.fen_signal.connect(self.on_fen_received)
        self.ros_node.log_signal.connect(self.append_log)
        self.ros_node.status_signal.connect(self.handle_status_update)
        
        if self.is_voice_mode:
            # 只有语音模式才监听触发信号
            if hasattr(self.ros_node, "voice_record_trigger_signal"):
                self.ros_node.voice_record_trigger_signal.connect(self.handle_voice_trigger)

    # ---------------- 业务逻辑 ----------------

    @Slot(bool)
    def handle_voice_trigger(self, trigger):
        """当机械臂完成动作，触发玩家开始说话"""
        if trigger:
            if self.voice_thread.isRunning():
                self.get_logger().info("语音线程已在运行，无需重复启动")
                return
            # 启动前重置实时文本显示
            self.voice_partial_label.setText("准备就绪，请说话...")
            self.voice_thread.current_fen = self.board.fen()
            self.voice_thread.start()
            self.start_timing()
            # 更新大字 UI 为录音状态
            self.voice_status_main.setText("● 正在录音")
            self.voice_status_main.setStyleSheet("color: #e74c3c;") # 红色闪烁感
            self.voice_partial_label.setText("请说出您的走法...")

    @Slot(str)
    def on_voice_partial_update(self, text):
        """显示实时识别的文字段落"""
        self.voice_partial_label.setText(f"「 {text} 」")

    @Slot(str)
    def update_voice_status(self, msg):
        """更新小字或错误提示"""
        self.append_log(f"语音系统: {msg}")
        if "无法识别" in msg:
            self.voice_status_main.setText("识别失败或走法有误")
            self.voice_status_main.setStyleSheet("color: #f39c12;")

    @Slot(str)
    def on_voice_move_detected(self, uci):
        """识别成功后的处理"""
        self.voice_status_main.setText("识别成功")
        self.voice_status_main.setStyleSheet("color: #27ae60;")
        self.voice_partial_label.setText(f"已发送指令: {uci}")
        
        self.ros_node.publish_voice_move(uci)
        self.stop_timing()
        
        # 本地预测性更新，增强响应感
        try:
            move = cchess.Move.from_uci(uci)
            self.board.push(move)
            self.visualizer.update_board(self.board.fen())
        except:
            pass

    @Slot(str)
    def on_fen_received(self, fen):
        if fen != self.board.fen():
            self.board = cchess.Board(fen)
            self.visualizer.update_board(fen)
            # 增加这一行，确保语音识别时参考的是最新棋盘
            if self.is_voice_mode and hasattr(self, 'voice_thread'):
                self.voice_thread.current_fen = fen
            self.append_log("棋盘已同步")

    def update_time_display(self):
        if self.is_timing and self.start_time:
            elapsed = int(time.time() - self.start_time)
            self.time_label.setText(f"{elapsed//60:02d}:{elapsed%60:02d}")

    def start_timing(self):
        self.start_time = time.time()
        self.is_timing = True
        self.timer.start(1000)

    def stop_timing(self):
        self.is_timing = False
        self.timer.stop()

    def append_log(self, message):
        self.log_box.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        # 自动滚动到底部
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())

    def open_menu(self):
        from .game_page import MenuDialog # 确保导入
        dialog = MenuDialog()
        dialog.end_game_signal.connect(self.handle_end_game)
        dialog.debug_send_signal.connect(self.handle_debug_send)
        dialog.exec()

    @Slot()
    def handle_end_game(self):
        self.ros_node.publish_uci_command("over")
        self.back_signal.emit()

    @Slot(str)
    def handle_debug_send(self, cmd):
        self.ros_node.publish_uci_command(cmd)
        self.append_log(f"发送指令: {cmd}")

    @Slot(str)
    def handle_status_update(self, status_msg: str):
        self.append_log(f"[状态] {status_msg}")
        if "check_detected" in status_msg:
            self.show_temporary_popup("将军！", color="#e74c3c")
        if status_msg.startswith("over:"):
            self.show_game_over_dialog("游戏结束！" + status_msg.split(":")[-1])

    def show_temporary_popup(self, message, color="#f39c12"):
        from .game_page import CustomMessageBox
        popup = CustomMessageBox(self, "提示", message, color)
        popup.show()

    def start_timing(self):
        self.start_time = time.time()
        self.is_timing = True
        self.timer.start(1000)

    def stop_timing(self):
        self.is_timing = False
        self.timer.stop()

    def append_log(self, message):
        self.log_box.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())

    def closeEvent(self, event):
        if self.is_voice_mode and self.voice_thread.isRunning():
            self.voice_thread.stop()
            self.voice_thread.wait()
        super().closeEvent(event)

    @Slot(str)
    def update_board(self, fen):
        self.chess_visualizer.update_board(fen)

    def trigger_move_piece_signal(self):
        self.ros_node.publish_move_piece_signal(True)
        self.append_log("已发布‘我移动了棋子’信号")
        
    @Slot()
    def on_reconfirm_status_clicked(self):
        """触发相机重新拍照"""
        self.ros_node.publish_image_trigger(True)
        self.append_log("系统提示：已请求相机重新确认当前棋盘状态。")

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
