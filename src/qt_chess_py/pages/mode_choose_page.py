# qt_chess_p/pages/mode_choose_page.py

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QButtonGroup, QGroupBox, QSizePolicy
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont, QCursor
from .game_page import WaitDialog # 导入 WaitDialog


class ModeChoosePage(QWidget):
    """模式选择页面 - 选择人机模式或机机模式"""
    
    mode_selected_signal = Signal(str)  # 发射选择的模式："HUMAN_AI" 或 "AI_AI"
    back_signal = Signal()              # 返回信号

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.selected_mode = "HUMAN_AI"  # 默认选择人机模式
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(60, 50, 60, 50)
        layout.setSpacing(50)

        # 标题
        title = QLabel("选择游戏模式")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Microsoft YaHei", 36, QFont.Bold))
        title.setStyleSheet("color: #2c3e50; margin-bottom: 40px;")
        layout.addWidget(title)

        # ===== 模式选择 =====
        mode_group = QGroupBox("请选择游戏模式")
        mode_group.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
        mode_layout = QHBoxLayout(mode_group)
        mode_layout.setContentsMargins(40, 40, 40, 40)
        mode_layout.setSpacing(60)

        self.mode_buttons = QButtonGroup(self)
        self.mode_buttons.setExclusive(True)

        # 基础按钮样式
        base_btn_style = """
            QPushButton {
                background-color: #5dade2;
                color: white;
                font-size: 26px;
                font-weight: 800;
                border-radius: 20px;
                padding: 28px 80px;
                border: 3px solid transparent;
                min-width: 200px;
                min-height: 100px;
            }
            QPushButton:hover {
                background-color: #85c1e9;
            }
        """

        # 人机模式按钮
        self.human_ai_btn = QPushButton("人机对战")
        self.human_ai_btn.setCheckable(True)
        self.human_ai_btn.setStyleSheet(base_btn_style + """
            QPushButton:checked {
                background-color: #27ae60;
                border: 3px solid #1e8449;
            }
        """)
        self.human_ai_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.human_ai_btn.setChecked(True)

        # 机机模式按钮
        self.ai_ai_btn = QPushButton("机机对战")
        self.ai_ai_btn.setCheckable(True)
        self.ai_ai_btn.setStyleSheet(base_btn_style + """
            QPushButton:checked {
                background-color: #e74c3c;
                border: 3px solid #922b21;
            }
        """)
        self.ai_ai_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.mode_buttons.addButton(self.human_ai_btn, 0)  # HUMAN_AI
        self.mode_buttons.addButton(self.ai_ai_btn, 1)     # AI_AI

        mode_layout.addWidget(self.human_ai_btn)
        mode_layout.addWidget(self.ai_ai_btn)
        layout.addWidget(mode_group)

        # ===== 模式说明 =====
        self.mode_description = QLabel()
        self.mode_description.setAlignment(Qt.AlignCenter)
        self.mode_description.setFont(QFont("Microsoft YaHei", 18))
        self.mode_description.setStyleSheet("""
            QLabel {
                background-color: #f8f9fa;
                border: 2px solid #dee2e6;
                border-radius: 10px;
                padding: 20px;
                color: #495057;
            }
        """)
        self.update_mode_description()
        layout.addWidget(self.mode_description)

        # ===== 按钮区域 =====
        button_layout = QHBoxLayout()
        button_layout.setSpacing(30)

        # 返回按钮
        back_btn = QPushButton("返回")
        back_btn.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
        back_btn.setStyleSheet("""
            QPushButton {
                background-color: #95a5a6;
                color: white;
                border-radius: 14px;
                padding: 18px 50px;
                font-size: 22px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #bdc3c7;
            }
        """)
        back_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        back_btn.clicked.connect(self.back_signal.emit)

        # 确认选择按钮
        confirm_btn = QPushButton("确认选择")
        confirm_btn.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
        confirm_btn.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                border-radius: 14px;
                padding: 20px 60px;
                font-size: 24px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #52be80;
            }
        """)
        confirm_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        confirm_btn.clicked.connect(self.confirm_selection)

        button_layout.addWidget(back_btn)
        button_layout.addWidget(confirm_btn)
        layout.addLayout(button_layout)

        layout.addStretch()

        # 连接信号
        self.human_ai_btn.toggled.connect(self.on_mode_changed)
        self.ai_ai_btn.toggled.connect(self.on_mode_changed)

    def on_mode_changed(self, checked):
        """模式选择改变时更新说明"""
        if checked:
            if self.human_ai_btn.isChecked():
                self.selected_mode = "HUMAN_AI"
            else:
                self.selected_mode = "AI_AI"
            self.update_mode_description()

    def update_mode_description(self):
        """更新模式说明文本"""
        if self.selected_mode == "HUMAN_AI":
            description = (
                "<b>人机对战模式</b><br>"
                "玩家与AI对战，可选择红方或黑方<br>"
                "包含完整的游戏设置选项"
            )
        else:
            description = (
                "<b>机机对战模式</b><br>"
                "两个AI自动对战，无需玩家操作<br>"
                "仅需设置AI难度和思考时间"
            )
        self.mode_description.setText(description)

    def confirm_selection(self):
        """确认选择并发射信号"""
        # 1. 立即发射信号，让MainWindow开始启动节点和准备下一页
        self.mode_selected_signal.emit(self.selected_mode)

        # 显示一个2秒的等待对话框，提示用户节点正在启动
        wait_dialog = WaitDialog(
            message_template="正在启动ROS节点，请稍候 {seconds} 秒...",
            wait_seconds=5,
            parent=self
        )
        # 2. 显示等待对话框，这会阻塞当前页面的交互，但不会阻塞MainWindow的页面切换
        wait_dialog.exec()