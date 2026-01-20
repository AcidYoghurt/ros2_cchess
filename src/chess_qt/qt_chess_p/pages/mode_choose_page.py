# qt_chess_p/pages/mode_choose_page.py

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QButtonGroup, QGroupBox, QSizePolicy
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont
from .game_page import WaitDialog

class ModeChoosePage(QWidget):
    """模式选择页面 - 新增独立的 VOICE_AI 模式"""
    
    mode_selected_signal = Signal(str)  # 发射 "HUMAN_AI", "AI_AI", 或 "VOICE_AI"
    back_signal = Signal()

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.selected_mode = "HUMAN_AI"
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(60, 50, 60, 50)
        layout.setSpacing(40)

        title = QLabel("选择游戏模式")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Microsoft YaHei", 36, QFont.Bold))
        layout.addWidget(title)

        mode_group = QGroupBox("请选择游戏模式")
        mode_group.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
        mode_layout = QHBoxLayout(mode_group)
        mode_layout.setContentsMargins(40, 40, 40, 40)
        mode_layout.setSpacing(30)

        self.mode_buttons = QButtonGroup(self)
        base_btn_style = """
            QPushButton {
                background-color: #5dade2; color: white; font-size: 24px; font-weight: 800;
                border-radius: 20px; padding: 20px; min-width: 180px; min-height: 120px;
            }
            QPushButton:hover { background-color: #85c1e9; }
            QPushButton:checked { border: 4px solid #fdfefe; }
        """

        # 三个同级别的模式按钮
        self.human_ai_btn = QPushButton("人机对战")
        self.human_ai_btn.setCheckable(True)
        self.human_ai_btn.setChecked(True)
        self.human_ai_btn.setStyleSheet(base_btn_style + "QPushButton:checked { background-color: #27ae60; }")

        self.voice_ai_btn = QPushButton("语音对弈")
        self.voice_ai_btn.setCheckable(True)
        self.voice_ai_btn.setStyleSheet(base_btn_style + "QPushButton:checked { background-color: #f39c12; }")

        self.ai_ai_btn = QPushButton("机机对战")
        self.ai_ai_btn.setCheckable(True)
        self.ai_ai_btn.setStyleSheet(base_btn_style + "QPushButton:checked { background-color: #e74c3c; }")

        for i, btn in enumerate([self.human_ai_btn, self.voice_ai_btn, self.ai_ai_btn]):
            self.mode_buttons.addButton(btn, i)
            mode_layout.addWidget(btn)

        layout.addWidget(mode_group)

        self.mode_description = QLabel()
        self.mode_description.setAlignment(Qt.AlignCenter)
        self.mode_description.setFont(QFont("Microsoft YaHei", 18))
        self.mode_description.setStyleSheet("background-color: #f8f9fa; border: 2px solid #dee2e6; border-radius: 10px; padding: 20px; color: #495057;")
        self.update_mode_description()
        layout.addWidget(self.mode_description)

        # 确认与返回
        button_layout = QHBoxLayout()
        back_btn = QPushButton("返回")
        back_btn.clicked.connect(self.back_signal.emit)
        confirm_btn = QPushButton("确认选择")
        confirm_btn.clicked.connect(self.confirm_selection)
        
        # 按钮样式统一应用（此处略过样式细节以聚焦逻辑）
        button_layout.addWidget(back_btn)
        button_layout.addWidget(confirm_btn)
        layout.addLayout(button_layout)

        self.human_ai_btn.toggled.connect(self.on_mode_changed)
        self.voice_ai_btn.toggled.connect(self.on_mode_changed)
        self.ai_ai_btn.toggled.connect(self.on_mode_changed)

    def on_mode_changed(self):
        btn_id = self.mode_buttons.checkedId()
        modes = {0: "HUMAN_AI", 1: "VOICE_AI", 2: "AI_AI"}
        self.selected_mode = modes.get(btn_id, "HUMAN_AI")
        self.update_mode_description()

    def update_mode_description(self):
        descs = {
            "HUMAN_AI": "<b>人机对战模式</b><br>玩家现实进行下棋操作，机械臂通过智能识别现实情况进行对弈，可自由选择阵营。",
            "VOICE_AI": "<b>语音对弈模式</b><br>玩家通过语音指令控制左机械臂(红方)，AI控制黑方。",
            "AI_AI": "<b>机机对战模式</b><br>两组AI机械臂自动对弈，整个系统自动运行。"
        }
        self.mode_description.setText(descs[self.selected_mode])

    def confirm_selection(self):
        self.mode_selected_signal.emit(self.selected_mode)
        WaitDialog(message_template="初始化中，请稍候 {seconds} 秒...", wait_seconds=3, parent=self).exec()