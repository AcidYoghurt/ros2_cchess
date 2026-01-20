# qt_chess_p/pages/settings_page.py

import json
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox,
    QPushButton, QButtonGroup, QGroupBox, QFormLayout, QDialog, QCheckBox,
    QSizePolicy, QScrollArea
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont


class SettingsPage(QWidget):
    back_signal = Signal()
    start_game_signal = Signal(dict)
    
    def __init__(self, ros_node, game_mode="HUMAN_AI"):
        super().__init__()
        self.ros_node = ros_node
        self.game_mode = game_mode  # "HUMAN_AI", "AI_AI" 或 "VOICE_AI"
        self.settings_data = {"think_time": 1.0, "limit_strength": False, "elo_rating": 2000}
        
        self.setStyleSheet("""
            QWidget {
                background-color: #2E3440;
                color: #ECEFF4;
                font-family: "Microsoft YaHei", sans-serif;
            }
        """)

        self.base_btn_style = """
            QPushButton {
                background-color: #5dade2;
                color: white;
                font-size: 26px;
                font-weight: 800;
                border-radius: 20px;
                padding: 28px 40px;
                border: 3px solid transparent;
                min-width: 180px;
                min-height: 80px;
            }
            QPushButton:hover { background-color: #85c1e9; }
            QPushButton:checked { border: 3px solid #ffffff; }
        """
        self.setup_ui()

    def setup_ui(self):
        # 清理旧布局，防止界面重叠或左上角出现异常小窗口
        if self.layout():
            QWidget().setLayout(self.layout()) 
            
        main_layout = QVBoxLayout(self)
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; }")
        main_layout.addWidget(scroll)

        content = QWidget()
        scroll.setWidget(content)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(60, 50, 60, 50)
        layout.setSpacing(40)

        # ===== 页面标题 =====
        titles = {
            "HUMAN_AI": "人机对战设置",
            "AI_AI": "机机对战设置",
            "VOICE_AI": "语音模式设置"
        }
        title = QLabel(titles.get(self.game_mode, "游戏设置"))
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Microsoft YaHei", 32, QFont.Bold))
        layout.addWidget(title)

        # ===== 核心配置区 =====
        if self.game_mode == "VOICE_AI":
            # 语音模式：仅显示黑方难度，玩家固定为红方
            self._build_voice_settings(layout)
        elif self.game_mode == "AI_AI":
            self._build_ai_ai_settings(layout)
        else:
            self._build_human_ai_settings(layout)

        # ===== 底部公共按钮 =====
        self._build_common_actions(layout)
        layout.addStretch()

    def _build_voice_settings(self, layout):
        group = QGroupBox("黑方 AI 难度 (对手)")
        group.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
        group_layout = QHBoxLayout(group)
        group_layout.setContentsMargins(30, 40, 30, 40)

        self.diff_buttons = QButtonGroup(self)
        levels = ["入门", "进阶", "大师"]
        for i, level in enumerate(levels):
            btn = QPushButton(level)
            btn.setCheckable(True)
            btn.setStyleSheet(self.base_btn_style + "QPushButton:checked { background-color: #34495e; }")
            if i == 1: btn.setChecked(True)
            self.diff_buttons.addButton(btn, i)
            group_layout.addWidget(btn)
        layout.addWidget(group)

    def _build_human_ai_settings(self, layout):
        # 阵营选择
        side_group = QGroupBox("选择您的阵营")
        side_group.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
        side_layout = QHBoxLayout(side_group)
        self.side_buttons = QButtonGroup(self)
        
        self.red_btn = QPushButton("红方")
        self.black_btn = QPushButton("黑方")
        for i, btn in enumerate([self.red_btn, self.black_btn]):
            btn.setCheckable(True)
            btn.setStyleSheet(self.base_btn_style + "QPushButton:checked { background-color: #27ae60; }")
            self.side_buttons.addButton(btn, i)
            side_layout.addWidget(btn)
        self.red_btn.setChecked(True)
        layout.addWidget(side_group)

        # 难度选择
        diff_group = QGroupBox("AI 难度")
        diff_group.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
        diff_layout = QHBoxLayout(diff_group)
        self.diff_buttons = QButtonGroup(self)
        for i, lvl in enumerate(["入门", "进阶", "大师"]):
            btn = QPushButton(lvl); btn.setCheckable(True)
            btn.setStyleSheet(self.base_btn_style + "QPushButton:checked { background-color: #34495e; }")
            if i == 1: btn.setChecked(True)
            self.diff_buttons.addButton(btn, i); diff_layout.addWidget(btn)
        layout.addWidget(diff_group)

    def _build_ai_ai_settings(self, layout):
        # AI_AI 模式显示红黑双方各自的难度
        for side_name in ["红方 AI", "黑方 AI"]:
            group = QGroupBox(f"{side_name} 难度")
            group.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
            group_layout = QHBoxLayout(group)
            group_layout.setContentsMargins(30, 30, 30, 30)
            
            btn_group = QButtonGroup(self)
            for i, lvl in enumerate(["入门", "进阶", "大师"]):
                btn = QPushButton(lvl); btn.setCheckable(True)
                btn.setStyleSheet(self.base_btn_style + "QPushButton:checked { background-color: #34495e; }")
                if i == 1: btn.setChecked(True)
                btn_group.addButton(btn, i); group_layout.addWidget(btn)
            
            if "红方" in side_name: self.red_diff_buttons = btn_group
            else: self.black_diff_buttons = btn_group
            layout.addWidget(group)

    def _build_common_actions(self, layout):
        btn_layout = QHBoxLayout()
        
        details_btn = QPushButton("详细设置...")
        details_btn.setStyleSheet("background-color: #8e44ad; color: white; font-size: 22px; border-radius: 15px; padding: 15px;")
        details_btn.clicked.connect(self.show_detailed_settings)
        
        start_btn = QPushButton("开始对弈")
        start_btn.setStyleSheet("background-color: #27ae60; color: white; font-size: 24px; font-weight: bold; border-radius: 15px; padding: 20px;")
        start_btn.clicked.connect(self.apply_settings)
        
        btn_layout.addWidget(details_btn, 1)
        btn_layout.addWidget(start_btn, 2)
        layout.addLayout(btn_layout)

    # ===== 详细设置窗口 =====
    def show_detailed_settings(self):
        dialog = QDialog(self)
        dialog_title = "人机对战详细设置" if self.game_mode == "HUMAN_AI" else "机机对战详细设置"
        dialog.setWindowTitle(dialog_title)
        dialog.resize(720, 480)

        form_layout = QFormLayout(dialog)
        form_layout.setContentsMargins(60, 50, 60, 50)
        form_layout.setSpacing(25)

        label_font = QFont("Microsoft YaHei", 20)
        field_font = QFont("Microsoft YaHei", 20)

        think_time_spin = QDoubleSpinBox()
        think_time_spin.setFont(field_font)
        think_time_spin.setRange(0.01, 60.0)
        think_time_spin.setSingleStep(0.1)
        think_time_spin.setValue(1.0)

        # 机机模式下不显示限制强度选项
        if self.game_mode == "HUMAN_AI":
            limit_strength_check = QCheckBox("限制强度 (ELO 模式)")
            limit_strength_check.setFont(label_font)

            elo_rating_spin = QDoubleSpinBox()
            elo_rating_spin.setFont(field_font)
            elo_rating_spin.setRange(1280, 3133)
            elo_rating_spin.setSingleStep(100)
            elo_rating_spin.setValue(2000)

        form_layout.addRow(QLabel("思考时间 (秒):", font=label_font), think_time_spin)

        if self.game_mode == "HUMAN_AI":
            form_layout.addRow(limit_strength_check)
            form_layout.addRow(QLabel("ELO 等级:", font=label_font), elo_rating_spin)

        ok_btn = QPushButton("确定")
        ok_btn.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
        ok_btn.setStyleSheet("""
            QPushButton {
                background-color: #5dade2;
                color: white;
                border-radius: 14px;
                padding: 18px 60px;
                font-size: 22px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #85c1e9;
            }
        """)
        ok_btn.clicked.connect(dialog.accept)
        form_layout.addRow(ok_btn)

        # 保存设置
        if self.game_mode == "HUMAN_AI":
            settings = {
                "think_time": think_time_spin.value(),
                "limit_strength": limit_strength_check.isChecked(),
                "elo_rating": int(elo_rating_spin.value()),
            }
        else:
            settings = {
                "think_time": think_time_spin.value(),
                "limit_strength": False,
                "elo_rating": 2000,
            }

        if dialog.exec():
            self.settings_data = settings
        else:
            self.settings_data = {"think_time": 1.0, "limit_strength": False, "elo_rating": 2000}

    def apply_settings(self):
        difficulty_map = {0: 5, 1: 10, 2: 20}
        extra = self.settings_data
        
        if self.game_mode == "VOICE_AI":
            # 语音模式逻辑：仅配置机械臂 2 (黑方 AI)
            black_level = difficulty_map.get(self.diff_buttons.checkedId(), 10)
            config_msg_2 = {
                "command": "config",
                "player_color": "RED",  # 机器人视角：黑方看到红方是对手
                "think_time": extra["think_time"],
                "skill_level": black_level,
            }
            if self.ros_node:
                self.ros_node.publish_rob_uci_command(json.dumps(config_msg_2, ensure_ascii=False))
            
            self.start_game_signal.emit({
                "config_1": None, "config_2": config_msg_2,
                "player_color": "RED", "game_mode": "VOICE_AI"
            })
        else:
            if self.game_mode == "HUMAN_AI":
                side = "RED" if self.side_buttons.checkedId() == 0 else "BLACK"
            else:
                side = "BLACK"  # 机机模式下双方都是 AI

            # 默认难度映射
            difficulty_map = {0: 5, 1: 10, 2: 20}

            # 如果是 AI_AI 模式，使用 AI 难度设置
            if self.game_mode == "AI_AI":
                red_skill_level = difficulty_map.get(self.red_diff_buttons.checkedId(), 10)
                black_skill_level = difficulty_map.get(self.black_diff_buttons.checkedId(), 10)
            else:
                # 在 HUMAN_AI 模式下，AI 端的难度使用 diff_buttons 选项
                red_skill_level = difficulty_map.get(self.diff_buttons.checkedId(), 10)
                black_skill_level = red_skill_level  # 假设 HUMAN_AI 模式下，红方和黑方使用相同的难度

            # 获取详细设置中的附加配置
            extra = getattr(self, "settings_data", {"think_time": 1.0, "limit_strength": False, "elo_rating": 2000})

            # 配置机械臂1和机械臂2的设置
            config_msg_1 = {
                "command": "config",
                "player_color": side,
                "think_time": extra["think_time"],
                "skill_level": red_skill_level,  # 机械臂1的难度
            }

            config_msg_2 = {
                "command": "config",
                "player_color": "RED",
                "think_time": extra["think_time"],
                "skill_level": black_skill_level,  # 机械臂2的难度
            }

            # HUMAN_AI 模式下，包含 ELO 等级
            if extra["limit_strength"] and self.game_mode == "HUMAN_AI":
                config_msg_1["elo"] = extra["elo_rating"]

            # 发送配置到 ROS
            if self.ros_node:
                self.ros_node.publish_uci_command(json.dumps(config_msg_1, ensure_ascii=False))
                self.ros_node.publish_rob_uci_command(json.dumps(config_msg_2, ensure_ascii=False))

            # 发射信号，开始游戏
            self.start_game_signal.emit({
                "config_1": config_msg_1,
                "config_2": config_msg_2,
                "player_color": side,
                "game_mode": self.game_mode
    })