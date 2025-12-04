# qt_chess_p/pages/settings_page.py

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
        self.game_mode = game_mode  # "HUMAN_AI" 或 "AI_AI"
         # 设置页面背景色
        self.setStyleSheet("""
            QWidget {
                background-color: #2E3440;
                color: #ECEFF4;
                font-family: "Microsoft YaHei", sans-serif;
            }
        """)


        # 定义基础按钮样式（在这里定义，确保所有模式都能访问）
        self.base_btn_style = """
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

        self.setup_ui()

    def setup_ui(self):
        # 滚动区域包装内容，防止缩小时溢出
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(scroll)

        content = QWidget()
        scroll.setWidget(content)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(60, 50, 60, 50)
        layout.setSpacing(50)

        # ===== 页面标题（显示当前模式） =====
        mode_title = "人机对战设置" if self.game_mode == "HUMAN_AI" else "机机对战设置"
        title = QLabel(mode_title)
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Microsoft YaHei", 28, QFont.Bold))
        title.setStyleSheet("color: #2c3e50; margin-bottom: 20px;")
        layout.addWidget(title)

        # ===== 难度选择（机机模式下显示两个难度选择） =====
        if self.game_mode == "AI_AI":
            diff_group = QGroupBox("难度选择")
            diff_group.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
            diff_layout = QVBoxLayout(diff_group)
            diff_layout.setContentsMargins(40, 40, 40, 40)
            diff_layout.setSpacing(60)

            self.red_diff_buttons = QButtonGroup(self)
            self.red_diff_buttons.setExclusive(True)
            self.black_diff_buttons = QButtonGroup(self)
            self.black_diff_buttons.setExclusive(True)

            # 红方难度选择
            red_difficulty_layout = QHBoxLayout()  # 使用水平布局
            red_difficulty_title = QLabel("红方难度")
            red_difficulty_title.setFont(QFont("Microsoft YaHei", 22))
            red_difficulty_layout.addWidget(red_difficulty_title)

            levels = ["入门", "进阶", "大师"]
            red_diff_style = self.base_btn_style + """
                QPushButton:checked {
                    background-color: #e74c3c;
                    border: 3px solid #922b21;
                }
            """
            for i, level in enumerate(levels):
                btn = QPushButton(level)
                btn.setCheckable(True)
                btn.setStyleSheet(red_diff_style)
                btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                if i == 1:
                    btn.setChecked(True)
                self.red_diff_buttons.addButton(btn, i)
                red_difficulty_layout.addWidget(btn)

            # 黑方难度选择
            black_difficulty_layout = QHBoxLayout()  # 使用水平布局
            black_difficulty_title = QLabel("黑方难度")
            black_difficulty_title.setFont(QFont("Microsoft YaHei", 22))
            black_difficulty_layout.addWidget(black_difficulty_title)

            black_diff_style = self.base_btn_style + """
                QPushButton:checked {
                    background-color: #2c3e50;
                    border: 3px solid #000000;
                }
            """
            for i, level in enumerate(levels):
                btn = QPushButton(level)
                btn.setCheckable(True)
                btn.setStyleSheet(black_diff_style)
                btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                if i == 1:
                    btn.setChecked(True)
                self.black_diff_buttons.addButton(btn, i)
                black_difficulty_layout.addWidget(btn)

            diff_layout.addLayout(red_difficulty_layout)
            diff_layout.addLayout(black_difficulty_layout)
            layout.addWidget(diff_group)

        # ===== 仅人机模式下的难度选择 =====
        if self.game_mode == "HUMAN_AI":
            side_group = QGroupBox("玩家选择阵营")
            side_group.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
            side_layout = QHBoxLayout(side_group)
            side_layout.setContentsMargins(40, 40, 40, 40)
            side_layout.setSpacing(60)

            self.side_buttons = QButtonGroup(self)
            self.side_buttons.setExclusive(True)

            # 红方按钮
            self.red_btn = QPushButton("红方")
            self.red_btn.setCheckable(True)
            self.red_btn.setStyleSheet(self.base_btn_style + """
                QPushButton:checked {
                    background-color: #e74c3c;
                    border: 3px solid #922b21;
                }
            """)
            self.red_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            # 黑方按钮
            self.black_btn = QPushButton("黑方")
            self.black_btn.setCheckable(True)
            self.black_btn.setStyleSheet(self.base_btn_style + """
                QPushButton:checked {
                    background-color: #2c3e50;
                    border: 3px solid #000000;
                }
            """)
            self.black_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            self.red_btn.setChecked(True)
            self.side_buttons.addButton(self.red_btn, 0)
            self.side_buttons.addButton(self.black_btn, 1)

            side_layout.addWidget(self.red_btn)
            side_layout.addWidget(self.black_btn)
            layout.addWidget(side_group)
            difficulty_group = QGroupBox("难度选择")
            difficulty_group.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
            diff_layout = QHBoxLayout(difficulty_group)
            diff_layout.setContentsMargins(40, 40, 40, 40)
            diff_layout.setSpacing(60)

            self.diff_buttons = QButtonGroup(self)
            self.diff_buttons.setExclusive(True)

            diff_style = self.base_btn_style + """
                QPushButton:checked {
                    background-color: #34495e;
                    border: 3px solid #1f2c38;
                }
            """

            levels = ["入门", "进阶", "大师"]
            for i, level in enumerate(levels):
                btn = QPushButton(level)
                btn.setCheckable(True)
                btn.setStyleSheet(diff_style)
                btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                if i == 1:
                    btn.setChecked(True)
                self.diff_buttons.addButton(btn, i)
                diff_layout.addWidget(btn)

            layout.addWidget(difficulty_group)
           

        # ===== 详细设置 =====
        details_btn = QPushButton("详细设置...")
        details_btn.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
        details_btn.setStyleSheet("""
            QPushButton {
                background-color: #8e44ad;
                color: white;
                border-radius: 18px;
                padding: 26px 100px;
                font-size: 26px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #9b59b6;
            }
            QPushButton:pressed {
                background-color: #6c3483;
            }
        """)
        details_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        details_btn.clicked.connect(self.show_detailed_settings)
        layout.addWidget(details_btn, alignment=Qt.AlignHCenter)

        # ===== 开始游戏 =====
        apply_btn = QPushButton("开始游戏")
        apply_btn.setFont(QFont("Microsoft YaHei", 30, QFont.Bold))
        apply_btn.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                border-radius: 24px;
                padding: 30px 120px;
                font-size: 30px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #52be80;
            }
            QPushButton:pressed {
                background-color: #1e8449;
            }
        """)
        apply_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        apply_btn.clicked.connect(self.apply_settings)
        layout.addWidget(apply_btn, alignment=Qt.AlignHCenter)

        layout.addStretch()

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

    # ===== 应用配置 =====
    def apply_settings(self):
        # 判断玩家选择的阵营（仅在 HUMAN_AI 模式下）
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
            import json
            self.ros_node.publish_uci_command(json.dumps(config_msg_1, ensure_ascii=False))
            self.ros_node.publish_rob_uci_command(json.dumps(config_msg_2, ensure_ascii=False))

        # 发射信号，开始游戏
        self.start_game_signal.emit({
            "config_1": config_msg_1,
            "config_2": config_msg_2,
            "player_color": side,
            "game_mode": self.game_mode
        })