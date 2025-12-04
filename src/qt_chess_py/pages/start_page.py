from PySide6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

class StartPage(QWidget):
    settings_signal = Signal()   # 进入游戏 → 跳到设置页
    quit_signal = Signal()       # 退出信号
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(60, 80, 60, 80)
        layout.setSpacing(40)

        # 标题
        title = QLabel("中国象棋")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("KaiTi", 48, QFont.Bold))
        title.setStyleSheet("color: #fafafa; margin-bottom: 60px;")

        # 按钮样式（统一）
        btn_style = """
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-size: 24px;
                font-weight: bold;
                border-radius: 12px;
                padding: 15px 30px;
            }
            QPushButton:hover {
                background-color: #ff6b5a;
            }
        """

        # 进入游戏按钮
        enter_btn = QPushButton("进入游戏")
        enter_btn.setStyleSheet(btn_style)
        enter_btn.setFixedHeight(80)
        enter_btn.clicked.connect(self.settings_signal.emit)

        # 退出按钮
        quit_btn = QPushButton("退出游戏")
        quit_btn.setStyleSheet(btn_style.replace("#e74c3c", "#34495e").replace("#ff6b5a", "#3e5870"))
        quit_btn.setFixedHeight(80)
        quit_btn.clicked.connect(self.quit_signal.emit)

        # 布局
        layout.addStretch()
        layout.addWidget(title)
        layout.addWidget(enter_btn)
        layout.addSpacing(20)
        layout.addWidget(quit_btn)
        layout.addStretch()
