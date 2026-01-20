from PySide6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

class StartPage(QWidget):
    settings_signal = Signal()   # 进入模式选择页
    history_signal = Signal()    # 进入历史记录页 (新增)
    quit_signal = Signal()       # 退出程序
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(60, 80, 60, 80)
        layout.setSpacing(30)

        # 标题
        title = QLabel("中国象棋")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("KaiTi", 64, QFont.Bold))
        title.setStyleSheet("color: #fafafa; margin-bottom: 40px;")

        # 统一按钮样式
        btn_style = """
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-size: 28px;
                font-weight: bold;
                border-radius: 15px;
                padding: 15px;
            }
            QPushButton:hover { background-color: #ff6b5a; }
        """
        
        # 按钮创建
        enter_btn = QPushButton("进入游戏")
        enter_btn.setStyleSheet(btn_style)
        enter_btn.setFixedHeight(90)
        enter_btn.clicked.connect(self.settings_signal.emit)

        # 历史记录按钮 (使用蓝色调区分)
        self.history_btn = QPushButton("查看历史对局")
        self.history_btn.setStyleSheet(btn_style.replace("#e74c3c", "#3498db").replace("#ff6b5a", "#5dade2"))
        self.history_btn.setFixedHeight(90)
        self.history_btn.clicked.connect(self.history_signal.emit)

        quit_btn = QPushButton("退出游戏")
        quit_btn.setStyleSheet(btn_style.replace("#e74c3c", "#34495e").replace("#ff6b5a", "#5d6d7e"))
        quit_btn.setFixedHeight(90)
        quit_btn.clicked.connect(self.quit_signal.emit)

        # 布局组装
        layout.addStretch()
        layout.addWidget(title)
        layout.addWidget(enter_btn)
        layout.addWidget(self.history_btn) 
        layout.addWidget(quit_btn)
        layout.addStretch()
