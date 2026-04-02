import sys
import os
import pandas as pd
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                               QHBoxLayout, QPushButton, QTableWidget,
                               QTableWidgetItem, QLabel, QMessageBox, QTabWidget, QHeaderView)
from PySide6.QtGui import QPixmap, QFont
from PySide6.QtCore import Qt
from ament_index_python.packages import get_package_share_directory

class RoboticArmEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        # 文件路径
        self.paths = {
            "左机械臂 (Left)": os.path.join(get_package_share_directory("machinery_chess_control_bringup"),"config","machinery","left","position.csv"),
            "右机械臂 (Right)": os.path.join(get_package_share_directory("machinery_chess_control_bringup"),"config","machinery","right","position.csv"),
        }
        self.tables = {}
        self.init_ui()
        self.reload_all_data()

    def init_ui(self):
        self.setWindowTitle('双机械臂棋盘点位编辑器')
        self.setGeometry(100, 100, 1000, 850) # 调大初始窗口尺寸

        # 定义全局大字体
        large_font = QFont("Microsoft YaHei", 12) # 12号字体
        header_font = QFont("Microsoft YaHei", 13, QFont.Weight.Bold)
        self.setFont(large_font)

        # 主布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- 1. 顶部图片参考区域 ---
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pixmap = QPixmap('squares.png')
        if not pixmap.isNull():
            self.image_label.setPixmap(pixmap.scaledToHeight(300, Qt.TransformationMode.SmoothTransformation))
        else:
            self.image_label.setText("【未找到棋盘参考图 chessboard_reference.png】")
            self.image_label.setStyleSheet("border: 2px dashed #aaa; padding: 20px;")
        main_layout.addWidget(self.image_label)

        # --- 2. 新增：图片下方说明文字 ---
        self.desc_label = QLabel("提示：o0 为被吃子放置点位")
        self.desc_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.desc_label.setStyleSheet("color: #D32F2F; font-weight: bold; margin-bottom: 10px;")
        main_layout.addWidget(self.desc_label)

        # --- 3. 中间选项卡区域 ---
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("QTabBar::tab { height: 40px; width: 200px; font-size: 14px; }")

        for name in self.paths.keys():
            table = QTableWidget()
            # 设置表格字体
            table.setFont(large_font)
            # 设置表头字体
            table.horizontalHeader().setFont(header_font)
            # 设置行高
            table.verticalHeader().setDefaultSectionSize(45)
            # 设置列宽（先给一个基础宽度，后面加载数据后再自动调整部分）
            table.horizontalHeader().setDefaultSectionSize(150)

            self.tables[name] = table
            self.tabs.addTab(table, name)

        main_layout.addWidget(self.tabs)

        # --- 4. 底部控制按钮 ---
        button_layout = QHBoxLayout()

        self.btn_reload = QPushButton("🔄 重新读取文件")
        self.btn_reload.setFixedHeight(50) # 按钮加高
        self.btn_reload.clicked.connect(self.reload_all_data)

        self.btn_save = QPushButton("💾 保存所有数据")
        self.btn_save.setFixedHeight(50)
        self.btn_save.setStyleSheet("""
            QPushButton {
                background-color: #2E7D32; 
                color: white; 
                font-weight: bold; 
                font-size: 16px;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #388E3C; }
        """)
        self.btn_save.clicked.connect(self.save_all_data)

        button_layout.addWidget(self.btn_reload)
        button_layout.addSpacing(30)
        button_layout.addWidget(self.btn_save)

        main_layout.addLayout(button_layout)

    def load_csv_to_table(self, file_path, table_widget):
        if not os.path.exists(file_path):
            return False

        try:
            df = pd.read_csv(file_path)
            table_widget.setRowCount(df.shape[0])
            table_widget.setColumnCount(df.shape[1])
            table_widget.setHorizontalHeaderLabels(df.columns)

            for i in range(df.shape[0]):
                for j in range(df.shape[1]):
                    item = QTableWidgetItem(str(df.iloc[i, j]))
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter) # 文字居中
                    table_widget.setItem(i, j, item)

            # 第一列（point）宽度稍小，其余列等分或自适应
            table_widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
            table_widget.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Interactive)
            table_widget.setColumnWidth(0, 100)

            return True
        except Exception as e:
            print(f"读取错误: {e}")
            return False

    def reload_all_data(self):
        for name, path in self.paths.items():
            self.load_csv_to_table(path, self.tables[name])

    def save_all_data(self):
        try:
            for name, path in self.paths.items():
                table = self.tables[name]
                rows = table.rowCount()
                cols = table.columnCount()
                if rows == 0: continue

                headers = [table.horizontalHeaderItem(i).text() for i in range(cols)]
                data = []
                for row in range(rows):
                    row_data = [table.item(row, col).text() if table.item(row, col) else "" for col in range(cols)]
                    data.append(row_data)

                os.makedirs(os.path.dirname(path), exist_ok=True)
                pd.DataFrame(data, columns=headers).to_csv(path, index=False)

            QMessageBox.information(self, "成功", "左右臂数据已同步保存！")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存失败: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    editor = RoboticArmEditor()
    editor.show()
    sys.exit(app.exec())
