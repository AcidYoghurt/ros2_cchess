import sys
import os
import pandas as pd
import serial
import time
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTableWidget, 
                             QTableWidgetItem, QLabel, QMessageBox, QTabWidget, 
                             QHeaderView, QLineEdit)
from PySide6.QtGui import QPixmap, QFont, QRegularExpressionValidator
from PySide6.QtCore import Qt, QRegularExpression

class RoboticArmEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        # 串口配置：请根据实际设备路径修改（如 /dev/ttyUSB0）
        self.config = {
            "左机械臂 (Left)": {"path": "/home/chess/Desktop/ros2_cchess/src/cchess_ros_control/machinery_chess_ros_control_bringup/config/machinery/left/position.csv", "port": "/dev/machineryLeftA"},
            "右机械臂 (Right)": {"path": "/home/chess/Desktop/ros2_cchess/src/cchess_ros_control/machinery_chess_ros_control_bringup/config/machinery/right/position.csv", "port": "/dev/machineryRightB"}
        }
        
        # 点位顺序序列
        self.point_sequence = []
        for r in range(9, -1, -1):
            for c in "abcdefghi":
                self.point_sequence.append(f"{c}{r}")
        self.point_sequence.append("o0")

        self.tables = {} 
        self.init_ui()
        self.reload_all_data()

    def init_ui(self):
        self.setWindowTitle('机械臂点位校准与编辑器')
        self.setGeometry(100, 100, 1150, 850)
        self.setFont(QFont("Microsoft YaHei", 12))

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # 1. 顶部图片与说明
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pixmap = QPixmap('chessboard_reference.png')
        if not pixmap.isNull():
            self.image_label.setPixmap(pixmap.scaledToHeight(220, Qt.TransformationMode.SmoothTransformation))
        main_layout.addWidget(self.image_label)

        desc = QLabel("说明：o0 (Z+200) | 其余 (Z+15) | 速度: 30% ")
        desc.setAlignment(Qt.AlignmentFlag.AlignCenter)
        desc.setStyleSheet("color: #D32F2F; font-weight: bold; margin-bottom: 5px;")
        main_layout.addWidget(desc)

        # 2. 测试控制面板
        test_panel = QHBoxLayout()
        test_panel.addStretch()
        
        # 回到原点按钮 (新加)
        self.btn_origin = QPushButton("🏠 回到原点")
        self.btn_origin.setFixedHeight(45)
        self.btn_origin.setStyleSheet("background-color: #455A64; color: white; padding: 0 15px;")
        self.btn_origin.clicked.connect(self.go_to_origin)
        
        self.test_input = QLineEdit()
        self.test_input.setPlaceholderText("a9")
        self.test_input.setFixedWidth(80)
        self.test_input.setFixedHeight(45)
        self.test_input.setValidator(QRegularExpressionValidator(QRegularExpression("^(o0|[a-i][0-9])$")))
        
        self.btn_test = QPushButton("🎯 测试当前点")
        self.btn_test.setFixedHeight(45)
        self.btn_test.clicked.connect(self.run_point_test)
        

        self.btn_next = QPushButton("⏭️ 下一个点位")
        self.btn_next.setFixedHeight(45)
        self.btn_next.setStyleSheet("background-color: #FB8C00; color: white; font-weight: bold;")
        self.btn_next.clicked.connect(self.go_to_next_point)

        self.btn_drop = QPushButton("⬇️ 落子测试")
        self.btn_drop.setFixedHeight(45)
        self.btn_drop.setStyleSheet("background-color: #C62828; color: white; font-weight: bold;")
        self.btn_drop.clicked.connect(self.run_point_drop)


        test_panel.addWidget(self.btn_origin) # 加入原点按钮
        test_panel.addSpacing(20)
        test_panel.addWidget(QLabel("点位:"))
        test_panel.addWidget(self.test_input)
        test_panel.addWidget(self.btn_test)
        test_panel.addWidget(self.btn_next)
        test_panel.addWidget(self.btn_drop)   
        test_panel.addStretch()
        main_layout.addLayout(test_panel)

        # === 批量偏置控制 ===
        offset_layout = QHBoxLayout()
        offset_layout.addStretch()

        offset_layout.addWidget(QLabel("批量偏置："))

        self.offset_axis = QLineEdit()
        self.offset_axis.setPlaceholderText("x / y / z")
        self.offset_axis.setFixedWidth(60)
        self.offset_axis.setAlignment(Qt.AlignCenter)
        self.offset_axis.setValidator(QRegularExpressionValidator(QRegularExpression("^[xyzXYZ]$")))

        self.offset_value = QLineEdit()
        self.offset_value.setPlaceholderText("偏置量")
        self.offset_value.setFixedWidth(80)
        self.offset_value.setValidator(QRegularExpressionValidator(
            QRegularExpression("^[-+]?[0-9]*\\.?[0-9]+$")
        ))

        self.btn_apply_offset = QPushButton("应用偏置")
        self.btn_apply_offset.setFixedHeight(40)
        self.btn_apply_offset.setStyleSheet("background-color: #1565C0; color: white; font-weight: bold;")
        self.btn_apply_offset.clicked.connect(self.apply_batch_offset)

        offset_layout.addWidget(self.offset_axis)
        offset_layout.addWidget(self.offset_value)
        offset_layout.addWidget(self.btn_apply_offset)

        offset_layout.addStretch()
        main_layout.addLayout(offset_layout)

        # 3. 中间表格选项卡
        self.tabs = QTabWidget()
        for name in self.config.keys():
            table = QTableWidget()
            table.verticalHeader().setDefaultSectionSize(40)
            self.tables[name] = table
            self.tabs.addTab(table, name)
        main_layout.addWidget(self.tabs)

        # 4. 底部按钮区
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        self.btn_reload = QPushButton("🔄 重新读取文件 (放弃更改)")
        self.btn_reload.setFixedHeight(40)
        self.btn_reload.clicked.connect(self.reload_all_data)

        self.btn_save = QPushButton("💾 保存所有修改至 CSV 文件")
        self.btn_save.setFixedHeight(50)
        self.btn_save.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold;")
        self.btn_save.clicked.connect(self.save_all_data)

        button_layout.addWidget(self.btn_reload)
        button_layout.addSpacing(20)
        button_layout.addWidget(self.btn_save)

        button_layout.addStretch()
        main_layout.addLayout(button_layout)


    # --- 逻辑功能 ---

    def go_to_origin(self):
        """发送回到原点指令"""
        arm_name = self.tabs.tabText(self.tabs.currentIndex())
        port = self.config[arm_name]["port"]
        self.send_serial(port, "Origin\r\n")

    def go_to_next_point(self):
        curr = self.test_input.text().strip().lower()
        if not curr or curr not in self.point_sequence:
            nxt = self.point_sequence[0]
        else:
            idx = self.point_sequence.index(curr)
            nxt = self.point_sequence[(idx + 1) % len(self.point_sequence)]
        self.test_input.setText(nxt)
        self.run_point_test()

    def run_point_test(self):
        point_id = self.test_input.text().strip().lower()
        if not point_id: return
        
        arm_name = self.tabs.tabText(self.tabs.currentIndex())
        port = self.config[arm_name]["port"]
        table = self.tables[arm_name]

        target_row = -1
        for r in range(table.rowCount()):
            if table.item(r, 0).text() == point_id:
                target_row = r
                break
        
        if target_row != -1:
            table.selectRow(target_row)
            table.scrollToItem(table.item(target_row, 0))
            try:
                x = table.item(target_row, 1).text()
                y = table.item(target_row, 2).text()
                z = float(table.item(target_row, 3).text())
                
                real_z = z + (170 if point_id == "o0" else 15)
                cmd = f"DescartesPoint_{x},{y},{real_z},30\r\n"
                self.send_serial(port, cmd)
            except Exception as e:
                print(f"数据错误: {e}")

    def run_point_drop(self):
        """真正落子测试：使用 CSV 中的真实 Z，不加偏移"""
        point_id = self.test_input.text().strip().lower()
        if not point_id:
            return

        arm_name = self.tabs.tabText(self.tabs.currentIndex())
        port = self.config[arm_name]["port"]
        table = self.tables[arm_name]

        target_row = -1
        for r in range(table.rowCount()):
            if table.item(r, 0).text() == point_id:
                target_row = r
                break

        if target_row != -1:
            table.selectRow(target_row)
            table.scrollToItem(table.item(target_row, 0))
            try:
                x = table.item(target_row, 1).text()
                y = table.item(target_row, 2).text()
                z = table.item(target_row, 3).text()  # ⚠️ 不加任何偏移

                cmd = f"DescartesPoint_{x},{y},{z},30\r\n"
                self.send_serial(port, cmd)
            except Exception as e:
                print(f"数据错误: {e}")


    def send_serial(self, port, cmd):
        """通用串口发送逻辑"""
        try:
            with serial.Serial(port, 115200, timeout=1) as ser:
                ser.write(cmd.encode('utf-8'))
                print(f"成功发送到 {port}: {cmd.strip()}")
        except Exception as e:
            QMessageBox.warning(self, "串口错误", f"无法连接 {port}\n{e}")

    def load_csv_to_table(self, path, table):
        if not os.path.exists(path): return
        df = pd.read_csv(path)
        table.setRowCount(df.shape[0])
        table.setColumnCount(df.shape[1])
        table.setHorizontalHeaderLabels(df.columns)
        for i in range(df.shape[0]):
            for j in range(df.shape[1]):
                item = QTableWidgetItem(str(df.iloc[i, j]))
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                if j == 0: # 锁定第一列
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                    item.setBackground(Qt.GlobalColor.lightGray)
                table.setItem(i, j, item)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

    def apply_batch_offset(self):
        axis = self.offset_axis.text().lower()
        if axis not in ("x", "y", "z"):
            QMessageBox.warning(self, "输入错误", "轴只能是 x / y / z")
            return

        try:
            offset = float(self.offset_value.text())
        except ValueError:
            QMessageBox.warning(self, "输入错误", "偏置量必须是数字")
            return

        arm_name = self.tabs.tabText(self.tabs.currentIndex())
        table = self.tables[arm_name]

        col_map = {"x": 1, "y": 2, "z": 3}
        col = col_map[axis]

        count = 0
        for r in range(table.rowCount()):
            point_id = table.item(r, 0).text()
            if point_id == "o0":
                continue

            try:
                old_val = float(table.item(r, col).text())
                new_val = old_val + offset
                table.item(r, col).setText(f"{new_val:.3f}")
                count += 1
            except Exception:
                pass

        QMessageBox.information(
            self,
            "完成",
            f"已对 {count} 个点的 {axis.upper()} 轴应用偏置 {offset}"
        )


    def reload_all_data(self):
        for name, info in self.config.items():
            self.load_csv_to_table(info["path"], self.tables[name])

    def save_all_data(self):
        try:
            for name, info in self.config.items():
                table = self.tables[name]
                headers = ['point', 'x', 'y', 'z']
                data = [[table.item(r, c).text() for c in range(4)] for r in range(table.rowCount())]
                pd.DataFrame(data, columns=headers).to_csv(info["path"], index=False)
            QMessageBox.information(self, "成功", "所有数据已保存！")
        except Exception as e:
            QMessageBox.critical(self, "保存失败", str(e))

    def closeEvent(self, event):
        """当用户关闭窗口时触发"""
        # 创建一个询问对话框
        reply = QMessageBox.question(
            self, 
            '确认退出', 
            "您有未保存的更改吗？建议在退出前保存所有点位数据。\n\n是否直接退出？",
            QMessageBox.StandardButton.Save | 
            QMessageBox.StandardButton.Discard | 
            QMessageBox.StandardButton.Cancel, 
            QMessageBox.StandardButton.Cancel
        )

        if reply == QMessageBox.StandardButton.Save:
            # 如果点击“保存”，执行保存逻辑并关闭
            self.save_all_data()
            event.accept()
        elif reply == QMessageBox.StandardButton.Discard:
            # 如果点击“放弃”，不保存直接关闭
            event.accept()
        else:
            # 如果点击“取消”或直接关掉弹窗，拦截关闭事件，保持窗口开启
            event.ignore()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    ex = RoboticArmEditor()
    ex.show()
    sys.exit(app.exec())
