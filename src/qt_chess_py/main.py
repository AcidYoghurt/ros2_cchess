# main.py

import sys
import locale
from PySide6.QtWidgets import QApplication
from main_window import MainWindow
import os

def main():
    # 设置locale
    try:
        locale.setlocale(locale.LC_ALL, '')
    except locale.Error:
        print("Warning: Could not set default locale.")
    
    # 创建应用
    app = QApplication(sys.argv)
    
    # 加载样式表
    current_file = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_file)
    with open(current_dir+'/'+"resources/styles.qss", "r") as f:
        app.setStyleSheet(f.read())
    
    # 创建主窗口并显示
    window = MainWindow()  
    window.show()
    
    # 运行应用
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
