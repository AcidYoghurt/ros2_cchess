import os
import re
import json
import cchess
import cchess.svg
import cairosvg

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QListWidget, QDialog, QTextEdit, QProgressBar
)
from PySide6.QtCore import Qt, Signal, QByteArray, QThread
from PySide6.QtGui import QFont, QPixmap
from openai import OpenAI

# ---------------- 异步分析线程 ----------------
class AnalysisThread(QThread):
    finished = Signal(str)
    error = Signal(str)

    def __init__(self, client, moves_text):
        super().__init__()
        self.client = client
        self.moves_text = moves_text

    def run(self):
        try:
            system_prompt = "你是一位象棋大师。请根据提供的招法序列分析整局棋，说明红黑方的优缺点及转折点，使用 Markdown 格式。"
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[{"role": "system", "content": system_prompt}, {"role": "user", "content": self.moves_text}]
            )
            analysis = response.choices[0].message.content
            self.finished.emit(analysis)
        except Exception as e:
            self.error.emit(str(e))

# ---------------- 带进度条的分析弹窗 ----------------
class AnalysisDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("大师级深度复盘报告")
        self.resize(650, 550)
        layout = QVBoxLayout(self)
        self.setStyleSheet("QDialog { background-color: #fdfaf5; }")

        # 状态提示标签
        self.status_label = QLabel("正在解析棋谱...")
        self.status_label.setStyleSheet("color: #5d4037; font-weight: bold;")
        layout.addWidget(self.status_label)

        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 0) # 繁忙模式：来回滚动
        self.progress_bar.setStyleSheet("""
            QProgressBar { border: 1px solid #bbb; border-radius: 5px; height: 10px; background: #eee; }
            QProgressBar::chunk { background-color: #4CAF50; width: 20px; }
        """)
        layout.addWidget(self.progress_bar)

        # 文本展示区
        self.text_area = QTextEdit()
        self.text_area.setReadOnly(True)
        self.text_area.setStyleSheet("""
            QTextEdit { 
                background-color: white; border: 1px solid #d7ccc8; 
                border-radius: 8px; padding: 15px; font-size: 14px; 
                color: #333; line-height: 1.6;
            }
        """)
        layout.addWidget(self.text_area)

        self.close_btn = QPushButton("合上卷轴")
        self.close_btn.setFixedHeight(35)
        self.close_btn.clicked.connect(self.accept)
        layout.addWidget(self.close_btn)

    def show_loading(self, message):
        """显示加载状态"""
        self.status_label.setText(message)
        self.status_label.show()
        self.progress_bar.show()

    def hide_loading(self):
        """分析完成后隐藏进度条"""
        self.status_label.hide()
        self.progress_bar.hide()

    def set_content(self, content):
        self.text_area.setMarkdown(content)

# ---------------- 主界面 ----------------
class HistoryPage(QWidget):
    back_signal = Signal()

    def __init__(self):
        super().__init__()
        
        self.games_dir = "games"
        self.step_fens = []
        self.full_move_list = []
        self.current_filename = "" 
        self.current_step = 0
        
        self.client = OpenAI(
            api_key="sk-vedzQ4e5pQbLzOL2K9frEJl9zbdiPt0BOCMCqHRHenmBM7BP", 
            base_url="https://api.chatanywhere.tech/v1"
        )
        
        self.setup_ui()
        self.refresh_file_list()

    def setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # --- Left ---
        left_container = QWidget()
        left_container.setFixedWidth(200)
        left_layout = QVBoxLayout(left_container)
        left_layout.addWidget(QLabel("历史对局记录"))
        self.file_list = QListWidget()
        self.file_list.itemClicked.connect(self.load_selected_pgn)
        left_layout.addWidget(self.file_list)
        refresh_btn = QPushButton("刷新列表")
        refresh_btn.clicked.connect(self.refresh_file_list)
        left_layout.addWidget(refresh_btn)
        main_layout.addWidget(left_container)

        # --- Center ---
        board_layout = QVBoxLayout()
        self.title_label = QLabel("对局详情")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setFont(QFont("Microsoft YaHei", 16, QFont.Bold))
        self.board_label = QLabel()
        self.board_label.setFixedSize(600, 600)
        self.board_label.setAlignment(Qt.AlignCenter)
        self.board_label.setStyleSheet("QLabel { background-color: #f5e8d5; border: 1px solid #999; }")
        board_layout.addWidget(self.title_label)
        board_layout.addWidget(self.board_label, alignment=Qt.AlignCenter)
        main_layout.addLayout(board_layout, stretch=1)

        # --- Right ---
        right_container = QWidget()
        right_container.setFixedWidth(220)
        right_layout = QVBoxLayout(right_container)
        right_layout.addWidget(QLabel("招法详情:"))
        self.move_list = QListWidget()
        self.move_list.itemClicked.connect(self.on_move_list_clicked)
        right_layout.addWidget(self.move_list)
        self.step_label = QLabel("步骤: 0 / 0")
        self.step_label.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.step_label)

        btn_layout = QHBoxLayout()
        self.prev_btn = QPushButton("上一步")
        self.next_btn = QPushButton("下一步")
        self.prev_btn.clicked.connect(self.show_prev)
        self.next_btn.clicked.connect(self.show_next)
        btn_layout.addWidget(self.prev_btn)
        btn_layout.addWidget(self.next_btn)
        right_layout.addLayout(btn_layout)

        self.ai_btn = QPushButton("🔍 分析整局")
        self.ai_btn.setFixedHeight(45)
        self.ai_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.ai_btn.setEnabled(False)
        self.ai_btn.clicked.connect(self.handle_ai_analysis)
        right_layout.addWidget(self.ai_btn)

        right_layout.addStretch()
        back_btn = QPushButton("返回主菜单")
        back_btn.setFixedHeight(40)
        back_btn.clicked.connect(self.back_signal.emit)
        right_layout.addWidget(back_btn)
        main_layout.addWidget(right_container)

    # ---------------- 业务逻辑 ----------------

    def handle_ai_analysis(self):
        cache_path = os.path.join(self.games_dir, f"{os.path.splitext(self.current_filename)[0]}_analysis.json")
        self.dialog = AnalysisDialog(self)
        
        if os.path.exists(cache_path):
            try:
                with open(cache_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.dialog.hide_loading()
                    self.dialog.set_content(data.get("analysis", ""))
                    self.dialog.exec()
                    return
            except: pass

        # 启动异步请求
        self.dialog.show_loading("大师正在拆解本局招法...")
        self.dialog.set_content("> 正在连接 AI 智库，请稍候。")
        self.dialog.show() # 非阻塞显示

        moves_text = " -> ".join(self.full_move_list)
        self.worker = AnalysisThread(self.client, moves_text)
        self.worker.finished.connect(lambda content: self.on_ai_finished(content, cache_path))
        self.worker.error.connect(self.on_ai_error)
        self.worker.start()

    def on_ai_finished(self, content, cache_path):
        self.dialog.hide_loading()
        self.dialog.set_content(content)
        # 缓存到本地
        try:
            with open(cache_path, 'w', encoding='utf-8') as f:
                json.dump({"analysis": content}, f, ensure_ascii=False)
        except: pass

    def on_ai_error(self, err):
        self.dialog.hide_loading()
        self.dialog.set_content(f"### ❌ 分析出现问题\n\n原因：{err}\n\n建议检查网络代理设置。")

    def load_selected_pgn(self, item):
        self.current_filename = item.text()
        file_path = os.path.join(self.games_dir, self.current_filename)
        try:
            board = cchess.Board.from_pgn(file_path)
            moves = list(board.move_stack)
            self.step_fens.clear()
            self.full_move_list.clear()
            move_names = ["起始位置"]
            temp = cchess.Board(fen=board._starting_fen)
            self.step_fens.append(temp.fen())
            for i, mv in enumerate(moves):
                m_notat = temp.move_to_notation(mv)
                self.full_move_list.append(m_notat)
                prefix = f"{i//2+1}." if i%2==0 else "   "
                move_names.append(f"{prefix} {m_notat}")
                temp.push(mv)
                self.step_fens.append(temp.fen())
            self.move_list.clear()
            self.move_list.addItems(move_names)
            self.current_step = 0
            self.update_display()
            self.ai_btn.setEnabled(True)
        except Exception as e: print(f"Error: {e}")

    def update_display(self):
        if not self.step_fens: return
        fen = self.step_fens[self.current_step]
        svg = cchess.svg.board(cchess.Board(fen), size=1200, coordinates=True, axes_type=1)
        png_bytes = cairosvg.svg2png(bytestring=svg.encode("utf-8"))
        pixmap = QPixmap()
        pixmap.loadFromData(png_bytes)
        pixmap = pixmap.scaled(self.board_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.board_label.setPixmap(pixmap)
        self.step_label.setText(f"步骤: {self.current_step} / {len(self.step_fens)-1}")

    def refresh_file_list(self):
        self.file_list.clear()
        os.makedirs(self.games_dir, exist_ok=True)
        self.file_list.addItems(sorted([f for f in os.listdir(self.games_dir) if f.endswith(".pgn")], reverse=True))

    def on_move_list_clicked(self, item):
        self.current_step = self.move_list.row(item); self.update_display()

    def show_prev(self):
        if self.current_step > 0: self.current_step -= 1; self.update_display()

    def show_next(self):
        if self.current_step < len(self.step_fens)-1: self.current_step += 1; self.update_display()