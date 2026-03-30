import os
import re
import json
import math
import cchess
import cchess.svg
import cairosvg

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QListWidget, QListWidgetItem, QDialog, QTextEdit, QProgressBar,
    QFrame, QAbstractItemView
)
from PySide6.QtCore import Qt, Signal, QThread, QTimer, QPointF, QSize
from PySide6.QtGui import QFont, QPixmap, QPainter, QPen, QColor, QPolygonF
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


class StepReviewThread(QThread):
    finished = Signal(dict)
    error = Signal(str)

    def __init__(self, client, move_list):
        super().__init__()
        self.client = client
        self.move_list = move_list

    def run(self):
        try:
            system_prompt = (
                "你是一位中国象棋教练。"
                "我会给你完整对局招法。请从第6步开始逐步点评每一步。"
                "每一步必须给 verdict(好/可行/不好)、reason、suggestion。"
                "如果 verdict 是 好，则 suggestion 为空字符串。"
                "只输出 JSON，不要输出任何额外文字。"
            )
            numbered_moves = "\n".join(f"{idx + 1}. {mv}" for idx, mv in enumerate(self.move_list))
            user_prompt = (
                "请按以下格式返回：\n"
                "{\n"
                '  "reviews": [\n'
                "    {\n"
                '      "step": 6,\n'
                '      "verdict": "好|可行|不好",\n'
                '      "reason": "简要原因",\n'
                '      "suggestion": "如果不是好，给出建议走法；如果是好则为空字符串"\n'
                "    }\n"
                "  ]\n"
                "}\n\n"
                "规则：\n"
                "- step 从 1 开始，对应第 N 个招法。\n"
                "- 只返回第6步到最后一步。\n"
                "- reason 和 suggestion 使用简体中文。\n\n"
                f"招法列表：\n{numbered_moves}"
            )

            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.2
            )
            raw_text = response.choices[0].message.content or ""
            payload = self._parse_json_payload(raw_text)
            if not isinstance(payload, dict):
                raise ValueError("AI 返回的 JSON 结构不是对象")
            self.finished.emit(payload)
        except Exception as e:
            self.error.emit(str(e))

    def _parse_json_payload(self, text):
        content = text.strip()
        block = re.search(r"```(?:json)?\s*([\s\S]*?)```", content, re.IGNORECASE)
        if block:
            content = block.group(1).strip()
        try:
            return json.loads(content)
        except Exception:
            start = content.find("{")
            end = content.rfind("}")
            if start >= 0 and end > start:
                return json.loads(content[start:end + 1])
            raise ValueError("AI 返回结果不是合法 JSON")

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


class WheelListWidget(QListWidget):
    """带轮盘观感的列表：居中项高亮、滚动后自动吸附到最近项。"""

    def __init__(self, item_height=48, parent=None):
        super().__init__(parent)
        self.base_item_height = item_height
        self._snap_to_current_once = False
        self._snap_timer = QTimer(self)
        self._snap_timer.setSingleShot(True)
        self._snap_timer.setInterval(120)
        self._snap_timer.timeout.connect(self.snap_to_nearest_item)

        self.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSpacing(4)

        self.currentRowChanged.connect(self.center_current_row)
        self.currentRowChanged.connect(lambda _: self.update_wheel_effect())
        self.verticalScrollBar().valueChanged.connect(self.update_wheel_effect)

    def set_text_items(self, texts):
        self.clear()
        for text in texts:
            item = QListWidgetItem(text)
            item.setTextAlignment(Qt.AlignCenter)
            item.setSizeHint(QSize(0, self.base_item_height))
            self.addItem(item)
        QTimer.singleShot(0, self.update_wheel_effect)

    def wheelEvent(self, event):
        super().wheelEvent(event)
        self._snap_timer.start()

    def mouseReleaseEvent(self, event):
        clicked_item = self.itemAt(event.pos())
        super().mouseReleaseEvent(event)
        if clicked_item is not None:
            self._snap_to_current_once = True
        self._snap_timer.start()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        QTimer.singleShot(0, self.update_wheel_effect)

    def center_current_row(self, row):
        if row < 0:
            return
        item = self.item(row)
        if item is None:
            return
        self.scrollToItem(item, QAbstractItemView.PositionAtCenter)

    def snap_to_nearest_item(self):
        if self.count() == 0:
            return
        if self._snap_to_current_once and self.currentRow() >= 0:
            self._snap_to_current_once = False
            self.center_current_row(self.currentRow())
            self.update_wheel_effect()
            return
        self._snap_to_current_once = False
        viewport_center = self.viewport().rect().center().y()
        nearest_row = -1
        nearest_dist = float("inf")
        for row in range(self.count()):
            rect = self.visualItemRect(self.item(row))
            if not rect.isValid():
                continue
            dist = abs(rect.center().y() - viewport_center)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_row = row
        if nearest_row >= 0 and nearest_row != self.currentRow():
            self.setCurrentRow(nearest_row)
        elif nearest_row >= 0:
            self.center_current_row(nearest_row)
        self.update_wheel_effect()

    def update_wheel_effect(self):
        if self.count() == 0:
            return
        viewport_center = self.viewport().rect().center().y()
        span = max(1, self.viewport().height() * 0.55)
        for row in range(self.count()):
            item = self.item(row)
            rect = self.visualItemRect(item)
            if not rect.isValid():
                continue
            dist = abs(rect.center().y() - viewport_center)
            weight = max(0.0, 1.0 - dist / span)

            font = QFont("Microsoft YaHei", 10 + int(weight * 4))
            font.setBold(weight > 0.62 or row == self.currentRow())
            item.setFont(font)

            alpha = 85 + int(weight * 170)
            item.setForeground(QColor(237, 242, 253, alpha))

            if row == self.currentRow():
                item.setBackground(QColor(92, 143, 212, 145))
            else:
                item.setBackground(QColor(74, 102, 154, 35 + int(weight * 35)))

# ---------------- 主界面 ----------------
class HistoryPage(QWidget):
    back_signal = Signal()

    def __init__(self):
        super().__init__()
        
        self.games_dir = "games"
        self.step_fens = []
        self.step_moves = []
        self.step_move_colors = []
        self.full_move_list = []
        self.step_reviews = []
        self.step_review_worker = None
        self.step_review_loading = False
        self.step_review_request_file = ""
        self.current_filename = "" 
        self.current_step = 0
        self.play_interval_ms = 900
        self.play_timer = QTimer(self)
        self.play_timer.setInterval(self.play_interval_ms)
        self.play_timer.timeout.connect(self.play_next_step)
        
        self.client = OpenAI(
            api_key="sk-vedzQ4e5pQbLzOL2K9frEJl9zbdiPt0BOCMCqHRHenmBM7BP", 
            base_url="https://api.chatanywhere.tech/v1"
        )
        
        self.setup_ui()
        self.refresh_file_list()

    def setup_ui(self):
        self.setObjectName("HistoryPage")
        self.setStyleSheet("""
            QWidget#HistoryPage {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #20283f, stop:1 #161d30);
            }
            QFrame#SideCard {
                background-color: rgba(34, 47, 76, 0.88);
                border: 1px solid rgba(157, 182, 228, 0.25);
                border-radius: 18px;
            }
            QLabel#PanelTitle {
                color: #e5ecfa;
                font-size: 17px;
                font-weight: 700;
                letter-spacing: 1px;
            }
            QLabel#BoardTitle {
                color: #edf2fd;
                font-size: 26px;
                font-weight: 700;
                letter-spacing: 2px;
            }
            QLabel#StatusLabel {
                color: #d4def3;
                font-size: 14px;
                font-weight: 600;
            }
            QLabel#SubPanelTitle {
                color: #e1eafc;
                font-size: 14px;
                font-weight: 700;
            }
            QLabel#BoardLabel {
                border-radius: 20px;
                border: 2px solid #8d9ec0;
                background-color: #f3ead5;
                color: #31405f;
                font-size: 30px;
                font-weight: 700;
            }
            QLabel#BoardLabel[empty="true"] {
                border-style: dashed;
                border-color: #7f8fb0;
                background: qradialgradient(cx:0.5, cy:0.4, radius:0.8,
                    fx:0.5, fy:0.45, stop:0 #2b3959, stop:1 #1b243a);
                color: #d5e0f4;
            }
            QListWidget#WheelList {
                background-color: rgba(255, 255, 255, 0.03);
                border: 1px solid rgba(196, 214, 248, 0.22);
                border-radius: 16px;
                padding: 12px 8px;
                outline: none;
            }
            QListWidget#WheelList::item {
                margin: 3px 8px;
                padding: 8px 10px;
                border-radius: 11px;
            }
            QTextEdit#StepReviewBox {
                background-color: rgba(22, 32, 53, 0.96);
                border: 1px solid rgba(186, 208, 246, 0.25);
                border-radius: 14px;
                color: #edf2ff;
                padding: 10px;
                font-size: 13px;
                selection-background-color: #4569a0;
            }
            QPushButton {
                min-height: 40px;
                border-radius: 10px;
                border: none;
                background-color: #4a5f86;
                color: #ecf2ff;
                font-weight: 600;
            }
            QPushButton:hover { background-color: #5872a5; }
            QPushButton:pressed { background-color: #43597f; }
            QPushButton:disabled {
                background-color: #344667;
                color: rgba(236, 242, 255, 0.45);
            }
            QPushButton#PlayButton {
                background-color: #2f6f96;
            }
            QPushButton#PlayButton:hover {
                background-color: #3e84ad;
            }
            QPushButton#StepAiButton {
                background-color: #5d4f96;
            }
            QPushButton#StepAiButton:hover {
                background-color: #7160b0;
            }
            QPushButton#AiButton {
                background-color: #3da54b;
                color: white;
            }
            QPushButton#AiButton:hover {
                background-color: #47b556;
            }
            QPushButton#BackButton {
                background-color: rgba(161, 180, 214, 0.20);
            }
        """)

        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(26, 24, 26, 24)
        main_layout.setSpacing(20)

        left_card = QFrame()
        left_card.setObjectName("SideCard")
        left_card.setFixedWidth(280)
        left_layout = QVBoxLayout(left_card)
        left_layout.setContentsMargins(16, 16, 16, 16)
        left_layout.setSpacing(12)
        left_title = QLabel("历史对局")
        left_title.setObjectName("PanelTitle")
        left_layout.addWidget(left_title)
        self.file_list = WheelListWidget(item_height=48)
        self.file_list.setObjectName("WheelList")
        self.file_list.itemClicked.connect(self.load_selected_pgn)
        left_layout.addWidget(self.file_list, stretch=1)
        refresh_btn = QPushButton("刷新列表")
        refresh_btn.clicked.connect(self.refresh_file_list)
        left_layout.addWidget(refresh_btn)
        main_layout.addWidget(left_card)

        center_container = QWidget()
        center_layout = QVBoxLayout(center_container)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(12)
        center_layout.addStretch()
        self.title_label = QLabel("历史复盘棋盘")
        self.title_label.setObjectName("BoardTitle")
        self.title_label.setAlignment(Qt.AlignCenter)
        center_layout.addWidget(self.title_label, alignment=Qt.AlignCenter)

        self.step_review_title = QLabel("AI 逐步复盘（第6步开始）")
        self.step_review_title.setObjectName("SubPanelTitle")
        self.step_review_title.setAlignment(Qt.AlignCenter)
        center_layout.addWidget(self.step_review_title, alignment=Qt.AlignCenter)

        self.step_review_box = QTextEdit()
        self.step_review_box.setObjectName("StepReviewBox")
        self.step_review_box.setReadOnly(True)
        self.step_review_box.setFixedSize(640, 130)
        center_layout.addWidget(self.step_review_box, alignment=Qt.AlignCenter)

        self.step_ai_btn = QPushButton("🧠 重新逐步点评")
        self.step_ai_btn.setObjectName("StepAiButton")
        self.step_ai_btn.setEnabled(False)
        self.step_ai_btn.setFixedWidth(220)
        self.step_ai_btn.clicked.connect(lambda: self.request_step_reviews(force_refresh=True))
        center_layout.addWidget(self.step_ai_btn, alignment=Qt.AlignCenter)

        self.board_label = QLabel("请选择历史对局")
        self.board_label.setObjectName("BoardLabel")
        self.board_label.setAlignment(Qt.AlignCenter)
        self.board_label.setProperty("empty", True)
        self.board_label.setFixedSize(640, 640)
        center_layout.addWidget(self.board_label, alignment=Qt.AlignCenter)
        center_layout.addStretch()
        main_layout.addWidget(center_container, stretch=1)

        right_card = QFrame()
        right_card.setObjectName("SideCard")
        right_card.setFixedWidth(280)
        right_layout = QVBoxLayout(right_card)
        right_layout.setContentsMargins(16, 16, 16, 16)
        right_layout.setSpacing(10)
        right_title = QLabel("招法详情")
        right_title.setObjectName("PanelTitle")
        right_layout.addWidget(right_title)
        self.move_list = WheelListWidget(item_height=44)
        self.move_list.setObjectName("WheelList")
        self.move_list.itemClicked.connect(self.on_move_list_clicked)
        right_layout.addWidget(self.move_list, stretch=1)
        self.step_label = QLabel("步骤: 0 / 0")
        self.step_label.setObjectName("StatusLabel")
        self.step_label.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.step_label)
        self.path_label = QLabel("走向: -")
        self.path_label.setObjectName("StatusLabel")
        self.path_label.setAlignment(Qt.AlignCenter)
        self.path_label.setWordWrap(True)
        right_layout.addWidget(self.path_label)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)
        self.prev_btn = QPushButton("上一步")
        self.next_btn = QPushButton("下一步")
        self.prev_btn.clicked.connect(self.show_prev)
        self.next_btn.clicked.connect(self.show_next)
        btn_layout.addWidget(self.prev_btn)
        btn_layout.addWidget(self.next_btn)
        right_layout.addLayout(btn_layout)

        self.play_btn = QPushButton("▶ 播放整局")
        self.play_btn.setObjectName("PlayButton")
        self.play_btn.setEnabled(False)
        self.play_btn.clicked.connect(self.toggle_playback)
        right_layout.addWidget(self.play_btn)

        self.ai_btn = QPushButton("🔍 分析整局")
        self.ai_btn.setObjectName("AiButton")
        self.ai_btn.setEnabled(False)
        self.ai_btn.clicked.connect(self.handle_ai_analysis)
        right_layout.addWidget(self.ai_btn)

        right_layout.addStretch()
        back_btn = QPushButton("返回主菜单")
        back_btn.setObjectName("BackButton")
        back_btn.clicked.connect(self.back_signal.emit)
        right_layout.addWidget(back_btn)
        main_layout.addWidget(right_card)

        self.reset_game_state("请选择历史对局")

    # ---------------- 业务逻辑 ----------------

    def reset_game_state(self, message="请选择历史对局"):
        self.stop_playback()
        self.step_fens.clear()
        self.step_moves.clear()
        self.step_move_colors.clear()
        self.full_move_list.clear()
        self.step_reviews = []
        self.step_review_loading = False
        self.step_review_request_file = ""
        self.current_filename = ""
        self.current_step = 0
        self.move_list.set_text_items([])
        self.set_board_empty_state(message)
        self.step_label.setText("步骤: 0 / 0")
        self.path_label.setText("走向: -")
        self.step_ai_btn.setEnabled(False)
        self.ai_btn.setEnabled(False)
        self.play_btn.setEnabled(False)
        self.step_review_box.setMarkdown("请选择历史对局后开始逐步点评。")
        self.update_step_buttons()

    def set_board_empty_state(self, text):
        self.board_label.setPixmap(QPixmap())
        self.board_label.setText(text)
        self.board_label.setProperty("empty", True)
        self.board_label.style().unpolish(self.board_label)
        self.board_label.style().polish(self.board_label)

    def set_board_content_state(self):
        self.board_label.setText("")
        self.board_label.setProperty("empty", False)
        self.board_label.style().unpolish(self.board_label)
        self.board_label.style().polish(self.board_label)

    def update_step_buttons(self):
        if self.play_timer.isActive():
            self.prev_btn.setEnabled(False)
            self.next_btn.setEnabled(False)
            return
        total_steps = len(self.step_fens) - 1
        has_game = total_steps >= 0 and len(self.step_fens) > 0
        self.prev_btn.setEnabled(has_game and self.current_step > 0)
        self.next_btn.setEnabled(has_game and self.current_step < total_steps)

    def step_review_cache_path(self):
        if not self.current_filename:
            return ""
        return os.path.join(
            self.games_dir,
            f"{os.path.splitext(self.current_filename)[0]}_step_review.json"
        )

    def request_step_reviews(self, force_refresh=False):
        if len(self.full_move_list) < 6:
            self.step_review_box.setMarkdown("对局步数不足 6 步，跳过逐步点评。")
            return
        if self.step_review_loading and self.step_review_request_file == self.current_filename:
            return

        cache_path = self.step_review_cache_path()
        if (not force_refresh) and cache_path and os.path.exists(cache_path):
            try:
                with open(cache_path, "r", encoding="utf-8") as f:
                    payload = json.load(f)
                self.step_reviews = self.normalize_step_reviews(payload)
                self.update_step_review_panel()
                return
            except Exception:
                pass

        self.step_review_loading = True
        self.step_ai_btn.setEnabled(False)
        self.step_review_box.setMarkdown("AI 正在逐步复盘，请稍候...")
        request_file = self.current_filename
        self.step_review_request_file = request_file
        self.step_review_worker = StepReviewThread(self.client, self.full_move_list)
        self.step_review_worker.finished.connect(
            lambda payload: self.on_step_reviews_finished(payload, cache_path, request_file)
        )
        self.step_review_worker.error.connect(lambda err: self.on_step_reviews_error(err, request_file))
        self.step_review_worker.start()

    def on_step_reviews_finished(self, payload, cache_path, request_file):
        if request_file != self.current_filename:
            if self.step_review_request_file == request_file:
                self.step_review_loading = False
                self.step_review_request_file = ""
            return
        self.step_review_loading = False
        self.step_review_request_file = ""
        self.step_reviews = self.normalize_step_reviews(payload)
        self.step_ai_btn.setEnabled(len(self.full_move_list) >= 6)
        self.update_step_review_panel()
        if not cache_path:
            return
        try:
            with open(cache_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def on_step_reviews_error(self, err, request_file):
        if request_file != self.current_filename:
            if self.step_review_request_file == request_file:
                self.step_review_loading = False
                self.step_review_request_file = ""
            return
        self.step_review_loading = False
        self.step_review_request_file = ""
        self.step_ai_btn.setEnabled(len(self.full_move_list) >= 6)
        self.step_review_box.setMarkdown(f"逐步点评失败：{err}")

    def normalize_step_reviews(self, payload):
        reviews = [None] * len(self.step_fens)
        if not isinstance(payload, dict):
            return reviews
        raw_reviews = payload.get("reviews", [])
        if not isinstance(raw_reviews, list):
            return reviews
        verdict_map = {
            "good": "好", "ok": "可行", "bad": "不好",
            "好": "好", "可行": "可行", "不好": "不好"
        }
        for item in raw_reviews:
            if not isinstance(item, dict):
                continue
            step = item.get("step")
            try:
                step = int(step)
            except Exception:
                continue
            if step < 6 or step >= len(self.step_fens):
                continue
            verdict = str(item.get("verdict", "")).strip()
            verdict = verdict_map.get(verdict.lower(), verdict_map.get(verdict, "可行"))
            reason = str(item.get("reason", "")).strip()
            suggestion = str(item.get("suggestion", "")).strip()
            if verdict == "好":
                suggestion = ""
            reviews[step] = {
                "verdict": verdict,
                "reason": reason if reason else "未提供原因",
                "suggestion": suggestion
            }
        return reviews

    def update_step_review_panel(self):
        if not self.step_fens:
            self.step_review_box.setMarkdown("请选择历史对局后开始逐步点评。")
            return
        if self.current_step == 0:
            self.step_review_box.setMarkdown("当前是起始局面，暂无点评。")
            return
        if self.current_step <= 5:
            self.step_review_box.setMarkdown(f"第 {self.current_step} 步属于前五步，按规则不显示点评。")
            return
        if self.step_review_loading:
            self.step_review_box.setMarkdown("AI 正在逐步复盘，请稍候...")
            return

        review = None
        if self.current_step < len(self.step_reviews):
            review = self.step_reviews[self.current_step]
        if not review:
            self.step_review_box.setMarkdown("该步暂无 AI 点评。可点击“重新逐步点评”重试。")
            return

        verdict = review.get("verdict", "可行")
        reason = review.get("reason", "未提供原因")
        suggestion = review.get("suggestion", "")
        verdict_style = {
            "好": ("✅", "▶"),
            "可行": ("⏸", "⏸"),
            "不好": ("⚠️", "⏪")
        }
        verdict_icon, pace_icon = verdict_style.get(verdict, ("⏸", "⏸"))
        text = (
            f"第 {self.current_step} 步：{verdict_icon} {verdict}\n\n"
            f"{pace_icon} 节奏：{'主动推进' if verdict == '好' else '可继续观察' if verdict == '可行' else '节奏受损'}\n"
            f"🔍 原因：{reason}\n"
        )
        if verdict != "好":
            if suggestion:
                text += f"\n💡 AI 建议：{suggestion}"
            else:
                text += "\n💡 AI 建议：建议优化子力协同与先手节奏。"
        else:
            text += "\n🎯 AI 提示：保持当前思路，持续压制对手空间。"
        self.step_review_box.setPlainText(text)

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
        self.stop_playback()
        self.current_filename = item.text()
        file_path = os.path.join(self.games_dir, self.current_filename)
        try:
            board = cchess.Board.from_pgn(file_path)
            moves = list(board.move_stack)
            self.step_fens.clear()
            self.step_moves.clear()
            self.step_move_colors.clear()
            self.full_move_list.clear()
            move_names = ["起始位置"]
            temp = cchess.Board(fen=board._starting_fen)
            self.step_fens.append(temp.fen())
            self.step_moves.append(None)
            self.step_move_colors.append(None)
            for i, mv in enumerate(moves):
                m_notat = temp.move_to_notation(mv)
                self.full_move_list.append(m_notat)
                self.step_moves.append(mv)
                self.step_move_colors.append(temp.turn)
                prefix = f"{i//2+1}." if i%2==0 else "   "
                move_names.append(f"{prefix} {m_notat}")
                temp.push(mv)
                self.step_fens.append(temp.fen())
            self.move_list.set_text_items(move_names)
            self.current_step = 0
            self.step_reviews = [None] * len(self.step_fens)
            self.select_current_step()
            self.update_display()
            self.ai_btn.setEnabled(True)
            self.play_btn.setEnabled(len(self.step_fens) > 1)
            self.step_ai_btn.setEnabled(len(self.full_move_list) >= 6)
            self.request_step_reviews(force_refresh=False)
        except Exception as e:
            self.reset_game_state("棋谱加载失败，请重新选择")
            print(f"Error: {e}")

    def update_display(self):
        if not self.step_fens:
            self.set_board_empty_state("请选择历史对局")
            self.step_label.setText("步骤: 0 / 0")
            self.path_label.setText("走向: -")
            self.update_step_buttons()
            self.update_step_review_panel()
            return

        fen = self.step_fens[self.current_step]
        board = cchess.Board(fen)
        last_move = self.step_moves[self.current_step] if self.current_step > 0 else None
        move_color = self.step_move_colors[self.current_step] if self.current_step > 0 else None
        svg = cchess.svg.board(
            board,
            size=1200,
            coordinates=True,
            axes_type=1,
            lastmove=last_move
        )
        png_bytes = cairosvg.svg2png(bytestring=svg.encode("utf-8"))
        pixmap = QPixmap()
        pixmap.loadFromData(png_bytes)
        pixmap = pixmap.scaled(self.board_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        if last_move is not None:
            pixmap = self.draw_move_arrow(pixmap, last_move, move_color)
        self.set_board_content_state()
        self.board_label.setPixmap(pixmap)
        self.step_label.setText(f"步骤: {self.current_step} / {len(self.step_fens)-1}")
        self.path_label.setText(f"走向: {self.format_move_direction(last_move, move_color)}")
        self.update_step_review_panel()
        self.update_step_buttons()
        self.move_list.update_wheel_effect()

    def refresh_file_list(self):
        os.makedirs(self.games_dir, exist_ok=True)
        files = sorted(
            [
                f for f in os.listdir(self.games_dir)
                if os.path.isfile(os.path.join(self.games_dir, f))
                and f.lower().endswith((".pgn", ".txt"))
            ],
            reverse=True
        )
        self.file_list.set_text_items(files)
        if self.file_list.count() > 0:
            self.file_list.setCurrentRow(0)
        if not self.step_fens:
            if files:
                self.set_board_empty_state("请选择历史对局")
            else:
                self.set_board_empty_state("暂无历史棋谱")

    def on_move_list_clicked(self, item):
        self.stop_playback()
        self.current_step = self.move_list.row(item)
        self.select_current_step()
        self.update_display()

    def show_prev(self):
        self.stop_playback()
        if self.current_step > 0:
            self.current_step -= 1
            self.select_current_step()
            self.update_display()

    def show_next(self):
        self.stop_playback()
        if self.current_step < len(self.step_fens)-1:
            self.current_step += 1
            self.select_current_step()
            self.update_display()

    def toggle_playback(self):
        if not self.step_fens or len(self.step_fens) <= 1:
            return

        if self.play_timer.isActive():
            self.stop_playback()
            return

        self.current_step = 0
        self.select_current_step()
        self.update_display()
        self.play_timer.start()
        self.play_btn.setText("⏸ 暂停播放")
        self.update_step_buttons()

    def play_next_step(self):
        if self.current_step >= len(self.step_fens) - 1:
            self.stop_playback()
            return

        self.current_step += 1
        self.select_current_step()
        self.update_display()

    def stop_playback(self):
        if self.play_timer.isActive():
            self.play_timer.stop()
        self.play_btn.setText("▶ 播放整局")
        self.update_step_buttons()

    def select_current_step(self):
        if self.move_list.count() <= self.current_step:
            return
        self.move_list.blockSignals(True)
        self.move_list.setCurrentRow(self.current_step)
        self.move_list.blockSignals(False)
        self.move_list.center_current_row(self.current_step)
        self.move_list.update_wheel_effect()

    def square_center_in_pixmap(self, square, pixmap):
        col_index = cchess.square_column(square)
        row_index = cchess.square_row(square)
        x_board = col_index * 100 - 400
        y_board = (9 - row_index) * 100 - 450
        x = (x_board + 600) * pixmap.width() / 1200.0
        y = (y_board + 600) * pixmap.height() / 1200.0
        return QPointF(x, y)

    def draw_move_arrow(self, pixmap, move, move_color):
        from_p = self.square_center_in_pixmap(move.from_square, pixmap)
        to_p = self.square_center_in_pixmap(move.to_square, pixmap)

        output = QPixmap(pixmap)
        painter = QPainter(output)
        painter.setRenderHint(QPainter.Antialiasing, True)
        arrow_color = QColor("#d32f2f") if move_color == cchess.RED else QColor("#212121")
        pen = QPen(arrow_color, 6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        painter.setPen(pen)
        painter.drawLine(from_p, to_p)

        dx = to_p.x() - from_p.x()
        dy = to_p.y() - from_p.y()
        length = math.hypot(dx, dy)
        if length > 1e-6:
            ux = dx / length
            uy = dy / length
            head_len = 18.0
            head_width = 12.0
            left = QPointF(
                to_p.x() - ux * head_len - uy * head_width / 2.0,
                to_p.y() - uy * head_len + ux * head_width / 2.0
            )
            right = QPointF(
                to_p.x() - ux * head_len + uy * head_width / 2.0,
                to_p.y() - uy * head_len - ux * head_width / 2.0
            )
            painter.setBrush(arrow_color)
            painter.drawPolygon(QPolygonF([to_p, left, right]))

        painter.end()
        return output

    def square_to_cn_coord(self, square):
        columns = "abcdefghi"
        col_index = cchess.square_column(square)
        row_index = cchess.square_row(square)
        if 0 <= col_index < len(columns):
            return f"{columns[col_index]}{row_index}"
        return str(square)

    def format_move_direction(self, move, move_color):
        if move is None:
            return "起始位置"
        notation = self.full_move_list[self.current_step - 1] if self.current_step > 0 else ""
        from_pos = self.square_to_cn_coord(move.from_square)
        to_pos = self.square_to_cn_coord(move.to_square)
        mover = "红方" if move_color == cchess.RED else "黑方"
        return f"{mover} {notation}: {from_pos} -> {to_pos}"
