# qt_chess_p/pages/game_page.py
import time
import json
import os
import re
import cchess
import cchess.svg
import cairosvg
import pyaudio
import dashscope
import difflib
from dashscope.audio.asr import Recognition, RecognitionCallback, RecognitionResult
from dashscope.common.error import InvalidParameter

from PySide6.QtCore import (
    Qt, Signal, Slot, QTimer, QEvent, QThread,
    QPropertyAnimation, QEasingCurve, QSize, QByteArray
)
from PySide6.QtGui import (
    QFont, QPixmap, QImage, QPainter, QColor
)
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTextEdit, QSizePolicy, QDialog, QLineEdit, QMessageBox,
    QProgressBar, QListWidget, QFrame, QGroupBox
)
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtSvgWidgets import QSvgWidget
# 填入你的阿里云 DashScope API Key
dashscope.api_key = "sk-4298611ab58b48f9a5deef4ba7d7ea2e"
# ================= 语音识别与大模型解析线程 =================
class VoiceRecognitionThread(QThread):
    recognized_move_signal = Signal(str)
    partial_result_signal = Signal(str)
    status_signal = Signal(str)
    finished_signal = Signal()

    def __init__(self, api_key=None):
        super().__init__()
        self._is_running = False
        self.current_fen = ""
        self.full_transcript = "" # 新增：用于记录本轮录音的所有文字
        self._fatal_asr_error = None

    @staticmethod
    def _normalize_text(value):
        return str(value or "").strip()

    def _classify_asr_error(self, status_code, code, message):
        """返回 (is_fatal, user_message)。fatal 表示不可重试，应停止自动重连。"""
        status = self._normalize_text(status_code)
        code_text = self._normalize_text(code)
        msg_text = self._normalize_text(message)
        merged = f"{code_text} {msg_text}".lower()

        # 不可恢复：鉴权/权限/额度/限流等
        fatal_status = {"401", "402", "403", "429"}
        fatal_keywords = [
            "quota", "token", "balance", "billing", "bill", "arrear", "insufficient",
            "exceed", "exhaust", "forbidden", "unauthorized", "permission denied",
            "欠费", "余额", "额度", "配额", "鉴权", "权限", "超限", "耗尽"
        ]
        is_fatal = status in fatal_status or any(k in merged for k in fatal_keywords)

        details = f"(status={status or 'unknown'}, code={code_text or 'unknown'})"
        if is_fatal:
            return True, f"语音识别不可用 {details}: {msg_text or '可能是 Token/余额/权限问题，请检查 DashScope 控制台。'}"
        return False, f"语音会话异常 {details}: {msg_text or '网络或服务波动，正在自动重连...'}"

    def run(self):
        self._is_running = True
        self.full_transcript = ""
        self._fatal_asr_error = None
        
        class ASRCallback(RecognitionCallback):
            def __init__(self, outer):
                self.outer = outer
            def on_event(self, result: RecognitionResult):
                sentence = result.get_sentence()
                if sentence:
                    text = sentence.get('text', '')
                    if text:
                        # 实时更新 UI 预览
                        self.outer.partial_result_signal.emit(text)
                        # 始终记录最新的识别内容
                        self.outer.full_transcript = text

            def on_error(self, result: RecognitionResult):
                status_code = getattr(result, "status_code", None)
                code = getattr(result, "code", "")
                message = getattr(result, "message", "")
                is_fatal, user_msg = self.outer._classify_asr_error(status_code, code, message)
                self.outer.status_signal.emit(user_msg)
                if is_fatal:
                    self.outer._fatal_asr_error = user_msg
                    self.outer._is_running = False

        callback = ASRCallback(self)
        recognition = None
        p = None
        stream = None
        reconnecting = False
        last_reconnect_ts = 0.0

        def start_recognition_session():
            rec = Recognition(
                model='paraformer-realtime-v1',
                format='pcm',
                sample_rate=16000,
                callback=callback
            )
            rec.start()
            return rec

        try:
            recognition = start_recognition_session()

            p = pyaudio.PyAudio()
            stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=3200)

            while self._is_running:
                if self._fatal_asr_error:
                    break

                data = stream.read(3200, exception_on_overflow=False)
                if not data:
                    continue

                if recognition is None:
                    if self._fatal_asr_error:
                        break
                    now = time.time()
                    if now - last_reconnect_ts < 1.0:
                        continue
                    last_reconnect_ts = now
                    try:
                        recognition = start_recognition_session()
                        if reconnecting:
                            self.status_signal.emit("语音会话已恢复")
                            reconnecting = False
                    except Exception:
                        if not reconnecting:
                            self.status_signal.emit("语音会话中断，正在重连...")
                            reconnecting = True
                        continue

                try:
                    recognition.send_audio_frame(data)
                except InvalidParameter:
                    # ASR 会话已停止时不要结束录音线程，尝试重连会话
                    if self._fatal_asr_error:
                        break
                    try:
                        recognition.stop()
                    except Exception:
                        pass
                    recognition = None
                    if not reconnecting:
                        self.status_signal.emit("语音会话中断，正在重连...")
                        reconnecting = True
                    continue
        except Exception as e:
            print(f"ASR Thread Error: {e}")
            if not self._fatal_asr_error:
                self.status_signal.emit("语音识别过程发生异常")
        finally:
            if recognition is not None:
                try:
                    recognition.stop()
                except InvalidParameter:
                    pass
                except Exception as e:
                    print(f"ASR Stop Error: {e}")

            if stream is not None:
                try:
                    stream.stop_stream()
                    stream.close()
                except Exception:
                    pass

            if p is not None:
                try:
                    p.terminate()
                except Exception:
                    pass

            # 线程停止后，如果刚才录到了话，立即交给 LLM 处理
            if self.full_transcript:
                self.process_text_with_llm(self.full_transcript)
            else:
                self.finished_signal.emit()

    def process_text_with_llm(self, natural_text):
        try:
            # 1. 【核心修正】在发送给 AI 之前，手动反转用户口中的“左右”
            # 因为 AI 的左右逻辑和象棋红方视角是相反的
            corrected_text = natural_text.replace("左", "临时").replace("右", "左").replace("临时", "右")
            print(f"DEBUG: 原始语音: {natural_text} -> 修正后发送给AI: {corrected_text}")

            board = cchess.Board(self.current_fen)
            legal_moves_notations = [board.move_to_notation(m) for m in board.legal_moves]

            # 2. 构造 Prompt（此时不再强求 AI 理解镜像，顺着它的逻辑来）
            prompt = (
                f"你是一个专业的中国象棋语音助手，现在为【红方】服务。\n\n"
                f"【棋盘规则】:\n"
                f"- 红方在下方，黑方在上方。向上为“进”，向下为“退”。\n"
                f"- 棋盘路数从右向左依次为：一、二、三、四、五、六、七、八、九。\n\n"
                f"【当前红方合法走法列表】: {', '.join(legal_moves_notations)}\n"
                f"【玩家语音指令】: \"{corrected_text}\"\n\n"
                f"任务：从合法走法列表中选出最符合意图的一个，只输出四个汉字，不要解释。"
            )

            # 4. 请求大模型
            response = dashscope.Generation.call(
                model='qwen-turbo',
                prompt=prompt
            )
            
            if response.status_code == 200:
                llm_output = response.output.text.strip()
                print(f"DEBUG: LLM 原始输出: {llm_output}")

                # 如果大模型判定为无关语音，直接丢弃
                if "未知" in llm_output:
                    self.status_signal.emit("未识别出有效的下棋指令")
                    return

                # 5. 字符清洗与繁简转换（针对红方）
                # 过滤掉非中文字符
                res_notation = re.sub(r'[^\u4e00-\u9fa5]', '', llm_output)
                
                # 核心处理：将常见的口语/简体字强制转换为 cchess 库识别的红方汉字
                char_map = {
                    '马': '傌', '馬': '傌',
                    '车': '俥', '車': '俥',
                    '象': '相',
                    '士': '仕',
                    '将': '帥', '帅': '帥',
                    '卒': '兵',
                    '砲': '炮'
                }
                for k, v in char_map.items():
                    res_notation = res_notation.replace(k, v)

                # 如果大模型话多，只截取最后4个字
                if len(res_notation) > 4: 
                    res_notation = res_notation[-4:]

                # 6. 匹配验证（精确匹配 + 模糊匹配）
                final_notation = None
                
                # 情况A：完全精准匹配到了合法走法
                if res_notation in legal_moves_notations:
                    final_notation = res_notation
                else:
                    # 情况B：差一两个字（如输出“傌二进”，漏了字），使用 difflib 进行相似度匹配
                    # cutoff=0.4 表示相似度达到 40% 即可候选
                    matches = difflib.get_close_matches(res_notation, legal_moves_notations, n=1, cutoff=0.4)
                    if matches:
                        final_notation = matches[0]
                        print(f"DEBUG: 模糊匹配修正 -> '{res_notation}' 修正为 '{final_notation}'")

                # 7. 最终执行
                if final_notation:
                    move = board.parse_notation(final_notation)
                    uci = move.uci()
                    self.status_signal.emit(f"识别成功: {final_notation}")
                    self.recognized_move_signal.emit(uci)
                else:
                    self.status_signal.emit(f"无法匹配该走法: {res_notation}")
            else:
                self.status_signal.emit(f"API请求失败: {response.code}")

        except Exception as e:
            print(f"LLM Process Error: {e}")
            self.status_signal.emit("解析过程发生异常")
        finally:
            self._is_running = False
            self.finished_signal.emit() # 恢复按钮状态
    def stop(self):
        self._is_running = False


class ChessVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.player_orientation = cchess.RED  # 默认红方在下
        self._current_fen = None
        self._base_pixmap = QPixmap()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.title_label = QLabel("当前局势")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
        self.title_label.setFixedHeight(65)
        self.title_label.setStyleSheet("""
            QLabel {
                color: #1E3A8A;
                background-color: #87CEEB;
                padding: 5px;
                border-radius: 8px;
                border: 2px solid #1E3A8A;
            }
        """)
        layout.addWidget(self.title_label, alignment=Qt.AlignHCenter)

        self.board_label = QLabel()
        self.board_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.board_label, stretch=1)

    def set_orientation(self, color_str):
        """设置棋盘视角：RED 或 BLACK"""
        old_orientation = self.player_orientation
        if color_str.upper() == "BLACK":
            self.player_orientation = cchess.BLACK
        else:
            self.player_orientation = cchess.RED

        # 视角变化时需要重建原始棋盘图
        if self._current_fen and self.player_orientation != old_orientation:
            self._rebuild_base_pixmap()
            self._apply_scaled_pixmap()

    def update_board(self, fen_str):
        """更新棋盘数据，并按当前可用空间显示。"""
        self._current_fen = fen_str
        self._rebuild_base_pixmap()
        self._apply_scaled_pixmap()

    def showEvent(self, event):
        super().showEvent(event)
        # 首次显示时延迟一帧重绘，确保布局尺寸稳定。
        QTimer.singleShot(0, self._apply_scaled_pixmap)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._apply_scaled_pixmap()

    def _rebuild_base_pixmap(self):
        """根据 FEN 渲染高分辨率棋盘原图，不依赖当前控件尺寸。"""
        if not self._current_fen:
            return
        try:
            board = cchess.Board(self._current_fen)

            # 尝试获取上一步（用于高亮）
            last_move = None
            try:
                last_move = board.peek()
            except Exception:
                pass

            svg_content = cchess.svg.board(
                board=board,
                size=1200,  # 用大尺寸避免细线丢失
                coordinates=True,
                axes_type=1,
                lastmove=last_move,
                checkers=board.checkers(),
                orientation=self.player_orientation,
                style="#board{fill:#f3e5ab; stroke:#5d4037}"
            )

            png_bytes = cairosvg.svg2png(bytestring=svg_content.encode("utf-8"))
            pixmap = QPixmap()
            if not pixmap.loadFromData(png_bytes):
                raise ValueError("QPixmap.loadFromData 失败")
            self._base_pixmap = pixmap
            self.board_label.clear()
        except Exception as e:
            print(f"棋盘渲染错误: {e}")
            self._base_pixmap = QPixmap()
            self.board_label.setText("棋盘渲染失败")

    def _apply_scaled_pixmap(self):
        """按 board_label 当前尺寸缩放并显示棋盘。"""
        if self._base_pixmap.isNull():
            return

        target_size = self.board_label.size()
        if target_size.width() <= 1 or target_size.height() <= 1:
            return

        scaled_pixmap = self._base_pixmap.scaled(
            target_size,
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.board_label.setPixmap(scaled_pixmap)


# ================= 自定义消息框 =================
class CustomMessageBox(QDialog):
    def __init__(self, parent=None, title="提示", message="", color="#f39c12", timeout=2000):
        super().__init__(parent)
        self.timeout = timeout
        self.setup_ui(title, message, color)
        self.setup_animation()
        
    def setup_ui(self, title, message, color):
        self.setWindowTitle(title)
        self.setModal(False)  # 非模态，不阻塞其他窗口
        self.setFixedSize(600, 300)  # 固定大小
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 30, 30, 30)
        
        # 消息标签
        self.message_label = QLabel(message)
        self.message_label.setAlignment(Qt.AlignCenter)
        self.message_label.setFont(QFont("Microsoft YaHei", 20, QFont.Bold))
        self.message_label.setWordWrap(True)
        self.message_label.setStyleSheet(f"""
            QLabel {{
                color: white;
                background-color: {color};
                padding: 30px;
                border-radius: 15px;
                font-size: 20px;
            }}
        """)
        
        layout.addWidget(self.message_label)
        
    def setup_animation(self):
        # 设置窗口标志：无边框、置顶
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        
        # 淡入动画
        self.fade_in_animation = QPropertyAnimation(self, b"windowOpacity")
        self.fade_in_animation.setDuration(300)
        self.fade_in_animation.setStartValue(0)
        self.fade_in_animation.setEndValue(1)
        self.fade_in_animation.setEasingCurve(QEasingCurve.OutCubic)
        
        # 淡出动画
        self.fade_out_animation = QPropertyAnimation(self, b"windowOpacity")
        self.fade_out_animation.setDuration(300)
        self.fade_out_animation.setStartValue(1)
        self.fade_out_animation.setEndValue(0)
        self.fade_out_animation.setEasingCurve(QEasingCurve.InCubic)
        self.fade_out_animation.finished.connect(self.close)
        
    def showEvent(self, event):
        super().showEvent(event)
        # 居中显示
        if self.parent():
            parent_rect = self.parent().frameGeometry()
            self.move(parent_rect.center() - self.rect().center())
        
        # 启动淡入动画
        self.fade_in_animation.start()
        
        # 设置定时器自动关闭
        QTimer.singleShot(self.timeout, self.start_fade_out)
        
    def start_fade_out(self):
        self.fade_out_animation.start()


# ================= 等待对话框 =================
class WaitDialog(QDialog):
    """一个模态对话框，用于在关闭节点时等待几秒钟。"""
    def __init__(self, message_template="正在关闭ROS节点，请等待 {seconds} 秒...", wait_seconds=6, parent=None):
        super().__init__(parent)
        self.wait_seconds = wait_seconds
        self.remaining_seconds = wait_seconds
        self.message_template = message_template

        self.setWindowTitle("请稍候...")
        self.setModal(True)
        self.setFixedSize(500, 250)  # 增大对话框尺寸

        layout = QVBoxLayout(self)
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(25)

        self.message_label = QLabel(self.message_template.format(seconds=self.remaining_seconds))
        self.message_label.setAlignment(Qt.AlignCenter)
        self.message_label.setFont(QFont("Microsoft YaHei", 16))  # 增大字体
        self.message_label.setStyleSheet("color: black;")  # 设置字体颜色为黑色
        self.message_label.setWordWrap(True)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, self.wait_seconds)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFormat("%v 秒")
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 10px;
                text-align: center;
                font-size: 14px;
                height: 25px;
                color: black;  /* 设置进度条文字颜色为黑色 */
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
                border-radius: 8px;
            }
        """)

        layout.addWidget(self.message_label)
        layout.addWidget(self.progress_bar)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_countdown)

    def showEvent(self, event: QEvent):
        """对话框显示时启动计时器"""
        self.timer.start(1000)
        super().showEvent(event)

    def update_countdown(self):
        """每秒更新一次倒计时和进度条"""
        self.remaining_seconds -= 1
        self.progress_bar.setValue(self.wait_seconds - self.remaining_seconds)
        
        if self.remaining_seconds > 0:
            self.message_label.setText(self.message_template.format(seconds=self.remaining_seconds))
        else:
            self.timer.stop()
            self.accept()






# ================= 菜单对话框 =================
class MenuDialog(QDialog):
    end_game_signal = Signal()
    debug_send_signal = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("游戏菜单")
        self.setFixedSize(400, 480)
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(25)
        layout.setContentsMargins(60, 50, 60, 50)

        btn_style = """
            QPushButton {
                font-size: 22px;
                padding: 16px;
                border-radius: 14px;
                font-weight: bold;
                min-height: 60px;
            }
        """

        # 结束游戏按钮
        self.end_game_btn = QPushButton("结束游戏")
        self.end_game_btn.setStyleSheet(btn_style + "background-color: #e74c3c; color: white;")

        # 单按钮控制暂停 / 继续
        self.pause_resume_btn = QPushButton("暂停游戏")
        self.pause_resume_btn.setCheckable(True)
        self.pause_resume_btn.setStyleSheet(btn_style + "background-color: #f39c12; color: white;")

        # 调试命令输入框
        self.debug_input = QLineEdit()
        self.debug_input.setPlaceholderText("请输入 UCI 命令（如 move e2e4）")
        self.debug_input.setStyleSheet("""
            QLineEdit {
                font-size: 20px;
                padding: 12px;
                border: 2px solid #bbb;
                border-radius: 10px;
                background-color: #fff;
            }
        """)

        # 发送调试命令按钮
        self.send_debug_btn = QPushButton("发送调试命令")
        self.send_debug_btn.setStyleSheet(btn_style + "background-color: #3498db; color: white;")

        # 关闭菜单按钮
        self.close_btn = QPushButton("关闭菜单")
        self.close_btn.setStyleSheet(btn_style + "background-color: #7f8c8d; color: white;")

        # 加入布局
        layout.addWidget(self.end_game_btn)
        layout.addWidget(self.pause_resume_btn)
        layout.addWidget(self.debug_input)
        layout.addWidget(self.send_debug_btn)
        layout.addWidget(self.close_btn)

        # 信号连接
        self.end_game_btn.clicked.connect(self.end_game_signal.emit)
        self.send_debug_btn.clicked.connect(self.send_debug_command)
        self.close_btn.clicked.connect(self.close)

        # 暂停/继续逻辑绑定
        self.pause_resume_btn.clicked.connect(self.toggle_pause_resume)

    def toggle_pause_resume(self):
        """暂停 / 继续游戏按钮切换逻辑"""
        if self.pause_resume_btn.isChecked():
            # 切换为暂停状态
            self.pause_resume_btn.setText("继续游戏")
            self.pause_resume_btn.setStyleSheet("""
                QPushButton {
                    font-size: 22px;
                    padding: 16px;
                    border-radius: 14px;
                    font-weight: bold;
                    min-height: 60px;
                    background-color: #27ae60;
                    color: white;
                }
            """)
            self.debug_send_signal.emit("pause")
        else:
            # 切换为继续状态
            self.pause_resume_btn.setText("暂停游戏")
            self.pause_resume_btn.setStyleSheet("""
                QPushButton {
                    font-size: 22px;
                    padding: 16px;
                    border-radius: 14px;
                    font-weight: bold;
                    min-height: 60px;
                    background-color: #f39c12;
                    color: white;
                }
            """)
            self.debug_send_signal.emit("resume")

    def send_debug_command(self):
        """从输入框发送UCI命令"""
        cmd = self.debug_input.text().strip()
        if cmd:
            self.debug_send_signal.emit(cmd)
            self.debug_input.clear()

# ================= 游戏主页面 =================
class GamePage(QWidget):
    back_signal = Signal()
    end_game_request_signal = Signal()

    def __init__(self, ros_node, player_color="RED", game_mode="HUMAN_AI"):
        super().__init__()
        self.ros_node = ros_node
        self.player_color = player_color
        self.game_mode = game_mode
        self.is_voice_mode = (game_mode == "VOICE_AI")
        self.voice_thread = None
        # 1. 核心状态初始化
        self.board = cchess.Board() # 初始棋盘
        self.is_timing = False
        self.start_time = None
        
        # 2. 计时器初始化
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time_display)

        # 3. 语音线程初始化
        if self.is_voice_mode:
            self.voice_thread = VoiceRecognitionThread("")
            self.voice_thread.recognized_move_signal.connect(self.on_voice_move_detected)
            self.voice_thread.partial_result_signal.connect(self.on_voice_partial_update)
            self.voice_thread.status_signal.connect(self.update_voice_status)

        # 4. 构建 UI
        self.setup_ui()
        
        # 5. 连接 ROS 信号
        self.setup_connections()
        
        # 6. 开局立即渲染初始棋盘 (修复识别节点晚的问题)
        self.visualizer.set_orientation(self.player_color)
        self.visualizer.update_board(self.board.fen())
        self.start_timing()
        if self.is_voice_mode and self.voice_thread is not None:
            self.voice_thread.finished_signal.connect(self.reset_record_button)

    def setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)
        
        # 左侧：棋盘 (变量名统一为 self.visualizer)
        self.visualizer = ChessVisualizer()
        main_layout.addWidget(self.visualizer, 3)

        # 右侧：控制面板
        right_panel = QVBoxLayout()
        
        # --- 语音交互区 (大字显示，修复对比度) ---
        if self.is_voice_mode:
            self.voice_group = QGroupBox("语音助手")
            self.voice_group.setStyleSheet("""
                QGroupBox {
                    background-color: #ffffff;
                    border: 3px solid #3498db;
                    border-radius: 15px;
                    margin-top: 20px;
                    font-weight: bold;
                    color: #2c3e50;
                }
            """)
            v_layout = QVBoxLayout(self.voice_group)
            
            # 大字状态标签
            self.voice_status_main = QLabel("等待开启")
            self.voice_status_main.setAlignment(Qt.AlignCenter)
            self.voice_status_main.setFont(QFont("Microsoft YaHei", 32, QFont.Bold))
            self.voice_status_main.setStyleSheet("color: #95a5a6; margin-top: 10px;") # 初始灰色
            
            # 新增：录音控制按钮
            self.record_btn = QPushButton("🎤 开始录音")
            self.record_btn.setMinimumHeight(80)
            self.record_btn.setFont(QFont("Microsoft YaHei", 20, QFont.Bold))
            self.record_btn.setStyleSheet("""
                QPushButton {
                    background-color: #3498db; color: white; border-radius: 15px;
                }
                QPushButton:hover { background-color: #2980b9; }
            """)
            self.record_btn.clicked.connect(self.toggle_voice_recording)

            # 实时识别文字内容
            self.voice_partial_label = QLabel("「 实时语音预览 」")
            self.voice_partial_label.setAlignment(Qt.AlignCenter)
            self.voice_partial_label.setWordWrap(True)
            self.voice_partial_label.setFont(QFont("Microsoft YaHei", 18))
            self.voice_partial_label.setStyleSheet("""
                background-color: #f7f9f9; 
                color: #2980b9; 
                padding: 10px; 
                border-radius: 8px;
                border: 1px dashed #bdc3c7;
            """)
            
            v_layout.addWidget(self.voice_status_main)
            v_layout.addWidget(self.voice_partial_label)
            v_layout.addWidget(self.record_btn)
            right_panel.addWidget(self.voice_group)

        # B. 机机模式：显示“AI自动对弈中”
        elif self.game_mode == "AI_AI":
            self.ai_status_group = QGroupBox("对弈状态")
            self.ai_status_group.setStyleSheet(self.get_group_box_style())
            ai_layout = QVBoxLayout(self.ai_status_group)
            self.ai_status_label = QLabel("🤖 AI 自动对弈中...")
            self.ai_status_label.setAlignment(Qt.AlignCenter)
            self.ai_status_label.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
            self.ai_status_label.setStyleSheet("color: #27ae60; padding: 20px;")
            ai_layout.addWidget(self.ai_status_label)
            right_panel.addWidget(self.ai_status_group)

        # --- 2. 时间显示 (通用) ---
        self.time_label = QLabel("00:00")
        self.time_label.setAlignment(Qt.AlignCenter)
        self.time_label.setFont(QFont("Arial", 45, QFont.Bold))
        self.time_label.setStyleSheet("""
            background-color: #2c3e50; 
            color: #ecf0f1; 
            border-radius: 15px; 
            padding: 10px;
            margin-bottom: 10px;
        """)
        right_panel.addWidget(self.time_label)

        # --- 3. 手动操作区 (仅普通人机模式可见) ---
        if self.game_mode == "HUMAN_AI" and not self.is_voice_mode:
            self.manual_btn = QPushButton("我已移动棋子")
            self.manual_btn.setMinimumHeight(80)
            self.manual_btn.setFont(QFont("Microsoft YaHei", 20, QFont.Bold))
            self.manual_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2ecc71; color: white; border-radius: 12px;
                }
                QPushButton:hover { background-color: #27ae60; }
            """)
            # 按钮2：重新确认当前状态 (新增)
            # self.reconfirm_btn = QPushButton("重新确认状态")
            # self.reconfirm_btn.setFixedHeight(60)
            # self.reconfirm_btn.setFont(QFont("Microsoft YaHei", 16))
            # self.reconfirm_btn.setStyleSheet("background-color: #f39c12; color: white; border-radius: 12px; margin-top: 5px;")
            # 连接到 move_callback 或对应函数
            # self.reconfirm_btn.clicked.connect(self.on_reconfirm_status_clicked)
            self.manual_btn.clicked.connect(self.trigger_move_piece_signal)
            # right_panel.addWidget(self.reconfirm_btn)
            right_panel.addWidget(self.manual_btn)

        # 日志框 (修复白底看不清的问题)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setPlaceholderText("系统日志...")
        self.log_box.setStyleSheet("""
            QTextEdit {
                border: 2px solid #bdc3c7;
                border-radius: 10px;
                background-color: #ffffff;
                color: #2c3e50;
                font-size: 16px;
                padding: 10px;
            }
        """)
        right_panel.addWidget(self.log_box)

        # 菜单按钮
        self.menu_btn = QPushButton("☰ 游戏菜单")
        self.menu_btn.setMinimumHeight(70)
        self.menu_btn.setFont(QFont("Microsoft YaHei", 20, QFont.Bold))
        self.menu_btn.setStyleSheet("""
            QPushButton {
                background-color: #34495e; color: white; border-radius: 12px;
            }
            QPushButton:hover { background-color: #5d6d7e; }
        """)
        self.menu_btn.clicked.connect(self.open_menu)
        right_panel.addWidget(self.menu_btn)

        main_layout.addLayout(right_panel, 2)

    def toggle_voice_recording(self):
        if not hasattr(self, 'voice_thread') or not self.voice_thread.isRunning():
            # 开始录音
            self.voice_thread.current_fen = self.board.fen()
            self.voice_thread.start()
            
            self.record_btn.setText("🛑 停止并识别")
            self.record_btn.setStyleSheet("background-color: #e74c3c; color: white; border-radius: 15px;")
            self.voice_status_main.setText("● 正在听你说...")
            self.voice_status_main.setStyleSheet("color: #e74c3c;")
        else:
            # 停止录音并触发 LLM
            self.voice_thread.stop()
            
            self.record_btn.setText("⌛ 处理中...")
            self.record_btn.setEnabled(False) # 暂时禁用防止重复点击
            self.voice_status_main.setText("正在解析意图...")
            self.voice_status_main.setStyleSheet("color: #f39c12;")
        
    def get_group_box_style(self):
        return """
            QGroupBox {
                background-color: #ffffff;
                border: 2px solid #3498db;
                border-radius: 10px;
                margin-top: 15px;
                font-weight: bold;
                color: #2c3e50;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }
        """
    
    def reset_record_button(self):
        """无论解析成功还是失败，都恢复按钮为可录音状态"""
        self.record_btn.setEnabled(True)
        self.record_btn.setText("🎤 开始录音")
        self.record_btn.setStyleSheet("background-color: #2ecc71; color: white; border-radius: 15px;")
        # 清除中间识别的文字预览，为下次做准备
        self.voice_partial_label.setText("")

    def setup_connections(self):
        # 连接 ROS 信号
        self.ros_node.fen_signal.connect(self.on_fen_received)
        self.ros_node.log_signal.connect(self.append_log)
        self.ros_node.status_signal.connect(self.handle_status_update)
        

    # ---------------- 业务逻辑 ----------------

    @Slot(bool)
    def handle_voice_trigger(self, trigger):
        """当机械臂完成动作，触发玩家开始说话"""
        if trigger:
            if self.voice_thread.isRunning():
                self.get_logger().info("语音线程已在运行，无需重复启动")
                return
            # 启动前重置实时文本显示
            self.voice_partial_label.setText("准备就绪，请说话...")
            self.voice_thread.current_fen = self.board.fen()
            self.voice_thread.start()
            self.start_timing()
            # 更新大字 UI 为录音状态
            self.voice_status_main.setText("● 正在录音")
            self.voice_status_main.setStyleSheet("color: #e74c3c;") # 红色闪烁感
            self.voice_partial_label.setText("请说出您的走法...")

    @Slot(str)
    def on_voice_partial_update(self, text):
        """显示实时识别的文字段落"""
        self.voice_partial_label.setText(f"「 {text} 」")

    @Slot(str)
    def update_voice_status(self, msg):
        """更新小字或错误提示"""
        self.append_log(f"语音系统: {msg}")
        
        # 只要 msg 包含以下关键词，说明一次“意图解析”尝试已经结束，不论成功还是失败
        # 失败的情况需要重置按钮
        error_keywords = [
            "失败", "异常", "不合规", "格式不对", "有误", "无法识别",
            "不可用", "token", "余额", "额度", "权限", "鉴权"
        ]
        
        if any(key in msg for key in error_keywords):
            self.record_btn.setText("🎤 重新录音")
            self.record_btn.setEnabled(True)
            self.record_btn.setStyleSheet("background-color: #3498db; color: white; border-radius: 15px;")
            self.voice_status_main.setText("解析失败")
            self.voice_status_main.setStyleSheet("color: #e67e22;") # 橙色提醒

    @Slot(str)
    def on_voice_move_detected(self, uci):
        """识别成功后的处理"""
        self.record_btn.setText("🎤 开始录音")
        self.record_btn.setEnabled(True)
        self.record_btn.setStyleSheet("background-color: #3498db; color: white; border-radius: 15px;")
        self.voice_status_main.setText("识别成功")
        self.voice_status_main.setStyleSheet("color: #27ae60;")
        self.voice_partial_label.setText(f"已发送指令: {uci}")
        
        self.ros_node.publish_voice_move(uci)
        self.stop_timing()
        
        # 本地预测性更新，增强响应感
        try:
            move = cchess.Move.from_uci(uci)
            self.board.push(move)
            self.visualizer.update_board(self.board.fen())
        except:
            pass

    @Slot(str)
    def on_fen_received(self, fen):
        if fen != self.board.fen():
            self.board = cchess.Board(fen)
            self.visualizer.update_board(fen)
            # 增加这一行，确保语音识别时参考的是最新棋盘
            if self.is_voice_mode and hasattr(self, 'voice_thread'):
                self.voice_thread.current_fen = fen
            self.append_log("棋盘已同步")
            
            # 检查当前是否是红方的回合，如果是，更新UI状态
            if self.is_voice_mode and self.board.turn == cchess.RED:
                self.record_btn.setText("🎤 开始录音")
                self.record_btn.setEnabled(True)
                self.record_btn.setStyleSheet("background-color: #3498db; color: white; border-radius: 15px;")
                self.voice_status_main.setText("轮到你了，请开始录音")
                self.voice_status_main.setStyleSheet("color: #3498db;") # 蓝色
                self.voice_partial_label.setText("准备就绪，请说话...")

    def update_time_display(self):
        if self.is_timing and self.start_time:
            elapsed = int(time.time() - self.start_time)
            self.time_label.setText(f"{elapsed//60:02d}:{elapsed%60:02d}")

    def start_timing(self):
        self.start_time = time.time()
        self.is_timing = True
        self.timer.start(1000)

    def stop_timing(self):
        self.is_timing = False
        self.timer.stop()

    def append_log(self, message):
        self.log_box.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        # 自动滚动到底部
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())

    def open_menu(self):
        from .game_page import MenuDialog # 确保导入
        dialog = MenuDialog()
        dialog.end_game_signal.connect(self.handle_end_game)
        dialog.debug_send_signal.connect(self.handle_debug_send)
        dialog.exec()

    @Slot()
    def handle_end_game(self):
        self.ros_node.publish_uci_command("over")
        self.back_signal.emit()

    @Slot(str)
    def handle_debug_send(self, cmd):
        self.ros_node.publish_uci_command(cmd)
        self.append_log(f"发送指令: {cmd}")

    @Slot(str)
    def handle_status_update(self, status_msg: str):
        self.append_log(f"[状态] {status_msg}")
        if "check_detected" in status_msg:
            self.show_temporary_popup("将军！", color="#e74c3c")
        if status_msg.startswith("over:"):
            self.show_game_over_dialog("游戏结束！" + status_msg.split(":")[-1])

    def show_temporary_popup(self, message, color="#f39c12"):
        from .game_page import CustomMessageBox
        popup = CustomMessageBox(self, "提示", message, color)
        popup.show()

    def start_timing(self):
        self.start_time = time.time()
        self.is_timing = True
        self.timer.start(1000)

    def stop_timing(self):
        self.is_timing = False
        self.timer.stop()

    def append_log(self, message):
        self.log_box.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())

    def closeEvent(self, event):
        if self.is_voice_mode and self.voice_thread.isRunning():
            self.voice_thread.stop()
            self.voice_thread.wait()
        super().closeEvent(event)

    @Slot(str)
    def update_board(self, fen):
        self.chess_visualizer.update_board(fen)

    def trigger_move_piece_signal(self):
        self.ros_node.publish_move_piece_signal(True)
        self.append_log("已发布‘我移动了棋子’信号")
        
    @Slot()
    def on_reconfirm_status_clicked(self):
        """触发相机重新拍照"""
        self.ros_node.publish_image_trigger(True)
        self.append_log("系统提示：已请求相机重新确认当前棋盘状态。")

    def open_menu(self):
        dialog = MenuDialog()
        dialog.end_game_signal.connect(self.handle_end_game)
        dialog.end_game_signal.connect(dialog.accept) 
        dialog.debug_send_signal.connect(self.handle_debug_send)
        dialog.exec()

    @Slot()
    def handle_end_game(self):
        self.stop_timing()
        self.ros_node.publish_uci_command("over")
        self.append_log("已发布结束信号：over")
        self.end_game_request_signal.emit()

    @Slot()
    def show_wait_dialog_and_go_back(self):
        wait_dialog = WaitDialog(
            message_template="正在关闭ROS节点，请等待 {seconds} 秒...",
            wait_seconds=6,
            parent=self
        )
        wait_dialog.exec()
        self.back_signal.emit()

    @Slot(str)
    def handle_debug_send(self, cmd):
        self.ros_node.publish_uci_command(cmd)
        if hasattr(self.ros_node, "publish_rob_uci_command"):
            self.ros_node.publish_rob_uci_command(cmd)

        if cmd.lower() == "pause":
            self.stop_timing()
            self.append_log("游戏已暂停")
        elif cmd.lower() == "resume":
            self.start_timing()
            self.append_log("游戏继续")
        else:
            self.append_log(f"已发送 UCI 指令: {cmd}")

    def show_temporary_popup(self, message, color="#f39c12"):
        """显示一个带淡入淡出动画的临时弹窗"""
        popup = CustomMessageBox(
            parent=self,
            title="提示",
            message=message,
            color=color,
            timeout=2000
        )
        popup.show()

    @Slot(bool)
    def handle_machinery_trigger(self, value):
        if self.game_mode == "HUMAN_AI" and self.player_color == "BLACK" and value and not self.is_timing:
            self.start_timing()
            self.append_log("收到机械臂触发信号，黑方计时开始")

    @Slot(str)
    def handle_status_update(self, status_msg: str):
        self.append_log(f"[状态] {status_msg}")
        
        # 检测是否是"将军"提示
        if "check_detected" in status_msg:
            self.show_temporary_popup("将军！", color="#f39c12")
        # 检测是否是"将军"提示
        if "移动无效" in status_msg:
            self.show_temporary_popup("移动无效", color="#d91908")    
        # 检测是否是结束游戏
        if status_msg.startswith("over:"):
            # 使用自定义的消息框来显示游戏结束信息
            game_over_msg ="游戏结束！" + status_msg.replace('over:', '').strip()
            self.show_game_over_dialog(game_over_msg)

    def show_game_over_dialog(self, message):
        """显示游戏结束对话框"""
        # 创建一个自定义的对话框来替代 QMessageBox
        dialog = QDialog(self)
        dialog.setWindowTitle("游戏结束")
        dialog.setModal(True)
        
        # 设置固定大小
        dialog.setFixedSize(800, 600)
        
        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(30)
        
        # 图标标签
        icon_label = QLabel("🎯")
        icon_label.setAlignment(Qt.AlignCenter)
        icon_label.setFont(QFont("Arial", 60))
        layout.addWidget(icon_label)
        
        # 消息标签
        message_label = QLabel(message)
        message_label.setAlignment(Qt.AlignCenter)
        message_label.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
        message_label.setWordWrap(True)  # 重要：允许文本自动换行
        message_label.setStyleSheet("""
            QLabel {
                color: #2c3e50;
                padding: 20px;
                background-color: #f8f9fa;
                border-radius: 15px;
                border: 2px solid #bdc3c7;
            }
        """)
        layout.addWidget(message_label)
        
        # 确定按钮
        ok_button = QPushButton("确定")
        ok_button.setFont(QFont("Microsoft YaHei", 20, QFont.Bold))
        ok_button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                border: none;
                border-radius: 10px;
                padding: 15px 30px;
                min-height: 50px;
            }
            QPushButton:hover {
                background-color: #2ecc71;
            }
        """)
        ok_button.clicked.connect(dialog.accept)
        layout.addWidget(ok_button)
        
        # 显示对话框并等待用户响应
        result = dialog.exec()
        
        # 无论用户点击什么按钮，都结束游戏并返回
        self.stop_timing()
        self.back_signal.emit()
        
    @Slot(str)
    def append_log(self, message):
        self.log_box.append(message)
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())
