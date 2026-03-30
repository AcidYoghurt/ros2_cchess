import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cchess
import re
import time

class VoiceGameNode(Node):
    UCI_DIRECT_PATTERN = re.compile(r'\b([a-i][0-9][a-i][0-9])\b', re.IGNORECASE)
    UCI_PAIR_PATTERN = re.compile(r'([a-i][0-9])\s*[,，\-\s>]*\s*([a-i][0-9])', re.IGNORECASE)

    def __init__(self):
        super().__init__('voice_game_node')
        self.get_logger().info("语音模式管理节点已启动。")

        # 1. 棋盘逻辑
        self.board = cchess.Board()
        
        # 2. 发布者
        self.arm_move_publisher = self.create_publisher(String, '/right/ai_move_topic', 10)
        self.fen_publisher = self.create_publisher(String, 'fen_topic', 10)
        self.status_publisher = self.create_publisher(String, 'status_topic', 10)
        
        # 3. 订阅者
        # 接收语音指令
        self.voice_move_sub = self.create_subscription(
            String, 'voice_move_topic', self.voice_move_callback, 10
        )
        
        # --- 订阅 FEN 话题以保持同步 ---
        self.fen_subscription = self.create_subscription(
            String, 'fen_topic', self.fen_sync_callback, 10
        )

        self.declare_parameter('voice_repeat_window_sec', 1.2)
        self.repeat_window_sec = float(self.get_parameter('voice_repeat_window_sec').value)
        self.last_command_key = ""
        self.last_command_time = 0.0


    def fen_sync_callback(self, msg):
        """当 AI 走棋或外部更新 FEN 时，同步内部棋盘状态"""
        new_fen = msg.data
        try:
            incoming_board = cchess.Board(new_fen)
        except Exception as e:
            self.get_logger().warn(f"收到无效FEN，忽略同步: {e}")
            return

        # 如果收到的 FEN 和内部不一致，则更新内部棋盘
        if new_fen != self.board.fen():
            self.board = incoming_board
            self.get_logger().info("内部棋盘已同步至最新局势。")

    def normalize_voice_move(self, raw_text):
        text = (raw_text or "").strip().lower()
        if not text:
            return None

        direct_match = self.UCI_DIRECT_PATTERN.search(text)
        if direct_match:
            return direct_match.group(1)

        pair_match = self.UCI_PAIR_PATTERN.search(text)
        if pair_match:
            return f"{pair_match.group(1)}{pair_match.group(2)}"

        compact = re.sub(r'[^a-i0-9]', '', text)
        for i in range(max(0, len(compact) - 3)):
            candidate = compact[i:i + 4]
            if re.fullmatch(r'[a-i][0-9][a-i][0-9]', candidate):
                return candidate
        
        # 尝试解析带方向的指令
        direction_move = self.parse_direction_move(text)
        if direction_move:
            return direction_move
        
        return None
    
    def parse_direction_move(self, text):
        """解析带方向的指令，如"右边的炮向左移动2步"""
        # 棋子类型映射
        piece_map = {
            '炮': 'c', '车': 'r', '马': 'n', '相': 'b', '士': 'a', '帅': 'k', '兵': 'p'
        }
        
        # 方向映射（红方视角）
        direction_map = {
            '左': 'left', '右': 'right', '前': 'forward', '后': 'backward'
        }
        
        # 提取棋子类型
        piece_type = None
        for key in piece_map:
            if key in text:
                piece_type = piece_map[key]
                break
        if not piece_type:
            return None
        
        # 提取方向
        direction = None
        for key in direction_map:
            if key in text:
                direction = direction_map[key]
                break
        if not direction:
            return None
        
        # 提取步数
        step_match = re.search(r'(\d+)步', text)
        if not step_match:
            return None
        steps = int(step_match.group(1))
        
        # 提取位置描述（如"右边的"）
        position_desc = None
        if '左边的' in text:
            position_desc = 'left'
        elif '右边的' in text:
            position_desc = 'right'
        elif '前面的' in text:
            position_desc = 'front'
        elif '后面的' in text:
            position_desc = 'back'
        
        # 找到符合条件的红方棋子
        candidate_pieces = []
        for square in cchess.SQUARES:
            piece = self.board.piece_at(square)
            if piece and piece.color == cchess.RED and piece.symbol().lower() == piece_type:
                candidate_pieces.append((square, piece))
        
        if not candidate_pieces:
            return None
        
        # 根据位置描述筛选棋子
        if position_desc == 'left':
            # 最左边的棋子（横坐标最小）
            candidate_pieces.sort(key=lambda x: cchess.square_column(x[0]))
        elif position_desc == 'right':
            # 最右边的棋子（横坐标最大）
            candidate_pieces.sort(key=lambda x: -cchess.square_column(x[0]))
        elif position_desc == 'front':
            # 最前面的棋子（纵坐标最大）
            candidate_pieces.sort(key=lambda x: -cchess.square_row(x[0]))
        elif position_desc == 'back':
            # 最后面的棋子（纵坐标最小）
            candidate_pieces.sort(key=lambda x: cchess.square_row(x[0]))
        
        # 取第一个符合条件的棋子
        target_square = candidate_pieces[0][0]
        from_col = cchess.square_column(target_square)
        from_row = cchess.square_row(target_square)
        
        # 根据方向计算目标位置
        to_col, to_row = from_col, from_row
        if direction == 'left':
            to_col = max(0, from_col - steps)
        elif direction == 'right':
            to_col = min(8, from_col + steps)
        elif direction == 'forward':
            to_row = min(9, from_row + steps)
        elif direction == 'backward':
            to_row = max(0, from_row - steps)
        
        # 转换为UCI格式
        from_square = cchess.square(from_col, from_row)
        to_square = cchess.square(to_col, to_row)
        
        # 检查移动是否合法
        try:
            move = cchess.Move(from_square, to_square)
            if move in self.board.legal_moves:
                return move.uci()
        except Exception:
            pass
        
        return None

    def is_duplicate_command(self, fen_before, uci_move):
        command_key = f"{fen_before}|{uci_move}"
        now = time.monotonic()
        is_duplicate = (
            command_key == self.last_command_key and
            (now - self.last_command_time) < self.repeat_window_sec
        )
        self.last_command_key = command_key
        self.last_command_time = now
        return is_duplicate

    def voice_move_callback(self, msg):
        raw_voice_text = msg.data
        uci_move = self.normalize_voice_move(raw_voice_text)

        if not uci_move:
            self.get_logger().warn(f"无法从语音结果解析UCI: '{raw_voice_text}'")
            self.publish_status("移动无效：语音格式错误，请重说")
            return

        self.get_logger().info(f"语音输入解析: '{raw_voice_text}' -> '{uci_move}'")
        
        # 在处理前，先确认是否轮到红方（玩家）走
        if self.board.turn != cchess.RED:
            self.get_logger().warn("现在不是红方回合，忽略语音指令。")
            self.publish_status("警告：现在是黑方思考时间")
            return

        if self.is_duplicate_command(self.board.fen(), uci_move):
            self.get_logger().warn(f"短时间重复语音指令，已忽略: {uci_move}")
            self.publish_status("提示：检测到重复指令，已忽略")
            return

        try:
            move = cchess.Move.from_uci(uci_move)

            moving_piece = self.board.piece_at(move.from_square)
            if moving_piece is None:
                self.get_logger().error(f"起点无棋子: {uci_move}")
                self.publish_status("移动无效：起点没有棋子")
                return

            if moving_piece.color != cchess.RED:
                self.get_logger().error(f"尝试移动非红方棋子: {uci_move}")
                self.publish_status("移动无效：只能移动红方棋子")
                return

            target_piece = self.board.piece_at(move.to_square)
            if target_piece is not None and target_piece.color == cchess.RED:
                self.get_logger().error(f"目标为己方棋子: {uci_move}")
                self.publish_status("移动无效：目标位置有己方棋子")
                return

            if move not in self.board.legal_moves:
                same_source_moves = [m.uci() for m in self.board.legal_moves if m.from_square == move.from_square]
                if same_source_moves:
                    hint = f"；可选走法: {', '.join(same_source_moves[:4])}"
                else:
                    hint = ""
                self.get_logger().error(f"非法走法: {uci_move}")
                self.publish_status(f"移动无效：不符合规则{hint}")
                return

            # 动作指令生成
            start_pos = uci_move[0:2]
            end_pos = uci_move[2:4]
            captured_piece = self.board.piece_at(move.to_square)

            move_parts = []
            if captured_piece is not None:
                move_parts.append(end_pos) # 吃子位置
                move_parts.append("o0")

            move_parts.extend([start_pos, end_pos])
            ai_move_msg_data = ",".join(move_parts)

            # 发布给机械臂执行
            arm_msg = String()
            arm_msg.data = ai_move_msg_data
            self.arm_move_publisher.publish(arm_msg)

            # 更新棋盘并发布（触发黑方）
            self.board.push(move)
            fen_msg = String()
            fen_msg.data = self.board.fen()
            self.fen_publisher.publish(fen_msg)

            self.check_game_status()

        except Exception as e:
            self.get_logger().error(f"处理错误: {e}")
            self.publish_status("移动无效：解析失败，请重说")

    def check_game_status(self):
        if self.board.is_check():
            self.publish_status("check_detected")
        if self.board.is_game_over():
            self.publish_status(f"over:{self.board.result()}")

    def publish_status(self, text):
        msg = String(); msg.data = text
        self.status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceGameNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
