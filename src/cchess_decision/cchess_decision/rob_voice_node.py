import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cchess

class VoiceGameNode(Node):
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



    def fen_sync_callback(self, msg):
        """当 AI 走棋或外部更新 FEN 时，同步内部棋盘状态"""
        new_fen = msg.data
        # 如果收到的 FEN 和内部不一致，则更新内部棋盘
        if new_fen != self.board.fen():
            self.board = cchess.Board(new_fen)
            self.get_logger().info("内部棋盘已同步至最新局势。")

    def voice_move_callback(self, msg):
        uci_move = msg.data 
        
        # 在处理前，先确认是否轮到红方（玩家）走
        if self.board.turn != cchess.RED:
            self.get_logger().warn("现在不是红方回合，忽略语音指令。")
            self.publish_status("警告：现在是黑方思考时间")
            return

        try:
            move = cchess.Move.from_uci(uci_move)
            
            if move in self.board.legal_moves:
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
            else:
                self.get_logger().error(f"非法走法: {uci_move}")
                self.publish_status("移动无效：不符合规则")
                
        except Exception as e:
            self.get_logger().error(f"处理错误: {e}")

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