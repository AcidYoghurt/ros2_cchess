import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from std_msgs.msg import Bool
from .reg_core.engine import ChineseChessGame
import cchess

class RobEngineNode(Node):
    def __init__(self):
        super().__init__('rob_engine_node')
        self.get_logger().info("双机中国象棋引擎节点已启动。等待UCI配置信息...")

        # 初始化核心游戏逻辑
        self.game = None

        # --- 状态和配置变量 ---
        self.is_configured = False
        self.config = {
            'player_color': cchess.BLACK,
            'think_time': 1.0,
            'difficulty_level': 5,
            'limit_strength': False,
            'elo_rating': 2000
        }

        # --- ROS 通信 ---
        self.uci_subscription = self.create_subscription(String, 'rob_uci_topic', self.uci_callback, 10)
        self.move_subscription = self.create_subscription(String, '/right/ai_move_topic', self.move_callback, 10)
        self.ai_move_publisher = self.create_publisher(String, 'chess_move_topic', 10)
        self.rob_move_publisher = self.create_publisher(String, '/left/ai_move_topic', 10)
        self.fen_publisher = self.create_publisher(String, 'fen_topic', 10)
        self.status_publisher = self.create_publisher(String, 'status_topic', 10)
        self.machinery_trigger_sub = self.create_subscription(Bool, '/right/machinery_pub_image_trigger', self.machinery_trigger_callback, 10)
        self.machinery_trigger_sub = self.create_subscription(Bool, '/left/machinery_pub_image_trigger', self.move_pub_callback, 10)
        #移动结果缓存区
        self.pending_engine_rob_move = None
        self.pending_engine_move = None
        # 状态管理
        self.is_paused = False
        self.pause_event = threading.Event()

    # ===============================================================
    # 初始化游戏
    # ===============================================================
    def init_game(self):
        if self.game is not None:
            self.get_logger().info("游戏已初始化，跳过。")
            return True

        try:
            config = self.config
            player_color = config['player_color']
            engine_path = self.get_engine_path()

            if not engine_path:
                self.get_logger().error("未能找到pikafish引擎，游戏无法启动。")
                msg = String()
                msg.data = "error:engine_not_found"
                self.status_publisher.publish(msg)
                return False

            self.game = ChineseChessGame(
                player_color=player_color,
                engine_path=engine_path,
                think_time=config['think_time'],
                difficulty_level=config['difficulty_level'],
                limit_strength=config['limit_strength'],
                elo_rating=config['elo_rating']
            )

            started = self.game.start_engine()
            if not started:
                self.get_logger().error("引擎启动失败。")
                msg = String()
                msg.data = "error:engine_start_failed"
                self.status_publisher.publish(msg)
                self.game = None
                return False

            self.is_configured = True
            self.get_logger().info("游戏核心已初始化并启动。")
            return True

        except Exception as e:
            self.get_logger().error(f"游戏初始化失败: {e}")
            msg = String()
            msg.data = f"error:game_init_exception:{e}"
            self.status_publisher.publish(msg)
            self.game = None
            return False

    # ===============================================================
    def get_engine_path(self):
        possible_paths = [
            "src/cchess_ros_decision/resource/pikafish",
            "pikafish",
            "./pikafish",
            "/usr/games/pikafish"
        ]
        for path in possible_paths:
            if os.path.exists(path):
                self.get_logger().info(f"找到引擎路径: {os.path.abspath(path)}")
                return os.path.abspath(path)
        self.get_logger().error("未找到引擎文件")
        return None

    # ===============================================================
    def move_callback(self, msg):
        if not self.is_configured:
            self.get_logger().warn("收到移动消息，但尚未配置。")
            s = String(); s.data = "error:not_configured"
            self.status_publisher.publish(s)
            return

        if self.game is None:
            s = String(); s.data = "error:no_game_instance"
            self.status_publisher.publish(s)
            return

        if self.is_paused:
            s = String(); s.data = "info:paused"
            self.status_publisher.publish(s)
            return

        self.get_logger().info(f'收到棋子移动消息: "{msg.data}"')
        
        try:
            parts = msg.data.split(',')  # 将字符串按逗号分割成列表
            if len(parts) >= 4 and parts[1] == 'o0':  # 检查第二部分是否为"o0"
                # 取最后两个部分（例如从"g3,o0,g4,g3"中取"g4"和"g3"）
                msg.data = ','.join(parts[2:])
            move_uci = msg.data.replace(',', '').replace(' ', '')
            success, message, outcome = self.game.make_player_move(move_uci)
            if not success:
                self.get_logger().warn(message)
                s = String()
                s.data = f"(rob2)error:illegal_move:{message}"
                self.status_publisher.publish(s)
                return

            self.publish_fen()

            # 检查将军
            self.check_for_check_and_checkmate()



            self.execute_engine_move()

        except Exception as e:
            self.get_logger().error(f"处理移动消息时出错: {e}")
            wrong_msg = String()
            wrong_msg.data = f"error:fail_to_handle_move:{e}"
            self.status_publisher.publish(wrong_msg)

    # ===============================================================
    def uci_callback(self, msg):
        """处理来自前端的信息内容"""
        self.get_logger().info(f'收到UCI消息: "{msg.data}"')
        data = msg.data.strip()

        # 处理暂停命令
        if data.lower() == "pause":
            if not self.is_paused:
                self.is_paused = True
                self.pause_event.clear()
                self.get_logger().info("收到暂停命令，游戏已暂停。")
                s = String()
                s.data = "info:paused"
                self.status_publisher.publish(s)
            else:
                self.get_logger().info("游戏已处于暂停状态。")
            return

        # 处理恢复命令（可选）
        if data.lower() == "resume":
            if self.is_paused:
                self.is_paused = False
                self.pause_event.set()
                self.get_logger().info("收到恢复命令，游戏继续。")
                s = String()
                s.data = "info:resumed"
                self.status_publisher.publish(s)
            else:
                self.get_logger().info("游戏未暂停，无需恢复。")
            return

        if data.upper() == "OVER":
            self.get_logger().info("收到游戏结束命令")
            self.handle_game_over_command()
            msg = String(); msg.data = "info:game_over_command"
            self.status_publisher.publish(msg)
            return

        try:
            json_data = json.loads(data)
            command = json_data.get("command")
            if command == "config" and not self.is_configured:
                self.handle_config_message(json_data)
                return
        except json.JSONDecodeError:
            pass

    
    def machinery_trigger_callback(self, msg: Bool):
        """当收到机械触发信号时，如果 pending_engine_rob_move 存在，则发布它"""
        if not msg.data:
            return  # 只在 True 时触发

        if self.pending_engine_rob_move is None:
            self.get_logger().info("收到机械触发信号，但当前没有待发布的引擎落子。")
            return

        self.get_logger().info("收到机械触发信号，正在发布引擎落子与FEN状态...")

        # 从缓存中取出待发布内容
        rob_move_msg_data = self.pending_engine_rob_move["msg"]
        outcome = self.pending_engine_rob_move["outcome"]

        # --- 发布引擎移动信息 ---
        rob_move_msg = String()
        rob_move_msg.data = rob_move_msg_data
        self.rob_move_publisher.publish(rob_move_msg)

        # --- 发布当前FEN状态 ---
        self.publish_fen()

        # --- 检查是否结束 ---
        if outcome:
            self.handle_game_over(outcome)
        self.pending_engine_rob_move = None

    def move_pub_callback(self, msg: Bool):
        """当收到机械触发信号时，如果 pending_engine_move 存在，则发布它"""
        if not msg.data:
            return  # 只在 True 时触发

        if self.pending_engine_move is None:
            self.get_logger().info("收到机械触发信号，但当前没有待发布的引擎落子。")
            return

        self.get_logger().info("收到机械触发信号，正在发布引擎落子与FEN状态...")

        # 从缓存中取出待发布内容
        ai_move_msg_data = self.pending_engine_move["msg"]
        outcome = self.pending_engine_move["outcome"]

        # --- 发布引擎移动信息 ---
        ai_move_msg = String()
        ai_move_msg.data = ai_move_msg_data
        self.ai_move_publisher.publish(ai_move_msg)

        # 检查将军
        self.check_for_check_and_checkmate()

        # --- 检查是否结束 ---
        if outcome:
            self.handle_game_over(outcome)

        # 清空缓存
        self.pending_engine_move = None

    # ===============================================================
    def handle_config_message(self, config_data):
        self.get_logger().info("收到配置信息，正在初始化游戏核心...")

        if "player_color" in config_data:
            color_str = config_data["player_color"].upper()
            self.config['player_color'] = cchess.RED if color_str == "RED" else cchess.BLACK
            self.get_logger().info(f"设置玩家颜色: {color_str}")

        if "elo" in config_data:
            self.config['elo_rating'] = int(config_data["elo"])
            self.config['limit_strength'] = True
            self.get_logger().info(f"设置ELO等级: {self.config['elo_rating']}")

        if "skill_level" in config_data:
            self.config['difficulty_level'] = int(config_data["skill_level"])
            self.config['limit_strength'] = True
            self.get_logger().info(f"设置难度等级: {self.config['difficulty_level']}")

        if "think_time" in config_data:
            self.config['think_time'] = float(config_data["think_time"])
            self.get_logger().info(f"设置思考时间: {self.config['think_time']}秒")

        if self.init_game() and self.config['player_color'] == cchess.BLACK:
            self.get_logger().info("玩家执黑，引擎先行。")
            self.execute_engine_move()

    # ===============================================================
    def execute_engine_move(self):
        if self.is_paused or self.game is None:
            return

        self.get_logger().info("引擎思考中...")
        s = String(); s.data = "info:机械臂2引擎思考中。。。"
        self.status_publisher.publish(s)

        try:
            success, move_uci, captured_info, outcome, error = self.game.make_engine_move()
             
        except Exception as e:
            self.get_logger().error(f"引擎思考抛出异常: {e}")
            s = String(); s.data = f"error:engine_exception:{e}"
            self.status_publisher.publish(s)
            return
        if success:
            # 发布引擎移动信息
            start_pos = move_uci[0:2]
            end_pos = move_uci[2:4]
            move_parts = []
            #如果发生吃子
            if captured_info:
                move_parts.append(captured_info)
                move_parts.append("o0")
            move_parts.append(start_pos)
            move_parts.append(end_pos)
            ai_move_msg_data = ",".join(move_parts)

            # 不立即发布，将引擎移动暂存在 pending_engine_move
            self.pending_engine_rob_move = {
                "msg": ai_move_msg_data,
                "outcome": outcome
            }

            chess_move_msg_data = self.game.get_current_fen()+', '+move_uci
            #打包成标准信息格式
            self.pending_engine_move = {
                "msg": chess_move_msg_data,
                "outcome": outcome
            }

            self.get_logger().info("引擎移动已生成，等待机械触发信号 (machinery_pub_image_trigger=True)...")
            
            if outcome:
                self.handle_game_over(outcome)
        else:
            self.get_logger().error(f"引擎移动失败: {error}")
            s = String()
            s.data = f"error:engine_move_failed:{error}"
            self.status_publisher.publish(s)
            
    def check_for_check_and_checkmate(self):
        """检查是否为将军或将死，并通知前端"""
    # 假设game对象提供了is_check和is_checkmate的方法
        if self.game.is_check():
            check_msg = String()
            check_msg.data = "info:check_detected"
            self.status_publisher.publish(check_msg)

    # ===============================================================
    def publish_fen(self):
        if self.game is None:
            return
        try:
            fen_msg = String()
            fen_msg.data = self.game.get_current_fen()
            self.fen_publisher.publish(fen_msg)
        except Exception as e:
            self.get_logger().error(f"发布FEN状态时出错: {e}")
            wrong_msg = String()
            wrong_msg.data = f"error:fen_publish_failed:{e}"
            self.status_publisher.publish(wrong_msg)

    # ===============================================================
    def handle_game_over_command(self):
        self.get_logger().info("正在执行游戏结束流程...")
        if self.game is not None:
            self.game.save_game()
            self.game = None
            self.is_configured = False
            self.get_logger().info("游戏已结束并重置。")
        else:
            self.get_logger().info("没有正在进行的游戏，无需结束。")

    # ===============================================================
    def handle_game_over(self, outcome):
        """处理游戏结束，自动检测将死"""
        result_msg = f"over:{outcome.termination.name},result:{outcome.result()}"
        self.get_logger().info(result_msg)

        # 如果是将死，则额外通知前端
        if outcome.termination.name.upper() == "CHECKMATE":
            checkmate_msg = String()
            checkmate_msg.data = "info:checkmate_detected"
            self.status_publisher.publish(checkmate_msg)

        if self.game:
            self.game.save_game()

        s = String()
        s.data = result_msg
        self.status_publisher.publish(s)

        # 清理
        self.game = None
        self.is_configured = False

    # ===============================================================
    def destroy_node(self):
        if self.game:
            try:
                self.game.stop_engine()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
