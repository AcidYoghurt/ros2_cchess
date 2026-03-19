import cchess
import cchess.engine
import os
import stat
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ChineseChessGame:
    """中国象棋游戏核心逻辑，不依赖ROS"""
    
    def __init__(self, player_color=cchess.RED, engine_path="pikafish", think_time=1.0,
                 difficulty_level=10, limit_strength=False, elo_rating=2000, board=None):
        self.board = board if board else cchess.Board()
        self.player_color = player_color
        self.think_time = think_time
        self.limit_strength = limit_strength
        self.elo_rating = elo_rating
        self.difficulty_level = difficulty_level
        self.engine_path = engine_path
        self.engine = None

    def is_check(self):
        """
        检查当前棋盘状态是否正在将军。
        返回: 如果当前行棋方被将军，则返回 True，否则返回 False。
        """
        return self.board.is_check()


    def start_engine(self):
        """启动象棋引擎"""
        try:
            if self.engine:
                self.engine.quit()
            
            engine_path_abs = os.path.abspath(self.engine_path)
            logger.info(f"尝试启动引擎: {engine_path_abs}")
            
            if not os.path.isfile(engine_path_abs):
                logger.error(f"未找到引擎文件: '{engine_path_abs}'")
                return False

            if not os.access(engine_path_abs, os.X_OK):
                os.chmod(engine_path_abs, os.stat(engine_path_abs).st_mode | stat.S_IEXEC)

            self.engine = cchess.engine.SimpleEngine.popen_uci(engine_path_abs)
            self.engine.configure({"Threads": 2, "Hash": 16})
            
            if self.limit_strength:
                self.engine.configure({"UCI_LimitStrength": "true", "UCI_Elo": str(self.elo_rating)})
            else:
                skill_level = max(0, min(20, self.difficulty_level))
                self.engine.configure({"UCI_LimitStrength": "false", "Skill Level": str(skill_level)})

            logger.info("引擎启动成功")
            return True
        except Exception as e:
            logger.error(f"引擎启动失败: {e}")
            return False

    def make_player_move(self, move_uci):
        """
        执行玩家移动
        
        Args:
            move_uci: UCI格式的移动字符串
            
        Returns:
            tuple: (success: bool, message: str, outcome: 游戏结果或None)
        """
        try:
            move = cchess.Move.from_uci(move_uci)
            if move in self.board.legal_moves:
                self.board.push(move)
                logger.info(f"成功执行玩家移动: {move_uci}")
                
                # 检查游戏状态
                outcome = self.board.outcome()
                return True, "移动成功", outcome
            else:
                logger.warning(f"玩家移动无效: {move_uci}")
                return False, f"移动无效: {move_uci}", None
                
        except Exception as e:
            logger.error(f"执行玩家移动时出错: {e}")
            return False, f"移动错误: {e}", None

    def make_engine_move(self):
        """
        执行引擎移动
        
        Returns:
            tuple: (success: bool, move_uci: str, captured_info: str, outcome: 游戏结果或None, error: str)
        """
        try:
            limit = cchess.engine.Limit(time=self.think_time)
            result = self.engine.play(self.board, limit)
            move = result.move

            if move and move in self.board.legal_moves:
                # 记录被吃的棋子信息
                captured_piece = self.board.piece_at(move.to_square)
                uci_notation = move.uci()
                
                # 执行移动
                self.board.push(move)
                
                # 准备被吃子信息
                captured_info = ""
                if captured_piece:
                    target_square = str(uci_notation)[-2:]
                    captured_info = target_square
                    logger.info(f"引擎移动: {uci_notation}，吃了{captured_piece}，被吃子坐标：{target_square}")
                else:
                    logger.info(f"引擎移动: {uci_notation}")
                
                # 检查游戏状态
                outcome = self.board.outcome()
                return True, uci_notation, captured_info, outcome, None
            else:
                logger.warning("引擎返回无效移动或未返回移动")
                return False, "", "", None, "引擎返回无效移动"
                
        except cchess.engine.EngineError as e:
            logger.error(f"引擎错误: {e}")
            return False, "", "", None, f"引擎错误: {e}"
        except Exception as e:
            logger.error(f"执行引擎移动时出错: {e}")
            return False, "", "", None, f"执行错误: {e}"

    def sync_board_state(self, fen_string):
        """
        同步棋盘状态
        
        Args:
            fen_string: FEN格式的棋盘状态
            
        Returns:
            bool: 同步是否成功
        """
        try:
            current_fen = self.board.fen().split(' ')[0]
            if fen_string != current_fen:
                self.board.set_fen(fen_string)
                logger.info("棋盘状态已同步")
                return True
            return True
        except Exception as e:
            logger.error(f"同步棋盘状态失败: {e}")
            return False

    def get_current_fen(self):
        """获取当前棋盘FEN状态"""
        return self.board.fen()

    def get_legal_moves(self):
        """获取当前合法移动列表"""
        return list(self.board.legal_moves)

    def check_game_status(self):
        """检查游戏状态"""
        return self.board.outcome()

    def save_game(self, filename=None):
        """保存游戏棋谱"""
        try:
            if not os.path.exists("games"):
                os.makedirs("games")
            
            if filename is None:
                filename = f"games/game_{len(os.listdir('games')) + 1 if os.path.exists('games') else 1}.pgn"
            else:
                filename = f"games/{filename}"
                
            with open(filename, "w", encoding="utf-8") as f:
                pgn = self.board.to_pgn(
                    red="人类玩家" if self.player_color == cchess.RED else "电脑引擎",
                    black="电脑引擎" if self.player_color == cchess.RED else "人类玩家"
                )
                f.write(pgn)
            logger.info(f"棋谱已保存到 {filename}")
            return True
        except Exception as e:
            logger.error(f"保存棋谱失败: {e}")
            return False

    def restart_game(self, player_color=None):
        """重新开始游戏"""
        try:
            self.board = cchess.Board()
            if player_color is not None:
                self.player_color = player_color
            logger.info("游戏已重新开始")
            return True
        except Exception as e:
            logger.error(f"重新开始游戏失败: {e}")
            return False

    def __del__(self):
        """析构函数，确保引擎正确关闭"""
        if hasattr(self, 'engine') and self.engine:
            try:
                self.engine.quit()
            except:
                pass