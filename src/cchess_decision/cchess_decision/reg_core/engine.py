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

    def _build_limit(self, time_limit=None, depth=None):
        """构建引擎分析限制条件"""
        limit_kwargs = {}
        if time_limit is not None:
            limit_kwargs["time"] = time_limit
        if depth is not None:
            limit_kwargs["depth"] = depth

        # 默认使用实例中的思考时间
        if not limit_kwargs:
            limit_kwargs["time"] = self.think_time
        return cchess.engine.Limit(**limit_kwargs)

    def _score_to_dict(self, pov_score):
        """将引擎分数对象转为可序列化字典"""
        if pov_score is None:
            return {
                "cp": None,
                "mate": None,
                "is_mate": False,
                "score_for_compare": None
            }

        return {
            "cp": pov_score.score(),
            "mate": pov_score.mate(),
            "is_mate": pov_score.is_mate(),
            "score_for_compare": pov_score.score(mate_score=100000)
        }

    def analyse_position(self, board=None, perspective=None, time_limit=None, depth=12, multipv=1):
        """
        分析一个局面并返回评分和主变化。

        Args:
            board: 可选，指定要分析的棋盘；默认分析当前 self.board
            perspective: 评分视角（cchess.RED/cchess.BLACK）；默认当前行棋方
            time_limit: 单次分析时间（秒），优先于默认 think_time
            depth: 搜索深度（可与 time_limit 一起使用）
            multipv: 返回前N条主变化

        Returns:
            dict: 包含评分、bestmove、pv等信息
        """
        if self.engine is None:
            started = self.start_engine()
            if not started:
                raise RuntimeError("引擎未启动，无法分析局面")

        working_board = board if board is not None else self.board
        pov = working_board.turn if perspective is None else perspective
        limit = self._build_limit(time_limit=time_limit, depth=depth)

        info_flags = cchess.engine.INFO_SCORE | cchess.engine.INFO_PV | cchess.engine.INFO_BASIC
        info = self.engine.analyse(working_board, limit, info=info_flags, multipv=max(1, int(multipv)))

        lines = info if isinstance(info, list) else [info]
        variations = []
        for line in lines:
            pv = line.get("pv", []) or []
            score_obj = line.get("score")
            pov_score = score_obj.pov(pov) if score_obj else None
            variations.append({
                "pv": [mv.uci() for mv in pv],
                "bestmove": pv[0].uci() if pv else None,
                "score": self._score_to_dict(pov_score),
                "depth": line.get("depth"),
                "nodes": line.get("nodes"),
                "time": line.get("time")
            })

        best = variations[0] if variations else {
            "pv": [],
            "bestmove": None,
            "score": self._score_to_dict(None),
            "depth": None,
            "nodes": None,
            "time": None
        }
        return {
            "fen": working_board.fen(),
            "turn": "RED" if working_board.turn == cchess.RED else "BLACK",
            "perspective": "RED" if pov == cchess.RED else "BLACK",
            "bestmove": best["bestmove"],
            "pv": best["pv"],
            "score": best["score"],
            "depth": best["depth"],
            "nodes": best["nodes"],
            "time": best["time"],
            "multipv": variations
        }

    def analyse_move(self, move_uci, board=None, time_limit=None, depth=12):
        """
        分析某一步棋质量，输出该步的CP损失（Centipawn Loss）。

        Returns:
            dict: {
                "move": str,
                "is_legal": bool,
                "bestmove": str|None,
                "is_bestmove": bool,
                "cp_loss": int|None,
                "quality": str,
                "before": dict,
                "after": dict,
                "error": str|None
            }
        """
        working_board = board.copy(stack=True) if board is not None else self.board.copy(stack=True)
        mover = working_board.turn

        before = self.analyse_position(
            board=working_board,
            perspective=mover,
            time_limit=time_limit,
            depth=depth,
            multipv=1
        )

        try:
            move = cchess.Move.from_uci(move_uci)
        except Exception as e:
            return {
                "move": move_uci,
                "is_legal": False,
                "bestmove": before.get("bestmove"),
                "is_bestmove": False,
                "cp_loss": None,
                "quality": "illegal",
                "before": before,
                "after": None,
                "error": f"UCI解析失败: {e}"
            }

        if move not in working_board.legal_moves:
            return {
                "move": move_uci,
                "is_legal": False,
                "bestmove": before.get("bestmove"),
                "is_bestmove": False,
                "cp_loss": None,
                "quality": "illegal",
                "before": before,
                "after": None,
                "error": "非法走子"
            }

        working_board.push(move)
        after = self.analyse_position(
            board=working_board,
            perspective=mover,
            time_limit=time_limit,
            depth=depth,
            multipv=1
        )

        before_score = before["score"]["score_for_compare"]
        after_score = after["score"]["score_for_compare"]
        cp_loss = None
        if before_score is not None and after_score is not None:
            cp_loss = before_score - after_score

        if cp_loss is None:
            quality = "unknown"
        elif cp_loss <= 20:
            quality = "excellent"
        elif cp_loss <= 60:
            quality = "good"
        elif cp_loss <= 120:
            quality = "inaccuracy"
        elif cp_loss <= 250:
            quality = "mistake"
        else:
            quality = "blunder"

        return {
            "move": move_uci,
            "is_legal": True,
            "bestmove": before.get("bestmove"),
            "is_bestmove": before.get("bestmove") == move_uci,
            "cp_loss": cp_loss,
            "quality": quality,
            "before": before,
            "after": after,
            "error": None
        }

    def score_moves(self, moves_uci, initial_fen=None, time_limit=None, depth=12):
        """
        给一盘棋（UCI走子序列）逐步打分。

        Args:
            moves_uci: list[str] 走子序列，如 ["h2e2", "h9g7", ...]
            initial_fen: 可选，起始局面FEN；默认当前局面
            time_limit: 每步分析时间（秒）
            depth: 每步搜索深度

        Returns:
            dict: 包含每一步评分、平均CP损失、失误统计
        """
        analysis_board = self.board.copy(stack=True)
        if initial_fen:
            analysis_board.set_fen(initial_fen)

        per_move = []
        valid_cp_loss = []
        quality_counter = {
            "excellent": 0,
            "good": 0,
            "inaccuracy": 0,
            "mistake": 0,
            "blunder": 0,
            "illegal": 0,
            "unknown": 0
        }

        for ply, move_uci in enumerate(moves_uci, start=1):
            mover = analysis_board.turn
            side = "RED" if mover == cchess.RED else "BLACK"
            move_result = self.analyse_move(
                move_uci=move_uci,
                board=analysis_board,
                time_limit=time_limit,
                depth=depth
            )
            move_result["ply"] = ply
            move_result["side"] = side
            per_move.append(move_result)

            quality = move_result.get("quality", "unknown")
            if quality not in quality_counter:
                quality = "unknown"
            quality_counter[quality] += 1

            cp_loss = move_result.get("cp_loss")
            if cp_loss is not None:
                valid_cp_loss.append(cp_loss)

            # 非法走子后停止后续分析
            if not move_result["is_legal"]:
                break

            analysis_board.push(cchess.Move.from_uci(move_uci))

        average_cp_loss = sum(valid_cp_loss) / len(valid_cp_loss) if valid_cp_loss else None
        return {
            "start_fen": initial_fen if initial_fen else self.board.fen(),
            "end_fen": analysis_board.fen(),
            "move_count": len(per_move),
            "average_cp_loss": average_cp_loss,
            "quality_counter": quality_counter,
            "moves": per_move
        }

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
