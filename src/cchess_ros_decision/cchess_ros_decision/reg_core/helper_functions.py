# helper_functions.py0
#此为棋盘识别辅助函数

def convert_coord(r, c):
    """
    该函数用于将棋盘转为二维数组格式（0-9,a-i）  
    """
    col_char = chr(ord('a') + c)
    row_num = 9 - r
    return f"{col_char}{row_num}"

def fen_to_board(fen: str):
    """将标准中国象棋 FEN 字符串转为 10x9 棋盘数组"""
    # 取棋盘部分（在空格前）
    fen_board_part = fen.split(" ")[0].strip()
    rows = fen_board_part.split("/")

    if len(rows) != 10:
        raise ValueError(f"FEN 行数错误：{len(rows)}，应为10行。")

    board = []
    for row in rows:
        expanded = []
        for ch in row:
            if ch.isdigit():
                expanded.extend(['.'] * int(ch))
            else:
                expanded.append(ch)
        if len(expanded) != 9:
            raise ValueError(f"FEN 行长度错误：{row} -> {len(expanded)}列，应为9列。")
        board.append(expanded)
    return board



def generate_fen_string(board_2d_array_short):
    """
    将二维数组形式的棋盘（10x9）转换为标准 FEN 字符串
    """
    fen_rows = []
    for row in board_2d_array_short:
        fen_row = ""
        empty_count = 0
        for cell in row:
            if cell == '.':
                empty_count += 1
            else:
                if empty_count > 0:
                    fen_row += str(empty_count)
                    empty_count = 0
                fen_row += cell
        if empty_count > 0:
            fen_row += str(empty_count)
        fen_rows.append(fen_row)
    return "/".join(fen_rows)

def is_board_flipped_by_kings(board_2d_array):
    """
    如果棋盘识别反转，就使其正常
    """
    red_king_pos = None
    black_king_pos = None
    for r in range(10):
        for c in range(9):
            if board_2d_array[r][c] == 'K':
                red_king_pos = (r, c)
            elif board_2d_array[r][c] == 'k':
                black_king_pos = (r, c)
    if red_king_pos is None or black_king_pos is None:
        return False
    return red_king_pos[0] < black_king_pos[0]


def compare_and_generate_move_fen(board1, board2):
    """
    对比两个棋盘状态，识别并返回棋子移动。
    返回移动前的 FEN 字符串和标准的走子字符串（例如：'a9a8'）。
    """
    # 找出所有不同的位置
    diff_positions = []
    for r in range(10):
        for c in range(9):
            if board1[r][c] != board2[r][c]:
                diff_positions.append((r, c))

    # 如果没有变化或变化过多，无法确定移动
    if len(diff_positions) != 2:
        return None, None

    # 分析两个变化位置
    pos1, pos2 = diff_positions

    # 遍历棋盘是通过一个二维数组，从上左到右下遍历，因此棋子所在方位会影响遍历情况

    # 情况1：标准移动（起点变空，终点有棋子）
    if board1[pos1[0]][pos1[1]] != '.' and board2[pos1[0]][pos1[1]] == '.' and \
       board1[pos2[0]][pos2[1]] == '.' and board2[pos2[0]][pos2[1]] != '.':
        start_pos = pos1
        end_pos = pos2
      

    # 情况2：标准移动（终点有棋子，起点变空） - 顺序互换
    elif board1[pos1[0]][pos1[1]] == '.' and board2[pos1[0]][pos1[1]] != '.' and \
         board1[pos2[0]][pos2[1]] != '.' and board2[pos2[0]][pos2[1]] == '.':
        start_pos = pos2
        end_pos = pos1
       

    # 情况3：吃子移动（起点变空，终点棋子变化）
    elif board1[pos1[0]][pos1[1]] != '.' and board2[pos1[0]][pos1[1]] == '.' and \
         board1[pos2[0]][pos2[1]] != '.' and board2[pos2[0]][pos2[1]] != '.':
        start_pos = pos1
        end_pos = pos2
        

    # 情况4：吃子移动（终点棋子变化，起点变空） - 顺序互换
    elif board1[pos1[0]][pos1[1]] != '.' and board2[pos1[0]][pos1[1]] != '.' and \
         board1[pos2[0]][pos2[1]] != '.' and board2[pos2[0]][pos2[1]] == '.':
        start_pos = pos2
        end_pos = pos1
        

    else:
        # 无法识别的变化模式
        return None, None

    fen1 = generate_fen_string(board1)
    move_str = f"{convert_coord(start_pos[0], start_pos[1])}{convert_coord(end_pos[0], end_pos[1])}"

    return fen1, move_str