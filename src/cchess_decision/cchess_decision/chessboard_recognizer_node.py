import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
import cv2
import traceback

# 导入棋盘识别模块和辅助函数
from .reg_core.chessboard_detector import ChessboardDetector
from .reg_core.helper_functions import (
    convert_coord, generate_fen_string, compare_and_generate_move_fen,
    fen_to_board, is_board_flipped_by_kings
)


class ChessboardRecognizerNode(Node):
    def __init__(self):
        super().__init__('chessboard_recognizer_node')

        self.image_after_move = None
        self.fen_before_move = None  # 新增：保存移动前 FEN

        # 声明参数（模型路径）
        self.declare_parameter('pose_model_path', 'onnx/pose/4_v6-0301.onnx')
        self.declare_parameter('full_classifier_model_path', 'onnx/layout_recognition/nano_v3-0319.onnx')

        # 订阅 FEN 话题（移动前棋盘）
        self.sub_fen = self.create_subscription(
            String,
            'fen_topic',
            self.callback_fen,
            10
        )

        # 订阅移动后图片
        self.sub_after = self.create_subscription(
            Image,
            'image_after_move',
            self.callback_after,
            10
        )

        # 发布棋子移动结果
        self.chess_move_publisher = self.create_publisher(
            String,
            'chess_move_topic',
            10
        )

        # 发布状态消息
        self.status_publisher = self.create_publisher(
            String,
            'status_topic',
            10
        )

        self.bridge = CvBridge()

        # 加载模型
        pose_model_path = self.get_parameter('pose_model_path').get_parameter_value().string_value
        full_classifier_model_path = self.get_parameter('full_classifier_model_path').get_parameter_value().string_value

        self.get_logger().info(f"加载模型: {pose_model_path}  {full_classifier_model_path}")

        try:
            package_share_dir = get_package_share_directory('cchess_decision')
            self.detector = ChessboardDetector(
                pose_model_path=os.path.join(package_share_dir, pose_model_path),
                full_classifier_model_path=os.path.join(package_share_dir, full_classifier_model_path)
            )
            self.get_logger().info('棋盘检测节点启动成功')
        except Exception as e:
            self.get_logger().error(f'节点启动失败： {e}')
            self.get_logger().error(traceback.format_exc())
            self.detector = None

    # ============================================================
    # 接收 FEN 消息
    # ============================================================
    def callback_fen(self, msg):
        self.get_logger().info(f'收到 FEN: {msg.data}')
        self.fen_before_move = msg.data.strip()

    # ============================================================
    # 接收移动后图片
    # ============================================================
    def callback_after(self, msg):
        self.get_logger().info('收到移动后图片')
        self.image_after_move = msg
        self.process_move()

     # ======================================================
    # 新增：图像增强（只在识别失败时使用）
    # ======================================================

    def enhance_images(self, img_bgr):
        results = []

        # 亮度 + 对比度
        results.append(
            cv2.convertScaleAbs(img_bgr, alpha=1.3, beta=25)
        )

        # CLAHE
        lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(2.0, (8, 8))
        l = clahe.apply(l)
        results.append(
            cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)
        )

        # HSV 红黑增强
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        s = cv2.add(s, 30)
        v = cv2.add(v, 15)
        results.append(
            cv2.cvtColor(cv2.merge((h, s, v)), cv2.COLOR_HSV2BGR)
        )

        # 轻度锐化
        kernel = np.array([[0, -1, 0],
                           [-1, 5, -1],
                           [0, -1, 0]])
        results.append(
            cv2.filter2D(img_bgr, -1, kernel)
        )

        return results



    # ============================================================
    # 棋盘识别逻辑
    # ============================================================
    def detect_board_from_image(self, cv_image):
        if self.detector is None:
            self.get_logger().error('核心检测器未初始化')
            return None

        def _run(img_bgr):
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            _, _, cells_labels_str, _, time_info = \
                self.detector.pred_detect_board_and_classifier(img_rgb)

            rows = cells_labels_str.split("\n")
            board_arr = [list(r) for r in rows]

            self.get_logger().info(f'识别耗时: {time_info}')
            return board_arr

        # ---------- 第一次：原图 ----------
        try:
            board = _run(cv_image)
            if board:
                return board
        except Exception:
            pass

        # ---------- 第二次：增强图 ----------
        self.get_logger().warn('原图识别失败，尝试增强图')
        for enhanced in self.enhance_images(cv_image):
            try:
                board = _run(enhanced)
                if board:
                    self.get_logger().warn('增强图识别成功')
                    return board
            except Exception:
                continue

        # ---------- 全部失败 ----------
        self.get_logger().error('原图 + 增强图 均无法识别棋盘')
        wrong_msg = String()
        wrong_msg.data = 'wrong_image: board recognition failed'
        self.status_publisher.publish(wrong_msg)
        return None


    # ============================================================
    # 核心处理逻辑
    # ============================================================
    def process_move(self):
        if not self.fen_before_move or not self.image_after_move:
            self.get_logger().info('等待 FEN 或 图片...')
            return

        try:
            cv_image_after = self.bridge.imgmsg_to_cv2(
                self.image_after_move,
                desired_encoding='bgr8'
            )

            board_before = fen_to_board(self.fen_before_move)
            if is_board_flipped_by_kings(board_before):
                self.get_logger().warn("移动前FEN方向为反，自动旋转180度")
                board_before = [row[::-1] for row in board_before[::-1]]

            # ====================================================
            # 第一次：原图完整流程
            # ====================================================
            self.get_logger().info('--- 原图识别移动 ---')
            board_after = self.detect_board_from_image(cv_image_after)

            fen1, move_str = None, None
            if board_after:
                fen1, move_str = compare_and_generate_move_fen(
                    board_before,
                    board_after
                )

            # ====================================================
            # 如果【没有检测到有效移动】，才尝试增强图
            # ====================================================
            if not (fen1 and move_str):
                self.get_logger().warn(
                    '原图未检测到有效移动，尝试增强图重新识别'
                )

                for enhanced in self.enhance_images(cv_image_after):
                    self.get_logger().info('--- 增强图识别移动 ---')
                    board_after = self.detect_board_from_image(enhanced)
                    if not board_after:
                        continue

                    fen1, move_str = compare_and_generate_move_fen(
                        board_before,
                        board_after
                    )

                    if fen1 and move_str:
                        self.get_logger().warn('增强图检测到有效移动')
                        break

            # ====================================================
            # 最终结果判定
            # ====================================================
            if fen1 and move_str:
                result = f"FEN: {fen1}, Move: {move_str}"
                self.get_logger().info(f"检测到移动: {result}")
                msg = String()
                msg.data = result
                self.chess_move_publisher.publish(msg)
            else:
                self.get_logger().info("未检测到有效移动")
                wrong_msg = String()
                wrong_msg.data = '未检测到有效移动'
                self.status_publisher.publish(wrong_msg)

            self.image_after_move = None

        except Exception as e:
            self.get_logger().error(f'处理失败: {e}')
            self.get_logger().error(traceback.format_exc())
            wrong_msg = String()
            wrong_msg.data = f'wrong_image: {e}'
            self.status_publisher.publish(wrong_msg)
            self.fen_before_move = None
            self.image_after_move = None



def main(args=None):
    rclpy.init(args=args)
    node = ChessboardRecognizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
