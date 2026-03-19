# base_om.py
import sys
import os

# 添加Miniconda的site-packages路径
conda_site_packages = '/usr/local/miniconda3/lib/python3.9/site-packages'
if conda_site_packages not in sys.path:
    sys.path.insert(0, conda_site_packages)

# 添加Miniconda的库路径（解决动态链接库依赖）
conda_lib_path = '/usr/local/miniconda3/lib'
os.environ['LD_LIBRARY_PATH'] = f"{conda_lib_path}:{os.environ.get('LD_LIBRARY_PATH', '')}"

import os
from typing import Tuple, List, Union, Any
import numpy as np
import cv2
from abc import ABC, abstractmethod

# 添加Ascend Python包路径
ascend_path = '/usr/local/Ascend/ascend-toolkit/latest/python/site-packages'
if ascend_path not in sys.path:
    sys.path.insert(0, ascend_path)

from ais_bench.infer.interface import InferSession


class BaseOM(ABC):
    def __init__(self, model_path: str, input_size: Tuple[int, int], device_id: int = 0):
        """初始化OM模型基类

        Args:
            model_path (str): OM模型路径
            input_size (tuple): 模型输入尺寸 (width, height)
            device_id (int): NPU设备ID，默认为0
        """
        self.device_id = device_id
        self.input_size = input_size
        self.session = None
        self.input_info = None
        self.output_info = None

        ret = self._load_model(model_path)
        if ret != 0:
            raise RuntimeError(f"Failed to load model: {model_path}")

    def _load_model(self, model_path: str) -> int:
        """加载OM模型"""
        print(f"尝试加载模型: {model_path}")

        # 检查文件是否存在
        if not os.path.exists(model_path):
            print(f"错误: 模型文件不存在: {model_path}")
            return -1

        # 检查文件是否可读
        if not os.access(model_path, os.R_OK):
            print(f"错误: 无法读取模型文件: {model_path}")
            return -1

        # 加载模型
        try:
            self.session = InferSession(model_path=model_path, device_id=self.device_id)
            print("模型加载成功")

            # 获取输入输出信息
            self.input_info = self.session.get_inputs()
            self.output_info = self.session.get_outputs()

            print("模型输入信息:")
            for i, input_tensor in enumerate(self.input_info):
                print(f"  Input {i}:")
                print(f"    Name: {input_tensor.name}")
                print(f"    Shape: {input_tensor.shape}")
                print(f"    Data type: {input_tensor.datatype}")
                print(f"    Size: {input_tensor.size}")

            print("模型输出信息:")
            for i, output_tensor in enumerate(self.output_info):
                print(f"  Output {i}:")
                print(f"    Name: {output_tensor.name}")
                print(f"    Shape: {output_tensor.shape}")
                print(f"    Data type: {output_tensor.datatype}")
                print(f"    Size: {output_tensor.size}")

            return 0

        except Exception as e:
            print(f"加载模型时发生异常: {e}")
            import traceback
            traceback.print_exc()
            return -1

    def run_inference(self, image: np.ndarray) -> List[np.ndarray]:
        """运行推理

        Args:
            image (np.ndarray): 输入图像数据，形状应与模型输入匹配

        Returns:
            List[np.ndarray]: 推理结果列表
        """
        if self.session is None:
            raise RuntimeError("Model not loaded")

        # 确保输入数据形状正确
        expected_shape = tuple(self.input_info[0].shape)
        if image.shape != expected_shape:
            raise ValueError(f"Input shape mismatch. Expected {expected_shape}, got {image.shape}")

        # 进行推理
        outputs = self.session.infer([image])
        return outputs

    @abstractmethod
    def preprocess_image(self, img_bgr: cv2.UMat, *args, **kwargs) -> np.ndarray:
        """图像预处理抽象方法"""
        pass

    @abstractmethod
    def pred(self, image: Union[cv2.UMat, str], *args, **kwargs) -> Any:
        """预测的抽象方法"""
        pass

    @abstractmethod
    def draw_pred(self, img: cv2.UMat, *args, **kwargs) -> cv2.UMat:
        """绘制预测结果的抽象方法"""
        pass

    def __del__(self):
        """析构函数，确保资源清理"""
        # InferSession会自动清理资源，不需要手动清理
        pass