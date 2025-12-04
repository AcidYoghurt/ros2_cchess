import cv2
import os
import json
import numpy as np

from .helper_4_kpt import BONE_NAMES

class Shape:

    @staticmethod
    def init_from_dict(data: dict):
        shape_ins = Shape(data["label"], data["points"], data["group_id"], data["shape_type"])

        return shape_ins

    def __init__(self, label="", points=None, group_id=1, shape_type=""):
        self.label = label
        self.score = None
        self.points = points
        self.group_id = group_id
        self.description = ""
        self.difficult = False
        self.shape_type = shape_type
        self.flags = {}
        self.attributes = {}
    
    def to_dict(self):
        return {
            "label": self.label,
            "score": self.score,
            "points": self.points,
            "group_id": self.group_id,
            "description": self.description,
            "difficult": self.difficult,
            "shape_type": self.shape_type,
            "flags": self.flags,
            "attributes": self.attributes
        }
    
class KeyPoint(Shape):
    def __init__(self, label="", point_xy=list[float, float], group_id=1):
        # 校验 point_xy 是否为 2 个元素的列表
        if len(point_xy) != 2:
            raise ValueError("point_xy 必须是一个包含 2 个元素的列表")
        super().__init__(label, [point_xy], group_id, "point")
    
class Rectangle(Shape):
    def __init__(self, label="A0", xyxy=list[float, float, float, float], group_id=1):
        
        if len(xyxy) != 4:
            raise ValueError("xyxy 必须是一个包含 4 个元素的列表")
        
        """
        bbox [左上角坐标, 右上角坐标, 右下角坐标, 左下角坐标] [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
        """
        x1, y1, x2, y2 = xyxy

        bbox = [
            [x1, y1],
            [x2, y1],
            [x2, y2],
            [x1, y2]
        ]
        
        super().__init__(label, bbox, group_id, "rectangle")

class Annotation:
    @staticmethod
    def init_from_dict(data: dict):
        """
        从 dict 初始化 Annotation 类
        """

        image_height = data["imageHeight"]
        image_width = data["imageWidth"]

        ann = Annotation(image_path=data["imagePath"], image_width=image_width, image_height=image_height)

        for shape in data["shapes"]:
            if shape["shape_type"] == "rectangle":
                ann.add_shape(Rectangle.init_from_dict(shape))
            elif shape["shape_type"] == "point":
                ann.add_shape(KeyPoint.init_from_dict(shape))

        return ann

    def __init__(self, image_path="", image_width=-1, image_height=-1):
        self.version = "2.4.4"
        self.flags = {}
        self.shapes = []
        self.image_data = None

        self.image_path = image_path
        self.image_height = image_height
        self.image_width = image_width
    
    def add_shape(self, shape: Rectangle | KeyPoint):
        self.shapes.append(shape.to_dict())
    
    def to_dict(self):
        if self.image_path == "":
            raise ValueError("image_path 不能为空")
        if self.image_height == -1 or self.image_width == -1:
            raise ValueError("image_height 和 image_width 不能为 -1")

        return {
            "version": self.version,
            "flags": self.flags,
            "shapes": self.shapes,
            "imagePath": self.image_path,
            "imageData": self.image_data,
            "imageHeight": self.image_height,
            "imageWidth": self.image_width
        }


def save_kpt_4_with_xanything(image_input: np.ndarray, image_ann_path, bbox: list[float, float, float, float], kpt_4: list[tuple[str, float, float]], save_dir: str):
    """
    保存 4 个关键点 和 一个 bbox 到 xanything 的 json 文件
    """
    x1, y1, x2, y2 = bbox
    x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)


    if image_input is None:
        raise ValueError("image_input 不能为 None")
    
    image_height, image_width = image_input.shape[:2]


    # image_ann_path 缺省 .json
    if not image_ann_path.endswith(".json"):
        image_ann_path = image_ann_path + ".json"

    # 读取 image_ann_path 的 文件名
    file_name = os.path.basename(image_ann_path)

    annotation = Annotation(file_name, image_width, image_height)

    kpt_4_dict = {}
    for bone_name, x, y in kpt_4:
        kpt_4_dict[bone_name] = [float(x), float(y)]

    for bone_name in BONE_NAMES:
        x, y = kpt_4_dict[bone_name]
        annotation.add_shape(KeyPoint(bone_name, [x, y]))

    # 添加 bbox
    annotation.add_shape(Rectangle("bbox", [x1, y1, x2, y2]))

    ann_file_path = os.path.join(save_dir, file_name)
    # 保存
    with open(ann_file_path, "w") as f:
        json.dump(annotation.to_dict(), f)

    # 保存图片
    image_input_rgb = image_input.copy()[:, :, ::-1]

    # print('ann_file_path:', ann_file_path.replace(".json", ".jpg"))

    cv2.imwrite(ann_file_path.replace(".json", ".jpg"), image_input_rgb)

def read_xanything_to_json(json_path) -> tuple[list[tuple[str, float, float]], list[float, float, float, float]]:
    """
    读取 xanything 的 json 文件
    """
    data = {}
    with open(json_path, "r") as f:
        data = json.load(f)

    # data 
    annotation = Annotation.init_from_dict(data)

    keypoints_4_dict: dict[str, list[float, float]] = {}
    # x1, y1, x2, y2
    bbox: list[float, float, float, float] = [] 

    for shape in annotation.shapes:
        if shape["shape_type"] == "point":
            keypoints_4_dict[shape["label"]] = [shape["points"][0][0], shape["points"][0][1]]
        elif shape["shape_type"] == "rectangle":
            bbox = [shape["points"][0][0], shape["points"][0][1], shape["points"][2][0], shape["points"][2][1]]

    keypoints_4: list[tuple[str, float, float]] = []

    for item in BONE_NAMES:
        keypoints_4.append((item, keypoints_4_dict[item][0], keypoints_4_dict[item][1]))

    return keypoints_4, bbox








