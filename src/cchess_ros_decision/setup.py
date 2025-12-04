from setuptools import setup, find_packages
import os
from glob import glob
from ament_index_python.packages import get_package_share_directory

package_name = 'cchess_ros_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 确保ONNX模型文件被安装到正确的位置
        (os.path.join('share', package_name, 'onnx', 'pose'), glob('resource/onnx/pose/*.onnx')),
        (os.path.join('share', package_name, 'onnx', 'layout_recognition'), glob('resource/onnx/layout_recognition/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nameless',
    maintainer_email='2654944298@qq.com',
    description='A ROS 2 package for Chinese chess.',
    license='Apache-2.0',
    tests_require=['pytest'],
     entry_points={
        'console_scripts': [
            'chessboard_recognizer_node = cchess_ros_decision.chessboard_recognizer_node:main',
            'chess_engine_node = cchess_ros_decision.chess_engine_node:main', 
            'chess_camera_publisher = cchess_ros_decision.chess_camera:main',
            'rob_engine_node = cchess_ros_decision.rob_engine_node:main'
        ],
    },
)
