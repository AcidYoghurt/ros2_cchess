import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'machinery_keyboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amiya',
    maintainer_email='1310946137@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'machinery_keyboard_control_node = machinery_keyboard_control.machinery_keyboard_control_node:main',
            'machinery_keypoint_debug_node = machinery_keyboard_control.machinery_keypoint_debug_node:main'
        ],
    },
)
