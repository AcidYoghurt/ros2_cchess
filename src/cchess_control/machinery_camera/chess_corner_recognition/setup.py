from setuptools import find_packages, setup
import glob

package_name = 'chess_corner_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
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
            'chess_corner_recognition_node = chess_corner_recognition.chess_corner_recognition_node:main',
            'artificial_chess_corner_recognition_node = chess_corner_recognition.artificial_chess_corner_recognition_node:main'
        ],
    },
)
