#!/bin/bash

unset http_proxy
unset https_proxy
unset all_proxy
unset HTTP_PROXY
unset HTTPS_PROXY
unset ALL_PROXY

conda activate ros2_cchess
source /opt/ros/humble/setup.bash
source install/setup.bash 
python3 src/chess_qt/qt_chess_p/main.py 

############## 调试 ##############
# ros2 launch machinery_chess_bringup machinery_chess_bringup.launch.py serial_port_name:=/dev/machineryLeftA namespace:=right/

# ros2 launch machinery_chess_bringup machinery_chess_bringup.launch.py serial_port_name:=/dev/machineryRightB namespace:=left/

