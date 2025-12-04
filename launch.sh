#!/bin/bash
source install/setup.bash 
python3 src/qt_chess_p/main.py 

############## 调试 ##############
# ros2 launch machinery_chess_bringup machinery_chess_bringup.launch.py serial_port_name:=/dev/MachineryA namespace:=right/

# ros2 launch machinery_chess_bringup machinery_chess_bringup.launch.py serial_port_name:=/dev/MachineryB namespace:=left/

