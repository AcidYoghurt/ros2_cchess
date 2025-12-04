# 环境配置说明
* 该程序使用了onnx网络模型用于识别棋盘局势，并且通过中国象棋库python-chinese-chess-main进行虚拟棋盘管理与走步合法性检测。

## 安装环境
- ROS2：Humble
- Ubuntu：22.04LTS 

## 安装依赖
- 首先在系统安装中国象棋库：[中国象棋库——github仓库](https://github.com/windshadow233/python-chinese-chess.git)
- 然后在ros2_cchess目录下打开**终端**并输入下面指令：
```bash
# 安装ros2依赖
pip install -r requirements.txt

# 安装ros2依赖
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 安装系统依赖
sudo apt update
sudo apt install libserial-dev
```

# 程序使用说明

## 程序注意事项

* **请勿直接通过右上角的“X”按钮关闭程序**，这可能导致后台节点继续运行，存在机械臂发生碰撞的风险。
* 如果程序崩溃或不小心关闭，请重新启动电脑，或者检查后台节点是否仍在运行，确保没有活跃节点。
* **机械臂停在原点**，但模式已选择时，可能是串口未正确连接，请检查串口连接状态。
* **正确退出程序**：在机械臂运行时，退出程序前应先暂停游戏，然后再结束游戏。

## 机机对战模式

* **RA 为红方，LB 为黑方**。如果 RA 错误地作为黑方，并且两个机械臂的位置差异较大，可能是串口连接错误，尝试交换串口连接。
* 在机机对战模式下，相机不参与棋盘状态检测，程序完全依赖于编码预设，因此请确保棋子的摆放准确，并尽量保证每个棋子位于中间。

## 人机对战模式

### 常见问题及解决方案

1. **未反馈“我移动了棋子”**：

   * 如果在输入“我移动了棋子”后没有任何反馈（调试信息窗口没有消息），很可能是相机设备未正确连接。尝试重新插拔相机，或者通过 `ls /dev/video*` 确认相机是否为 `video0`。如果不是，修改 `src/cchess_ros/cchess_ros/chess_camera.py` 文件第60行的设备序号。

2. **调试窗口显示“无效移动”**：

   * **棋子识别错误**：如车被误识别为卒，马被误识别为象等，尝试将调试窗口中的“无效移动：”后面的内容复制到 [象棋AI网站](https://xiangqiai.com/#/) 进行比对。通过调整错误识别的棋子的角度来解决识别问题。
   * **移动多个棋子**：检查可视化UI中的棋盘与实际棋盘是否一致，确保未不小心移动多个棋子。

3. **弹窗提示“非法移动”**：

   * 说明您所走的棋步不合规，例如马不走日、象不走田，或者没有破局的情况下被将军。

4. **出现“无效移动：”信息，且包含“x”**：

   * 可能是棋盘被遮挡，确保相机视野内的棋盘没有遮挡。

5. **无法解决问题时**：

   * 如果无法通过调试解决问题，可以尝试手动输入棋子的移动。格式为：`h2e2`（棋盘原点位于RA方向的一角，横向为a-i，纵向为0-9）。

## 调试命令

1. 启动右侧机械臂：

   ```bash
   ros2 launch machinery_chess_bringup machinery_chess_bringup.launch.py serial_port_name:=/dev/MachineryA namespace:=right/
   ```

2. 启动左侧机械臂：

   ```bash
   ros2 launch machinery_chess_bringup machinery_chess_bringup.launch.py serial_port_name:=/dev/MachineryB namespace:=left/
   ```


