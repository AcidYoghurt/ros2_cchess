# ROS2指擎机械臂控制层

**模块结构**：

```bash
machinery_ros2control
├── machinery_control	# 通过决策层发出的指令来操控机械臂
├── machinery_controller	# ros2_control中的controller
├── machinery_hardware_interface	# ros2_control中的hardware_interface
├── machinery_keyboard_control	# 键盘操操控机械臂
└── machinery_ros2control_bringup	# 测试machinery_ros2control模块
```

---

**这是ros2_cchess的控制层，里面包括了**：

- ros2_control相关：

  - ```machinery_controller```：该项目所需要的controller，里面包括了 ```笛卡尔积控制器(cartesian_move_controller) ```和 ```吸嘴控制器(gripper_suction_controller)```，这些控制器都实现了：
    - 上层软件 发送指令到 controller
    - controller 反馈状态到 上层软件 
    - controller 发送指令到 hardware_interface
    - hardware_interface 反馈状态到 controller

  - ```machinery_hardware_interface```：为指擎机械臂做的hardware_interface，里面包括吸嘴控制、笛卡尔积坐标控制、串口通讯，实现了：
    - 接收来自 controller 的指令
    - 向串口发送消息
    - 接收来自串口的反馈信息
    - 发送反馈信息给controller

- 人机交互部分```machinery_keyboard_control```：

  - ```machinery_keyboard_control_node.py```：使用键盘操控机械臂吸嘴的笛卡尔积位置，W前，S后，A左，D右，Q上，E下。会时时显示目前吸嘴的坐标

  - ```machinery_keypoint_debug_node.py```：使用键盘操控机械臂吸嘴，运动到关键点，搭配config文件使用：

    ```yaml
    /**:
      # 该config文件只针对棋盘关键点检测 machinery_keypoint_debug.launch.py
      ros__parameters:
        points_to_move: 'all' # all表示走全部点
        # points_to_move: "['a0','a9','i0','i9','i8']"  # 你也可以选择要走的点位
    ```

- 启动部分```machinery_ros2control_bringup```：
  - **config文件夹**：因为采用了namespace设计，所以一个namespace对应一个文件夹，如```left/```对于```config/left```
  - **urdf文件夹**：用于配置ros2_control相关参数，主要是搭配hardware_interface使用
  - **launch文件夹**：
    - ```machinery_keyboard.launch.py```：用键盘操控机械臂自由上下左右移动
    - ```machinery_keypoint_debug.launch.py```：测试机械臂是否能够到达棋盘点位
    - ```machinery_ros2control.launch.py```：测试ros2_control模块，即controller和hardware_interface
