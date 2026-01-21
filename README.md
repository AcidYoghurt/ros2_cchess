# ROS 2 中国象棋机器人 (ros2_cchess) 项目文档

> [!WARNING]
>
> 如果想要修改或对本项目二次开发，请先学习ros2与python有关知识

## 1. 项目简介

本项目 ros2_cchess 是一个基于 ROS 2 开发的中国象棋对弈机器人系统。它集成了**机器视觉**（识别棋盘与棋子）、**博弈算法**（象棋AI引擎）和**机械臂控制**（执行落子）三个核心模块。

本项目旨在提供一个模块化、易于调试的机器人对弈开发框架，适合用于 ROS 2 学习、算法验证及机械臂应用开发。

- 项目结构：

  ```bash
  ros2_cchess项目结构
  ├── cchess_ros_control	# 执行与感知层
  │   ├── machinery_camera	# 视觉识别【未使用】
  │   │   ├── chess_corner_recognition	# 棋盘角点识别
  │   │   └── pub_camera	# 启动相机
  │   ├── machinery_chess_ros_control_bringup	# 测试执行与感知层
  │   ├── machinery_model	# 机器模型【未使用】
  │   └── machinery_ros2control	# 硬件控制，使用了ros2_control框架
  │       ├── machinery_control	# 通过决策层发出的指令来操控机械臂
  │       ├── machinery_controller	# ros2_control中的controller
  │       ├── machinery_hardware_interface	# ros2_control中的hardware_interface
  │       ├── machinery_keyboard_control	# 键盘操操控机械臂
  │       └── machinery_ros2control_bringup	# 测试machinery_ros2control模块
  ├── cchess_ros		# 决策层
  │       ├── resource	# 象棋引擎pikafish与象棋识别模型onnx
  │       ├── cchess_ros	# 相关决策节点
  │       ├── chess_camera.py	# 实际相机控制节点
  │       ├── chessboard_recognizer_node.py	# 象棋与棋盘识别节点
  │		├── chess_engine_node.py	# 象棋主引擎（全模式都会使用的决策节点）
  │       └── rob_engine_node.py	# 象棋副引擎（只有启动两台机械臂时用于控制第二台机械臂）
  ├── chess_qt	# 应用层，负责与用户交互，显示棋盘状态
  │   ├── qt_chess_p	# 中国象棋对弈系统主程序
  │   └── qt_point	# 中国象棋点位配置主程序
  └── ros2_chess_bringup	# 集成层，负责统筹全局，一键启动所有节点
  ```



## 2. 环境搭建

### 2.1 系统要求

- **操作系统**: Ubuntu 22.04LTS
- **语言**: Python 3.10+, C++

### 2.2 ROS2安装

使用小鱼ROS一键安装：

```bas
wget http://fishros.com/install -O fishros && . fishros
```

> [!WARNING]
>
> ROS2一定要选择humble版本

### 2.3 依赖安装

首先安装编译项目前所需依赖

```bash	
# 1. 安装系统级依赖 (OpenCV, 串口工具等)
sudo apt update
sudo apt install git python3-pip ros-humble-cv-bridge ros-humble-image-transport

# 2. 安装 Python 依赖
pip3 install opencv-python numpy pyserial
```

### 2.3 编译项目

> [!NOTE]
>
> 在执行rosdep相关指令时，如果出现Error，忽略不计

```bash
# 克隆本项目
git clone https://github.com/AcidYoghurt/ros2_cchess.git

# 返回工作空间根目录
cd cchess_ws

# 安装ros2依赖
rosdep init
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 安装python依赖
pip install onnxruntime PySide6
```



## 3. 代码框架

```Mermaid
graph LR
    A[摄像头&虚拟棋盘] -->|图像数据 /camera/image_raw & 棋盘数据 /fen_topic| B(棋盘管理节点 Vision_Node)
    B -->|棋盘状态 FEN串| C(决策节点 Logic_Node)
    C -->|下一步落子指令| D(运动控制节点 Motion_Node)
    D -->|串口/驱动指令| E[机械臂硬件]
```

- **Topic (话题)**: 用于各模块间的高频数据传输（如图像、关节角度）。
- **Service (服务)**: 用于请求/响应式操作（如“请求计算下一步”、“请求复位”）。
- **Action (动作)**: 用于耗时的机械臂运动控制。



## 4. 核心模块说明

为了方便理解和二次开发，代码主要分为以下四个功能包/模块：

### 4.1 用户感知层

> - **功能**：负责展示可视化和可操作页面
> - **核心功能包**：```chess_qt```与```ros2_chess_bringup```
>
> - **启动逻辑**：QT线程管理全局ros2界面启动文件（即ros2_chess_bringup里面的launch文件），通过启动QT游戏界面的各种模式来启动不同的launch文件

#### 4.1.1 launch文件

- ```machinery_chess_bringup.launch.py```：启动控制层节点

- ```cchess_rob.launch.py```：启动决策层中的机机对弈节点
- ```cchess_person.launch.py```：启动决策层中的人机对弈节点

#### 4.1.2 配置文件

- 在**chess_qt/qt_chess_p/config**文件夹中有一个**config.yaml**
- 这是用于在不同游戏模式下配置**不同的launch文件**



### 4.2 视觉模块

> - **功能**: 负责从摄像头读取图像，进行图像矫正、棋盘定位和棋子识别。
> - **核心代码**: chess_camera.py
> - **关键逻辑**:高斯滤波去噪。霍夫变换 识别圆形棋子。透视变换 将斜视角的棋盘拉正。

#### 4.2.1 通信接口

该节点起到承上启下的作用，连接硬件相机与图像处理算法：

- **订阅者 (Input Triggers)**:machinery_pub_image_trigger (Bool): 接收到 True 时，延时1秒（等待机械臂稳定）拍摄“移动前”的棋盘状态。person_pub_image_trigger (Bool): 接收到 True 时，立即拍摄“移动后”的棋盘状态。
- **发布者 (Output Data)**:image_before_move (Image): 发布移动前的原始图像数据。image_after_move (Image): 发布移动后的原始图像数据。status_topic (Image/String): 发布相机状态或报错信息。

#### 4.2.2 关键逻辑实现

1. **多线程图像采集 (Threading)**:为了防止 OpenCV 的 camera.read() 阻塞 ROS 2 的主循环（导致无法及时响应回调），代码中开启了一个守护线程 camera_capture_thread 持续在后台读取最新的视频帧。**优势**：保证了当触发信号到来时，系统能立即获取到内存中最新的画面，而不是一张积压在缓冲区的老旧图片。
2. **OpenCV与ROS格式转换**:使用 cv_bridge 库将 OpenCV 的 numpy.ndarray 格式转换为 ROS 标准的 sensor_msgs/msg/Image 消息，编码格式为 bgr8。

#### 4.2.3 调试与参数修改

在 chess_camera.py 中，最常需要修改的是**摄像头设备索引**。

- **问题描述**：如果启动节点后报错“无法打开摄像头”，通常是因为电脑连接了多个摄像头（如笔记本自带相机+USB相机）。

- **修改位置**：找到 start_camera 函数：

  ```python
  def start_camera(self):
      # ...
      try:
          # 0通常是笔记本自带相机或第一个USB设备
          # 如果使用的是外接USB相机，可能需要改为 1 或 2
          self.camera = cv2.VideoCapture(0) 
      # ...
  ```

- **图像延时调试**：
  如果发现拍摄的画面机械臂还在抖动，可以在 machinery_trigger_callback 中调整 time.sleep(1) 的时间长度。

  

### 4.3 决策模块

> - **功能**: 该模块是机器人的“大脑”，负责连接感知与执行。它不仅负责下棋算法（AI），还负责将视觉识别到的图像信息转化为具体的棋步逻辑（FEN串与UCI指令）。
> - **核心功能包**: cchess_ros_decision

决策模块由三个核心节点组成，分别处理**状态识别**、**人机对弈逻辑**和**双臂对弈逻辑**。

#### 4.3.1 状态推演节点 (chessboard_recognizer_node.py)

该节点是连接“视觉图像”与“游戏逻辑”的桥梁。它不直接控制机械臂，而是负责告诉系统：“刚才发生了什么”。

- **工作流程**:**输入**: 订阅 image_after_move (移动后的照片) 和 fen_topic (移动前的棋局状态)。**处理**:调用 **ONNX 模型** (RTMPOSE) 识别棋盘的四个角点和关键点。执行**透视变换**，将倾斜的棋盘图片拉正。调用 **分类模型** (Full Classifier) 识别每一个格子上是什么棋子。**核心逻辑**: 将识别出的新盘面与上一轮的 FEN 串进行对比，通过差异反推出刚才走了哪一步（例如检测到 h2 变空，e2 多了一个炮，推导出 h2e2）。**输出**: 向 chess_move_topic 发布走法字符串（如 fen_str, h2e2）。
- **关键依赖**:该节点依赖 onnx/pose/ 和 onnx/layout_recognition/ 下的模型文件，请确保路径正确。**自动纠偏**: 代码中包含 is_board_flipped_by_kings 逻辑，会自动根据“将/帅”的位置判断棋盘是否被放反了（倒置），并自动旋转数据。

#### 4.3.2 象棋AI核心节点

本项目支持两种对弈模式，由不同的节点控制：

**A. 人机对弈 (chess_engine_node.py)**

- **场景**: 只有一台机械臂（Right），人类在真实棋盘上走棋，机械臂应答。
- **逻辑**:接收 chess_move_topic（人类的走法）。验证走法是否合规（Legal Move）。调用 **Pikafish** 引擎计算最佳应手。发布 ```/right/ai_move_topic``` 指挥机械臂落子。检测“将军”或“将死”状态并通报 UI。

**B. 双机/左右互博 (rob_engine_node.py)**

- **场景**: 两台机械臂（Left 和 Right）互相下棋。
- **特殊机制 (同步锁)**:为了防止两台机械臂撞车或在未归位时拍照，该节点引入了**触发器机制**。它会缓存引擎的计算结果（pending_engine_move），只有当收到硬件层发的 machinery_pub_image_trigger (机械臂已归位静止) 信号后，才会发布下一步动作指令。

**C. 语音人机模式**

- 复用了人机对弈模块，只是人方面用的是语音控制

#### 4.3.3 底层引擎与算法 (reg_core)

- **UCI 引擎 (engine.py)**:本项目封装了开源强大的象棋引擎 **Pikafish**。**权限提示**: 如果运行报错 Permission denied，通常是因为 resource/pikafish 二进制文件没有执行权限。代码中已有 os.chmod 尝试修复，但建议手动检查：codeBash`chmod +x src/cchess_ros_decision/resource/pikafish`
- **AI 难度配置**:可以通过向 uci_topic 发送 JSON 指令动态调整难度，支持：think_time: 思考时间（秒）。elo: 限制 ELO 分数（如 1500, 2000）。skill_level: 限制层级（0-20）。

#### 4.3.4 调试指南

如果你发现机器人**“看着棋盘发呆，不走棋”**，请按以下顺序排查：

1. **检查模型路径**: 确认 chessboard_recognizer_node 启动日志是否显示 加载模型: ...onnx 成功。
2. **检查 FEN 状态**: 使用 ros2 topic echo /fen_topic。如果 FEN 串和当前棋盘不一致，AI 会认为你还没走完或走了非法步，导致卡住。
3. **检查引擎**: 确认系统中安装了 pikafish 或项目路径下有该文件。
4. **光照影响**: 视觉识别对反光敏感，若识别出的棋子类别错误（如把“兵”认成“炮”），会导致 FEN 推演失败。请尝试调整环境光或使用 rqt 查看识别的置信度。



### 4.4 运动控制模块 

> - **功能**: 将逻辑坐标转换为机械臂的物理坐标，并进行逆运动学解算。
> - **核心功能包**: cchess_ros_control

在该模块中：

- 每一层中如果有bringup文件夹，则该bringup文件夹只专门针对该层及子层；
- 如果没有就是没写

#### 4.4.1 bring_up文件夹

- ```machinery_chess_ros_control_bringup```：启动控制层节点

- ```machinery_ros2control_bringup```：启动ros2_control相关节点

#### 4.4.2 配置文件

在```machinery_chess_ros_control_bringup```和```machinery_ros2control_bringup```中，因为运动控制模块采用域名设计，所以每一个域名对应一个实体机械臂，每一个域名对应一个机械臂配置。如：

```bash
config
└── machinery
    ├── left
    └── right
```

- ```left```：对应 **left/** 域名，里面的配置文件都是配置 **left/** 域名的机械臂
- ```right```：对应 **right/** 域名，里面的配置文件都是配置 **right/** 域名的机械臂



**配置文件介绍**：

- ```machinery_controllers.yaml```：该文件是ros2_control中的controller配置，属于machinery_ros2control模块

- ```machinery.yaml```：该文件是机械臂的**初始点**和**自定义原点配置**

  - **初始点**：

    <img src="assets/6970841cbcf1c.png" alt="初始点.png" style="zoom: 25%;" />

  - **自定义原点（防止碰撞）**：

    <img src="assets/6970841d6be30.png" alt="自定义原点.png" style="zoom:25%;" />

- ```points_to_move.yaml```：该文件是专门为了machinery_keyboard_control中的machinery_keypoint_debug_node.py调试用，规定要走什么点位



### 4.4 决策模块二次开发说明

本系统的二次开发工作主要集中在**决策模块（Decision Layer）**。该模块负责象棋对弈逻辑的计算，并通过 ROS 话题与机械臂运动层进行通信，实现“走棋决策 → 机械臂执行”的完整闭环。

#### 4.4.1 决策模块与运动层接口说明

决策模块通过话题接口向运动层下发机械臂的移动指令，当前系统定义的话题如下：

- `/right/ai_move_topic`：左机械臂控制话题  
- `/left/ai_move_topic`：右机械臂控制话题  

话题消息格式为：(g3,o0)

[^消息解释]: （表示机械臂从棋盘坐标 **g3** 移动至 **o0**。）



您可以在该模块中自由修改或扩展以下内容：

- 棋子移动目标点位的计算逻辑  
- 机械臂执行顺序或策略  
- 多步决策、特殊规则处理等高级逻辑  

只需保证最终输出的目标点位格式符合上述约定，即可与运动层无缝对接。

#### 4.4.2 象棋引擎说明与替换方式

当前系统默认采用 **Pikafish** 作为中国象棋引擎：

- 官方网站：https://www.pikafish.com  

系统通过 **标准 UCI（Universal Chess Interface）格式** 与引擎进行通信，用于获取走步决策、局面评估等信息。

如您需要接入自研引擎或其他象棋算法，请注意：

- 决策模块仅依赖 **标准 UCI 输出格式**
- 只要引擎能够正确输出 UCI 走法，即可直接替换现有引擎
- 无需对机械臂控制或运动层代码进行修改

#### 4.4.3 中国象棋规则与棋局管理库

系统内部使用 Python 中国象棋库完成以下功能：

- 虚拟棋局创建与维护  
- 中国象棋规则判断  
- 棋局状态管理与输出  
- 走法合法性校验  
- 棋盘状态可视化  

该库的开源地址为：  
https://github.com/windshadow233/python-chinese-chess

在进行二次开发时，您可以基于该库扩展更多功能，例如自定义规则、调试模式或棋局数据记录等。

#### 4.4.4 视觉识别系统与走法输出接口

本系统当前采用的视觉识别方案为：

- **棋盘四角定位 + 棋子分类识别**
- 不直接输出 YOLO 风格的像素坐标
- 视觉模型仅识别并记录棋盘上每一个“标准点位”的棋子状态

该方案本质上等价于“识别棋盘状态”，而非“检测像素位置”。

所使用的中国象棋识别库开源地址为：  
https://github.com/TheOne1006/chinese-chess-recognition

在此基础上，系统对原始开源代码进行了适度修改，新增了：

- 前后棋局状态对比  
- 自动判断人类或机械臂的走棋步骤  
- 走法生成与校验逻辑  

#### 4.4.5 自定义视觉系统接入说明

如您希望将视觉识别系统替换为自研方案（例如基于 YOLO 的检测模型），只需满足以下条件：

- 最终通过接口话题 `chess_move_topic`

- 输出 **标准 UCI 走法字符串**，例如：g3g4 

  [^消息解释]: （表示棋子从 **g3** 移动到 **g4**。）

  只要满足该接口约定，无需关心内部识别方式，系统即可正确完成后续决策与机械臂执行流程。
