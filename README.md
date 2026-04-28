# Apple_robot_xjtu

## 项目概述

这是一个基于双臂Rokae机器人的智能苹果采摘系统，集成了深度学习目标检测、双臂协调控制、SLAM导航和Qt图形界面等多种技术。系统通过视觉识别苹果位置，自动分配采摘任务给左右双臂，实现高效、安全的自动化采摘作业。

## 系统功能

### 1. 智能视觉检测功能

**功能描述：**
- 使用YOLOv8 ONNX深度学习模型进行苹果目标检测
- 结合ZED双目相机获取RGB图像和深度点云数据
- 实现2D图像检测与3D空间坐标的精确融合
- 实时发布检测结果和可视化图像

**技术特点：**
- 支持多目标同时检测
- 自动计算苹果的3D空间坐标（x, y, z）
- 检测结果实时可视化显示
- 点云数据融合，提高定位精度

**核心文件：**
- `img_detect/src/apple_detect.cpp` - 苹果检测主程序
- `img_detect/src/yolov8_onnx.cpp` - YOLOv8模型推理
- `img_detect/src/yolov8_utils.cpp` - 工具函数

**使用方法：**
```bash
# 启动视觉检测节点
rosrun img_detect apple_detect
```

**发布话题：**
- `/ap_robot/chatter_appleInfo` - 苹果检测信息（包含ID、坐标等）
- `/ap_robot/chatter_detectImg` - 检测结果可视化图像

### 2. 智能任务分配功能

**功能描述：**
- 根据苹果的空间位置自动分配采摘任务
- 实现双臂并行工作，提高采摘效率
- 智能调度算法，优化采摘顺序

**分配策略：**
- **空间分区：** x < 0 的苹果分配给左臂，x >= 0 的苹果分配给右臂
- **深度优先：** 每个区域内的苹果按z坐标（深度）从近到远排序，优先采摘最近的
- **并行执行：** 左右双臂可同时工作，互不干扰

**核心文件：**
- `task_assign/src/target_assign.cpp` - 任务分配主程序

**使用方法：**
```bash
# 启动任务分配节点
rosrun task_assign target_assign
```

**服务接口：**
- `/left_pickTarget` - 左臂采摘服务
- `/right_pickTarget` - 右臂采摘服务

### 3. 双臂机器人控制功能

**功能描述：**
- 控制左右两个Rokae xMate SR4机器人
- 实现复杂的苹果采摘动作序列
- 包含碰撞检测和安全保护机制
- 支持多种运动模式和速度控制

**控制特点：**
- **双臂独立控制：** 左右臂可独立执行任务
- **协调采摘：** 支持双臂协同工作
- **安全保护：** 多层碰撞检测，防止机器人碰撞
- **精确控制：** 支持关节空间和笛卡尔空间控制

**采摘动作流程：**
1. 移动到苹果上方预备位置
2. 缓慢下降到抓取位置
3. 控制夹爪抓取苹果
4. 抬起并移动到放置位置
5. 释放苹果
6. 返回初始位置

**核心文件：**
- `rokae/src/robot_control_node.cpp` - 机器人控制主节点
- `rokae/src/demoboth.cpp` - 双臂协同演示
- `rokae/src/leftRokae.cpp` - 左臂控制
- `rokae/src/rightRokae.cpp` - 右臂控制

**使用方法：**
```bash
# 启动机器人控制节点
rosrun rokae robot_control_node
```

### 4. Qt图形界面功能

**功能描述：**
- 提供直观的图形用户界面
- 集成RViz可视化显示
- 支持一键启动和控制系统各模块
- 实时显示系统状态和运行数据

**界面功能模块：**

#### 4.1 导航控制模块
- **启动SLAM建图：** 启动gmapping进行地图构建
- **启动导航：** 启动move_base进行路径规划
- **启动相机：** 启动ZED相机驱动
- **启动激光雷达：** 启动Livox激光雷达
- **数据录制：** 录制rosbag数据
- **停止所有模块：** 一键停止所有导航相关模块

#### 4.2 采摘控制模块
- **一键启动采摘系统：** 自动启动视觉检测、任务分配、机器人控制等所有采摘相关模块
- **紧急停止：** 立即停止所有采摘动作，确保安全
- **左臂归位：** 控制左臂返回初始位置
- **右臂归位：** 控制右臂返回初始位置

#### 4.3 标定模块
- **眼在手上标定：** 相机安装在机械臂末端的手眼标定
- **眼在手外标定：** 相机固定在工作空间的手眼标定
- **可视化预览：** 实时显示标定画面
- **交互式确认：** 用户可接受或跳过每个标定姿态

#### 4.4 实时显示模块
- **RViz集成：** 嵌入式RViz显示机器人状态、地图、点云等
- **视觉检测显示：** 实时显示苹果检测结果
- **苹果坐标表格：** 显示检测到的苹果ID和3D坐标
- **系统日志：** 实时显示系统运行日志
- **状态指示灯：** 显示各模块运行状态

**核心文件：**
- `qt_ros_nav/src/mainwindow.cpp` - 主窗口界面
- `qt_ros_nav/src/ros_node.cpp` - ROS节点通信
- `qt_ros_nav/src/main.cpp` - 程序入口

**使用方法：**
```bash
# 启动Qt图形界面
rosrun qt_ros_nav qt_ros_nav
```

### 5. 手眼标定功能

**功能描述：**
- 支持两种标定模式：眼在手上和眼在手外
- 自动化标定流程，提高标定效率
- 提供可视化界面，便于用户确认
- 支持标定结果保存和加载

**标定模式：**

#### 5.1 眼在手上标定
- **适用场景：** 相机安装在机械臂末端
- **标定对象：** 相机坐标系到机械臂末端坐标系的变换
- **标定流程：**
  1. 选择要标定的机械臂（左臂或右臂）
  2. 点击"启动眼在手上"按钮
  3. 机械臂自动移动到多个标定姿态
  4. 每个姿态显示标定画面
  5. 用户确认或跳过当前姿态
  6. 完成所有姿态后计算标定结果

#### 5.2 眼在手外标定
- **适用场景：** 相机固定在工作空间
- **标定对象：** 相机坐标系到机器人基座坐标系的变换
- **标定流程：**
  1. 选择要标定的机械臂
  2. 点击"启动眼在手外"按钮
  3. 机械臂移动到多个标定位置
  4. 相机拍摄标定板图像
  5. 完成标定计算

**核心文件：**
- `auto_calibrate_eye_in_hand.py` - 眼在手上标定脚本
- `auto_calibrate_eye_to_hand.py` - 眼在手外标定脚本

**使用方法：**
```bash
# 眼在手上标定
python auto_calibrate_eye_in_hand.py

# 眼在手外标定
python auto_calibrate_eye_to_hand.py
```

### 6. 通信功能

**功能描述：**
- 处理串口通信协议
- 实现上位机与下位机之间的数据交换
- 支持多种通信模式

**核心文件：**
- `comm/src/serial.cpp` - 串口通信实现

### 7. 点云发布功能

**功能描述：**
- 发布点云数据用于3D感知
- 支持多种点云格式

**核心文件：**
- `pub_pcd/src/pcd_publisher.cpp` - 点云发布节点

## 完整使用流程

### 方式一：通过Qt图形界面操作（推荐）

#### 1. 启动系统
```bash
# 确保roscore已运行
roscore

# 在另一个终端启动Qt界面
rosrun qt_ros_nav qt_ros_nav
```

#### 2. 导航模块操作
1. 点击"启动LIO"按钮启动激光雷达惯性里程计
2. 点击"启动Fusion"按钮启动传感器融合
3. 点击"启动相机"按钮启动ZED相机
4. 点击"启动激光雷达"按钮启动Livox激光雷达
5. 观察状态指示灯，确认各模块正常运行

#### 3. 采摘模块操作
1. 点击"一键启动采摘系统"按钮
2. 系统自动启动以下模块：
   - 视觉检测节点（apple_detect）
   - 任务分配节点（target_assign）
   - 机器人控制节点（robot_control_node）
3. 观察实时显示的苹果检测结果
4. 查看苹果坐标表格中的检测数据
5. 系统自动分配采摘任务给双臂
6. 观察双臂执行采摘动作

#### 4. 紧急情况处理
- 如遇紧急情况，点击"紧急停止"按钮
- 系统立即停止所有采摘动作
- 确认安全后可重新启动采摘系统

#### 5. 机械臂归位
- 点击"左臂归位"按钮控制左臂返回初始位置
- 点击"右臂归位"按钮控制右臂返回初始位置

#### 6. 手眼标定操作
1. 切换到"标定"标签页
2. 选择要标定的机械臂（左臂或右臂）
3. 根据需求选择标定模式：
   - 点击"启动眼在手上"进行眼在手上标定
   - 点击"启动眼在手外"进行眼在手外标定
4. 观察标定画面预览
5. 对每个标定姿态：
   - 确认画面正确后点击"接受当前姿态"
   - 如有问题可点击"跳过当前姿态"
6. 完成所有姿态后系统自动计算标定结果
7. 如需中断，点击"终止标定"按钮

### 方式二：命令行手动启动

#### 1. 启动ROS核心
```bash
roscore
```

#### 2. 启动视觉检测
```bash
rosrun img_detect apple_detect
```

#### 3. 启动任务分配
```bash
rosrun task_assign target_assign
```

#### 4. 启动机器人控制
```bash
rosrun rokae robot_control_node
```

#### 5. 启动Qt界面（可选）
```bash
rosrun qt_ros_nav qt_ros_nav
```

## 系统架构

```
双臂机器人采摘系统
├── 视觉检测模块 (img_detect)
│   ├── YOLOv8目标检测
│   ├── ZED双目相机驱动
│   ├── 点云数据处理
│   └── 3D坐标计算
├── 任务分配模块 (task_assign)
│   ├── 空间分区算法
│   ├── 深度优先排序
│   └── 双臂并行调度
├── 机器人控制模块 (rokae)
│   ├── 左臂控制
│   ├── 右臂控制
│   ├── 双臂协同
│   └── 安全保护
├── Qt图形界面 (qt_ros_nav)
│   ├── 导航控制
│   ├── 采摘控制
│   ├── 标定功能
│   └── 实时显示
├── 通信模块 (comm)
│   └── 串口通信
└── 点云发布模块 (pub_pcd)
    └── 点云数据发布
```

## 技术栈

- **ROS Noetic**: 机器人操作系统框架
- **Qt5**: 图形用户界面开发
- **OpenCV**: 计算机视觉处理
- **YOLOv8**: 深度学习目标检测
- **PCL**: 点云处理库
- **Eigen**: 线性代数和几何运算
- **ZED SDK**: 双目相机驱动
- **Rokae SDK**: 机器人硬件接口
- **RViz**: 3D可视化工具
- **move_base**: 导航路径规划
- **gmapping**: SLAM建图

## 硬件平台

- **机器人**: Rokae xMate SR4双臂机器人
  - 左臂IP: 192.168.2.161
  - 右臂IP: 192.168.2.160
- **相机**: Intel RealSense D405 或 ZED双目相机
- **传感器**: Livox激光雷达
- **计算平台**: 支持ROS Noetic的Ubuntu系统

## 安装说明

### 1. 系统要求
- Ubuntu 20.04 LTS
- ROS Noetic
- 支持CUDA的NVIDIA显卡（用于YOLOv8推理）

### 2. 依赖项安装

```bash
# 安装ROS Noetic
sudo apt update
sudo apt install ros-noetic-desktop-full

# 安装视觉相关包
sudo apt install ros-noetic-cv-bridge ros-noetic-tf2-eigen \
ros-noetic-image-transport ros-noetic-pcl-conversions \
ros-noetic-vision-opencv

# 安装导航相关包
sudo apt install ros-noetic-move-base ros-noetic-gmapping \
ros-noetic-amcl ros-noetic-map-server

# 安装Qt开发环境
sudo apt install qt5-default qtbase5-dev-tools \
libqt5svg5-dev

# 安装其他依赖
sudo apt install libeigen3-dev libopencv-dev libpcl-dev \
python3-pip

# 安装Python依赖
pip3 install onnxruntime opencv-python numpy
```

### 3. 编译步骤

```bash
# 进入工作空间
cd ~/dual_rokae_ws

# 编译工作空间
catkin_make

# 设置环境变量
source devel/setup.bash

# 永久设置环境变量（可选）
echo "source ~/dual_rokae_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 配置说明

### 1. 网络配置
确保机器人控制器与计算机在同一网络中：
- 左臂IP: 192.168.2.161
- 右臂IP: 192.168.2.160
- 计算机IP: 192.168.2.x

### 2. 相机配置
根据使用的相机类型修改相应的launch文件和参数。

### 3. 模型文件
将YOLOv8 ONNX模型文件放置在正确位置：
- `img_detect/src/best.onnx`

## 常见问题

### 1. 机器人连接失败
- 检查网络连接
- 确认机器人IP地址正确
- 检查机器人控制器是否开机

### 2. 视觉检测无结果
- 检查相机是否正常工作
- 确认模型文件路径正确
- 检查光照条件

### 3. 双臂无法协同工作
- 确认两个机器人都已连接
- 检查任务分配节点是否正常运行
- 查看ROS话题和服务状态

## 开发团队

西安交通大学机器人研究所

## 许可证

本项目遵循MIT许可证（参见LICENSE文件）

## 更新日志

- v1.0.0: 初始版本，包含基本的双臂采摘功能
  - 集成视觉检测、导航和控制模块
  - 提供完整的Qt图形界面
  - 支持手眼标定功能
  - 实现智能任务分配算法



  cat /tmp/picking_system.log 2>/dev/null; echo "==="; tail -20 /tmp/right_robot_control_node.log 2>/dev/null; echo "==="; tail -20 /tmp/left_robot_control_node.log 2>/dev/null; echo "==="; tail -20 /tmp/target_assign.log 2>/dev/null; echo "==="; tail -20 /tmp/advance_vision_node.log 2>/dev/null   
  - 日志查看