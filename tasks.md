# 苹果采摘机器人总成软件任务清单

> 说明：本文件基于 `2026-04-23` 的仓库现状整理，目标是把机械臂视觉抓取、导航、建图、标定和 RViz 可视化整合成一个统一的总成软件。

---

## 1. 当前现状

### 已具备的能力

- `img_detect` 已完成基于视觉的苹果检测。
- 机械臂基于视觉的抓取流程已经实现。
- `qt_ros_nav` 已经搭起总成软件雏形。
- `qt_ros_nav` 里已经嵌入 `RViz`，并接入了地图、点云、机器人模型等显示。
- `qt_ros_nav` 已经有导航控制、话题录制、采摘控制、视觉反馈和标定页占位。

### 当前结构

- `src/qt_ros_nav/ui/mainwindow.ui`：主界面布局。
- `src/qt_ros_nav/src/mainwindow.cpp`：界面逻辑与按钮控制。
- `src/qt_ros_nav/src/ros_node.cpp`：ROS 订阅、发布与数据转发。
- `src/qt_ros_nav/launch/robot_navigation_2.launch`：导航启动入口。
- `src/qt_ros_nav/config/*.yaml`：导航参数配置。
- `src/qt_ros_nav/maps/*`：地图文件。

---

## 2. `qt_ros_nav` 现有功能

### 已实现

- RViz 嵌入到主界面。
- 导航相关按钮已接入：
  - 融合 SLAM
  - 激光 SLAM
  - 相机
  - 激光雷达
  - 话题录制
  - 关闭导航系统
- 采摘相关按钮已接入：
  - 一键启动采摘
  - 一键急停
  - 左臂 Home
  - 右臂 Home
- 视觉反馈已接入：
  - 检测图像显示
  - 苹果数量显示
  - 苹果坐标表格
- ROS 数据已接入：
  - `/map`
  - `/odom`
  - `/amcl_pose`
  - `/scan`
  - `/ap_robot/chatter_detectImg`
  - `/ap_robot/chatter_appleInfo`
  - 左右臂状态话题

### 仍需完善

- 标定页目前还是占位。
- 采摘页的手眼标定流程还没形成完整闭环。
- 导航和采摘之间还缺少统一调度层。
- 参数、路径、话题名仍有部分硬编码。
- 代码里存在可进一步整理的旧文件和中间产物。

---

## 3. 总成软件目标

- 把导航、建图、视觉识别、机械臂控制、夹爪控制和标定统一到一个软件里。
- 主界面可直接完成日常操作，不依赖多个零散窗口。
- RViz 作为核心可视化区域，实时展示地图、点云、机器人位姿和任务状态。
- 采摘流程能从“识别目标”一直串到“任务分配、机械臂执行、状态反馈”。

---

## 4. 待办任务

### P0 - 先补齐总成框架

- [ ] 统一软件功能边界，明确导航、采摘、标定、监控四个区域。
- [ ] 把 `qt_ros_nav` 的启动流程整理成稳定的总入口。
- [ ] 统一日志输出和状态显示规则。
- [ ] 清理重复或临时文件，保留真正参与构建的源码。

### P1 - 导航与建图

- [ ] 梳理建图、定位、导航的启动顺序。
- [ ] 把地图保存、加载、回放流程补完整。
- [ ] 增加导航目标点下发与状态回显。
- [ ] 统一导航参数和 launch 文件。
- [ ] 让 RViz 中的地图、点云、TF、机器人模型显示更稳定。

### P2 - 采摘作业闭环

- [ ] 完善采摘系统一键启动与急停逻辑。
- [ ] 让检测图像、苹果数量、苹果坐标与采摘流程联动。
- [ ] 把左右臂状态显示做成实时、可靠的状态面板。
- [ ] 补齐左臂 Home、右臂 Home、夹爪开合等手动控制。
- [ ] 将任务分配、机械臂控制、视觉检测的接口统一起来。

### P3 - 机械臂自动标定

- [ ] 设计标定页流程。
- [ ] 补齐手眼标定的数据采集、计算和结果保存。
- [ ] 统一左右臂标定结果的管理方式。
- [ ] 明确标定结果如何回写到总成软件配置中。

### P4 - 配置化与工程化

- [ ] 把 IP、话题名、路径、地图文件改成可配置项。
- [ ] 统一 launch、yaml、bag、map 的目录管理。
- [ ] 补充启动失败、话题缺失、节点异常退出的处理。
- [ ] 为后续演示和部署准备一键启动脚本。

### P5 - 验证与交付

- [ ] 按“建图 - 定位 - 导航 - 识别 - 抓取 - 反馈”顺序做联调。
- [ ] 记录每个功能模块的验收条件。
- [ ] 输出一份面向使用者的操作说明。
- [ ] 整理成适合论文演示和答辩展示的版本。

---

## 5. 推荐开发顺序

1. 先稳定 `qt_ros_nav` 的总成框架。
2. 再把采摘闭环和状态回显补齐。
3. 然后完善标定页。
4. 最后做配置化、联调和交付整理。



## ‘2026-4-24’开发日志

已完成。以下是所做修改的总结：

mainwindow.h
- 添加 #include <QRadioButton>
- 添加两个成员变量：calib_left_arm_radio_ 和 calib_right_arm_radio_

mainwindow.cpp
- 添加 #include <QButtonGroup>、<QRadioButton>、<QGroupBox>
- 在标定界面面板的提示文本与状态标签之间，添加了“选择机械臂”分组框，包含两个单选按钮（左臂 192.168.2.161 / 右臂 192.168.2.160，默认为右臂）
- 更新了 launchCalibration() 函数，使其读取单选按钮选择结果，并向 rosrun 传递参数 _robot_ip:=<IP>，同时在日志与状态栏中显示机械臂名称

auto_handeye_calibration.cpp
- 删除了第 609–611 行具有误导性的重复三元表达式 —— evaluateConsistency 函数内部已通过 eye_in_hand 布尔参数自动处理手眼在机/手眼在外标定，因此直接传递原始的 R_gripper2base_list / t_gripper2base_list 始终是正确的

src/img_detect/src/vision_node.cpp 现在只做一件事：打开 ZED、持续发布 /ap_robot/debug_img 和 /ap_robot/camera_info，不再等机械臂服务、不跑 YOLO、不掺采摘逻辑。
src/rokae/src/auto_handeye_calibration.cpp 也补了实时预览：即使还没识别到棋盘，/handeye/preview 也会先发当前画面，这样 UI 不会再空白。


src/rokae/src/auto_handeye_calibration.cpp (line 421) 现在是“边显示边等待确认”的流程，画面会持续刷新，不再是单帧停顿；按“接受/跳过”后才进入下一位姿。
标定结果现在按每次运行自动保存到独立时间戳目录，里面会有 camera_matrix.txt、dist_coeffs.txt、accepted_poses.csv 和每个接受位姿的图像，不会互相覆盖。
标定结束后会先回到初始位姿，再断开机器人。
src/qt_ros_nav/src/mainwindow.cpp (line 298) 我加了窗口关闭时的进程收尾，避免 QProcess: Destroyed while process ("bash") is still running。


完成所有修改，总结一下改了什么：

auto_handeye_calibration.cpp — 连接稳定性修复：

connectRobot() — 加入3次重试逻辑，每次失败等3秒再试，并打印连接进度日志
新增 reconnectRobot(int max_retries=3) — 销毁旧连接，重建TCP连接，最多重试3次
recoverRobot() — 先尝试就地恢复（stop/reset），如果失败则自动调用 reconnectRobot() 重建TCP
loadCurrentPose 调用（两处）— 全部包裹 try-catch，断连时自动触发重连，重连失败才终止标定
waitForCameraInfo() — 超时从10秒增加到30秒，并加入等待进度日志


## 软件画面不显示
找到问题了。有两个层面：

问题1： updateCalibrationImage 第594行 if (!calib_running_) return; — 标定未运行时直接丢弃所有画面。

问题2（根本原因）： 标定画面只来自 /handeye/preview（标定节点发布），但标定节点在连接机器人期间（很慢）还没进入帧循环，不会发布任何画面。应该也订阅 vision_node 发布的 /ap_robot/debug_img 作为相机实时预览。

解决方案：让 calib_preview_label_ 同时显示 /ap_robot/debug_img（相机实时画面），标定运行时 /handeye/preview 优先。

修复完成。总结改动：

之前的问题：

calib_preview_label_ 只订阅 /handeye/preview（标定节点发布），标定节点在连接机器人期间不会发布任何画面
updateCalibrationImage 有 if (!calib_running_) return; 的硬性检查，标定未运行时一切画面都被丢弃
修复：

ros_node.h/cpp — 新增订阅 /ap_robot/debug_img（来自 vision_node），通过 signal_calib_camera_image 信号发送
mainwindow.cpp — 新增 updateCalibCameraImage() 槽：标定未运行时显示 vision_node 的实时相机画面；标定运行中时 /handeye/preview 优先显示
updateCalibrationImage — 移除 if (!calib_running_) return;，标定运行中的画面不再被阻断
现在标定Tab的行为是：

vision_node 运行 → 立刻看到相机实时画面
点击启动标定 → 画面切换为标定节点的 /handeye/preview（带棋盘格检测覆盖）
标定结束 → 自动恢复显示 vision_node 的实时画面



## 现在很奇怪，打开标定软件有画面，但是点击启动标定后，画面就没了，而且没法标定
问题清楚了：点击启动标定后 calib_running_ 变为 true，updateCalibCameraImage 立刻停止显示 /ap_robot/debug_img，但标定节点还在连接机器人，/handeye/preview 还没开始发布，所以画面空白。

修复：用 calib_preview_ready_ 代替 calib_running_ 来判断是否切换画面源——只有标定节点真正开始发布画面后才停止显示相机实时流。

这些是 VSCode 的 includePath 配置警告，不影响 catkin 编译。

修改已完成：现在 updateCalibCameraImage 用 calib_preview_ready_ 而不是 calib_running_ 来判断。这样点击启动标定后，在标定节点还没开始发布 /handeye/preview 的期间，/ap_robot/debug_img 的相机实时画面会继续显示，直到收到第一帧标定画面才切换。



左臂与右臂的关键差异总结：

项目	右臂	左臂
IP	192.168.2.160	192.168.2.161
服务名	right_pickTarget	left_pickTarget
安装位置	y=-0.150	y=+0.150
安装旋转	Rx(+90°)	Rx(-90°) ← 关键区别
关节名	right_joint1~6	left_joint1~6
标定文件	camera_pose_right.txt	camera_pose_left.txt
夹爪关闭指令	...0x02, 0x58...	...0x03, 0x84... ← 不同
放置点	{0.508, -0.189, 0.164, ...}	{0.06, 0.27, -0.26, ...}
FK 的安装变换 buildLeftMountTransform() 中 rx=-1.5708 替换了右臂的 rx=+1.5708，这使得连杆碰撞监控能正确计算左臂各关节在 base_link 系中的真实位置。

注意：HOME_RX/RY/RZ 和 DROP_POSE 来自 leftRokaegodemo 的示教值，首次运行前建议先手动验证放置点位姿是否安全。