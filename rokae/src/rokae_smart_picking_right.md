# Rokae xMateSR4 智能采摘与全机身避障控制节点 (右臂版)

## 📌 项目简介————实现代码，robot_control_node.cpp

本节点是一个面向复杂非结构化环境（如农业采摘、凌乱桌面抓取）的工业级 ROS 控制节点。针对**侧装模式**的 Rokae xMateSR4 机械臂，实现了从视觉目标解析、多维度姿态采样，到全机身动态/静态避障的完整闭环控制。

区别于传统的仅进行“末端点防撞”的简单策略，本节点在底层 C++ 中硬编码了机械臂的正向运动学（FK），利用多线程以 20Hz 的频率在三维空间中构建机械臂的“动态肉身（胶囊体阵列）”，实现了毫秒级的全连杆碰撞拦截与紧急制动。

## ✨ 核心特性

1. **硬编码正向运动学 (Fast FK)**
   - 摒弃了计算开销较大的 ROS TF 树，直接根据 xMateSR4 的 DH 参数（关节偏移与旋转轴）硬编码解析正向运动学。
   - 输入 6 个关节角，瞬间输出 8 个关键节点（基座、6个关节原点、工具末端）的三维坐标，计算极速且不依赖外部组件。
2. **全连杆安全包裹 (Full-Body Capsule Collision)**
   - **动态避障**：将 7 段连杆抽象为 7 个粗细不同的空间胶囊体（半径 0.04m ~ 0.08m），与实时的 PCL 点云进行线段到点的距离解算。
   - **静态避障**：构建了工作台/架子的 AABB 包围盒，防止机械臂在规划或运动中发生“自杀式”撞击。
3. **斐波那契多视角抓取规划 (Fibonacci Sphere Sampling)**
   - 针对单个目标点，利用斐波那契球面采样生成 120 个切入角度。
   - 配合智能姿态解算（优先贴合 Home 姿态），自动过滤掉容易引发奇异点或与机架干涉的危险位姿，选出最优解。
4. **异步多线程与原子级急停 (Asynchronous E-Stop)**
   - 剥离出独立的 `linkCollisionMonitorThread` 监控线程。
   - 利用 `std::atomic<bool>` 与主运动线程进行无锁通信。一旦发现轨迹受阻，在机械臂撞击前即可触发 `robot.stopMove()` 实施紧急制动，并自动切换至备选策略。

## ⚙️ 系统架构与 ROS 接口

### 订阅 (Subscribers)
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/ap_robot/obstacles` | `sensor_msgs::PointCloud2` | 相机坐标系下的障碍物/背景点云 |
| `/joint_states` | `sensor_msgs::JointState` | 机械臂实时关节角状态 |
| `ap_robot/serial_ack` | `std_msgs::Bool` | 夹爪串口指令完成的反馈信号 |

### 发布 (Publishers)
| Topic | Type | Description |
| :--- | :--- | :--- |
| `ap_robot/serial` | `comm::serialData` | 发送给硬件底层的串口控制报文（控制夹爪开合） |

### 服务 (Services)
| Service Name | Type | Description |
| :--- | :--- | :--- |
| `right_pickTarget` | `task_assign::Target` | 接收目标点的 $(x, y, z)$ 坐标，触发采摘全流程 |

## 🛠️ 参数配置 (ROS Params)

在 `launch` 文件中可动态调整以下参数，以适配不同的抓取场景：

| 参数名 | 默认值 | 描述 |
| :--- | :--- | :--- |
| `dist_pre` | `0.15` (m) | 抓取预备点距离目标的长度 |
| `dist_ret` | `0.10` (m) | 抓取完成后直线撤退的距离 |
| `gripper_open_wait` | `0.5` (s) | 夹爪完全张开所需等待时间 |
| `gripper_close_wait` | `0.6` (s) | 夹爪完全闭合所需等待时间 |
| `max_reach` | `1.5` (m) | 机械臂允许探出的最大安全距离 |
| `collision_radius` | `0.08` (m) | 抓取末端路径碰撞检测的胶囊体半径 |
| `approach_speed` | `60.0` (%) | 执行 `MoveL` 进给与撤退时的安全速度 |
| `transit_speed` | `200.0` (%) | 执行 `MoveJ` 大范围转移时的过渡速度 |
| `num_samples` | `120` | 斐波那契球面采样数 |

**静态架子 AABB 包围盒参数:**
- `rack_xmin`, `rack_xmax`: `[-0.25, 0.25]`
- `rack_ymin`, `rack_ymax`: `[-0.35, 0.35]`
- `rack_zmin`, `rack_zmax`: `[0.00, 0.885]`

## 🚀 动作执行流 (Workflow)

当收到 `right_pickTarget` 服务调用时，系统执行以下状态机：

1. **坐标转换**：加载 `camera_pose_right.txt` 与 `camera_depth_scale_right.txt`，将目标转换至基座坐标系。
2. **合法性校验**：检查目标是否超出臂展（`MAX_REACH`）或陷入静态基座死区（`RackAABB`）。
3. **策略生成**：基于目标点生成 120 个接近向量，按姿态得分降序排序。
4. **尝试抓取**：
   - 遍历前 20 个最优策略。
   - **预检**：检查“预备点”及“进给线段”是否与点云或架子发生碰撞。
   - **执行**：开启监控线程 $\rightarrow$ 打开夹爪 $\rightarrow$ MoveJ 至预备点 $\rightarrow$ MoveL 进给 $\rightarrow$ 闭合夹爪 $\rightarrow$ MoveL 撤退。
   - **异常处理**：若移动中途触发多线程连杆碰撞，则抛出异常并尝试下一个候选策略。
5. **放置与复位**：移动至预设的 Drop 关节角，释放夹爪，完成单次节拍。

## ⚠️ 注意事项
- **网络连接**：本节点写死了机械臂 IP `192.168.2.160`，如更换环境请在代码或配置文件中修改。
- **夹爪协议**：`leftClosedata` 与 `leftOpendata` 的 16 进制报文针对特定型号电爪硬编码，适配新硬件需修改数据帧。