再进一步，我们就不能满足于“列出3个策略让机器人去试”这种离散的、碰运气的方法了。

作为研究生研究，你需要实现 **“基于采样与评价函数的连续空间最优抓取合成” (Sampling-based Grasp Synthesis with Cost Function Optimization)**。

### 核心思想升级

* **以前 (工程级):** 人为规定“前、上、侧”三个固定方向。
* **现在 (研究级):**
1. **球面采样 (Spherical Sampling):** 在苹果表面生成密集的切向空间（例如 50 个候选进给向量）。
2. **通用数学解算 (General Solver):** 编写一个通用的“LookAt”算法，输入任意向量，自动计算出机械臂末端的 6D 姿态（旋转矩阵）。
3. **代价函数 (Cost Function):** 建立评分标准 ，选出得分最高的那个去执行。



---

### 1. 数学核心：通用 Look-At 算法

你需要一个数学工具：**给定一个进给向量  (Z轴)，自动算出使得夹爪“水平”或“垂直”的旋转矩阵 **。

这涉及到 **Gram-Schmidt 正交化** 过程。

**在 `robot_control_node.cpp` 中添加这个核心数学函数：**

```cpp
// 引入 Eigen 几何模块
#include <Eigen/Geometry> 

/**
 * @brief 研究生级核心算法：从进给向量反解旋转矩阵
 * @param approach_vec 进给向量 (指向苹果中心，即末端 Z 轴)
 * @param preferred_x_axis 期望的 X 轴方向 (通常是世界坐标系的 -Z 或 水平方向，决定了夹爪的开口方向)
 * @return std::vector<double> 欧拉角 {Rx, Ry, Rz} (Rokae 标准)
 */
std::vector<double> computeOptimalPose(Eigen::Vector3d approach_vec, Eigen::Vector3d preferred_x_axis = Eigen::Vector3d(0, 0, -1)) {
    // 1. Z轴：归一化进给向量
    Eigen::Vector3d z_axis = approach_vec.normalized();

    // 2. Y轴：通过叉乘生成 (垂直于 Z 和 期望 X)
    // 注意：如果 Z 和 期望 X 平行（比如垂直抓取），叉乘会失效，需要特殊处理
    Eigen::Vector3d y_axis = z_axis.cross(preferred_x_axis);
    
    if (y_axis.norm() < 0.01) {
        // 奇异情况处理：如果进给方向和期望X平行，换一个辅助轴
        y_axis = z_axis.cross(Eigen::Vector3d(0, 1, 0));
    }
    y_axis.normalize();

    // 3. X轴：由 Y 和 Z 确定 (保证右手系)
    Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized();

    // 4. 构建旋转矩阵
    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    // 5. 转为欧拉角 (根据 Rokae 文档，通常是 Z-Y-X 或 X-Y-Z，这里假设 ZYX)
    // 务必注意：Eigen 的 eulerAngles 返回范围可能跳变，工业机器人通常需要连续解
    // 这里为了演示，直接返回标准解
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0); // Z, Y, X 顺序

    return {euler[2], euler[1], euler[0]}; // 返回 Rx, Ry, Rz
}

```

---

### 2. 策略升级：斐波那契球面采样 (Fibonacci Lattice)

不要只试 3 个点，我们要生成均匀分布在苹果上半球面的采样点。

```cpp
struct GraspTask {
    Eigen::Vector3d pos;       // 目标点
    Eigen::Vector3d approach;  // 进给方向
    std::vector<double> rpy;   // 姿态
    double score;              // 评分
};

// 生成候选抓取集合
std::vector<GraspTask> generateGraspCandidates(Eigen::Vector3d apple_center, int num_samples = 30) {
    std::vector<GraspTask> candidates;
    
    // 黄金分割角
    double phi = M_PI * (3.0 - std::sqrt(5.0)); 

    for (int i = 0; i < num_samples; ++i) {
        // 1. 生成球面均匀点 (y 从 1 到 0，只取上半球)
        double y = 1 - (i / float(num_samples - 1)); 
        double radius = std::sqrt(1 - y * y);
        double theta = phi * i;

        double x = std::cos(theta) * radius;
        double z = std::sin(theta) * radius;

        // 2. 得到相对于球心的方向向量 (从外部指向球心)
        // 注意：x,z 是水平面，y 是垂直轴 (根据采样公式)，需映射到机器人坐标系
        // 假设机器人坐标系 Z 是高，X 是前
        Eigen::Vector3d vec_on_sphere(x, z, y); // 这里的 y 对应高度
        Eigen::Vector3d approach = -vec_on_sphere.normalized(); // 进给方向：指向球心

        // 3. 过滤掉不合理的角度 (比如从下往上抓，会被树枝挡住)
        // 如果进给方向的 Z 分量 > 0 (说明是从下往上捅)，丢弃
        if (approach.z() > 0.1) continue; 

        // 4. 计算对应的姿态
        // 期望夹爪开口水平 (X轴指向侧面)
        std::vector<double> rpy = computeOptimalPose(approach, Eigen::Vector3d(0, 1, 0));

        GraspTask task;
        task.pos = apple_center;
        task.approach = approach;
        task.rpy = rpy;
        task.score = 0.0; // 待评分

        candidates.push_back(task);
    }
    return candidates;
}

```

---

### 3. 评估函数 (Cost Function)

这是你论文的**核心创新点**。为什么选A不选B？要有理有据。

```cpp
double evaluateGrasp(const GraspTask& task, Eigen::Vector3d apple_pos) {
    double score = 100.0;

    // 1. 距离惩罚 (Distance Cost)
    // 离机器人底座越远，刚度越差，分数越低
    double dist_to_base = apple_pos.norm();
    score -= dist_to_base * 10.0; 

    // 2. 高度惩罚 (Height Cost)
    // 进给向量越水平越好 (Z轴分量越接近0越好)，因为垂直抓取容易碰到上方树枝
    double vertical_component = std::abs(task.approach.z());
    score -= vertical_component * 30.0;

    // 3. 侧向偏移惩罚 (Alignment Cost)
    // 我们希望进给方向尽量正对着机器人 (X轴)，不要太偏侧面 (Y轴大)
    // 这样避免机械臂以奇怪的姿态扭曲
    double side_component = std::abs(task.approach.y());
    score -= side_component * 20.0;

    // 4. (可选) 如果有OctoMap，在这里加入碰撞检测
    // if (checkCollision(task)) return -1.0;

    return score;
}

```

---

### 4. 整合到 `pickCallBack`

将上述模块放入你的主逻辑中，现在的逻辑变成了**“生成 -> 评分 -> 排序 -> 执行”**。

```cpp
// 在 pickCallBack 内部替换原来的逻辑：

// 1. 获取苹果坐标
Eigen::Vector3d target_pos(p_base(0), p_base(1), p_base(2));

// 2. 生成 50 个采样候选
auto candidates = generateGraspCandidates(target_pos, 50);

// 3. 对每个候选评分
for (auto& task : candidates) {
    task.score = evaluateGrasp(task, target_pos);
}

// 4. 排序 (分数从高到低)
std::sort(candidates.begin(), candidates.end(), 
          [](const GraspTask& a, const GraspTask& b) { return a.score > b.score; });

// 5. 依次尝试执行 (Top-K Strategy)
bool success = false;
for (const auto& best_task : candidates) {
    if (best_task.score < 0) break; // 分数太低的不试

    ROS_INFO("尝试最优策略: Score=%.1f, Dir=[%.2f, %.2f, %.2f]", 
             best_task.score, best_task.approach.x(), best_task.approach.y(), best_task.approach.z());

    // 解包数据
    double Rx = best_task.rpy[0];
    double Ry = best_task.rpy[1];
    double Rz = best_task.rpy[2];

    // 计算预备点
    Eigen::Vector3d pre_pos = target_pos - best_task.approach * DIST_PRE_GRASP;
    Eigen::Vector3d retreat_pos = target_pos - best_task.approach * DIST_RETREAT;

    // ... 下面是 MoveL 执行逻辑，同上一轮代码 ...
    // try {
    //    Move to Pre_Pos
    //    Move to Target
    //    Grasp
    //    Move to Retreat
    //    success = true;
    //    break; // 成功了就退出循环
    // } catch (...) {
    //    continue; // 失败了试下一个分数高的
    // }
}

```

### 总结：你的论文/项目提升点

1. **从“离散”到“连续”：** 你不再受限于几个固定的方向，你的算法可以在半球面上找到**任意**可行的最优角度。
2. **通用性：** 你的 `computeOptimalPose` 结合了线性代数，适用于任何位置的苹果，即使苹果在角落里，算法也能算出对应的旋转矩阵，而不是靠硬编码。
3. **评价体系：** 你引入了 `evaluateGrasp`，这意味着你开始考虑**机器人的运动学约束**（刚度、工作空间、避障倾向），这才是研究生级别的机器人控制。

这个架构写出来，你的代码就不再是一个简单的 Demo，而是一个具备**局部路径规划 (Local Path Planning)** 能力的智能采摘系统。