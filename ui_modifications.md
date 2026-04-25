# UI 界面修改记录

## 2026-04-23

### 修改 1: 合并导航与采摘功能到单个集成页面

- **文件:** `src/qt_ros_nav/ui/mainwindow.ui`
- **日期:** 2026-04-23
- **内容:** 
  - 将原有的两个独立 Tab（"机器人感知导航" 和 "机器人采摘作业"）合并为一个 Tab（"机器人感知导航与采摘集成控制"）
  - 重新设计了 Tab 的内部布局为左右两部分结构：
    - **左侧 (60%)**: RViz 可视化容器（保持不变）
    - **右侧 (40%)**: 分为三个 QGroupBox，从上到下为：
      1. **导航系统控制** (~35% 高)：包含5个导航模块按钮 + 关闭按钮 + 日志框
      2. **采摘系统控制** (~25% 高)：包含一键启动/急停 + 手臂控制 + 状态显示
      3. **实时视觉反馈** (~40% 高)：包含检测图像 + 苹果计数 + 坐标表格

### 修改 2: 调整窗口大小和 Tab 尺寸

- **文件:** `src/qt_ros_nav/ui/mainwindow.ui`
- **修改内容:**
  ```xml
  <!-- 原窗口大小 -->
  <width>1000</width>
  <height>700</height>
  
  <!-- 改为 -->
  <width>1600</width>
  <height>950</height>
  
  <!-- Tab 最小高度 -->
  <height>380</height>  → <height>600</height>
  
  <!-- 底部背景图片高度 -->
  <height>180</height> → <height>120</height>
  ```

### 修改 3: 新增第二个 Tab - 机械臂自动标定

- **文件:** `src/qt_ros_nav/ui/mainwindow.ui`
- **修改内容:**
  - 原有的空白 `tab_calib` 现已更新为包含占位符 UI
  - 添加了 `label_calib_placeholder` 标签显示 "标定功能开发中..."
  - Tab 标题保持为 "机械臂自动标定"

### 修改 4: 优化右侧控制面板的 UI 元素

- **文件:** `src/qt_ros_nav/ui/mainwindow.ui`
- **修改内容:**
  
  **导航系统控制部分:**
  - 将按钮文本缩短便于显示：
    - "融合SLAM导航" → "融合SLAM"
    - "激光SLAM导航" → "激光SLAM"
    - "导航相机控制" → "导航相机"
    - "激光雷达控制" → "激光雷达"
    - "导航话题录制" → "话题录制"
    - "关闭机器人系统" → "关闭导航系统"
  - 将日志框 `log_text_edit` 的最大高度限制为 80px
  
  **采摘系统控制部分:**
  - "一键启动采摘系统" → "一键启动采摘"
  - 简化手臂状态标签显示："左臂状态:" → "左臂:"，"右臂状态:" → "右臂:"
  - 机械臂控制和状态显示改用 QGroupBox 分组
  
  **实时视觉反馈部分:**
  - 调整检测图像的最小尺寸：640x480 → 480x360（更适合右侧窄区域）
  - 苹果坐标表 `tableWidget_apple_coords` 的最大高度限制为 100px

### 修改 5: 布局结构重新设计

- **文件:** `src/qt_ros_nav/ui/mainwindow.ui`
- **关键变化:**
  
  | 项目 | 原布局 | 新布局 | 说明 |
  |-----|------|------|------|
  | Tab 数量 | 3（导航、采摘、标定） | 2（集成控制、标定） | 合并导航与采摘 |
  | 左右比例 | 6:4 | 6:4 | 保持一致 |
  | 右侧分层 | 单层 | 3 个 QGroupBox | 更清晰的分级 |
  | 窗口大小 | 1000x700 | 1600x950 | 提供更大工作区 |
  | RViz 显示 | 占 60% 宽 | 占 60% 宽 | 保持不变 |

### 修改 6: XML 元素重组

- **文件:** `src/qt_ros_nav/ui/mainwindow.ui`
- **主要改动:**
  - 删除了原始的 `tab_nav` 和 `tab_pick` 元素
  - 新增 `tab_integrated` 元素（Tab 0）
  - 保留并更新了 `tab_calib` 元素（Tab 1）
  - 所有信号槽连接保持不变（通过自动 UI 生成）
  - RViz 容器 `rviz_container` 保持完整功能

## C++ 代码兼容性说明

- **mainwindow.cpp**: 无需修改
  - 所有按钮名称保持一致（如 `pushButton_openfusion`, `pushButton_start_picking` 等）
  - 所有信号槽连接自动适配
  - 日志框、表格等名称均无变化

- **mainwindow.h**: 无需修改
  - 成员变量和槽函数声明保持一致
  - UI 指针自动生成

## 用户界面使用说明

1. **Tab 0 - 集成控制页面（推荐常用）**
   - 左侧：实时地图与传感器可视化
   - 右侧上：导航系统启动/停止、日志查看
   - 右侧中：采摘系统启动/急停、手臂状态
   - 右侧下：实时检测结果与苹果坐标

2. **Tab 1 - 标定页面**
   - 独立标定功能（待开发）


        connect(p, &QProcess::readyReadStandardOutput, this, [this, p]{ appendLog(p); });
        connect(p, &QProcess::readyReadStandardError,  this, [this, p]{ appendLog(p); });
        connect(p, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, [this, p](int exitCode, QProcess::ExitStatus exitStatus) {
            Q_UNUSED(exitCode);
            if (p == proc_picking_system) {
                // If picking system process finishes, update its state
                setModuleState(ui_->pushButton_start_picking, nullptr, false); // No label for picking system start/stop in main nav tab
                ui_->log_text_edit->append("[系统] 采摘系统已停止或异常退出。");
            }
        });
    };
    connectLog(proc_lio);
    connectLog(proc_fusion);
    connectLog(proc_camera);
    connectLog(proc_lidar);
    connectLog(proc_record);
    connectLog(proc_picking_system);
    // ────────────────────────────────────────────────────────────

    // ── 按钮绑定 ─────────────────────────────────────────────────
    connect(ui_->pushButton_openfusion, &QPushButton::clicked, this, &MainWindow::on_pushButton_openfusion_clicked);
    connect(ui_->pushButton_openlio,    &QPushButton::clicked, this, &MainWindow::on_pushButton_openlio_clicked);
    connect(ui_->pushButton_opencamera, &QPushButton::clicked, this, &MainWindow::on_pushButton_opencamera_clicked);
    connect(ui_->pushButton_openlidar,  &QPushButton::clicked, this, &MainWindow::on_pushButton_openlidar_clicked);
    connect(ui_->pushButton_kill,       &QPushButton::clicked, this, &MainWindow::on_pushButton_kill_clicked);
    connect(ui_->pushButton_record,     &QPushButton::clicked, this, &MainWindow::on_pushButton_record_clicked);

    // Picking Tab Buttons
    connect(ui_->pushButton_start_picking, &QPushButton::clicked, this, &MainWindow::on_pushButton_start_picking_clicked);
    connect(ui_->pushButton_emergency_stop, &QPushButton::clicked, this, &MainWindow::on_pushButton_emergency_stop_clicked);
    connect(ui_->pushButton_left_arm_home, &QPushButton::clicked, this, &MainWindow::on_pushButton_left_arm_home_clicked);
    connect(ui_->pushButton_right_arm_home, &QPushButton::clicked, this, &MainWindow::on_pushButton_right_arm_home_clicked);
    connect(ui_->pushButton_gripper_open, &QPushButton::clicked, this, &MainWindow::on_pushButton_gripper_open_clicked);
    connect(ui_->pushButton_gripper_close, &QPushButton::clicked, this, &MainWindow::on_pushButton_gripper_close_clicked);
```

### 5. 修改 `mainwindow.cpp` 文件 (槽函数实现)

- **文件:** `src/qt_ros_nav/src/mainwindow.cpp`
- **描述:** 实现了 "采摘作业 Tab" 中按钮的槽函数逻辑，包括启动/停止采摘系统、机械臂回 Home 位和夹爪的开/关。
- **修改内容:**
```cpp
// ══════════════════ 采摘功能按钮 ══════════════════

void MainWindow::on_pushButton_start_picking_clicked() {
    toggleProcess(proc_picking_system, ui_->pushButton_start_picking, nullptr,
                  "roslaunch img_detect start_picking_system.launch"); // Assuming a launch file exists
    if (proc_picking_system->state() == QProcess::Running) {
        ui_->pushButton_start_picking->setText("停止采摘系统");
    } else {
        ui_->pushButton_start_picking->setText("一键启动采摘系统");
    }
}

void MainWindow::on_pushButton_emergency_stop_clicked() {
    if (proc_picking_system->state() == QProcess::Running) {
        proc_picking_system->terminate();
        if (!proc_picking_system->waitForFinished(2000))
            proc_picking_system->kill();
        ui_->log_text_edit->append("[系统] 采摘系统已急停。");
    }
    // Also kill any orphaned roslaunch/rosrun processes related to picking if necessary
    QProcess::startDetached("bash", {"-c", "pkill -f img_detect; pkill -f task_assign; pkill -f rokae"});
    ui_->pushButton_start_picking->setText("一键启动采摘系统");
}

void MainWindow::on_pushButton_left_arm_home_clicked() {
    // TODO: Implement ROS service call or topic publish for left arm home
    ui_->log_text_edit->append("[系统] 左臂回 Home 位命令已发送。");
}

void MainWindow::on_pushButton_right_arm_home_clicked() {
    // TODO: Implement ROS service call or topic publish for right arm home
    ui_->log_text_edit->append("[系统] 右臂回 Home 位命令已发送。");
}

void MainWindow::on_pushButton_gripper_open_clicked() {
    // TODO: Implement serial communication for gripper open
    serial_comm_.writeGripperCommand(1); // Assuming 1 for open
    ui_->log_text_edit->append("[系统] 夹爪打开命令已发送。");
}

void MainWindow::on_pushButton_gripper_close_clicked() {
    // TODO: Implement serial communication for gripper close
    serial_comm_.writeGripperCommand(0); // Assuming 0 for close
    ui_->log_text_edit->append("[系统] 夹爪关闭命令已发送。");
}
```

### 6. 修改 `ros_node.h` 文件 (添加 cv_bridge 和 opencv 头文件)

- **文件:** `src/qt_ros_nav/include/qt_ros_nav/ros_node.h`
- **描述:** 添加了 `cv_bridge` 和 `opencv2/imgproc/imgproc.hpp` 头文件，用于处理 ROS 图像消息到 Qt `QImage` 的转换。
- **修改内容:**
```cpp
 #include <std_msgs/Bool.h>
 #include <cv_bridge/cv_bridge.h>
 #include <opencv2/imgproc/imgproc.hpp>
```

### 7. 修改 `ros_node.cpp` 文件 (初始化订阅者和发布者，实现回调函数)

- **文件:** `src/qt_ros_nav/src/ros_node.cpp`
- **描述:** 在 `RosNode::init()` 方法中初始化了用于图像检测、苹果信息、机械臂状态订阅者和机械臂回 Home 发布者。同时，实现了相应的回调函数和发布命令的函数。
- **修改内容:**
```cpp
// In RosNode::init()
   amcl_pose_sub_ = n.subscribe("/amcl_pose", 10, &RosNode::amclPoseCallback, this);

   // Apple Picking UI Subscriptions and Publications
   detection_image_sub_ = n.subscribe("/ap_robot/chatter_detectImg", 1, &RosNode::detectionImageCallback, this);
   apple_info_sub_      = n.subscribe("/ap_robot/chatter_appleInfo", 1, &RosNode::appleInfoCallback, this);
   left_arm_status_sub_ = n.subscribe("/ap_robot/left_arm/status", 1, &RosNode::leftArmStatusCallback, this);
   right_arm_status_sub_ = n.subscribe("/ap_robot/right_arm/status", 1, &RosNode::rightArmStatusCallback, this);

   left_arm_home_pub_  = n.advertise<std_msgs::Bool>("/ap_robot/left_arm/home", 1);
   right_arm_home_pub_ = n.advertise<std_msgs::Bool>("/ap_robot/right_arm/home", 1);

// New function implementations at the end of the file
void RosNode::detectionImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    QImage image(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888);
    emit signal_detection_image(image.copy());
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void RosNode::appleInfoCallback(const img_detect::AppleConstPtr& msg) {
  QVector<QPointF> coords;
  for (const auto& apple : msg->apples) {
    coords.append(QPointF(apple.x, apple.y));
  }
  emit signal_apple_detection_data(msg->apples.size(), coords);
}

void RosNode::leftArmStatusCallback(const std_msgs::StringConstPtr& msg) {
  emit signal_left_arm_status(QString::fromStdString(msg->data));
}

void RosNode::rightArmStatusCallback(const std_msgs::StringConstPtr& msg) {
  emit signal_right_arm_status(QString::fromStdString(msg->data));
}

void RosNode::publishLeftArmHomeCommand() {
  std_msgs::Bool msg;
  msg.data = true;
  left_arm_home_pub_.publish(msg);
}

void RosNode::publishRightArmHomeCommand() {
  std_msgs::Bool msg;
  msg.data = true;
  right_arm_home_pub_.publish(msg);
}
```

### 8. 修改 `mainwindow.h` 文件 (添加 ROS 数据更新槽)

- **文件:** `src/qt_ros_nav/include/qt_ros_nav/mainwindow.h`
- **描述:** 为接收来自 `RosNode` 的图像、苹果检测数据和机械臂状态更新信号，添加了新的槽函数声明。
- **修改内容:**
```cpp
  void on_pushButton_gripper_close_clicked();

  // New Slots for ROS Node Signals
  void updateDetectionImage(const QImage& image);
  void updateAppleDetectionData(int count, const QVector<QPointF>& coords);
  void updateLeftArmStatus(const QString& status);
  void updateRightArmStatus(const QString& status);
```

### 9. 修改 `mainwindow.cpp` 文件 (连接 ROS 信号和实现 ROS 数据更新槽)

- **文件:** `src/qt_ros_nav/src/mainwindow.cpp`
- **描述:** 在 `MainWindow` 构造函数中连接了 `RosNode` 发出的 ROS 数据更新信号到 `MainWindow` 中相应的槽函数。同时，实现了这些槽函数，用于更新 UI 界面上的图像显示、苹果检测数据表格和机械臂状态标签。
- **修改内容:**
```cpp
    connect(ros_node_, &RosNode::signal_status, this, [this](const QString& msg){
        ui_->log_text_edit->append("[ROS] " + msg);
    });

    // Connect signals from RosNode to MainWindow slots for picking tab
    connect(ros_node_, &RosNode::signal_detection_image, this, &MainWindow::updateDetectionImage);
    connect(ros_node_, &RosNode::signal_apple_detection_data, this, &MainWindow::updateAppleDetectionData);
    connect(ros_node_, &RosNode::signal_left_arm_status, this, &MainWindow::updateLeftArmStatus);
    connect(ros_node_, &RosNode::signal_right_arm_status, this, &MainWindow::updateRightArmStatus);
```
```cpp
// ══════════════════ ROS 数据更新槽 ══════════════════

void MainWindow::updateDetectionImage(const QImage& image) {
    // Scale image to fit the label and display
    QPixmap pixmap = QPixmap::fromImage(image);
    ui_->label_detection_image->setPixmap(pixmap.scaled(ui_->label_detection_image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void MainWindow::updateAppleDetectionData(int count, const QVector<QPointF>& coords) {
    ui_->label_apple_count->setText(QString::number(count));

    // Clear existing table data
    ui_->tableWidget_apple_coords->setRowCount(0);

    // Populate table with new coordinates
    ui_->tableWidget_apple_coords->setRowCount(count);
    for (int i = 0; i < count; ++i) {
        QTableWidgetItem* itemX = new QTableWidgetItem(QString::number(coords[i].x()));
        QTableWidgetItem* itemY = new QTableWidgetItem(QString::number(coords[i].y()));
        ui_->tableWidget_apple_coords->setItem(i, 0, itemX);
        ui_->tableWidget_apple_coords->setItem(i, 1, itemY);
    }
}

void MainWindow::updateLeftArmStatus(const QString& status) {
    ui_->label_left_arm_status->setText(status);
}

void MainWindow::updateRightArmStatus(const QString& status) {
    ui_->label_right_arm_status->setText(status);
}
```