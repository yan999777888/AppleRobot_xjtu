#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QProcess>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDateTime>
#include <QDir>
#include <QPixmap>
#include <QFrame>
#include <QTableWidgetItem>
#include <QSizePolicy>
#include <QButtonGroup>
#include <QRadioButton>
#include <QGroupBox>

#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/display_group.h>

static const QString kStyleRunning = "QLabel { background-color:#27ae60; color:white; border-radius:4px; }";
static const QString kStyleStopped = "QLabel { background-color:#95a5a6; color:white; border-radius:4px; }";

namespace {

QString rosEnvPrefix() {
    const QString ws = QDir::homePath() + "/dual_rokae_ws";
    return QString("if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi; "
                   "if [ -f %1/devel/setup.bash ]; then source %1/devel/setup.bash; fi; ")
        .arg(ws);
}

QString wrapRosCommand(const QString& command) {
    return rosEnvPrefix() + command;
}

}  // namespace

MainWindow::MainWindow(int argc, char** argv, QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow),
      ros_node_(new RosNode(argc, argv)), is_moving_(false)
{
    ui_->setupUi(this);

    // ── RViz 嵌入 ──────────────────────────────────────────────
    render_panel_ = new rviz::RenderPanel();
    // 在加入布局前强制给定非零尺寸；OGRE 在 initialize() 时需要有效的
    // viewport 才能计算包围盒，零尺寸会触发 AxisAlignedBox 断言崩溃。
    render_panel_->setMinimumSize(1, 1);
    render_panel_->resize(800, 600);

    QVBoxLayout* rviz_layout = new QVBoxLayout(ui_->rviz_container);
    rviz_layout->addWidget(render_panel_);
    rviz_layout->setContentsMargins(0, 0, 0, 0);

    manager_ = new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(), manager_);
    manager_->initialize();
    manager_->setFixedFrame("map");

    rviz::Display* grid = manager_->createDisplay("rviz/Grid", "Grid", true);
    grid->subProp("Color")->setValue(QColor(180, 180, 180));
    Q_UNUSED(manager_->createDisplay("rviz/TF", "TF", true))
    rviz::Display* robot_model = manager_->createDisplay("rviz/RobotModel", "Robot", true);
    robot_model->subProp("Robot Description")->setValue("robot_description");
    rviz::Display* map_display = manager_->createDisplay("rviz/Map", "Map", false);
    map_display->subProp("Topic")->setValue("/map");
    rviz::Display* point_cloud = manager_->createDisplay("rviz/PointCloud2", "LiDAR", false);
    point_cloud->subProp("Topic")->setValue("/livox/lidar");
    point_cloud->subProp("Style")->setValue("Flat Squares");
    point_cloud->subProp("Color Transformer")->setValue("AxisColor");
    manager_->startUpdate();
    // ────────────────────────────────────────────────────────────

    // ── ROS 节点 ─────────────────────────────────────────────────
    if (!ros_node_->init())
        ui_->label_title->setText("ROS Node 初始化失败：请先启动 roscore");

    connect(ros_node_, &RosNode::signal_status, this, [this](const QString& msg){
        ui_->log_text_edit->append("[ROS] " + msg);
    });
    connect(ros_node_, &RosNode::signal_detection_image,      this, &MainWindow::updateDetectionImage);
    connect(ros_node_, &RosNode::signal_calibration_image,    this, &MainWindow::updateCalibrationImage);
    connect(ros_node_, &RosNode::signal_calib_camera_image,  this, &MainWindow::updateCalibCameraImage);
    connect(ros_node_, &RosNode::signal_calibration_status,  this, [this](const QString& msg){
        ui_->log_text_edit->append("[标定] " + msg);
        if (calib_status_label_) {
            calib_status_label_->setText(msg);
            calib_status_label_->setStyleSheet(
                "QLabel { background:#e0f2fe; color:#0f172a; border-radius:10px; padding:8px 10px; }");
        }
    });
    connect(ros_node_, &RosNode::signal_apple_detection_data, this, &MainWindow::updateAppleDetectionData);
    connect(ros_node_, &RosNode::signal_left_arm_status,      this, &MainWindow::updateLeftArmStatus);
    connect(ros_node_, &RosNode::signal_right_arm_status,     this, &MainWindow::updateRightArmStatus);
    // ────────────────────────────────────────────────────────────

    // ── 进程创建 ─────────────────────────────────────────────────
    proc_lio            = new QProcess(this);
    proc_fusion         = new QProcess(this);
    proc_camera         = new QProcess(this);
    proc_lidar          = new QProcess(this);
    proc_record         = new QProcess(this);
    proc_picking_system = new QProcess(this);
    proc_calib          = new QProcess(this);
    proc_calib_cam_     = new QProcess(this);

    auto connectLog = [this](QProcess* p) {
        connect(p, &QProcess::readyReadStandardOutput, this, [this, p]{ appendLog(p); });
        connect(p, &QProcess::readyReadStandardError,  this, [this, p]{ appendLog(p); });
    };
    connectLog(proc_lio);
    connectLog(proc_fusion);
    connectLog(proc_camera);
    connectLog(proc_lidar);
    connectLog(proc_record);
    connectLog(proc_picking_system);
    connectLog(proc_calib);
    connectLog(proc_calib_cam_);

    // proc_picking_system 退出时恢复按钮文字
    connect(proc_picking_system,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int, QProcess::ExitStatus){
                ui_->pushButton_start_picking->setText("一键启动采摘系统");
                ui_->log_text_edit->append("[系统] 采摘系统已停止或异常退出");
            });
    connect(proc_calib,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int exitCode, QProcess::ExitStatus exitStatus){
                setCalibrationRunning(false,
                    exitStatus == QProcess::NormalExit
                        ? QString("标定节点已退出 (code=%1)").arg(exitCode)
                        : "标定节点异常退出");
            });
    connect(proc_calib_cam_,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int exitCode, QProcess::ExitStatus exitStatus){
                ui_->log_text_edit->append(
                    exitStatus == QProcess::NormalExit
                        ? QString("[标定] 相机源已退出 (code=%1)").arg(exitCode)
                        : "[标定] 相机源异常退出");
            });
    // ────────────────────────────────────────────────────────────

    // ── 按钮绑定（导航 Tab）──────────────────────────────────────
    connect(ui_->pushButton_openfusion, &QPushButton::clicked, this, &MainWindow::on_pushButton_openfusion_clicked);
    connect(ui_->pushButton_openlio,    &QPushButton::clicked, this, &MainWindow::on_pushButton_openlio_clicked);
    connect(ui_->pushButton_opencamera, &QPushButton::clicked, this, &MainWindow::on_pushButton_opencamera_clicked);
    connect(ui_->pushButton_openlidar,  &QPushButton::clicked, this, &MainWindow::on_pushButton_openlidar_clicked);
    connect(ui_->pushButton_kill,       &QPushButton::clicked, this, &MainWindow::on_pushButton_kill_clicked);
    connect(ui_->pushButton_record,     &QPushButton::clicked, this, &MainWindow::on_pushButton_record_clicked);

    // ── 按钮绑定（采摘 Tab）──────────────────────────────────────
    connect(ui_->pushButton_start_picking,  &QPushButton::clicked, this, &MainWindow::on_pushButton_start_picking_clicked);
    connect(ui_->pushButton_emergency_stop, &QPushButton::clicked, this, &MainWindow::on_pushButton_emergency_stop_clicked);
    connect(ui_->pushButton_left_arm_home,  &QPushButton::clicked, this, &MainWindow::on_pushButton_left_arm_home_clicked);
    connect(ui_->pushButton_right_arm_home, &QPushButton::clicked, this, &MainWindow::on_pushButton_right_arm_home_clicked);
    // ────────────────────────────────────────────────────────────

    // ── 雷达话题检测定时器 ────────────────────────────────────────
    lidar_check_timer_ = new QTimer(this);
    lidar_check_timer_->setSingleShot(false);
    connect(lidar_check_timer_, &QTimer::timeout, this, &MainWindow::checkLidarTopic);
    calib_cam_check_timer_ = new QTimer(this);
    calib_cam_check_timer_->setSingleShot(false);
    connect(calib_cam_check_timer_, &QTimer::timeout, this, &MainWindow::checkCalibCamTopic);
    // ────────────────────────────────────────────────────────────

    // 初始化导航 Tab 状态灯
    setModuleState(ui_->pushButton_openfusion, ui_->label_fusion, false);
    setModuleState(ui_->pushButton_openlio,    ui_->label_lio,    false);
    setModuleState(ui_->pushButton_opencamera, ui_->label_camera, false);
    setModuleState(ui_->pushButton_openlidar,  ui_->label_lidar,  false);
    setModuleState(ui_->pushButton_record,     ui_->label_record, false);

    // 初始化苹果坐标表格（4列：ID / X / Y / Z）
    ui_->tableWidget_apple_coords->setColumnCount(4);
    ui_->tableWidget_apple_coords->setHorizontalHeaderLabels({"ID", "X", "Y", "Z"});
    ui_->tableWidget_apple_coords->horizontalHeader()->setStretchLastSection(true);

    auto* calib_layout = qobject_cast<QVBoxLayout*>(ui_->tab_calib->layout());
    if (calib_layout) {
        ui_->label_calib_placeholder->hide();

        auto* root = new QHBoxLayout();
        root->setSpacing(16);

        auto* previewCard = new QFrame();
        previewCard->setObjectName("calibPreviewCard");
        previewCard->setStyleSheet(
            "QFrame#calibPreviewCard { background:#111827; border:1px solid #334155; border-radius:16px; }"
            "QLabel { color:#e5e7eb; }");
        auto* previewLayout = new QVBoxLayout(previewCard);
        previewLayout->setContentsMargins(16, 16, 16, 16);
        previewLayout->setSpacing(10);
        auto* previewTitle = new QLabel("标定画面预览");
        previewTitle->setStyleSheet("font-size:18px; font-weight:600; color:#f8fafc;");
        calib_preview_label_ = new QLabel("等待标定画面...");
        calib_preview_label_->setAlignment(Qt::AlignCenter);
        calib_preview_label_->setMinimumSize(860, 540);
        calib_preview_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        calib_preview_label_->setStyleSheet(
            "QLabel { background:#0f172a; color:#94a3b8; border:1px solid #334155; border-radius:12px; }");
        previewLayout->addWidget(previewTitle);
        previewLayout->addWidget(calib_preview_label_, 1);

        auto* sideCard = new QFrame();
        sideCard->setObjectName("calibSideCard");
        sideCard->setStyleSheet(
            "QFrame#calibSideCard { background:white; border:1px solid #e2e8f0; border-radius:16px; }");
        auto* sideLayout = new QVBoxLayout(sideCard);
        sideLayout->setContentsMargins(16, 16, 16, 16);
        sideLayout->setSpacing(12);

        auto* title = new QLabel("机械臂自动标定");
        title->setStyleSheet("font-size:20px; font-weight:700; color:#0f172a;");
        calib_hint_label_ = new QLabel("按步骤查看每一帧标定图，确认 RGB 轴方向正确后再接受。");
        calib_hint_label_->setWordWrap(true);
        calib_hint_label_->setStyleSheet("color:#475569; line-height:1.4;");

        // ── 机械臂选择 ──────────────────────────────────────────
        auto* armGroupBox = new QGroupBox("选择机械臂");
        armGroupBox->setStyleSheet(
            "QGroupBox { font-size:13px; font-weight:600; color:#334155; "
            "border:1px solid #cbd5e1; border-radius:8px; margin-top:6px; padding-top:6px; }"
            "QGroupBox::title { subcontrol-origin:margin; left:8px; }");
        auto* armRow = new QHBoxLayout(armGroupBox);
        armRow->setContentsMargins(8, 4, 8, 6);
        calib_left_arm_radio_  = new QRadioButton("左臂 (192.168.2.161)");
        calib_right_arm_radio_ = new QRadioButton("右臂 (192.168.2.160)");
        calib_right_arm_radio_->setChecked(true);  // default: right arm
        auto* armBtnGroup = new QButtonGroup(this);
        armBtnGroup->addButton(calib_left_arm_radio_);
        armBtnGroup->addButton(calib_right_arm_radio_);
        armRow->addWidget(calib_left_arm_radio_);
        armRow->addWidget(calib_right_arm_radio_);
        // ────────────────────────────────────────────────────────

        calib_status_label_ = new QLabel("标定节点未启动");
        calib_status_label_->setStyleSheet("QLabel { background:#f1f5f9; color:#0f172a; border-radius:10px; padding:8px 10px; }");

        calib_eye_in_hand_button_ = new QPushButton("启动眼在手上");
        calib_eye_to_hand_button_ = new QPushButton("启动眼在手外");
        calib_accept_button_ = new QPushButton("接受当前姿态");
        calib_reject_button_ = new QPushButton("跳过当前姿态");
        calib_abort_button_ = new QPushButton("终止标定");

        calib_accept_button_->setEnabled(false);
        calib_reject_button_->setEnabled(false);
        calib_abort_button_->setEnabled(false);

        auto* startRow = new QHBoxLayout();
        startRow->addWidget(calib_eye_in_hand_button_);
        startRow->addWidget(calib_eye_to_hand_button_);

        sideLayout->addWidget(title);
        sideLayout->addWidget(calib_hint_label_);
        sideLayout->addWidget(armGroupBox);
        sideLayout->addWidget(calib_status_label_);
        sideLayout->addLayout(startRow);
        sideLayout->addSpacing(8);
        sideLayout->addWidget(calib_accept_button_);
        sideLayout->addWidget(calib_reject_button_);
        sideLayout->addWidget(calib_abort_button_);
        sideLayout->addStretch(1);

        root->addWidget(previewCard, 3);
        root->addWidget(sideCard, 1);
        calib_layout->addLayout(root);

        connect(calib_eye_in_hand_button_, &QPushButton::clicked, this, [this]{
            launchCalibration("eye_in_hand");
        });
        connect(calib_eye_to_hand_button_, &QPushButton::clicked, this, [this]{
            launchCalibration("eye_to_hand");
        });
        connect(calib_accept_button_, &QPushButton::clicked, this, [this]{
            ros_node_->publishCalibrationDecision(1);
            calib_preview_ready_ = false;
            if (calib_accept_button_) calib_accept_button_->setEnabled(false);
            if (calib_reject_button_) calib_reject_button_->setEnabled(false);
            if (calib_status_label_) {
                calib_status_label_->setText("已接受当前姿态，等待机械臂移动到下一位姿");
                calib_status_label_->setStyleSheet(
                    "QLabel { background:#dcfce7; color:#166534; border-radius:10px; padding:8px 10px; }");
            }
            ui_->log_text_edit->append("[标定] 已点击接受当前姿态");
        });
        connect(calib_reject_button_, &QPushButton::clicked, this, [this]{
            ros_node_->publishCalibrationDecision(2);
            calib_preview_ready_ = false;
            if (calib_accept_button_) calib_accept_button_->setEnabled(false);
            if (calib_reject_button_) calib_reject_button_->setEnabled(false);
            if (calib_status_label_) {
                calib_status_label_->setText("已跳过当前姿态，等待机械臂移动到下一位姿");
                calib_status_label_->setStyleSheet(
                    "QLabel { background:#fef9c3; color:#854d0e; border-radius:10px; padding:8px 10px; }");
            }
            ui_->log_text_edit->append("[标定] 已点击跳过当前姿态");
        });
        connect(calib_abort_button_, &QPushButton::clicked, this, [this]{
            ros_node_->publishCalibrationDecision(3);
            setCalibrationRunning(false, "已终止标定");
        });
    }
}

MainWindow::~MainWindow() {
    for (QProcess* p : {proc_lio, proc_fusion, proc_camera, proc_lidar, proc_record, proc_picking_system, proc_calib, proc_calib_cam_}) {
        if (p && p->state() == QProcess::Running) {
            p->terminate();
            if (!p->waitForFinished(1000)) p->kill();
        }
    }
    if (lidar_check_timer_) lidar_check_timer_->stop();
    if (calib_cam_check_timer_) calib_cam_check_timer_->stop();
    if (manager_) { manager_->stopUpdate(); delete manager_; }
    delete ros_node_;
    delete ui_;
}

// ── 辅助 ────────────────────────────────────────────────────────

void MainWindow::appendLog(QProcess* process) {
    QString text = process->readAllStandardOutput();
    text += process->readAllStandardError();
    if (!text.trimmed().isEmpty())
        ui_->log_text_edit->append(text.trimmed());
}

void MainWindow::setModuleState(QPushButton* button, QLabel* label, bool running) {
    // Keep descriptive text and add running indicator
    QString originalText = button->property("originalText").toString();
    if (originalText.isEmpty()) {
        originalText = button->text();
        button->setProperty("originalText", originalText);
    }
    
    if (running) {
        button->setText(originalText + " (运行中)");
        // Button turns green when running
        button->setStyleSheet("QPushButton { background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #5cb85c, stop:1 #449d44); color: white; border: none; border-radius: 6px; }"
                            "QPushButton:hover { background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #6bcf6b, stop:1 #55ad55); }"
                            "QPushButton:pressed { background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #449d44, stop:1 #367f36); }");
    } else {
        button->setText(originalText);
        // Button returns to blue when stopped
        button->setStyleSheet("QPushButton { background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #73b5ff, stop:1 #4285f4); color: white; border: none; border-radius: 6px; }"
                            "QPushButton:hover { background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #86c0ff, stop:1 #5294ff); }"
                            "QPushButton:pressed { background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #4285f4, stop:1 #3367d6); }");
    }
    
    if (label)
        label->setStyleSheet(running ? kStyleRunning : kStyleStopped);
}

void MainWindow::toggleProcess(QProcess* proc, QPushButton* btn, QLabel* lbl,
                                const QString& launchCmd) {
    if (proc->state() == QProcess::Running) {
        proc->terminate();
        if (!proc->waitForFinished(2000)) proc->kill();
        setModuleState(btn, lbl, false);
        ui_->log_text_edit->append("[系统] 已停止：" + launchCmd);
    } else {
        proc->start("bash", QStringList() << "-lc" << wrapRosCommand(launchCmd));
        setModuleState(btn, lbl, true);
        ui_->log_text_edit->append("[系统] 已启动：" + launchCmd);
    }
}

void MainWindow::launchCalibration(const QString& mode) {
    if (proc_calib->state() == QProcess::Running) {
        ui_->log_text_edit->append("[标定] 节点已在运行");
        return;
    }

    const bool eye_to_hand = (mode == "eye_to_hand");
    const QString imageTopic = eye_to_hand
        ? "/ap_robot/calib_raw_img"
        : "/camera/color/image_raw";
    const QString cameraInfoTopic = eye_to_hand
        ? "/ap_robot/camera_info"
        : "/camera/color/camera_info";

    // Determine arm IP from radio selection (default right arm if widget not ready)
    const bool useLeft = calib_left_arm_radio_ && calib_left_arm_radio_->isChecked();
    const QString robotIp = useLeft ? "192.168.2.161" : "192.168.2.160";
    const QString armName = useLeft ? "左臂" : "右臂";

    calib_pending_mode_ = mode;
    calib_pending_robot_ip_ = robotIp;
    calib_pending_arm_name_ = armName;
    calib_pending_image_topic_ = imageTopic;
    calib_pending_camera_info_topic_ = cameraInfoTopic;

    calib_preview_ready_ = false;
    calib_waiting_camera_ready_ = false;
    setCalibrationRunning(true, QString("标定准备中: %1 [%2]").arg(mode, armName));

    if (eye_to_hand) {
        if (proc_calib_cam_->state() != QProcess::Running) {
            const QString camCmd = wrapRosCommand("rosrun img_detect vision_node");
            proc_calib_cam_->start("bash", QStringList() << "-lc" << camCmd);
            if (!proc_calib_cam_->waitForStarted(1500)) {
                ui_->log_text_edit->append("[标定] ZED 相机源启动较慢，先继续启动标定");
            } else {
                ui_->log_text_edit->append("[标定] 已启动 ZED 相机源 vision_node");
            }
        } else {
            ui_->log_text_edit->append("[标定] ZED 相机源已在运行");
        }
        calib_waiting_camera_ready_ = true;
        if (isCalibrationCameraReady()) {
            if (calib_cam_check_timer_) calib_cam_check_timer_->stop();
            calib_waiting_camera_ready_ = false;
            startCalibrationProcess(calib_pending_mode_, calib_pending_robot_ip_, calib_pending_arm_name_,
                                    calib_pending_image_topic_, calib_pending_camera_info_topic_);
        } else {
            ui_->log_text_edit->append("[标定] 等待相机话题 /ap_robot/calib_raw_img 和 /ap_robot/camera_info");
            if (calib_cam_check_timer_) calib_cam_check_timer_->start(500);
        }
    } else {
        ui_->log_text_edit->append("[标定] 眼在手上模式需要外部 D405 图像源 /camera/color/image_raw 和 /camera/color/camera_info");
        startCalibrationProcess(calib_pending_mode_, calib_pending_robot_ip_, calib_pending_arm_name_,
                                calib_pending_image_topic_, calib_pending_camera_info_topic_);
    }
}

void MainWindow::checkCalibCamTopic() {
    if (!calib_waiting_camera_ready_) {
        return;
    }

    if (!isCalibrationCameraReady()) {
        return;
    }

    if (calib_cam_check_timer_) calib_cam_check_timer_->stop();
    calib_waiting_camera_ready_ = false;
    ui_->log_text_edit->append("[标定] 相机话题已就绪，准备启动标定节点");
    startCalibrationProcess(calib_pending_mode_, calib_pending_robot_ip_, calib_pending_arm_name_,
                            calib_pending_image_topic_, calib_pending_camera_info_topic_);
}

bool MainWindow::isCalibrationCameraReady() const {
    const int rc = QProcess::execute("bash", QStringList() << "-lc" << wrapRosCommand(
                                     "rostopic list 2>/dev/null | grep -q /ap_robot/calib_raw_img && "
                                     "rostopic list 2>/dev/null | grep -q /ap_robot/camera_info"));
    if (rc == 0) {
        return true;
    }
    return false;
}

void MainWindow::setCalibrationRunning(bool running, const QString& message) {
    calib_running_ = running;
    if (!running) {
        calib_preview_ready_ = false;
        calib_waiting_camera_ready_ = false;
        calib_pending_mode_.clear();
        calib_pending_robot_ip_.clear();
        calib_pending_arm_name_.clear();
        calib_pending_image_topic_.clear();
        calib_pending_camera_info_topic_.clear();
        if (calib_cam_check_timer_) calib_cam_check_timer_->stop();
        if (calib_preview_label_) {
            calib_preview_label_->clear();
            calib_preview_label_->setText("等待标定画面...");
        }
    }

    if (calib_eye_in_hand_button_) calib_eye_in_hand_button_->setEnabled(!running);
    if (calib_eye_to_hand_button_) calib_eye_to_hand_button_->setEnabled(!running);
    if (calib_accept_button_) calib_accept_button_->setEnabled(running);
    if (calib_reject_button_) calib_reject_button_->setEnabled(running);
    if (calib_abort_button_) calib_abort_button_->setEnabled(running);
    if (calib_left_arm_radio_) calib_left_arm_radio_->setEnabled(!running);
    if (calib_right_arm_radio_) calib_right_arm_radio_->setEnabled(!running);
    if (calib_status_label_) {
        calib_status_label_->setText(message.isEmpty()
                                         ? (running ? "标定运行中" : "标定已停止")
                                         : message);
        calib_status_label_->setStyleSheet(running
                                               ? "QLabel { background:#dcfce7; color:#166534; border-radius:10px; padding:8px 10px; }"
                                               : "QLabel { background:#f1f5f9; color:#0f172a; border-radius:10px; padding:8px 10px; }");
    }
}

void MainWindow::startCalibrationProcess(const QString& mode,
                                         const QString& robotIp,
                                         const QString& armName,
                                         const QString& imageTopic,
                                         const QString& cameraInfoTopic) {
    if (proc_calib->state() == QProcess::Running) {
        ui_->log_text_edit->append("[标定] 标定节点已在运行");
        return;
    }

    if (proc_picking_system->state() == QProcess::Running) {
        ui_->log_text_edit->append("[标定] 检测到采摘系统正在运行，先停止以释放机械臂连接");
        proc_picking_system->terminate();
        if (!proc_picking_system->waitForFinished(2000)) proc_picking_system->kill();
        ui_->pushButton_start_picking->setText("一键启动采摘系统");
    }

    QProcess::startDetached("bash", QStringList() << "-lc"
                            << wrapRosCommand("pkill -f robot_control_node; pkill -f leftRokae; pkill -f rightRokae; pkill -f task_assign"));

    const QString cmd = QString(
        "rosrun rokae auto_handeye_calibration "
        "_mode:=%1 "
        "_robot_ip:=%2 "
        "_interactive_confirm:=true "
        "_image_topic:=%3 "
        "_camera_info_topic:=%4")
        .arg(mode, robotIp, imageTopic, cameraInfoTopic);
    proc_calib->start("bash", QStringList() << "-lc" << wrapRosCommand(cmd));
    if (!proc_calib->waitForStarted(3000)) {
        setCalibrationRunning(false, "标定节点启动失败");
        ui_->log_text_edit->append(QString("[标定] 启动失败：%1 [%2 / %3]").arg(mode, armName, robotIp));
        return;
    }

    ui_->log_text_edit->append(QString("[标定] 已启动: %1 [%2 / %3]").arg(mode, armName, robotIp));
    setCalibrationRunning(true, QString("标定节点运行中: %1 [%2]").arg(mode, armName));
}

// ══════════════════ 导航 Tab 按钮 ══════════════════

void MainWindow::on_pushButton_openlio_clicked() {
    toggleProcess(proc_lio, ui_->pushButton_openlio, ui_->label_lio,
                  "roslaunch orchard_lio mapping_mid360.launch");
}

void MainWindow::on_pushButton_openfusion_clicked() {
    toggleProcess(proc_fusion, ui_->pushButton_openfusion, ui_->label_fusion,
                  "roslaunch orchard_fusion_mapping mapping_mid360.launch");
}

void MainWindow::on_pushButton_opencamera_clicked() {
    toggleProcess(proc_camera, ui_->pushButton_opencamera, ui_->label_camera,
                  "roslaunch usb_cam usb_cam-test.launch");
}

void MainWindow::on_pushButton_openlidar_clicked() {
    if (proc_lidar->state() == QProcess::Running) {
        proc_lidar->terminate();
        if (!proc_lidar->waitForFinished(2000)) proc_lidar->kill();
        lidar_check_timer_->stop();
        setModuleState(ui_->pushButton_openlidar, ui_->label_lidar, false);
    } else {
        proc_lidar->start("bash", QStringList() << "-lc"
                          << wrapRosCommand("roslaunch livox_ros_driver2 msg_MID360.launch"));
        setModuleState(ui_->pushButton_openlidar, ui_->label_lidar, true);
        ui_->log_text_edit->append("[系统] 雷达启动中，等待话题 /livox/lidar ...");
        lidar_check_timer_->start(500);
    }
}

void MainWindow::checkLidarTopic() {
    const int rc = QProcess::execute("bash", QStringList() << "-lc"
                                     << wrapRosCommand("rostopic list 2>/dev/null | grep -q /livox/lidar"));
    if (rc == 0) {
        lidar_check_timer_->stop();
        ui_->log_text_edit->append("[系统] 雷达话题 /livox/lidar 已就绪");
    }
}

void MainWindow::on_pushButton_record_clicked() {
    if (proc_record->state() == QProcess::Running) {
        proc_record->terminate();
        if (!proc_record->waitForFinished(2000)) proc_record->kill();
        setModuleState(ui_->pushButton_record, ui_->label_record, false);
        ui_->log_text_edit->append("[系统] 话题录制已停止");
    } else {
        QString savePath = "/home/ubuntu/bag_record";
        QString ts  = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
        QString cmd = QString(
            "mkdir -p %1 && rosbag record -O %1/%2.bag "
            "/livox/imu /livox/lidar /usb_cam/image_raw /map /odom /amcl_pose"
        ).arg(savePath, ts);
        proc_record->start("bash", QStringList() << "-lc" << wrapRosCommand(cmd));
        setModuleState(ui_->pushButton_record, ui_->label_record, true);
        ui_->log_text_edit->append("[系统] 话题录制已开始 → " + savePath + "/" + ts + ".bag");
    }
}

void MainWindow::on_pushButton_kill_clicked() {
    for (QProcess* p : {proc_lio, proc_fusion, proc_camera, proc_lidar, proc_record}) {
        if (p->state() == QProcess::Running) {
            p->terminate();
            p->waitForFinished(1000);
            if (p->state() == QProcess::Running) p->kill();
        }
    }
    if (proc_calib->state() == QProcess::Running) {
        proc_calib->terminate();
        proc_calib->waitForFinished(1000);
        if (proc_calib->state() == QProcess::Running) proc_calib->kill();
    }
    if (proc_calib_cam_->state() == QProcess::Running) {
        proc_calib_cam_->terminate();
        proc_calib_cam_->waitForFinished(1000);
        if (proc_calib_cam_->state() == QProcess::Running) proc_calib_cam_->kill();
    }
    if (calib_cam_check_timer_) calib_cam_check_timer_->stop();
    lidar_check_timer_->stop();
    setModuleState(ui_->pushButton_openlio,    ui_->label_lio,    false);
    setModuleState(ui_->pushButton_openfusion, ui_->label_fusion, false);
    setModuleState(ui_->pushButton_opencamera, ui_->label_camera, false);
    setModuleState(ui_->pushButton_openlidar,  ui_->label_lidar,  false);
    setModuleState(ui_->pushButton_record,     ui_->label_record, false);
    setCalibrationRunning(false, "标定节点已停止");
    QProcess::startDetached("bash", QStringList() << "-c" << "pkill -f roslaunch; pkill -f rosrun");
    ui_->log_text_edit->append("[系统] 所有节点已停止");
}

// ══════════════════ 采摘 Tab 按钮 ══════════════════

void MainWindow::on_pushButton_start_picking_clicked() {
    if (proc_picking_system->state() == QProcess::Running) {
        proc_picking_system->terminate();
        if (!proc_picking_system->waitForFinished(2000)) proc_picking_system->kill();
        ui_->pushButton_start_picking->setText("一键启动采摘系统");
        ui_->log_text_edit->append("[系统] 采摘系统已停止");
    } else {
        proc_picking_system->start("bash", QStringList() << "-lc"
                                   << wrapRosCommand("roslaunch img_detect start_picking_system.launch"));
        ui_->pushButton_start_picking->setText("停止采摘系统");
        ui_->log_text_edit->append("[系统] 采摘系统已启动");
    }
}

void MainWindow::on_pushButton_emergency_stop_clicked() {
    if (proc_picking_system->state() == QProcess::Running) {
        proc_picking_system->terminate();
        if (!proc_picking_system->waitForFinished(2000)) proc_picking_system->kill();
    }
    QProcess::startDetached("bash", QStringList() << "-c" << "pkill -f img_detect; pkill -f task_assign; pkill -f rokae");
    ui_->pushButton_start_picking->setText("一键启动采摘系统");
    ui_->log_text_edit->append("[系统] 采摘系统已急停");
}

void MainWindow::on_pushButton_left_arm_home_clicked() {
    ros_node_->publishLeftArmHomeCommand();
    ui_->log_text_edit->append("[系统] 左臂回 Home 位命令已发送");
}

void MainWindow::on_pushButton_right_arm_home_clicked() {
    ros_node_->publishRightArmHomeCommand();
    ui_->log_text_edit->append("[系统] 右臂回 Home 位命令已发送");
}

// ══════════════════ ROS 数据更新槽 ══════════════════

void MainWindow::updateDetectionImage(const QImage& image) {
    ui_->label_detection_image->setPixmap(
        QPixmap::fromImage(image).scaled(
            ui_->label_detection_image->size(),
            Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void MainWindow::updateCalibrationImage(const QImage& image) {
    if (!calib_preview_label_) return;
    calib_preview_label_->setPixmap(
        QPixmap::fromImage(image).scaled(
            calib_preview_label_->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
    calib_preview_ready_ = true;
    if (calib_accept_button_) calib_accept_button_->setEnabled(calib_running_);
    if (calib_reject_button_) calib_reject_button_->setEnabled(calib_running_);
}

void MainWindow::updateCalibCameraImage(const QImage& image) {
    if (!calib_preview_label_) return;
    if (calib_preview_ready_) return;
    calib_preview_label_->setPixmap(
        QPixmap::fromImage(image).scaled(
            calib_preview_label_->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
}

void MainWindow::updateAppleDetectionData(int count, const QVector<QPointF>& coords) {
    ui_->label_apple_count->setText(QString("检测到苹果数量: %1").arg(count));

    ui_->tableWidget_apple_coords->setRowCount(count);
    for (int i = 0; i < count; ++i) {
        ui_->tableWidget_apple_coords->setItem(i, 0, new QTableWidgetItem(QString::number(i)));
        ui_->tableWidget_apple_coords->setItem(i, 1, new QTableWidgetItem(QString::number(coords[i].x(), 'f', 3)));
        ui_->tableWidget_apple_coords->setItem(i, 2, new QTableWidgetItem(QString::number(coords[i].y(), 'f', 3)));
        ui_->tableWidget_apple_coords->setItem(i, 3, new QTableWidgetItem("—"));  // Z 列由 signal 扩展后填入
    }
}

void MainWindow::updateLeftArmStatus(const QString& status) {
    ui_->label_left_arm_status->setText(status);
}

void MainWindow::updateRightArmStatus(const QString& status) {
    ui_->label_right_arm_status->setText(status);
}
