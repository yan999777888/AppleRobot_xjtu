#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QPainter>
#include <QGraphicsView>
#include <QDebug>
#include <QSerialPortInfo>
#include <QProcess>
#include <QThread>

// 定义成员变量（在类里已经声明）
MainWindow::MainWindow(int argc, char** argv, QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow), ros_node_(new RosNode(argc, argv)), serial_comm_(), is_moving_(false)
{
    ui_->setupUi(this);

    // 初始化 ROS
    if (!ros_node_->init()) {
        ui_->label_title->setText("ROS Node 初始化失败");
    }

    // 按钮绑定
    connect(ui_->pushButton_openfusion, &QPushButton::clicked, this, &MainWindow::on_pushButton_openfusion_clicked);
    connect(ui_->pushButton_openlio,    &QPushButton::clicked, this, &MainWindow::on_pushButton_openlio_clicked);
    connect(ui_->pushButton_opencamera, &QPushButton::clicked, this, &MainWindow::on_pushButton_opencamera_clicked);
    connect(ui_->pushButton_openlidar,  &QPushButton::clicked, this, &MainWindow::on_pushButton_openlidar_clicked);
    connect(ui_->pushButton_kill,       &QPushButton::clicked, this, &MainWindow::on_pushButton_kill_clicked);

    // 创建独立进程，每个按钮一个
    proc_lio     = new QProcess(this);
    proc_fusion  = new QProcess(this);
    proc_camera  = new QProcess(this);
    proc_lidar   = new QProcess(this);
    proc_record = new QProcess(this);

}

MainWindow::~MainWindow() {
    delete ros_node_;
    delete ui_;
}

// ===================== 激光SLAM =====================
void MainWindow::on_pushButton_openlio_clicked()
{
    ui_->label_title->setText("正在启动激光建图...");

    // 先关闭旧的
    if (proc_lio->state() == QProcess::Running) {
        proc_lio->kill();
        proc_lio->waitForFinished();
    }

    proc_lio->start("bash", QStringList() << "-c" << "roslaunch orchard_lio mapping_mid360.launch");
}

// ===================== 融合SLAM =====================
void MainWindow::on_pushButton_openfusion_clicked()
{
    ui_->label_title->setText("正在启动融合建图...");

    if (proc_fusion->state() == QProcess::Running) {
        proc_fusion->kill();
        proc_fusion->waitForFinished();
    }

    proc_fusion->start("bash", QStringList() << "-c" << "roslaunch orchard_fusion_mapping mapping_mid360.launch");
}

// ===================== 相机 =====================
void MainWindow::on_pushButton_opencamera_clicked()
{
    ui_->label_title->setText("正在启动相机...");

    if (proc_camera->state() == QProcess::Running) {
        proc_camera->kill();
        proc_camera->waitForFinished();
    }

    proc_camera->start("bash", QStringList() << "-c" << "roslaunch usb_cam usb_cam-test.launch");
}

// ===================== 雷达 =====================
void MainWindow::on_pushButton_openlidar_clicked()
{
    ui_->label_title->setText("正在启动雷达...");

    if (proc_lidar->state() == QProcess::Running) {
        proc_lidar->kill();
        proc_lidar->waitForFinished();
    }

    proc_lidar->start("bash", QStringList() << "-c" << "roslaunch livox_ros_driver2 msg_MID360.launch");
}

// topic_record

void MainWindow::on_pushButton_record_clicked()
{
    ui_->label_title->setText("正在启动话题录制...");

    // 先停止已运行的录制
    if (proc_record->state() == QProcess::Running) {
        proc_record->kill();
        proc_record->waitForFinished();
        ui_->label_title->setText("话题录制已停止");
        return;
    }

    // 录制命令（带时间戳 + 保存路径 + 指定话题）
    QString cmd = R"(
        SAVE_PATH="/home/ubuntu/bag_record";
        mkdir -p $SAVE_PATH;
        BAG_NAME="$SAVE_PATH/$(date +%Y-%m-%d_%H-%M-%S).bag";
        rosbag record -O $BAG_NAME /livox/imu /livox/lidar /usb_cam/image_raw;
    )";

    proc_record->start("bash", QStringList() << "-c" << cmd);
}

// ===================== 关闭所有 =====================
void MainWindow::on_pushButton_kill_clicked()
{
    ui_->label_title->setText("已停止所有节点");

    // 关闭每个进程
    if (proc_lio->state() == QProcess::Running) {
        proc_lio->kill();
        proc_lio->waitForFinished();
    }
    if (proc_fusion->state() == QProcess::Running) {
        proc_fusion->kill();
        proc_fusion->waitForFinished();
    }
    if (proc_camera->state() == QProcess::Running) {
        proc_camera->kill();
        proc_camera->waitForFinished();
    }
    if (proc_lidar->state() == QProcess::Running) {
        proc_lidar->kill();
        proc_lidar->waitForFinished();
    }

    //强制杀死所有 ROS 相关进程（彻底退出）
    QProcess::startDetached("pkill -f roslaunch");
    QProcess::startDetached("pkill -f rosrun");
    QProcess::startDetached("pkill -f rosout");
    QProcess::startDetached("pkill -f image_view");
    QProcess::startDetached("pkill -f usb_cam");
    QProcess::startDetached("rosnode kill -a");

    //关闭窗口
    this->close();
}