#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QTimer>
#include <QPushButton>
#include <QLabel>
#include <QFrame>
#include <QRadioButton>
#include <QString>
#include "ros_node.h"
#include "./ui_mainwindow.h"

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>

Q_DECLARE_METATYPE(QProcess::ExitStatus)

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = nullptr);
  ~MainWindow();

private slots:
  void on_pushButton_openlio_clicked();
  void on_pushButton_openfusion_clicked();
  void on_pushButton_opencamera_clicked();
  void on_pushButton_openlidar_clicked();
  void on_pushButton_kill_clicked();
  void on_pushButton_record_clicked();
  void checkLidarTopic();
  void checkCalibCamTopic();
  void appendLog(QProcess* process);

  // 采摘 Tab 槽
  void on_pushButton_start_picking_clicked();
  void on_pushButton_emergency_stop_clicked();
  void on_pushButton_left_arm_home_clicked();
  void on_pushButton_right_arm_home_clicked();

  // RosNode 信号对应的 UI 更新槽
  void updateDetectionImage(const QImage& image);
  void updateCalibrationImage(const QImage& image);
  void updateCalibCameraImage(const QImage& image);
  void updateAppleDetectionData(int count, const QVector<QPointF>& coords);
  void updateLeftArmStatus(const QString& status);
  void updateRightArmStatus(const QString& status);

private:
  void setModuleState(QPushButton* button, QLabel* label, bool running);
  void toggleProcess(QProcess* proc, QPushButton* btn, QLabel* lbl,
                     const QString& launchCmd);
  void launchCalibration(const QString& mode);
  void startCalibrationProcess(const QString& mode,
                               const QString& robotIp,
                               const QString& armName,
                               const QString& imageTopic,
                               const QString& cameraInfoTopic);
  bool isCalibrationCameraReady() const;
  void setCalibrationRunning(bool running, const QString& message = QString());
  void updateBasePickLockUi();

  Ui::MainWindow* ui_;
  RosNode* ros_node_;
  bool ros_ready_ {false};
  bool is_moving_;

  QProcess* proc_lio;
  QProcess* proc_fusion;
  QProcess* proc_camera;
  QProcess* proc_lidar;
  QProcess* proc_record;
  QProcess* proc_picking_system;
  QProcess* proc_calib;
  QProcess* proc_calib_cam_;

  QTimer* lidar_check_timer_;
  QTimer* calib_cam_check_timer_;

  QPushButton* calib_eye_in_hand_button_ {nullptr};
  QPushButton* calib_eye_to_hand_button_ {nullptr};
  QPushButton* calib_accept_button_ {nullptr};
  QPushButton* calib_reject_button_ {nullptr};
  QPushButton* calib_abort_button_ {nullptr};
  QPushButton* calib_start_cam_button_ {nullptr};
  QPushButton* base_pick_lock_button_ {nullptr};
  QLabel* calib_status_label_ {nullptr};
  QLabel* calib_preview_label_ {nullptr};
  QLabel* calib_hint_label_ {nullptr};
  QLabel* calib_cam_status_label_ {nullptr};
  QRadioButton* calib_left_arm_radio_ {nullptr};
  QRadioButton* calib_right_arm_radio_ {nullptr};
  bool calib_running_ {false};
  bool calib_preview_ready_ {false};
  bool calib_waiting_camera_ready_ {false};
  bool base_pick_enabled_ {false};
  bool picking_system_running_ {false};
  QString gripper_serial_port_ {"/dev/ttyUSB0"};
  QString calib_pending_mode_;
  QString calib_pending_robot_ip_;
  QString calib_pending_arm_name_;
  QString calib_pending_image_topic_;
  QString calib_pending_camera_info_topic_;

  rviz::RenderPanel*          render_panel_;
  rviz::VisualizationManager* manager_;
};

#endif // MAINWINDOW_H
