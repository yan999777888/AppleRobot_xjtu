#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <QThread>
#include <QImage>
#include <QVector>
#include <QPointF>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <img_detect/Apple.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

class RosNode : public QThread {
  Q_OBJECT

public:
  RosNode(int argc, char** argv);
  virtual ~RosNode();
  bool init();

  void publishLeftArmHomeCommand();
  void publishRightArmHomeCommand();
  void publishCalibrationDecision(int decision);

protected:
  void run() override;

signals:
  void signal_mapUpdated(const QImage& map);
  void signal_robotPose(float x, float y, float yaw);
  void signal_status(const QString& msg);
  void signal_cmdVel(float linear, float angular);
  void signal_detection_image(const QImage& image);
  void signal_calibration_image(const QImage& image);
  void signal_calib_camera_image(const QImage& image);
  void signal_calibration_status(const QString& status);
  void signal_apple_detection_data(int count, const QVector<QPointF>& coords);
  void signal_left_arm_status(const QString& status);
  void signal_right_arm_status(const QString& status);

public slots:
  void slot_startMapping();
  void slot_startNavigation();
  void slot_saveMap();
  void slot_loadMap();
  void slot_setGoal(float x, float y, float yaw);
  void slot_cmdVel(float linear, float angular);

private:
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void detectionImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void calibrationImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void calibCameraImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void calibrationStatusCallback(const std_msgs::StringConstPtr& msg);
  void appleInfoCallback(const img_detect::AppleConstPtr& msg);
  void leftArmStatusCallback(const std_msgs::StringConstPtr& msg);
  void rightArmStatusCallback(const std_msgs::StringConstPtr& msg);

  int init_argc_;
  char** init_argv_;
  // NodeHandle 使用值类型并用指针管理生命周期，确保在 init() 后持续有效
  ros::NodeHandle* nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher goal_pub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber amcl_pose_sub_;
  ros::ServiceClient map_saver_client_;
  ros::ServiceClient map_loader_client_;
  ros::Subscriber detection_image_sub_;
  ros::Subscriber calibration_image_sub_;
  ros::Subscriber calib_camera_image_sub_;
  ros::Subscriber calibration_status_sub_;
  ros::Subscriber apple_info_sub_;
  ros::Publisher left_arm_home_pub_;
  ros::Publisher right_arm_home_pub_;
  ros::Publisher calibration_decision_pub_;
  ros::Subscriber left_arm_status_sub_;
  ros::Subscriber right_arm_status_sub_;
  bool is_mapping_;
  bool is_navigating_;
};

#endif // ROS_NODE_H
