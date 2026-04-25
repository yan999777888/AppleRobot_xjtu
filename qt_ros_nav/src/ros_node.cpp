#include "ros_node.h"
#include <QDebug>

RosNode::RosNode(int argc, char** argv)
    : init_argc_(argc), init_argv_(argv), nh_(nullptr),
      is_mapping_(false), is_navigating_(false) {}

RosNode::~RosNode() {
  if (ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
  delete nh_;
}

bool RosNode::init() {
  ros::init(init_argc_, init_argv_, "qt_ros_nav");
  if (!ros::master::check()) {
    emit signal_status("ROS Master 未找到！");
    return false;
  }
  ros::start();

  // 在堆上创建 NodeHandle，确保其生命周期与 RosNode 一致
  nh_ = new ros::NodeHandle();

  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  goal_pub_    = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  laser_sub_     = nh_->subscribe("/scan",       10, &RosNode::laserCallback,    this);
  odom_sub_      = nh_->subscribe("/odom",       10, &RosNode::odomCallback,     this);
  map_sub_       = nh_->subscribe("/map",        10, &RosNode::mapCallback,      this);
  amcl_pose_sub_ = nh_->subscribe("/amcl_pose", 10, &RosNode::amclPoseCallback, this);

  detection_image_sub_  = nh_->subscribe("/ap_robot/chatter_detectImg",  1, &RosNode::detectionImageCallback,  this);
  calibration_image_sub_ = nh_->subscribe("/handeye/preview",             1, &RosNode::calibrationImageCallback, this);
  calib_camera_image_sub_ = nh_->subscribe("/ap_robot/debug_img",          1, &RosNode::calibCameraImageCallback, this);
  calibration_status_sub_ = nh_->subscribe("/handeye/status",             1, &RosNode::calibrationStatusCallback, this);
  apple_info_sub_       = nh_->subscribe("/ap_robot/chatter_appleInfo",  1, &RosNode::appleInfoCallback,       this);
  left_arm_status_sub_  = nh_->subscribe("/ap_robot/left_arm/status",    1, &RosNode::leftArmStatusCallback,   this);
  right_arm_status_sub_ = nh_->subscribe("/ap_robot/right_arm/status",   1, &RosNode::rightArmStatusCallback,  this);

  left_arm_home_pub_  = nh_->advertise<std_msgs::Bool>("/ap_robot/left_arm/home",  1);
  right_arm_home_pub_ = nh_->advertise<std_msgs::Bool>("/ap_robot/right_arm/home", 1);
  calibration_decision_pub_ = nh_->advertise<std_msgs::Int32>("/handeye/decision", 1);

  map_saver_client_  = nh_->serviceClient<std_srvs::Empty>("/map_saver");
  map_loader_client_ = nh_->serviceClient<std_srvs::Empty>("/map_loader");

  start();
  emit signal_status("ROS 节点已初始化");
  return true;
}

void RosNode::run() {
  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  emit signal_status("ROS 节点已关闭");
}

void RosNode::slot_startMapping() {
  if (!is_mapping_) {
    is_mapping_ = true;
    emit signal_status("SLAM 已启动");
  }
}

void RosNode::slot_startNavigation() {
  if (!is_navigating_) {
    is_navigating_ = true;
    emit signal_status("导航已启动");
  }
}

void RosNode::slot_saveMap() {
  std_srvs::Empty srv;
  emit signal_status(map_saver_client_.call(srv) ? "地图保存成功" : "地图保存失败！");
}

void RosNode::slot_loadMap() {
  std_srvs::Empty srv;
  emit signal_status(map_loader_client_.call(srv) ? "地图加载成功" : "地图加载失败！");
}

void RosNode::slot_setGoal(float x, float y, float yaw) {
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id    = "map";
  goal.header.stamp       = ros::Time::now();
  goal.pose.position.x    = x;
  goal.pose.position.y    = y;
  goal.pose.orientation.z = std::sin(yaw / 2.0f);
  goal.pose.orientation.w = std::cos(yaw / 2.0f);
  goal_pub_.publish(goal);
  emit signal_status(QString("导航目标已发送: (%1, %2, %3)").arg(x).arg(y).arg(yaw));
}

void RosNode::slot_cmdVel(float linear, float angular) {
  geometry_msgs::Twist cmd;
  cmd.linear.x  = linear;
  cmd.angular.z = angular;
  cmd_vel_pub_.publish(cmd);
  emit signal_cmdVel(linear, angular);
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

void RosNode::publishCalibrationDecision(int decision) {
  std_msgs::Int32 msg;
  msg.data = decision;
  calibration_decision_pub_.publish(msg);
}

void RosNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& /*msg*/) {}

void RosNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  float x   = static_cast<float>(msg->pose.pose.position.x);
  float y   = static_cast<float>(msg->pose.pose.position.y);
  float yaw = 2.0f * std::atan2(static_cast<float>(msg->pose.pose.orientation.z),
                                 static_cast<float>(msg->pose.pose.orientation.w));
  emit signal_robotPose(x, y, yaw);
}

void RosNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  int width  = static_cast<int>(msg->info.width);
  int height = static_cast<int>(msg->info.height);
  QImage map(width, height, QImage::Format_Grayscale8);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx   = x + (height - y - 1) * width;
      int value = msg->data[idx];
      uchar color = (value == -1) ? 128u : (value == 0) ? 255u : 0u;
      map.setPixel(x, y, qRgb(color, color, color));
    }
  }
  emit signal_mapUpdated(map);
}

void RosNode::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  float x   = static_cast<float>(msg->pose.pose.position.x);
  float y   = static_cast<float>(msg->pose.pose.position.y);
  float yaw = 2.0f * std::atan2(static_cast<float>(msg->pose.pose.orientation.z),
                                  static_cast<float>(msg->pose.pose.orientation.w));
  emit signal_robotPose(x, y, yaw);
}

void RosNode::detectionImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    // QImage 不拥有数据，必须 copy() 才能安全跨线程传递
    QImage image(cv_ptr->image.data,
                 cv_ptr->image.cols,
                 cv_ptr->image.rows,
                 static_cast<int>(cv_ptr->image.step),
                 QImage::Format_RGB888);
    emit signal_detection_image(image.rgbSwapped().copy());
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void RosNode::calibrationImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    QImage image(cv_ptr->image.data,
                 cv_ptr->image.cols,
                 cv_ptr->image.rows,
                 static_cast<int>(cv_ptr->image.step),
                 QImage::Format_RGB888);
    emit signal_calibration_image(image.rgbSwapped().copy());
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void RosNode::appleInfoCallback(const img_detect::AppleConstPtr& msg) {
  // Apple.msg 字段：id[], x[], y[], z[], size[]（并行数组，无 apples 子字段）
  int count = static_cast<int>(msg->id.size());
  QVector<QPointF> coords;
  coords.reserve(count);
  for (int i = 0; i < count; ++i) {
    coords.append(QPointF(msg->x[i], msg->y[i]));
  }
  emit signal_apple_detection_data(count, coords);
}

void RosNode::leftArmStatusCallback(const std_msgs::StringConstPtr& msg) {
  emit signal_left_arm_status(QString::fromStdString(msg->data));
}

void RosNode::rightArmStatusCallback(const std_msgs::StringConstPtr& msg) {
  emit signal_right_arm_status(QString::fromStdString(msg->data));
}

void RosNode::calibCameraImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    QImage image(cv_ptr->image.data,
                 cv_ptr->image.cols,
                 cv_ptr->image.rows,
                 static_cast<int>(cv_ptr->image.step),
                 QImage::Format_RGB888);
    emit signal_calib_camera_image(image.rgbSwapped().copy());
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void RosNode::calibrationStatusCallback(const std_msgs::StringConstPtr& msg) {
  emit signal_calibration_status(QString::fromStdString(msg->data));
}
