#include "ros/ros.h"
#include "ros/package.h"
#include "rokae/robot.h"
#include "rokae/utility.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <system_error>
#include <ctime>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

using namespace rokae;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr auto kPreviewPollInterval = std::chrono::milliseconds(30);

enum class Mode {
  EyeInHand,
  EyeToHand,
};

struct Config {
  Mode mode{Mode::EyeInHand};
  std::string mode_name{"eye_in_hand"};
  std::string robot_ip;
  std::string image_topic;
  std::string camera_info_topic;
  std::string save_dir;
  int checker_rows{8};
  int checker_cols{6};
  double square_size{0.0405};
  int move_speed{200};
  double settle_time{2.0};
  int num_captures{5};
  int num_poses{20};
  double pos_dx{0.06};
  double pos_dy{0.06};
  double pos_dz{0.04};
  double rot_delta{0.25};
  double wait_timeout{30.0};
  double decision_timeout{0.0};
  double min_pos_dist{0.02};
  double min_rot_dist{5.0};
  bool interactive_confirm{false};
  bool show_opencv_preview{false};
  std::string hand_eye_method{"TSAI"};
  std::string run_stamp;
};

struct DetectionResult {
  bool found{false};
  Eigen::Matrix3d R_target2cam{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_target2cam{Eigen::Vector3d::Zero()};
  cv::Mat vis;
};

struct SolveResult {
  std::string name;
  Eigen::Matrix4d transform{Eigen::Matrix4d::Identity()};
  double mean_error_mm{0.0};
  double max_error_mm{0.0};
  double std_error_mm{0.0};
  double det{0.0};
};

Eigen::Matrix3d eulerToMatrix(double rx, double ry, double rz) {
  const Eigen::AngleAxisd ax(rx, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ay(ry, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd az(rz, Eigen::Vector3d::UnitZ());
  return (az * ay * ax).toRotationMatrix();
}

Eigen::Matrix4d poseToMatrix(const std::array<double, 6>& pose) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = eulerToMatrix(pose[3], pose[4], pose[5]);
  T.block<3, 1>(0, 3) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  return T;
}

Eigen::Matrix4d inverseRigid(const Eigen::Matrix4d& T) {
  Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
  inv.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
  inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * T.block<3, 1>(0, 3);
  return inv;
}

void saveMatrixTxt(const std::filesystem::path& path, const Eigen::Matrix4d& T) {
  std::ofstream ofs(path);
  ofs << std::fixed << std::setprecision(9);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << T(i, j);
      if (j < 3) ofs << ' ';
    }
    ofs << '\n';
  }
}

void saveMatrixTxt(const std::filesystem::path& path, const cv::Mat& mat) {
  std::ofstream ofs(path);
  ofs << std::fixed << std::setprecision(9);
  for (int r = 0; r < mat.rows; ++r) {
    for (int c = 0; c < mat.cols; ++c) {
      ofs << mat.at<double>(r, c);
      if (c + 1 < mat.cols) ofs << ' ';
    }
    ofs << '\n';
  }
}

void addTextBanner(cv::Mat& image, const std::string& text, const cv::Point& origin,
                   double scale, const cv::Scalar& color, int thickness) {
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

void decoratePreview(cv::Mat& image, const cv::Mat& reference, const std::string& pose_text) {
  if (image.empty()) return;
  const int h = image.rows;
  const int w = image.cols;

  addTextBanner(image, pose_text, cv::Point(12, 30), 0.78, cv::Scalar(0, 0, 0), 3);
  addTextBanner(image, pose_text, cv::Point(12, 30), 0.78, cv::Scalar(255, 255, 255), 1);
  addTextBanner(image,
                "Accept: GUI button / SPACE   Reject: GUI button / R   Abort: GUI button / ESC",
                cv::Point(12, 58), 0.52, cv::Scalar(0, 0, 0), 2);
  addTextBanner(image,
                "Accept: GUI button / SPACE   Reject: GUI button / R   Abort: GUI button / ESC",
                cv::Point(12, 58), 0.52, cv::Scalar(0, 255, 255), 1);

  if (!reference.empty()) {
    const int rw = std::max(1, w / 4);
    const int rh = std::max(1, h / 4);
    cv::Mat inset;
    cv::resize(reference, inset, cv::Size(rw, rh));
    const cv::Rect roi(w - rw - 10, 10, rw, rh);
    inset.copyTo(image(roi));
    cv::rectangle(image, roi, cv::Scalar(0, 255, 255), 2);
    addTextBanner(image, "Last accepted", cv::Point(w - rw - 8, 6), 0.42, cv::Scalar(0, 255, 255), 1);
  }
}

sensor_msgs::ImagePtr matToImageMsg(const cv::Mat& image) {
  cv_bridge::CvImage out;
  out.encoding = sensor_msgs::image_encodings::BGR8;
  out.image = image;
  return out.toImageMsg();
}

cv::Mat eigenMatrixToCv(const Eigen::Matrix3d& R) {
  cv::Mat mat(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      mat.at<double>(r, c) = R(r, c);
    }
  }
  return mat;
}

cv::Mat eigenVectorToCv(const Eigen::Vector3d& t) {
  cv::Mat mat(3, 1, CV_64F);
  mat.at<double>(0, 0) = t.x();
  mat.at<double>(1, 0) = t.y();
  mat.at<double>(2, 0) = t.z();
  return mat;
}

bool detectCheckerboard(const cv::Mat& color_image,
                        const cv::Mat& camera_matrix,
                        const cv::Mat& dist_coeffs,
                        const cv::Size& board_size,
                        double square_size,
                        DetectionResult& out) {
  if (color_image.empty() || camera_matrix.empty()) return false;

  cv::Mat gray;
  if (color_image.channels() == 3) {
    cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = color_image.clone();
  }

  std::vector<cv::Point2f> corners;
  bool found = false;
#if CV_VERSION_MAJOR >= 4
  found = cv::findChessboardCornersSB(
      gray, board_size, corners,
      cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
#endif
  if (!found) {
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                      cv::CALIB_CB_NORMALIZE_IMAGE;
    found = cv::findChessboardCorners(gray, board_size, corners, flags);
  }
  if (!found) {
    out.found = false;
    out.vis = color_image.clone();
    return false;
  }

  if (corners.size() > 0) {
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
  }

  std::vector<cv::Point3f> object_points;
  object_points.reserve(static_cast<size_t>(board_size.area()));
  for (int row = 0; row < board_size.height; ++row) {
    for (int col = 0; col < board_size.width; ++col) {
      object_points.emplace_back(static_cast<float>(col * square_size),
                                 static_cast<float>(row * square_size),
                                 0.0f);
    }
  }

  cv::Mat rvec, tvec;
  if (!cv::solvePnP(object_points, corners, camera_matrix, dist_coeffs, rvec, tvec)) {
    out.found = false;
    out.vis = color_image.clone();
    return false;
  }

  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);
  R_cv.convertTo(R_cv, CV_64F);
  Eigen::Matrix3d R;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R(r, c) = R_cv.at<double>(r, c);
    }
  }

  out.found = true;
  out.R_target2cam = R;
  out.t_target2cam = Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
  out.vis = color_image.clone();
  cv::drawChessboardCorners(out.vis, board_size, corners, found);
  cv::drawFrameAxes(out.vis, camera_matrix, dist_coeffs, rvec, tvec, square_size * 2.0);
  return true;
}

std::vector<std::array<double, 6>> generatePoses(const std::array<double, 6>& base_pose,
                                                 int num_poses,
                                                 double dx,
                                                 double dy,
                                                 double dz,
                                                 double drot) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist_x(-dx, dx);
  std::uniform_real_distribution<double> dist_y(-dy, dy);
  std::uniform_real_distribution<double> dist_z(-dz, dz);
  std::uniform_real_distribution<double> dist_r(-drot, drot);

  std::vector<std::array<double, 6>> poses;
  poses.reserve(static_cast<size_t>(num_poses));
  poses.push_back(base_pose);
  for (int i = 1; i < num_poses; ++i) {
    std::array<double, 6> pose = base_pose;
    pose[0] += dist_x(rng);
    pose[1] += dist_y(rng);
    pose[2] += dist_z(rng);
    pose[3] += dist_r(rng);
    pose[4] += dist_r(rng);
    pose[5] += dist_r(rng);
    poses.push_back(pose);
  }
  return poses;
}

bool isPoseTooSimilar(const Eigen::Matrix4d& T_new,
                      const std::vector<Eigen::Matrix3d>& R_list,
                      const std::vector<Eigen::Vector3d>& t_list,
                      double min_pos_dist,
                      double min_rot_dist) {
  for (size_t i = 0; i < R_list.size(); ++i) {
    const double pos_diff_mm = (T_new.block<3, 1>(0, 3) - t_list[i]).norm() * 1000.0;
    const Eigen::Matrix3d R_diff = T_new.block<3, 3>(0, 0) * R_list[i].transpose();
    const double cos_a = std::clamp((R_diff.trace() - 1.0) / 2.0, -1.0, 1.0);
    const double rot_diff = std::acos(cos_a) * 180.0 / kPi;
    if (pos_diff_mm < min_pos_dist * 1000.0 && rot_diff < min_rot_dist) {
      return true;
    }
  }
  return false;
}

std::array<double, 6> loadCurrentPose(BaseRobot& robot, std::error_code& ec) {
  const auto pose = robot.posture(CoordinateType::flangeInBase, ec);
  return {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};
}

std::string makeTimestampStamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  #if defined(_WIN32)
  localtime_s(&tm, &tt);
  #else
  localtime_r(&tt, &tm);
  #endif
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return ss.str();
}

class CameraBuffer {
 public:
  CameraBuffer(ros::NodeHandle nh, const std::string& image_topic, const std::string& camera_info_topic)
      : nh_(std::move(nh)) {
    image_sub_ = nh_.subscribe(image_topic, 1, &CameraBuffer::imageCallback, this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &CameraBuffer::cameraInfoCallback, this);
  }

  bool hasCameraInfo() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return has_camera_info_;
  }

  bool hasImage() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return !latest_image_.empty();
  }

  bool getFrame(cv::Mat& image, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (latest_image_.empty() || !has_camera_info_) return false;
    image = latest_image_.clone();
    camera_matrix = camera_matrix_.clone();
    dist_coeffs = dist_coeffs_.clone();
    return true;
  }

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      const auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      std::lock_guard<std::mutex> lk(mutex_);
      latest_image_ = cv_ptr->image.clone();
    } catch (const cv_bridge::Exception& e) {
      ROS_WARN("image conversion failed: %s", e.what());
    }
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
      camera_matrix_.at<double>(i / 3, i % 3) = msg->K[i];
    }
    const size_t dist_size = msg->D.empty() ? 5 : msg->D.size();
    dist_coeffs_ = cv::Mat(1, static_cast<int>(dist_size), CV_64F, cv::Scalar(0));
    for (size_t i = 0; i < msg->D.size(); ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->D[i];
    }
    has_camera_info_ = true;
  }

  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  mutable std::mutex mutex_;
  cv::Mat latest_image_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool has_camera_info_{false};
};

class AutoHandEyeCalibrator {
 public:
  explicit AutoHandEyeCalibrator(ros::NodeHandle nh)
      : nh_(std::move(nh)) {
    loadConfig();
    camera_ = std::make_unique<CameraBuffer>(nh_, config_.image_topic, config_.camera_info_topic);
    preview_pub_ = nh_.advertise<sensor_msgs::Image>("/handeye/preview", 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>("/handeye/status", 1, true);
    decision_sub_ = nh_.subscribe("/handeye/decision", 1, &AutoHandEyeCalibrator::decisionCallback, this);
  }

  void decisionCallback(const std_msgs::Int32ConstPtr& msg) {
    {
      std::lock_guard<std::mutex> lk(decision_mutex_);
      latest_decision_ = msg->data;
      decision_received_ = true;
    }
    ROS_INFO_STREAM("received calibration decision: " << msg->data);
    decision_cv_.notify_all();
  }

  int waitForDecision() {
    std::unique_lock<std::mutex> lk(decision_mutex_);
    if (config_.decision_timeout > 0.0) {
      const bool ok = decision_cv_.wait_for(
          lk, std::chrono::duration<double>(config_.decision_timeout),
          [&] { return decision_received_ || !ros::ok(); });
      if (!ok) {
        ROS_WARN_STREAM("wait for calibration decision timed out after "
                        << config_.decision_timeout << " s");
        return 2;
      }
    } else {
      decision_cv_.wait(lk, [&] { return decision_received_ || !ros::ok(); });
    }
    return latest_decision_;
  }

  int run() {
    if (!waitForCameraInfo()) {
      ROS_ERROR("camera info not received");
      publishStatus("标定失败：未收到相机信息");
      return 1;
    }

    if (!connectRobot()) {
      publishStatus("标定失败：机械臂连接失败");
      return 1;
    }

    std::array<double, 6> base_pose;
    try {
      base_pose = loadCurrentPose(*robot_, ec_);
    } catch (...) {
      ROS_ERROR("failed to read initial robot pose");
      shutdownRobot();
      return 1;
    }
    ROS_INFO_STREAM("base pose: "
                    << base_pose[0] << ", " << base_pose[1] << ", " << base_pose[2] << ", "
                    << base_pose[3] << ", " << base_pose[4] << ", " << base_pose[5]);

    if (!prepareSaveDir()) {
      publishStatus("标定失败：保存目录创建失败");
      return 1;
    }

    const cv::Size board_size(config_.checker_cols, config_.checker_rows);
    const std::vector<std::array<double, 6>> poses =
        generatePoses(base_pose, config_.num_poses, config_.pos_dx, config_.pos_dy, config_.pos_dz, config_.rot_delta);

    ROS_INFO_STREAM("generated poses: " << poses.size());

    std::vector<Eigen::Matrix3d> R_gripper2base_list;
    std::vector<Eigen::Vector3d> t_gripper2base_list;
    std::vector<Eigen::Matrix3d> R_target2cam_list;
    std::vector<Eigen::Vector3d> t_target2cam_list;

    int valid_count = 0;
    int collision_count = 0;
    cv::Mat reference_vis;

    cv::Mat camera_matrix, dist_coeffs;
    if (!camera_->getFrame(last_image_, camera_matrix, dist_coeffs)) {
      ROS_ERROR("failed to read first camera frame");
      shutdownRobot();
      publishStatus("标定失败：未读取到相机图像");
      return 1;
    }
    saveMatrixTxt(save_dir_path_ / "camera_matrix.txt", camera_matrix);
    saveMatrixTxt(save_dir_path_ / "dist_coeffs.txt", dist_coeffs);

    for (size_t i = 0; i < poses.size() && ros::ok(); ++i) {
      const auto& pose = poses[i];
      ROS_INFO_STREAM("pose " << (i + 1) << "/" << poses.size()
                      << " x=" << pose[0] << " y=" << pose[1] << " z=" << pose[2]
                      << " rx=" << pose[3] << " ry=" << pose[4] << " rz=" << pose[5]);

      if (i > 0) {
        if (!moveToPose(pose)) {
          ++collision_count;
          publishStatus("当前位姿移动失败，已跳过。当前有效位姿: " + std::to_string(valid_count));
          recoverRobot();
          continue;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(config_.settle_time));
      } else {
        ROS_INFO("use current pose as the first calibration sample");
      }

      std::array<double, 6> actual_pose;
      try {
        actual_pose = loadCurrentPose(*robot_, ec_);
      } catch (const rokae::Exception& e) {
        ROS_WARN("loadCurrentPose failed: %s, attempting reconnect", e.what());
        if (!reconnectRobot()) {
          ROS_ERROR("reconnect failed, aborting calibration");
          shutdownRobot();
          publishStatus("标定中断：机械臂重连失败。当前有效位姿: " + std::to_string(valid_count));
          return 1;
        }
        ++collision_count;
        continue;
      } catch (...) {
        ROS_WARN("loadCurrentPose failed (unknown), attempting reconnect");
        if (!reconnectRobot()) {
          ROS_ERROR("reconnect failed, aborting calibration");
          shutdownRobot();
          publishStatus("标定中断：机械臂重连失败。当前有效位姿: " + std::to_string(valid_count));
          return 1;
        }
        ++collision_count;
        continue;
      }
      const Eigen::Matrix4d T_actual = poseToMatrix(actual_pose);
      if (isPoseTooSimilar(T_actual, R_gripper2base_list, t_gripper2base_list,
                           config_.min_pos_dist, config_.min_rot_dist)) {
        ROS_WARN("skip duplicated pose");
        continue;
      }

      {
        std::lock_guard<std::mutex> lk(decision_mutex_);
        decision_received_ = false;
        latest_decision_ = 0;
      }

      std::vector<DetectionResult> detections;
      cv::Mat latest_vis;
      DetectionResult latest_det;
      const auto pose_wait_start = std::chrono::steady_clock::now();

      while (ros::ok()) {
        cv::Mat frame;
        if (camera_->getFrame(frame, camera_matrix, dist_coeffs)) {
          DetectionResult det;
          if (detectCheckerboard(frame, camera_matrix, dist_coeffs, board_size, config_.square_size, det)) {
            detections.push_back(det);
            if (detections.size() > static_cast<size_t>(std::max(1, config_.num_captures))) {
              detections.erase(detections.begin());
            }
            latest_det = det;
          } else {
            latest_det.found = false;
            latest_det.vis = frame.clone();
          }

          const cv::Mat& base_preview = latest_det.found ? latest_det.vis : frame;
          latest_vis = base_preview.clone();
          decoratePreview(latest_vis, reference_vis,
                          "Pose " + std::to_string(i + 1) + "/" + std::to_string(poses.size()));
          addTextBanner(latest_vis, latest_det.found ? "Checkerboard found" : "Searching checkerboard",
                        cv::Point(12, 64), 0.7, cv::Scalar(0, 0, 0), 3);
          addTextBanner(latest_vis, latest_det.found ? "Checkerboard found" : "Searching checkerboard",
                        cv::Point(12, 64), 0.7,
                        latest_det.found ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 255), 1);

          preview_pub_.publish(matToImageMsg(latest_vis));
          if (config_.show_opencv_preview) {
            cv::imshow("auto_handeye_calibration", latest_vis);
            cv::waitKey(1);
          }
        }

        std::unique_lock<std::mutex> lk(decision_mutex_);
        if (decision_received_) {
          break;
        }
        if (!config_.interactive_confirm) {
          break;
        }
        if (config_.decision_timeout > 0.0) {
          const bool signaled = decision_cv_.wait_for(
              lk, kPreviewPollInterval,
              [&] { return decision_received_ || !ros::ok(); });
          if (!signaled && std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - pose_wait_start).count() > config_.decision_timeout) {
            ROS_WARN("decision timeout, treat as reject");
            latest_decision_ = 2;
            decision_received_ = true;
            break;
          }
        } else {
          decision_cv_.wait_for(lk, kPreviewPollInterval,
                                [&] { return decision_received_ || !ros::ok(); });
        }
      }

      const int decision = config_.interactive_confirm ? latest_decision_ : 1;
      if (decision == 3) {
        ROS_WARN("abort requested");
        shutdownRobot();
        publishStatus("已终止标定。当前有效位姿: " + std::to_string(valid_count));
        return 0;
      }
      if (decision != 1) {
        ROS_INFO("pose rejected");
        publishStatus("已拒绝当前位姿。当前有效位姿: " + std::to_string(valid_count));
        continue;
      }
      if (detections.empty()) {
        ROS_WARN("checkerboard not found");
        publishStatus("当前位姿未识别到棋盘，已跳过。当前有效位姿: " + std::to_string(valid_count));
        continue;
      }

      const DetectionResult det = detections[detections.size() / 2];
      if (!latest_vis.empty()) {
        preview_pub_.publish(matToImageMsg(latest_vis));
      }

      const std::string image_name = (save_dir_path_ / ("pose_" + padIndex(valid_count + 1) + ".png")).string();
      cv::imwrite(image_name, det.vis);
      std::ofstream pose_log(save_dir_path_ / "accepted_poses.csv", std::ios::app);
      pose_log << valid_count + 1 << ','
               << actual_pose[0] << ',' << actual_pose[1] << ',' << actual_pose[2] << ','
               << actual_pose[3] << ',' << actual_pose[4] << ',' << actual_pose[5] << ','
               << image_name << '\n';

      R_gripper2base_list.push_back(T_actual.block<3, 3>(0, 0));
      t_gripper2base_list.push_back(T_actual.block<3, 1>(0, 3));
      R_target2cam_list.push_back(det.R_target2cam);
      t_target2cam_list.push_back(det.t_target2cam);
      reference_vis = det.vis.clone();
      ++valid_count;
      ROS_INFO_STREAM("accepted samples: " << valid_count);
      publishStatus("已接受当前位姿，当前有效位姿: " + std::to_string(valid_count));
    }

    if (collision_count > 0) {
      ROS_WARN_STREAM("collision or timeout count: " << collision_count);
    }

    if (valid_count < 3) {
      ROS_ERROR("need at least 3 valid poses");
      shutdownRobot();
      publishStatus("标定失败：有效位姿不足 3 个。当前有效位姿: " + std::to_string(valid_count));
      return 1;
    }

    const bool eye_in_hand = config_.mode == Mode::EyeInHand;
    std::vector<Eigen::Matrix3d> first_R;
    std::vector<Eigen::Vector3d> first_t;

    if (eye_in_hand) {
      first_R = R_gripper2base_list;
      first_t = t_gripper2base_list;
    } else {
      first_R.reserve(R_gripper2base_list.size());
      first_t.reserve(t_gripper2base_list.size());
      for (size_t i = 0; i < R_gripper2base_list.size(); ++i) {
        Eigen::Matrix4d T_bg = Eigen::Matrix4d::Identity();
        T_bg.block<3, 3>(0, 0) = R_gripper2base_list[i];
        T_bg.block<3, 1>(0, 3) = t_gripper2base_list[i];
        const Eigen::Matrix4d T_gb = inverseRigid(T_bg);
        first_R.push_back(T_gb.block<3, 3>(0, 0));
        first_t.push_back(T_gb.block<3, 1>(0, 3));
      }
    }

    std::vector<SolveResult> solve_results;
    const std::vector<std::pair<std::string, cv::HandEyeCalibrationMethod>> methods = {
        {"TSAI", cv::CALIB_HAND_EYE_TSAI},
        {"PARK", cv::CALIB_HAND_EYE_PARK},
        {"HORAUD", cv::CALIB_HAND_EYE_HORAUD},
        {"ANDREFF", cv::CALIB_HAND_EYE_ANDREFF},
        {"DANIILIDIS", cv::CALIB_HAND_EYE_DANIILIDIS},
    };

    const std::string primary_method = config_.hand_eye_method;
    std::string best_method;
    double best_error = std::numeric_limits<double>::infinity();
    Eigen::Matrix4d chosen_transform = Eigen::Matrix4d::Identity();

    for (const auto& [name, method] : methods) {
      cv::Mat R_out, t_out;
      std::vector<cv::Mat> cv_first_R, cv_target_R;
      std::vector<cv::Mat> cv_first_t, cv_target_t;
      for (size_t i = 0; i < first_R.size(); ++i) {
        cv_first_R.push_back(eigenMatrixToCv(first_R[i]));
        cv_first_t.push_back(eigenVectorToCv(first_t[i]));
        cv_target_R.push_back(eigenMatrixToCv(R_target2cam_list[i]));
        cv_target_t.push_back(eigenVectorToCv(t_target2cam_list[i]));
      }

      try {
        cv::calibrateHandEye(cv_first_R, cv_first_t, cv_target_R, cv_target_t, R_out, t_out,
                             static_cast<cv::HandEyeCalibrationMethod>(method));

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            T(r, c) = R_out.at<double>(r, c);
          }
          T(r, 3) = t_out.at<double>(r, 0);
        }

        const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        const double det = R.determinant();
        const double ortho = (R * R.transpose() - Eigen::Matrix3d::Identity()).norm();
        const auto [mean_err, max_err, std_err] = evaluateConsistency(
            R_gripper2base_list,
            t_gripper2base_list,
            R_target2cam_list,
            t_target2cam_list,
            T,
            eye_in_hand);

        SolveResult result;
        result.name = name;
        result.transform = T;
        result.mean_error_mm = mean_err;
        result.max_error_mm = max_err;
        result.std_error_mm = std_err;
        result.det = det;
        solve_results.push_back(result);

        const std::string suffix = eye_in_hand ? "cam2gripper" : "cam2base";
        saveMatrixTxt(save_dir_path_ / ("T_" + suffix + "_" + name + ".txt"), T);

        if (mean_err < best_error) {
          best_error = mean_err;
          best_method = name;
          chosen_transform = T;
        }

        ROS_INFO_STREAM(name << " mean error: " << mean_err << " mm, det=" << det << ", ortho=" << ortho);
      } catch (const cv::Exception& e) {
        ROS_WARN("calibrateHandEye failed for %s: %s", name.c_str(), e.what());
      }
    }

    if (solve_results.empty()) {
      ROS_ERROR("no hand-eye solution available");
      shutdownRobot();
      return 1;
    }

    const auto primary_it = std::find_if(
        methods.begin(), methods.end(),
        [&](const auto& item) { return item.first == primary_method; });
    const std::string chosen_name =
        (primary_it != methods.end()) ? primary_method : best_method;

    const Eigen::Matrix4d primary_transform =
        selectTransform(solve_results, chosen_name, chosen_transform);
    const Eigen::Matrix4d inverse_transform = inverseRigid(primary_transform);

    const std::string suffix = eye_in_hand ? "cam2gripper" : "cam2base";
    saveMatrixTxt(save_dir_path_ / ("T_" + suffix + ".txt"), primary_transform);
    saveMatrixTxt(save_dir_path_ / (eye_in_hand ? "T_gripper2cam.txt" : "T_base2cam.txt"), inverse_transform);

    auto [mean_err, max_err, std_err] = evaluateConsistency(
        R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list,
        primary_transform, eye_in_hand);

    std::ofstream summary(save_dir_path_ / "calibration_summary.txt");
    summary << "mode=" << config_.mode_name << '\n'
            << "method=" << chosen_name << '\n'
            << "best_method=" << best_method << '\n'
            << "valid_poses=" << valid_count << '\n'
            << "mean_error_mm=" << mean_err << '\n'
            << "max_error_mm=" << max_err << '\n'
            << "std_error_mm=" << std_err << '\n';

    ROS_INFO_STREAM("saved calibration to " << save_dir_path_.string());
    publishStatus("标定完成，已保存结果。有效位姿总数: " + std::to_string(valid_count));
    if (!moveToPose(base_pose)) {
      ROS_WARN("return to start pose failed, trying reset");
      recoverRobot();
    }
    shutdownRobot();
    return 0;
  }

  private:
  std::tuple<double, double, double> evaluateConsistency(
      const std::vector<Eigen::Matrix3d>& R_gripper2base_list,
      const std::vector<Eigen::Vector3d>& t_gripper2base_list,
      const std::vector<Eigen::Matrix3d>& R_target2cam_list,
      const std::vector<Eigen::Vector3d>& t_target2cam_list,
      const Eigen::Matrix4d& T,
      bool eye_in_hand) const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(R_gripper2base_list.size());
    for (size_t i = 0; i < R_gripper2base_list.size(); ++i) {
      Eigen::Matrix4d T_bg = Eigen::Matrix4d::Identity();
      T_bg.block<3, 3>(0, 0) = R_gripper2base_list[i];
      T_bg.block<3, 1>(0, 3) = t_gripper2base_list[i];

      Eigen::Matrix4d T_ct = Eigen::Matrix4d::Identity();
      T_ct.block<3, 3>(0, 0) = R_target2cam_list[i];
      T_ct.block<3, 1>(0, 3) = t_target2cam_list[i];

      Eigen::Vector3d p;
      if (eye_in_hand) {
        const Eigen::Matrix4d T_bt = T_bg * T * T_ct;
        p = T_bt.block<3, 1>(0, 3);
      } else {
        const Eigen::Matrix4d T_gt = inverseRigid(T_bg) * T * T_ct;
        p = T_gt.block<3, 1>(0, 3);
      }
      points.push_back(p);
    }

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
      mean += p;
    }
    mean /= static_cast<double>(points.size());

    std::vector<double> errs;
    errs.reserve(points.size());
    for (const auto& p : points) {
      errs.push_back((p - mean).norm() * 1000.0);
    }

    double sum = 0.0;
    double sum_sq = 0.0;
    double max_err = 0.0;
    for (double err : errs) {
      sum += err;
      sum_sq += err * err;
      max_err = std::max(max_err, err);
    }
    const double mean_err = sum / static_cast<double>(errs.size());
    const double std_err = std::sqrt(std::max(0.0, sum_sq / static_cast<double>(errs.size()) - mean_err * mean_err));
    return {mean_err, max_err, std_err};
  }

  Eigen::Matrix4d selectTransform(const std::vector<SolveResult>& results,
                                  const std::string& name,
                                  const Eigen::Matrix4d& fallback) const {
    const auto it = std::find_if(results.begin(), results.end(),
                                 [&](const SolveResult& item) { return item.name == name; });
    if (it != results.end()) {
      return it->transform;
    }
    return fallback;
  }

  bool prepareSaveDir() {
    std::error_code ec;
    std::filesystem::create_directories(save_dir_path_, ec);
    if (ec) {
      ROS_ERROR_STREAM("failed to create save dir: " << ec.message());
      return false;
    }
    const auto pose_log_path = save_dir_path_ / "accepted_poses.csv";
    if (!std::filesystem::exists(pose_log_path)) {
      std::ofstream pose_log(pose_log_path);
      pose_log << "index,x,y,z,rx,ry,rz,image\n";
    }
    return true;
  }

  bool waitForCameraInfo() {
    const auto start = std::chrono::steady_clock::now();
    ROS_INFO("waiting for camera info on %s ...", config_.camera_info_topic.c_str());
    while (ros::ok()) {
      if (camera_->hasCameraInfo() && camera_->hasImage()) {
        ROS_INFO("camera info received");
        return true;
      }
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed > 30.0) {
        return false;
      }
      if (static_cast<int>(elapsed) % 5 == 0 && static_cast<int>(elapsed) > 0) {
        ROS_INFO_THROTTLE(5, "still waiting for camera... (%.0fs)", elapsed);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  bool connectRobot() {
    constexpr int kMaxRetries = 3;
    for (int attempt = 1; attempt <= kMaxRetries && ros::ok(); ++attempt) {
      ROS_INFO("connecting to robot %s (attempt %d/%d)...", config_.robot_ip.c_str(), attempt, kMaxRetries);
      try {
        robot_ = std::make_unique<xMateRobot>(config_.robot_ip);
        auto check_ec = [&](const char* step) -> bool {
          if (ec_) {
            ROS_WARN_STREAM(step << " failed: " << ec_.message() << " (" << ec_.value() << ")");
            return false;
          }
          return true;
        };

        ec_.clear();
        ROS_INFO("  - setOperateMode(automatic)");
        robot_->setOperateMode(OperateMode::automatic, ec_);
        if (!check_ec("setOperateMode")) throw std::runtime_error("setOperateMode failed");

        ec_.clear();
        ROS_INFO("  - setPowerState(true)");
        robot_->setPowerState(true, ec_);
        if (!check_ec("setPowerState")) throw std::runtime_error("setPowerState failed");

        ec_.clear();
        ROS_INFO("  - setMotionControlMode(NrtCommand)");
        robot_->setMotionControlMode(MotionControlMode::NrtCommand, ec_);
        if (!check_ec("setMotionControlMode")) throw std::runtime_error("setMotionControlMode failed");

        ec_.clear();
        robot_->setDefaultZone(0, ec_);
        if (!check_ec("setDefaultZone")) throw std::runtime_error("setDefaultZone failed");

        ec_.clear();
        robot_->setDefaultSpeed(config_.move_speed, ec_);
        if (!check_ec("setDefaultSpeed")) throw std::runtime_error("setDefaultSpeed failed");

        // 注册全局运动完成事件回调
        ec_.clear();
        robot_->setEventWatcher(Event::moveExecution, [this](const EventInfo& info) {
          try {
            const auto id      = std::any_cast<std::string>(info.at(EventInfoKey::MoveExecution::ID));
            const auto index   = std::any_cast<int>(info.at(EventInfoKey::MoveExecution::WaypointIndex));
            const bool reached = std::any_cast<bool>(info.at(EventInfoKey::MoveExecution::ReachTarget));
            std::error_code move_error;
            try {
              move_error = std::any_cast<std::error_code>(info.at(EventInfoKey::MoveExecution::Error));
            } catch (...) {
            }
            {
              std::lock_guard<std::mutex> lk(move_mutex_);
              last_move_id_      = id;
              last_move_index_   = index;
              last_move_reached_ = reached;
              last_move_error_   = move_error;
              move_event_ready_  = true;
            }
            move_cv_.notify_all();
          } catch (...) {}
        }, ec_);
        if (!check_ec("setEventWatcher")) throw std::runtime_error("setEventWatcher failed");

        ROS_INFO("robot connected successfully");
        return true;
      } catch (const std::runtime_error& e) {
        ROS_WARN("robot connection stage failed: %s", e.what());
        robot_.reset();
      } catch (const rokae::Exception& e) {
        ROS_WARN("robot connection attempt %d failed: %s", attempt, e.what());
        robot_.reset();
      } catch (const std::exception& e) {
        ROS_WARN("robot connection attempt %d failed: %s", attempt, e.what());
        robot_.reset();
      }
      if (attempt < kMaxRetries) {
        ROS_INFO("retrying in 3 seconds...");
        std::this_thread::sleep_for(std::chrono::seconds(3));
      }
    }
    ROS_ERROR("robot connection failed after %d attempts", kMaxRetries);
    return false;
  }

  void recoverRobot() {
    // 先尝试就地恢复（连接还在时）
    try {
      robot_->stop(ec_);
    } catch (...) {}
    std::this_thread::sleep_for(std::chrono::seconds(1));
    try {
      robot_->moveReset(ec_);
    } catch (...) {}
    std::this_thread::sleep_for(std::chrono::seconds(1));
    try {
      robot_->setOperateMode(OperateMode::automatic, ec_);
      robot_->setPowerState(true, ec_);
      robot_->setMotionControlMode(MotionControlMode::NrtCommand, ec_);
      return;
    } catch (...) {}

    // 就地恢复失败，尝试 TCP 重连
    ROS_WARN("in-place recovery failed, attempting TCP reconnection...");
    reconnectRobot();
  }

  bool reconnectRobot(int max_retries = 3) {
    for (int attempt = 1; attempt <= max_retries && ros::ok(); ++attempt) {
      ROS_INFO("reconnect attempt %d/%d to %s ...", attempt, max_retries, config_.robot_ip.c_str());
      try {
        robot_.reset();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        robot_ = std::make_unique<xMateRobot>(config_.robot_ip);
        auto check_ec = [&](const char* step) -> bool {
          if (ec_) {
            ROS_WARN_STREAM(step << " failed: " << ec_.message() << " (" << ec_.value() << ")");
            return false;
          }
          return true;
        };

        ec_.clear();
        robot_->setOperateMode(OperateMode::automatic, ec_);
        if (!check_ec("setOperateMode")) throw std::runtime_error("setOperateMode failed");

        ec_.clear();
        robot_->setPowerState(true, ec_);
        if (!check_ec("setPowerState")) throw std::runtime_error("setPowerState failed");

        ec_.clear();
        robot_->setMotionControlMode(MotionControlMode::NrtCommand, ec_);
        if (!check_ec("setMotionControlMode")) throw std::runtime_error("setMotionControlMode failed");

        ec_.clear();
        robot_->setDefaultZone(0, ec_);
        if (!check_ec("setDefaultZone")) throw std::runtime_error("setDefaultZone failed");

        ec_.clear();
        robot_->setDefaultSpeed(config_.move_speed, ec_);
        if (!check_ec("setDefaultSpeed")) throw std::runtime_error("setDefaultSpeed failed");

        ec_.clear();
        robot_->setEventWatcher(Event::moveExecution, [this](const EventInfo& info) {
          try {
            const auto id      = std::any_cast<std::string>(info.at(EventInfoKey::MoveExecution::ID));
            const auto index   = std::any_cast<int>(info.at(EventInfoKey::MoveExecution::WaypointIndex));
            const bool reached = std::any_cast<bool>(info.at(EventInfoKey::MoveExecution::ReachTarget));
            std::error_code move_error;
            try {
              move_error = std::any_cast<std::error_code>(info.at(EventInfoKey::MoveExecution::Error));
            } catch (...) {
            }
            {
              std::lock_guard<std::mutex> lk(move_mutex_);
              last_move_id_      = id;
              last_move_index_   = index;
              last_move_reached_ = reached;
              last_move_error_   = move_error;
              move_event_ready_  = true;
            }
            move_cv_.notify_all();
          } catch (...) {}
        }, ec_);
        if (!check_ec("setEventWatcher")) throw std::runtime_error("setEventWatcher failed");

        ROS_INFO("reconnected successfully on attempt %d", attempt);
        return true;
      } catch (const std::runtime_error& e) {
        ROS_WARN("reconnect attempt %d failed: %s", attempt, e.what());
        robot_.reset();
      } catch (const rokae::Exception& e) {
        ROS_WARN("reconnect attempt %d failed: %s", attempt, e.what());
        robot_.reset();
      } catch (...) {
        ROS_WARN("reconnect attempt %d failed (unknown error)", attempt);
        robot_.reset();
      }
      std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    ROS_ERROR("all reconnect attempts failed");
    return false;
  }

  bool moveToPose(const std::array<double, 6>& pose) {
    std::string cmd_id;
    try {
      MoveJCommand cmd({pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]}, config_.move_speed, 0);
      robot_->moveAppend({cmd}, cmd_id, ec_);
      {
        std::lock_guard<std::mutex> lk(move_mutex_);
        move_event_ready_ = false;
        last_move_id_.clear();
        last_move_index_ = -1;
        last_move_reached_ = false;
        last_move_error_.clear();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      robot_->moveStart(ec_);
      return waitForFinish(cmd_id, 0);
    } catch (const rokae::Exception& e) {
      ROS_WARN("moveToPose failed: %s", e.what());
      return false;
    }
  }

  bool waitForFinish(const std::string& traj_id, int index) {
    std::unique_lock<std::mutex> lk(move_mutex_);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(config_.wait_timeout);
    while (ros::ok()) {
      if (move_event_ready_ && last_move_id_ == traj_id && last_move_index_ == index) {
        if (last_move_error_) {
          ROS_ERROR_STREAM("move execution error: " << last_move_error_.message());
          return false;
        }
        return last_move_reached_;
      }

      if (move_cv_.wait_until(lk, deadline) == std::cv_status::timeout) {
        ROS_WARN("waitForFinish timed out after %.1f s", config_.wait_timeout);
        return false;
      }
    }
    return false;
  }

  void shutdownRobot() {
    if (!robot_) return;
    try {
      robot_->setPowerState(false, ec_);
      robot_->disconnectFromRobot(ec_);
    } catch (...) {
    }
  }

  void loadConfig() {
    nh_.param<std::string>("robot_ip", config_.robot_ip, "192.168.2.161");
    nh_.param<std::string>("mode", config_.mode_name, std::string("eye_in_hand"));
    nh_.param<std::string>("image_topic", config_.image_topic, std::string());
    nh_.param<std::string>("camera_info_topic", config_.camera_info_topic, std::string());
    nh_.param<std::string>("hand_eye_method", config_.hand_eye_method, std::string("TSAI"));
    nh_.param("checker_rows", config_.checker_rows, config_.checker_rows);
    nh_.param("checker_cols", config_.checker_cols, config_.checker_cols);
    nh_.param("square_size", config_.square_size, config_.square_size);
    nh_.param("move_speed", config_.move_speed, config_.move_speed);
    nh_.param("settle_time", config_.settle_time, config_.settle_time);
    nh_.param("num_captures", config_.num_captures, config_.num_captures);
    nh_.param("num_poses", config_.num_poses, config_.num_poses);
    nh_.param("pos_dx", config_.pos_dx, config_.pos_dx);
    nh_.param("pos_dy", config_.pos_dy, config_.pos_dy);
    nh_.param("pos_dz", config_.pos_dz, config_.pos_dz);
    nh_.param("rot_delta", config_.rot_delta, config_.rot_delta);
    nh_.param("wait_timeout", config_.wait_timeout, config_.wait_timeout);
    nh_.param("decision_timeout", config_.decision_timeout, config_.decision_timeout);
    nh_.param("interactive_confirm", config_.interactive_confirm, config_.interactive_confirm);
    nh_.param("show_opencv_preview", config_.show_opencv_preview, config_.show_opencv_preview);
    config_.run_stamp = makeTimestampStamp();

    if (config_.mode_name == "eye_to_hand") {
      config_.mode = Mode::EyeToHand;
      if (config_.image_topic.empty()) {
        config_.image_topic = "/ap_robot/calib_raw_img";
      }
      if (config_.camera_info_topic.empty()) {
        config_.camera_info_topic = "/ap_robot/camera_info";
      }
      if (config_.checker_rows == 8 && config_.checker_cols == 6) {
        config_.checker_rows = 5;
        config_.checker_cols = 5;
      }
      if (config_.square_size == 0.0405) {
        config_.square_size = 0.0285;
      }
      config_.save_dir = "calibration_results_eye_to_hand";
    } else {
      config_.mode = Mode::EyeInHand;
      if (config_.image_topic.empty()) {
        config_.image_topic = "/camera/color/image_raw";
      }
      if (config_.camera_info_topic.empty()) {
        config_.camera_info_topic = "/camera/color/camera_info";
      }
      config_.save_dir = "calibration_results";
    }

    const std::string package_path = ros::package::getPath("rokae");
    if (package_path.empty()) {
      save_dir_path_ = std::filesystem::current_path() / config_.save_dir / config_.run_stamp;
    } else {
      save_dir_path_ = std::filesystem::path(package_path) / "src" / config_.save_dir / config_.run_stamp;
    }
    ROS_INFO_STREAM("image topic: " << config_.image_topic);
    ROS_INFO_STREAM("camera info topic: " << config_.camera_info_topic);
    ROS_INFO_STREAM("save dir: " << save_dir_path_.string());
  }

  static std::string padIndex(int idx) {
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << idx;
    return ss.str();
  }

  ros::NodeHandle nh_;
  Config config_;
  std::filesystem::path save_dir_path_;
  std::unique_ptr<CameraBuffer> camera_;
  std::unique_ptr<xMateRobot> robot_;
  std::error_code ec_;
  cv::Mat last_image_;
  ros::Publisher preview_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber decision_sub_;
  std::mutex decision_mutex_;
  std::condition_variable decision_cv_;
  int latest_decision_{0};
  bool decision_received_{false};
  // 运动完成事件（由 setEventWatcher 回调写入，waitForFinish 读取）
  std::mutex move_mutex_;
  std::condition_variable move_cv_;
  std::string last_move_id_;
  int last_move_index_{-1};
  bool last_move_reached_{false};
  std::error_code last_move_error_;
  bool move_event_ready_{false};

  void publishStatus(const std::string& text) {
    ROS_INFO_STREAM(text);
    std_msgs::String msg;
    msg.data = text;
    status_pub_.publish(msg);
  }
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "auto_handeye_calibration");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  AutoHandEyeCalibrator app(nh);
  return app.run();
}
