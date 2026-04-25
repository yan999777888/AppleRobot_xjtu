#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace {

struct CalibrationConfig {
  int checker_rows{5};
  int checker_cols{5};
  double square_size{0.0285};
  bool show_window{false};
  std::string frame_id{"zed_left_camera_frame"};
};

cv::Mat buildCameraMatrix(const sl::CameraInformation& camera_info) {
  const auto& left = camera_info.camera_configuration.calibration_parameters.left_cam;
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = left.fx;
  K.at<double>(1, 1) = left.fy;
  K.at<double>(0, 2) = left.cx;
  K.at<double>(1, 2) = left.cy;
  return K;
}

cv::Mat buildDistCoeffs() {
  return cv::Mat::zeros(1, 5, CV_64F);
}

sensor_msgs::CameraInfo buildCameraInfo(const cv::Mat& K,
                                        const cv::Mat& D,
                                        int width,
                                        int height,
                                        const std::string& frame_id) {
  sensor_msgs::CameraInfo msg;
  msg.header.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(width);
  msg.height = static_cast<uint32_t>(height);
  msg.distortion_model = "plumb_bob";

  for (int i = 0; i < 9; ++i) {
    msg.K[i] = K.at<double>(i / 3, i % 3);
    msg.R[i] = (i % 4 == 0) ? 1.0 : 0.0;
  }

  msg.P[0] = K.at<double>(0, 0);
  msg.P[1] = 0.0;
  msg.P[2] = K.at<double>(0, 2);
  msg.P[3] = 0.0;
  msg.P[4] = 0.0;
  msg.P[5] = K.at<double>(1, 1);
  msg.P[6] = K.at<double>(1, 2);
  msg.P[7] = 0.0;
  msg.P[8] = 0.0;
  msg.P[9] = 0.0;
  msg.P[10] = 1.0;
  msg.P[11] = 0.0;

  msg.D.resize(static_cast<size_t>(D.cols));
  for (int i = 0; i < D.cols; ++i) {
    msg.D[static_cast<size_t>(i)] = D.at<double>(0, i);
  }

  return msg;
}

bool detectCheckerboard(const cv::Mat& bgr,
                        const cv::Mat& camera_matrix,
                        const cv::Mat& dist_coeffs,
                        const cv::Size& board_size,
                        double square_size,
                        cv::Mat& vis) {
  vis = bgr.clone();
  if (bgr.empty()) {
    return false;
  }

  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  const int flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                    cv::CALIB_CB_NORMALIZE_IMAGE |
                    cv::CALIB_CB_FAST_CHECK;
  std::vector<cv::Point2f> corners;
  const bool found = cv::findChessboardCorners(gray, board_size, corners, flags);
  if (!found) {
    return false;
  }

  cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
  cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

  cv::drawChessboardCorners(vis, board_size, corners, found);

  if (!camera_matrix.empty()) {
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
    if (cv::solvePnP(object_points, corners, camera_matrix, dist_coeffs, rvec, tvec)) {
      cv::drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, square_size * 2.0);
    }
  }

  return true;
}

void overlayText(cv::Mat& image, const std::string& text, const cv::Point& origin, double scale,
                 const cv::Scalar& color, int thickness) {
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  CalibrationConfig cfg;
  pnh.param("checker_rows", cfg.checker_rows, cfg.checker_rows);
  pnh.param("checker_cols", cfg.checker_cols, cfg.checker_cols);
  pnh.param("square_size", cfg.square_size, cfg.square_size);
  pnh.param("show_window", cfg.show_window, cfg.show_window);
  pnh.param<std::string>("frame_id", cfg.frame_id, cfg.frame_id);

  ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("ap_robot/debug_img", 1);
  ros::Publisher pub_calib_raw_image = nh.advertise<sensor_msgs::Image>("ap_robot/calib_raw_img", 1);
  ros::Publisher pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("ap_robot/camera_info", 1, true);

  sl::Camera zed;
  sl::InitParameters init_params;
  init_params.coordinate_units = sl::UNIT::METER;
  init_params.camera_resolution = sl::RESOLUTION::HD720;
  init_params.camera_fps = 30;

  if (zed.open(init_params) != sl::ERROR_CODE::SUCCESS) {
    ROS_ERROR("ZED open failed");
    return 1;
  }

  const sl::CameraInformation camera_info = zed.getCameraInformation();
  const cv::Mat camera_matrix = buildCameraMatrix(camera_info);
  const cv::Mat dist_coeffs = buildDistCoeffs();

  sensor_msgs::CameraInfo cam_msg = buildCameraInfo(camera_matrix, dist_coeffs, 0, 0, cfg.frame_id);

  if (cfg.show_window) {
    cv::namedWindow("vision_calibration_preview", cv::WINDOW_NORMAL);
  }

  ros::Rate rate(30);
  sl::Mat left_image;
  while (ros::ok()) {
    if (zed.grab() != sl::ERROR_CODE::SUCCESS) {
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    zed.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::CPU);
    const int width = left_image.getWidth();
    const int height = left_image.getHeight();
    cv::Mat bgra(height, width, CV_8UC4, left_image.getPtr<sl::uchar1>(sl::MEM::CPU));

    cv::Mat bgr;
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

    cv::Mat vis;
    const bool found = detectCheckerboard(
        bgr, camera_matrix, dist_coeffs,
        cv::Size(cfg.checker_cols, cfg.checker_rows),
        cfg.square_size, vis);

    overlayText(vis, "Handeye calibration source", cv::Point(20, 36), 0.85,
                cv::Scalar(0, 255, 255), 2);
    overlayText(vis,
                found ? "Checkerboard found" : "Searching checkerboard",
                cv::Point(20, 70), 0.7,
                found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
    overlayText(vis,
                "Publish: /ap_robot/debug_img  /ap_robot/camera_info",
                cv::Point(20, 104), 0.6,
                cv::Scalar(255, 255, 255), 1);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = cfg.frame_id;

    cam_msg.header = header;
    cam_msg.width = static_cast<uint32_t>(width);
    cam_msg.height = static_cast<uint32_t>(height);
    pub_camera_info.publish(cam_msg);

    const sensor_msgs::ImagePtr raw_msg =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, bgr).toImageMsg();
    pub_calib_raw_image.publish(raw_msg);

    const sensor_msgs::ImagePtr image_msg =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, vis).toImageMsg();
    pub_image.publish(image_msg);

    if (cfg.show_window) {
      cv::imshow("vision_calibration_preview", vis);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q') {
        break;
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  zed.close();
  if (cfg.show_window) {
    cv::destroyAllWindows();
  }
  return 0;
}
