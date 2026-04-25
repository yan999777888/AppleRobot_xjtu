#include "ros/ros.h"
#include "task_assign/Target.h"
#include "img_detect/Apple.h"
#include "img_detect/yolov8_onnx.h"
#include "img_detect/yolov8_utils.h"// 你的头文件
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <atomic>

// PCL 库
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace sl;

// 参数
const float APPLE_R_MIN = 0.025; // 最小半径
const float APPLE_R_MAX = 0.055; // 最大半径
const int STABILITY_FRAMES = 5;  // 防抖帧数

struct Point3D { float x, y, z; };
ros::Publisher pub_obstacles;
ros::Publisher pub_debug_img;
ros::Publisher pub_apples;

// [核心] 点云处理：拟合苹果 + 提取障碍物
// ---------------------------------------------------------
// [终极调试版] 处理点云：分级保底 + 详细日志
// ---------------------------------------------------------
// ---------------------------------------------------------
// [稳定版] 前景滤波 + 质心计算
// 只计算距离相机最近的 10cm 范围内的点 (过滤背景树叶)
// ---------------------------------------------------------
Point3D processPointCloud(const sl::Mat& zed_cloud, const cv::Rect& box) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. 提取 ROI 点云 (全分辨率采样)
    float min_z = 100.0; // 用于寻找最近点
    
    for (int r = box.y; r < box.y + box.height; r++) { 
        for (int c = box.x; c < box.x + box.width; c++) {
            sl::float4 pt;
            zed_cloud.getValue(c, r, &pt);
            
            // 基础过滤：只取有效且在合理范围内的点
            if (std::isfinite(pt.z) && pt.z > 0.2 && pt.z < 2.0) {
                cloud->points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
                // 记录最近点的深度
                if (pt.z < min_z) min_z = pt.z;
            }
        }
    }
    
    // 用英文日志，避免乱码
    ROS_INFO("ROI Size: %dx%d | Total Points: %ld | Min Depth: %.3f", 
             box.width, box.height, cloud->size(), min_z);

    if (cloud->size() < 10) {
        ROS_WARN("Points < 10, Blind Mode Active");
        // 盲抓保底：假设深度 0.6m
        return {0.0, 0.0, 0.6}; 
    }

    // ==========================================
    // [核心算法] 前景滤波 (Foreground Filtering)
    // ==========================================
    // 原理：苹果一定是框里离相机最近的物体。
    // 我们只取 [min_z, min_z + 10cm] 范围内的点来算平均值。
    // 这样可以完美剔除背景的树叶和树枝。
    
    float sx = 0, sy = 0, sz = 0;
    int valid_fg_count = 0;
    float depth_threshold = min_z + 0.10; // 只看最近的 10cm 厚度

    // 用于发布避障点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& p : cloud->points) {
        if (p.z <= depth_threshold) {
            // 这是苹果表面的点
            sx += p.x;
            sy += p.y;
            sz += p.z;
            valid_fg_count++;
        } else {
            // 这是背景或障碍物
            obstacles->points.push_back(p);
        }
    }

    if (valid_fg_count == 0) return {0,0,0}; // 理论上不可能发生

    // 计算前景质心
    Point3D centroid;
    centroid.x = sx / valid_fg_count;
    centroid.y = sy / valid_fg_count;
    centroid.z = sz / valid_fg_count;

    // 半径补偿：质心在苹果表面，我们要抓球心，所以往里推 4cm
    centroid.z += 0.04; 

    // 发布障碍物 (背景点)
    if (!obstacles->empty()) {
        // 降采样一下，防止太密
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(obstacles);
        sor.setLeafSize(0.02f, 0.02f, 0.02f);
        sor.filter(*cloud_filtered);

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud_filtered, msg);
        msg.header.frame_id = "zed_left_camera_frame"; 
        msg.header.stamp = ros::Time::now();
        pub_obstacles.publish(msg);
    }

    ROS_INFO("Stable Target: [%.3f, %.3f, %.3f] (Fg Points: %d)", 
             centroid.x, centroid.y, centroid.z, valid_fg_count);
    
    return centroid;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    pub_obstacles = nh.advertise<sensor_msgs::PointCloud2>("ap_robot/obstacles", 1);
    pub_debug_img = nh.advertise<sensor_msgs::Image>("ap_robot/debug_img", 1);
    pub_apples    = nh.advertise<img_detect::Apple>("ap_robot/chatter_appleInfo", 1);

    Yolov8Onnx yolo;
    yolo.ReadModel("/home/y/dual_rokae_ws/src/img_detect/src/best.onnx", true);

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    init_params.coordinate_units = sl::UNIT::METER;
    zed.open(init_params);

    // 每个检测框独立防抖缓冲（按检测框索引存，简化为全局最近帧列表）
    // 使用 map<int, deque> 按苹果 id 防抖过于复杂，此处改为：
    // 每帧把所有稳定苹果打包发布，防抖由 task_assign 侧的策略决定
    bool auto_mode = false;

    while (nh.ok()) {
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            sl::Mat left, zed_cloud;
            zed.retrieveImage(left, VIEW::LEFT);
            zed.retrieveMeasure(zed_cloud, MEASURE::XYZ);

            cv::Mat frame(left.getHeight(), left.getWidth(), CV_8UC4, left.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::Mat rgb; cv::cvtColor(frame, rgb, cv::COLOR_BGRA2BGR);

            vector<OutputSeg> results;

            if (yolo.OnnxDetect(rgb, results) && auto_mode) {
                img_detect::Apple apple_msg;
                apple_msg.header.stamp = ros::Time::now();

                for (uint32_t i = 0; i < results.size(); i++) {
                    auto& res = results[i];
                    Point3D center = processPointCloud(zed_cloud, res.box);

                    // 过滤无效点
                    if (center.z <= 0 || center.z > 2.0) continue;

                    apple_msg.id.push_back(i);
                    apple_msg.x.push_back(center.x);
                    apple_msg.y.push_back(center.y);
                    apple_msg.z.push_back(center.z);
                    apple_msg.size.push_back(
                        (res.box.width + res.box.height) * 0.5f / rgb.cols);

                    cv::rectangle(rgb, res.box, cv::Scalar(0,255,0), 2);
                    cv::putText(rgb,
                        "x:" + to_string(center.x).substr(0,5),
                        {res.box.x, res.box.y - 5}, 1, 1.2, {0,255,255}, 1);
                }

                if (!apple_msg.id.empty()) {
                    pub_apples.publish(apple_msg);
                    ROS_INFO_THROTTLE(1.0, "发布 %ld 个苹果", apple_msg.id.size());
                }
            } else if (yolo.OnnxDetect(rgb, results)) {
                // MANUAL 模式只画框不发布
                for (auto& res : results)
                    cv::rectangle(rgb, res.box, cv::Scalar(0,200,0), 2);
            }

            cv::putText(rgb, auto_mode ? "AUTO" : "MANUAL", {20,40}, 1, 2, {0,255,0}, 2);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb).toImageMsg();
            pub_debug_img.publish(msg);

            cv::imshow("ZED View", rgb);

            char key = cv::waitKey(10);
            if(key=='a') auto_mode = !auto_mode;
            if(key=='q') break;
        }
        ros::spinOnce();
    }
    zed.close();
    return 0;
}