#include "ros/ros.h"
#include "task_assign/Target.h"
#include "img_detect/yolov8_onnx.h"
#include "img_detect/yolov8_utils.h"
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL 库
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

// 【新增】RViz 可视化消息头文件
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace sl;

// 参数
const int STABILITY_FRAMES = 5;  // 防抖帧数

struct Point3D { float x, y, z; };

ros::Publisher pub_obstacles;
ros::Publisher pub_debug_img;
ros::Publisher pub_marker; // 【新增】发布目标 Marker

// ---------------------------------------------------------
// [稳定版] 前景滤波 + 质心计算 + 障碍物分离
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
        return {0.0, 0.0, 0.6}; // 盲抓保底
    }

    // ==========================================
    // [核心算法] 前景滤波 (Foreground Filtering)
    // ==========================================
    
    float sx = 0, sy = 0, sz = 0;
    int valid_fg_count = 0;
    float depth_threshold = min_z + 0.10; // 只看最近的 10cm 厚度

    // 用于发布避障点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& p : cloud->points) {
        if (p.z <= depth_threshold) {
            // 这是苹果表面的点
            sx += p.x; sy += p.y; sz += p.z;
            valid_fg_count++;
        } else {
            // 这是背景或障碍物
            obstacles->points.push_back(p);
        }
    }

    if (valid_fg_count == 0) return {0,0,0}; 

    // 计算前景质心
    Point3D centroid;
    centroid.x = sx / valid_fg_count;
    centroid.y = sy / valid_fg_count;
    centroid.z = sz / valid_fg_count;

    // 半径补偿：质心在苹果表面，我们要抓球心，所以往里推 4cm
    centroid.z += 0.04; 

    // 发布障碍物 (背景点) -> 这部分你之前已经有了，现在可以在 RViz 里看到
    if (!obstacles->empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(obstacles);
        sor.setLeafSize(0.02f, 0.02f, 0.02f); // 降采样
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
    ros::ServiceClient client = nh.serviceClient<task_assign::Target>("left_pickTarget");
    
    // 初始化 Publisher
    pub_obstacles = nh.advertise<sensor_msgs::PointCloud2>("ap_robot/obstacles", 1);
    pub_debug_img = nh.advertise<sensor_msgs::Image>("ap_robot/debug_img", 1);
    // 【新增】发布目标 Marker
    pub_marker = nh.advertise<visualization_msgs::Marker>("ap_robot/target_marker", 1);

    Yolov8Onnx yolo;
    yolo.ReadModel("/home/y/dual_rokae_ws/src/img_detect/src/best.onnx", true);

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    init_params.coordinate_units = sl::UNIT::METER;
    zed.open(init_params);

    std::deque<Point3D> buffer;
    bool auto_mode = false;
    bool is_busy = false;

    while (nh.ok()) {
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            sl::Mat left, zed_cloud;
            zed.retrieveImage(left, VIEW::LEFT);
            zed.retrieveMeasure(zed_cloud, MEASURE::XYZ);

            cv::Mat frame(left.getHeight(), left.getWidth(), CV_8UC4, left.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::Mat rgb; cv::cvtColor(frame, rgb, cv::COLOR_BGRA2BGR);

            vector<OutputSeg> results;
            Point3D best_apple = {0,0,0};
            bool found = false;

            if (yolo.OnnxDetect(rgb, results)) {
                float min_dist = 100.0;
                for (auto& res : results) {
                    Point3D center = processPointCloud(zed_cloud, res.box);
                    if (center.z > 0 && center.z < min_dist) {
                        min_dist = center.z;
                        best_apple = center;
                        found = true;
                    }
                    cv::rectangle(rgb, res.box, cv::Scalar(0,255,0), 2);
                }
            }

            // ==========================================
            // 【新增】RViz 可视化逻辑
            // ==========================================
            if (found) {
                visualization_msgs::Marker marker;
                // 注意：Frame ID 必须和 PointCloud 的 Frame ID 一致
                marker.header.frame_id = "zed_left_camera_frame"; 
                marker.header.stamp = ros::Time::now();
                marker.ns = "apple_target";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::SPHERE; // 画一个球
                marker.action = visualization_msgs::Marker::ADD;
                
                // 设置位置 (YOLO+PCL 算出来的坐标)
                marker.pose.position.x = best_apple.x;
                marker.pose.position.y = best_apple.y;
                marker.pose.position.z = best_apple.z;
                marker.pose.orientation.w = 1.0;

                // 设置尺寸 (直径 8cm)
                marker.scale.x = 0.08;
                marker.scale.y = 0.08;
                marker.scale.z = 0.08;

                // 设置颜色 (绿色，半透明)
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.8;

                marker.lifetime = ros::Duration(0.1); // 只显示0.1秒，防止重影
                pub_marker.publish(marker);
            }
            // ==========================================

            if (found) buffer.push_back(best_apple);
            else if (!buffer.empty()) buffer.pop_front();
            if (buffer.size() > STABILITY_FRAMES) buffer.pop_front();

            if (auto_mode && !is_busy && buffer.size() == STABILITY_FRAMES) {
                float sx=0,sy=0,sz=0;
                for(auto& p:buffer){sx+=p.x; sy+=p.y; sz+=p.z;}
                
                is_busy = true;
                task_assign::Target srv;
                srv.request.x = sx/buffer.size();
                srv.request.y = sy/buffer.size();
                srv.request.z = sz/buffer.size();
                
                ROS_INFO("发送目标: [%.3f, %.3f, %.3f]", srv.request.x, srv.request.y, srv.request.z);
                client.call(srv);
                buffer.clear();
                is_busy = false;
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