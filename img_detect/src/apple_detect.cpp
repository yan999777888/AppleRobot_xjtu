#include "ros/ros.h"
#include "img_detect/Apple.h"
#include "img_detect/yolov8_onnx.h"
#include "img_detect/yolov8_utils.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sl/Camera.hpp>

using namespace std;
using namespace sl;

cv::Mat rgb_image;
sensor_msgs::PointCloud2ConstPtr cloud_msg;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // 将 ROS 图像消息转换为 OpenCV 图像
        rgb_image = cv_bridge::toCvCopy(msg, "bgra8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgra8'.", msg->encoding.c_str());
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    cloud_msg = msg;
}

//使用ZED ROS包
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"apple_detect");
    ros::NodeHandle nh;
    ros::Publisher pub_chatter_appleInfo = nh.advertise<img_detect::Apple>("ap_robot/chatter_appleInfo",100);
    ros::Subscriber pointCloud_sub = nh.subscribe("/zed2/zed_node/point_cloud/cloud_registered", 1, pointCloudCallback);
    // 创建图像传输对象
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub = it.subscribe("/zed2/zed_node/rgb/image_rect_color", 1, imageCallback);
    image_transport::Publisher pub_detectImg = it.advertise("ap_robot/chatter_detectImg", 1);

    //初始化yolo模型
    Yolov8Onnx task_detect_onnx; //创建检测器
    string detect_model_path = "best.onnx"; //模型加载路径
    if (task_detect_onnx.ReadModel(detect_model_path, true)) {
		ROS_INFO("yolo模型加载成功!" );
	}
    else {
		ROS_INFO("yolo模型加载失败!" ); 
		return -1;
	}
    vector<OutputSeg> detect_result;
    int img_height, img_width;
    cv::Rect box;
    img_detect::Apple apple;
    int rect_x, rect_y, rect_width, rect_height, rect_centerx, rect_centery;
    int point_index;
    float x, y, z;
    sensor_msgs::ImagePtr detected_image;// 创建ROS图像消息对象

    ros::Rate rate(10); // 设置发布频率为10Hz
    while (nh.ok())
    {   
        if(cloud_msg != nullptr && task_detect_onnx.OnnxDetect(rgb_image, detect_result)){
            for(int i = 0; i < detect_result.size();i++){
                box = detect_result[i].box;     //获取矩形框中心的像素坐标
                rect_x = detect_result[i].box.x;
                rect_y = detect_result[i].box.y;
                rect_width = detect_result[i].box.width;
                rect_height = detect_result[i].box.height;
                rect_centerx = rect_x + rect_width / 2;
                rect_centery = rect_y + rect_height / 2;
                // 计算点云数据中的索引
                point_index = rect_centerx * cloud_msg->width + rect_centery;
                // 检查索引是否有效
                if (point_index >= cloud_msg->width * cloud_msg->height) {
                    return false;
                }
                // 获取点的起始位置
                const uint8_t* point_ptr = &cloud_msg->data[point_index * cloud_msg->point_step];

                // 解析 x, y, z 字段
                x = *reinterpret_cast<const float*>(point_ptr + cloud_msg->fields[0].offset);
                y = *reinterpret_cast<const float*>(point_ptr + cloud_msg->fields[1].offset);
                z = *reinterpret_cast<const float*>(point_ptr + cloud_msg->fields[2].offset);

                if (std::isnan(z) || std::isinf(z) || z <= 0) {
                    // std::cout << "Invalid depth value at the given pixel coordinates." << std::endl;
                    continue;
                }
                // std::cout << "点云坐标为: x=" << camera_point.x << " y=" << camera_point.y << " z=" << camera_point.z << std::endl;
                // float width = rect_width * z / fx;
                // float height = rect_height * z / fy;
                // std::cout << "苹果直径大小为 " << min(width, height) << " m" << std::endl;

                apple.header.stamp = ros::Time::now();
                apple.id.push_back(i);
                apple.x.push_back(x);
                apple.y.push_back(y);
                apple.z.push_back(z);
                // apple.size.push_back(min(width, height));

                rectangle(rgb_image, box, cv::Scalar(0, 255, 0), 3, 8);   //画出矩形
            }
            pub_chatter_appleInfo.publish(apple);
            detected_image = cv_bridge::CvImage(std_msgs::Header(), "bgra8", rgb_image).toImageMsg();// 将OpenCV格式的图像转换为ROS图像消息
            // cv::Mat cvImage = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGRA8)->image;
            // cv::imshow("cvImage", cvImage);
            // cv::waitKey(100);
            pub_detectImg.publish(detected_image);// 发布图像消息
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
