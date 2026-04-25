#include "ros/ros.h"
#include "img_detect/Apple.h"
#include "img_detect/yolov8_onnx.h"
#include "img_detect/yolov8_utils.h"
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace sl;

//使用ZED SDK
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"img_detect");
    ros::NodeHandle nh;
    ros::Publisher pub_chatter_appleInfo = nh.advertise<img_detect::Apple>("ap_robot/chatter_appleInfo",100);
    ros::Publisher pub_detectImg = nh.advertise<sensor_msgs::Image>("ap_robot/chatter_detectImg", 10);
    //初始化yolo模型
    Yolov8Onnx task_detect_onnx; //创建检测器
    string detect_model_path = "/home/ubuntu/dual_rokae_ws/src/img_detect/src/best.onnx"; //模型加载路径
    if (task_detect_onnx.ReadModel(detect_model_path, true)) {
		ROS_INFO("yolo模型加载成功!" );
	}
    else {
		ROS_INFO("yolo模型加载失败!" ); 
		return -1;
	}

    //初始化ZED相机
    Camera zed;
    InitParameters init_params;//设置相机初始参数
    init_params.sdk_verbose=true;
    init_params.depth_mode=DEPTH_MODE::ULTRA;
    init_params.coordinate_units=UNIT::METER; //以毫米为单位
    init_params.camera_resolution = RESOLUTION::HD1080;
    init_params.depth_stabilization=true;  //稳定深度
    init_params.camera_fps=30;
    init_params.coordinate_system=COORDINATE_SYSTEM::IMAGE;//使用符合opencv的图像坐标系，RIGHT_HANDED_Z_UP_X_FWD适用于ros
    // init_params.depth_minimum_distance=30;  //设置最小距离0.3m
    // init_params.depth_maximum_distance=300; //设置最大距离5m
    
    RuntimeParameters runParameters;//设置运行时动态参数
    runParameters.confidence_threshold=95; //深度置信度，低于置信度70的深度值被认为是准确的
    // runParameters.enable_depth=true;
    // runParameters.enable_fill_mode=false;//采用标准感知模式
    // runParameters.texture_confidence_threshold=100;
    if(zed.open(init_params) != ERROR_CODE::SUCCESS)
    {
        ROS_INFO("zed2 open failed!");
        return -1;
    }
 
    vector<OutputSeg> detect_result;
    cv::Mat cv_left_img;
    int img_height, img_width;
    sensor_msgs::ImagePtr ros_image;// 创建ROS图像消息对象

    // 获取相机内参
    sl::CameraInformation camera_info = zed.getCameraInformation();
    float fx = camera_info.camera_configuration.calibration_parameters.left_cam.fx;
    float fy = camera_info.camera_configuration.calibration_parameters.left_cam.fy;
    // float cx = camera_info.camera_configuration.calibration_parameters.left_cam.cx;
    // float cy = camera_info.camera_configuration.calibration_parameters.left_cam.cy;

    ros::Rate rate(10); // 设置发布频率为10Hz
    while (nh.ok())
    {
        if(zed.grab(runParameters) != ERROR_CODE::SUCCESS)  //从相机中获取最新图像，并进行矫正
        {
            ROS_INFO("获取图像失败!");
            continue;
        }
        sl::Mat rected_left_img;  //获取sl格式左图
        sl::Mat depth_image;  //获取sl格式深度图
        zed.retrieveImage(rected_left_img,VIEW::LEFT, sl::MEM::CPU);
        zed.retrieveMeasure(depth_image, sl::MEASURE::DEPTH, sl::MEM::CPU);
        img_height = rected_left_img.getHeight();
        img_width = rected_left_img.getWidth();
        cv_left_img = cv::Mat(img_height, img_width, CV_8UC4, rected_left_img.getPtr<sl::uchar1>(sl::MEM::CPU));

        img_detect::Apple apple;
        if(task_detect_onnx.OnnxDetect(cv_left_img, detect_result)){
            
            for(int i = 0; i < detect_result.size();i++){
                cv::Rect box = detect_result[i].box;     //获取矩形框中心的像素坐标
                int rect_x = detect_result[i].box.x;
                int rect_y = detect_result[i].box.y;
                int rect_width = detect_result[i].box.width;
                int rect_height = detect_result[i].box.height;
                int rect_centerx = rect_x + rect_width / 2;
                int rect_centery = rect_y + rect_height / 2;

                // 获取点云数据并转换为相机坐标
                sl::Mat point_cloud;
                zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZ);
                sl::float4 camera_point;
                point_cloud.getValue(rect_centerx, rect_centery, &camera_point);                
                if (std::isnan(camera_point.z) || std::isinf(camera_point.z) || camera_point.z <= 0) {
                    // std::cout << "Invalid depth value at the given pixel coordinates." << std::endl;
                    continue;
                }

                // std::cout << "点云坐标为: x=" << camera_point.x << " y=" << camera_point.y << " z=" << camera_point.z << std::endl;
                float width = rect_width * camera_point.z / fx;
                float height = rect_height * camera_point.z / fy;
                // std::cout << "苹果直径大小为 " << min(width, height) << " m" << std::endl;

                apple.header.stamp = ros::Time::now();
                apple.id.push_back(i);
                apple.x.push_back(camera_point.x);
                apple.y.push_back(camera_point.y);
                apple.z.push_back(camera_point.z);
                apple.size.push_back(min(width, height));

                rectangle(cv_left_img, box, cv::Scalar(0, 255, 0), 3, 8);   //画出矩形
            }
            pub_chatter_appleInfo.publish(apple);
            ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgra8", cv_left_img).toImageMsg();// 将OpenCV格式的图像转换为ROS图像消息
            // cv::Mat cvImage = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGRA8)->image;
            // cv::imshow("cvImage", cvImage);
            // cv::waitKey(100);
            pub_detectImg.publish(ros_image);// 发布图像消息
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
