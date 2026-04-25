#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pcd_points", 1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 发布静态 TF：zed2_left_camera_frame → world
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transform;
    static_transform.header.stamp = ros::Time::now();
    static_transform.header.frame_id = "world";
    static_transform.child_frame_id = "zed2_left_camera_frame";
    static_transform.transform.translation.x = 0.12256;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.82;
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;

    static_broadcaster.sendTransform(static_transform);

    // 修改为你的PCD文件路径
    std::string pcd_path = "src/pub_pcd/pcd/pointcloud.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_path, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read PCD file at %s", pcd_path.c_str());
        return -1;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "zed2_left_camera_frame";  // 或 "base_link"，根据需要

    ros::Rate loop_rate(10);  // 每秒发布一次

    while (ros::ok())
    {
        output.header.stamp = ros::Time::now();
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
