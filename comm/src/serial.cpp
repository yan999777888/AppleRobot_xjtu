#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <thread>
#include "comm/serialData.h"
#include <std_msgs/Bool.h>

serial::Serial sp;
ros::Publisher pub_ack;

// 串口写回调（发送指令）
void arrayCallback(const comm::serialData::ConstPtr& msg) {
    sp.write(reinterpret_cast<const uint8_t*>(msg->data.data()), msg->data.size());
}

// 串口读取线程：解析应答帧并发布 ack
// 步进电机驱动器完成后回发帧，帧头为 0xaa 0x55，帧尾为 0x0d 0x0a
void readThread() {
    std::vector<uint8_t> buf;
    while (ros::ok()) {
        if (sp.available()) {
            std::string raw = sp.read(sp.available());
            for (uint8_t b : raw) buf.push_back(b);

            // 扫描完整帧：0xaa 0x55 ... 0x0d 0x0a
            for (size_t i = 0; i + 1 < buf.size(); ++i) {
                if (buf[i] == 0xaa && buf[i + 1] == 0x55) {
                    // 从 i+2 向后找帧尾
                    for (size_t j = i + 2; j + 1 < buf.size(); ++j) {
                        if (buf[j] == 0x0d && buf[j + 1] == 0x0a) {
                            // 找到完整帧，发布 ack
                            std_msgs::Bool ack;
                            ack.data = true;
                            pub_ack.publish(ack);
                            buf.erase(buf.begin(), buf.begin() + j + 2);
                            i = 0; // 重新扫描
                            break;
                        }
                    }
                    break;
                } else {
                    buf.erase(buf.begin()); // 丢弃无效字节
                    --i;
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string port_name = "/dev/ttyUSB0";
    int baudrate = 115200;
    int timeout_ms = 100;
    pnh.param<std::string>("port", port_name, port_name);
    pnh.param("baudrate", baudrate, baudrate);
    pnh.param("timeout_ms", timeout_ms, timeout_ms);

    serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
    sp.setPort(port_name);
    sp.setBaudrate(static_cast<uint32_t>(baudrate));
    sp.setTimeout(timeout);

    try {
        sp.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if (!sp.isOpen()) {
        ROS_ERROR_STREAM("Port open failed.");
        return -1;
    }
    ROS_INFO_STREAM(port_name << " is opened.");

    pub_ack = nh.advertise<std_msgs::Bool>("ap_robot/serial_ack", 10);
    ros::Subscriber sub = nh.subscribe("ap_robot/serial", 10, arrayCallback);

    // 启动串口读取线程
    std::thread reader(readThread);
    reader.detach();

    ros::spin();

    sp.close();
    return 0;
}
