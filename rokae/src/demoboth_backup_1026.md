```
/**
 * @file pick_demo_stable.cpp
 * @brief 稳定版：左右臂顺序执行（保持连接，不断电）
 * 改循环次数，   定位到Line 285, const int loops = 2;
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <atomic>

#include "rokae/robot.h"
#include "print_helper.hpp"
#include "rokae/utility.h"

#include "ros/ros.h"
#include "comm/serialData.h"

using namespace rokae;
using namespace std::chrono_literals;

// ----- 全局常量 -----
static const std::string Rightip = "192.168.2.160";
static const std::string Leftip  = "192.168.2.161";

// ----- 工具函数：健壮的 waitForFinish -----
bool waitForFinish(BaseRobot &robot, const std::string &traj_id, int waypoint_index, double timeout_sec = 20.0) {
    using namespace rokae::EventInfoKey::MoveExecution;
    error_code ec;
    auto start = std::chrono::steady_clock::now();

    while (ros::ok()) {
        auto info = robot.queryEventInfo(Event::moveExecution, ec);
        if (ec) {
            ROS_WARN_STREAM("[waitForFinish] queryEventInfo error: " << ec.message());
            // 若 sdk 报错，稍作等待再重试，避免立刻退出
            std::this_thread::sleep_for(100ms);
        } else {
            // 防护：确保存在所需 key
            if (info.find(ID) != info.end() && info.find(WaypointIndex) != info.end()) {
                try {
                    auto _id = std::any_cast<std::string>(info.at(ID));
                    auto _idx = std::any_cast<int>(info.at(WaypointIndex));

                    // 如果 event 返回 Error 字段且非空，则打印并返回 false
                    if (info.find(Error) != info.end()) {
                        try {
                            auto _ec_any = info.at(Error);
                            if (_ec_any.has_value()) {
                                auto _ec = std::any_cast<error_code>(_ec_any);
                                if (_ec) {
                                    ROS_ERROR_STREAM("[waitForFinish] Move error: " << _ec.message());
                                    return false;
                                }
                            }
                        } catch (const std::bad_any_cast&) {
                            // 忽略 bad cast: 有些实现可能以不同类型返回 Error
                        }
                    }

                    if (_id == traj_id && _idx == waypoint_index) {
                        // 检查 ReachTarget 字段（若存在）
                        bool reached = false;
                        if (info.find(ReachTarget) != info.end()) {
                            try {
                                reached = std::any_cast<bool>(info.at(ReachTarget));
                            } catch (const std::bad_any_cast&) {
                                reached = true; // 若无法 cast，则假设已到达以避免死等
                            }
                        } else {
                            reached = true; // 没有该字段时也当作到达
                        }

                        if (reached) {
                            ROS_INFO_STREAM("[waitForFinish] traj " << traj_id << " waypoint " << waypoint_index << " finished.");
                            return true;
                        } else {
                            ROS_WARN_STREAM_THROTTLE(5, "[waitForFinish] traj matched but ReachTarget==false, continuing wait...");
                        }
                    }
                } catch (const std::bad_any_cast &e) {
                    ROS_WARN_STREAM("[waitForFinish] bad_any_cast: " << e.what());
                }
            }
        }

        // 超时判断
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if (elapsed > timeout_sec) {
            ROS_ERROR_STREAM("[waitForFinish] timeout after " << timeout_sec << "s for traj " << traj_id << " index " << waypoint_index);
            return false;
        }

        // 让出 CPU，且让 ROS 回调/底层事件处理能跑
        ros::spinOnce();
        std::this_thread::sleep_for(50ms);
    }

    return false; // ros not ok
}

// ----- 打印运动执行信息（Event watcher） -----
void printInfo(const rokae::EventInfo &info) {
    using namespace rokae::EventInfoKey::MoveExecution;
    try {
        std::string id = std::any_cast<std::string>(info.at(ID));
        int idx = std::any_cast<int>(info.at(WaypointIndex));
        bool reach = false;
        if (info.find(ReachTarget) != info.end()) reach = std::any_cast<bool>(info.at(ReachTarget));
        error_code ec = std::any_cast<error_code>(info.at(Error));
        std::string remark = "";
        if (info.find(Remark) != info.end()) remark = std::any_cast<std::string>(info.at(Remark));
        ROS_INFO_STREAM("[Event] ID:" << id << " Index:" << idx << " Reach:" << (reach ? "YES" : "NO") << " Error:" << ec.message() << " Remark:" << remark);
    } catch (const std::bad_any_cast &e) {
        ROS_WARN_STREAM("[printInfo] bad_any_cast: " << e.what());
    } catch (const std::exception &e) {
        ROS_WARN_STREAM("[printInfo] exception: " << e.what());
    }
}

// ----- 封装：对单次 MoveJCommand 执行并等待 -----
// 返回 true: 执行成功并到达；false: 超时或出错
bool execMoveOnce(BaseRobot &robot, const MoveJCommand &cmd, double timeout_sec = 20.0) {
    std::error_code ec;
    std::string traj_id;
    robot.moveReset(ec);
    if (ec) {
        ROS_ERROR_STREAM("[execMoveOnce] moveReset error: " << ec.message());
        return false;
    }
    robot.moveAppend({cmd}, traj_id, ec);
    if (ec) {
        ROS_ERROR_STREAM("[execMoveOnce] moveAppend error: " << ec.message());
        return false;
    }
    robot.moveStart(ec);
    if (ec) {
        ROS_ERROR_STREAM("[execMoveOnce] moveStart error: " << ec.message());
        return false;
    }
    // 单 waypoint 情况，等待 waypoint index = 0
    bool ok = waitForFinish(robot, traj_id, 0, timeout_sec);
    return ok;
}

// ----- BothDemo 类：持有对机器人对象的引用（不在方法中频繁 connect/disconnect） -----
class BothDemo {
public:
    BothDemo(xMateRobot &rbot, xMateRobot &lbot, ros::Publisher &pub)
      : rbot_(rbot), lbot_(lbot), pub_serial_(pub) {}

    // demogoline: 按原始坐标不改动
    bool demogoline(const std::string &ip, int /*i*/) {
        try {
            if (ip == Rightip) {
                MoveJCommand moveJ_godemoline({0.88,-0.08,0.23,-M_PI_2,30*M_PI_1,-103*M_PI_1});
                return execMoveOnce(rbot_, moveJ_godemoline, 25.0);
            } else {
                MoveJCommand moveJ_godemoline({0.89,0.26,-0.02,103*M_PI_1,-83*M_PI_1,78*M_PI_1});
                return execMoveOnce(lbot_, moveJ_godemoline, 25.0);
            }
        } catch (const rokae::Exception &e) {
            ROS_ERROR_STREAM("[demogoline] exception: " << e.what());
            return false;
        }
    }

    bool demogohome(const std::string &ip, int /*i*/) {
        try {
            if (ip == Rightip) {
                MoveJCommand moveJ_gohome({0.23, -0.23, 0.26, M_PI_2, 70*M_PI_1, 39*M_PI_1});
                return execMoveOnce(rbot_, moveJ_gohome, 20.0);
            } else {
                MoveJCommand moveJ_gohome({0.28, 0.47, 0.084, 157*M_PI_1, -78*M_PI_1, 15*M_PI_1});
                return execMoveOnce(lbot_, moveJ_gohome, 20.0);
            }
        } catch (const rokae::Exception &e) {
            ROS_ERROR_STREAM("[demogohome] exception: " << e.what());
            return false;
        }
    }

    bool demogorelese(const std::string &ip, int /*i*/) {
        try {
            if (ip == Rightip) {
                // 右臂原始实现中：先 gohome 再 tran
                MoveJCommand moveJ_gohome({0.23, -0.23, 0.26, M_PI_2, 70*M_PI_1, 39*M_PI_1});
                MoveJCommand movej_tran({0.05,-0.29,0.26,96*M_PI_1,67*M_PI_1,6*M_PI_1});
                // 这里按原意 append 两段并等待最后一段完成
                std::error_code ec;
                std::string traj_id;
                rbot_.moveReset(ec);
                rbot_.moveAppend({moveJ_gohome}, traj_id, ec);
                rbot_.moveAppend({movej_tran}, traj_id, ec);
                rbot_.moveStart(ec);
                bool ok = waitForFinish(rbot_, traj_id, 0, 25.0); // 第二段 index = 1
                return ok;
            } else {
                // 左臂原始实现：tran1 + tran2
                MoveJCommand tran1({0.22,0.268,-0.2,-179*M_PI_1,-36*M_PI_1,0});
                MoveJCommand tran2({0.088,0.268,-0.284,-179*M_PI_1,5*M_PI_1,0});
                std::error_code ec;
                std::string traj_id;
                lbot_.moveReset(ec);
                lbot_.moveAppend({tran1}, traj_id, ec);
                lbot_.moveAppend({tran2}, traj_id, ec);
                lbot_.moveStart(ec);
                bool ok = waitForFinish(lbot_, traj_id, 0, 25.0);
                return ok;
            }
        } catch (const rokae::Exception &e) {
            ROS_ERROR_STREAM("[demogorelese] exception: " << e.what());
            return false;
        }
    }

    // 公开发布串口命令接口（如果需要）
    void publishSerial(const comm::serialData &d) {
        pub_serial_.publish(d);
    }

private:
    xMateRobot &rbot_;
    xMateRobot &lbot_;
    ros::Publisher &pub_serial_;
};

int main(int argc, char *argv[]) 
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"pick_demo_stable");
    ros::NodeHandle nh;

    // Publisher for serial commands (手爪)
    ros::Publisher pub_serial = nh.advertise<comm::serialData>("ap_robot/serial",10);

    // 串口数据（保持你原来的数据）
    comm::serialData rightClosedata, rightOpendata, leftClosedata, leftOpendata;
    rightClosedata.data = {0xaa, 0x55, 0x03, 0x01,  0x04, 0xb0, 0x50, 0x05, 0x27, 0x10, 0x3c, 0x01, 0x02, 0x03, 0x0d, 0x0a};
    rightOpendata.data  = {0xaa, 0x55, 0x03, 0x00,  0x04, 0xb0, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};
    leftClosedata.data  = {0xaa, 0x55, 0x02, 0x00, 0x04, 0xb0, 0x50, 0x05, 0x27, 0x10, 0x3c, 0x01, 0x02, 0x03, 0x0d, 0x0a};
    leftOpendata.data   = {0xaa, 0x55, 0x02, 0x01, 0x04, 0xb0, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};

    // ----- 连接并配置两台机械臂（只做一次） -----
    std::error_code ec;
    ROS_INFO("[main] Connecting to robots...");

    xMateRobot robotR(Rightip);
    xMateRobot robotL(Leftip);

    // 设置基础模式（只做一次）
    robotR.setOperateMode(OperateMode::automatic, ec);
    if (ec) ROS_ERROR_STREAM("[main] robotR setOperateMode error: " << ec.message());
    robotL.setOperateMode(OperateMode::automatic, ec);
    if (ec) ROS_ERROR_STREAM("[main] robotL setOperateMode error: " << ec.message());

    robotR.setPowerState(true, ec);
    if (ec) ROS_ERROR_STREAM("[main] robotR setPowerState error: " << ec.message());
    robotL.setPowerState(true, ec);
    if (ec) ROS_ERROR_STREAM("[main] robotL setPowerState error: " << ec.message());

    robotR.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robotL.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robotR.setDefaultZone(50, ec);
    robotL.setDefaultZone(50, ec);
    robotR.setDefaultSpeed(350, ec);
    robotL.setDefaultSpeed(200, ec);

    Toolset defaultToolset;
    robotR.setToolset(defaultToolset, ec);
    robotL.setToolset(defaultToolset, ec);

    // 只注册一次事件监听，避免多次注册导致干扰
    robotR.setEventWatcher(Event::moveExecution, printInfo, ec);
    robotL.setEventWatcher(Event::moveExecution, printInfo, ec);

    ROS_INFO("[main] Robots configured. Entering main loop.");

    // 创建 BothDemo 实例
    BothDemo demo(robotR, robotL, pub_serial);

    // 循环次数（你要求 2 次）
    const int loops = 2;

    for (int i = 0; i < loops && ros::ok(); ++i) {
        ROS_INFO_STREAM("=== Main loop iteration: " << i << " ===");

        // 1) 右臂：移动到采摘位
        ROS_INFO("[main] Right arm: demogoline");
        if (!demo.demogoline(Rightip, i)) {
            ROS_ERROR("[main] Right demogoline failed");
            // 继续尝试下一步或根据策略决定退出，这里选择继续但记录错误
        }

        // 右手爪闭合（串口）
        ROS_INFO("[main] publish right close");
        pub_serial.publish(rightClosedata);
        // 给手爪一定时间执行（手爪动作由串口固件决定），保留短暂停顿并允许 ROS 回调运行
        for (int k = 0; k < 20 && ros::ok(); ++k) { ros::spinOnce(); std::this_thread::sleep_for(50ms); }

        // 右臂：释放/回到释放位（demogorelese）
        ROS_INFO("[main] Right arm: demogorelese");
        if (!demo.demogorelese(Rightip, i)) {
            ROS_ERROR("[main] Right demogorelese failed");
        }

        // 右手爪打开
        ROS_INFO("[main] publish right open");
        pub_serial.publish(rightOpendata);
        for (int k = 0; k < 20 && ros::ok(); ++k) { ros::spinOnce(); std::this_thread::sleep_for(50ms); }

        // 2) 左臂：移动到采摘位
        ROS_INFO("[main] Left arm: demogoline");
        if (!demo.demogoline(Leftip, i)) {
            ROS_ERROR("[main] Left demogoline failed");
        }

        // 左手爪闭合
        ROS_INFO("[main] publish left close");
        pub_serial.publish(leftClosedata);
        for (int k = 0; k < 20 && ros::ok(); ++k) { ros::spinOnce(); std::this_thread::sleep_for(50ms); }

        // 左臂释放
        ROS_INFO("[main] Left arm: demogorelese");
        if (!demo.demogorelese(Leftip, i)) {
            ROS_ERROR("[main] Left demogorelese failed");
        }

        // 左手爪打开
        ROS_INFO("[main] publish left open");
        pub_serial.publish(leftOpendata);
        for (int k = 0; k < 20 && ros::ok(); ++k) { ros::spinOnce(); std::this_thread::sleep_for(50ms); }

        ROS_INFO_STREAM("=== Loop " << i << " finished ===");
        // 小间隔，给控制器和手爪一点缓冲
        // for (int k = 0; k < 20 && ros::ok(); ++k) { ros::spinOnce(); std::this_thread::sleep_for(50ms); }
    }

    // 主循环结束：安全停机与断开连接（只在退出时执行一次）
    ROS_INFO("[main] All loops done, stopping robots and disconnecting...");
    robotR.stop(ec);
    robotL.stop(ec);
    robotR.setPowerState(false, ec);
    robotL.setPowerState(false, ec);
    robotR.disconnectFromRobot(ec);
    robotL.disconnectFromRobot(ec);

    ROS_INFO("[main] Exit.");
    return 0;
}

```
