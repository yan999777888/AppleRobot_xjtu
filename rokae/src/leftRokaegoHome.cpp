/**
 * @file sdk_example.cpp
 * @brief SDK各接口使用示例
 *
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include "rokae/utility.h"
#include "ros/ros.h"

using namespace rokae;
std::ostream &os = std::cout; ///< print to console

/**
 * @brief 等待运动结束 - 通过查询路径ID及路点序号是否已完成的方式
 */
void waitForFinish(BaseRobot &robot, const std::string &traj_id, int index){
  using namespace rokae::EventInfoKey::MoveExecution;
  error_code ec;
  while(true) {
    auto info = robot.queryEventInfo(Event::moveExecution, ec);
    auto _id = std::any_cast<std::string>(info.at(ID));
    auto _index = std::any_cast<int>(info.at(WaypointIndex));
    if(auto _ec = std::any_cast<error_code>(info.at(Error))) {
      print(std::cout, "路径", _id, ":", _index, "错误:", _ec.message());
      return;
    }
    if(_id == traj_id && _index == index) {
      if(std::any_cast<bool>(info.at(ReachTarget))) {
        print(std::cout, "路径", traj_id, ":", index, "已完成");
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

/**
 * @brief 打印运动执行信息
 */
void printInfo(const rokae::EventInfo &info) {
  using namespace rokae::EventInfoKey::MoveExecution;
  print(std::cout, "[运动执行信息] ID:", std::any_cast<std::string>(info.at(ID)), "Index:", std::any_cast<int>(info.at(WaypointIndex)),
        "已完成: ", std::any_cast<bool>(info.at(ReachTarget)) ? "YES": "NO", std::any_cast<error_code>(info.at(Error)),
          std::any_cast<std::string>(info.at(Remark)));
}

/**
 * @brief main program
 */
int main() {

  try {
    // *** 1. 连接机器人 ***
    std::string ip = "192.168.2.161";
    std::error_code ec;
    xMateRobot robot(ip);  // 此处连接的是协作6轴机型

    // *** 2. Switch to auto mode and motor on ***
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    // *** 3. set default speed and turning zone ***
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.setDefaultZone(50, ec); // 可选：设置默认转弯区
    robot.setDefaultSpeed(200, ec); // 可选：设置默认速度
    
    Toolset defaultToolset;
    robot.setToolset(defaultToolset, ec);

    robot.setEventWatcher(Event::moveExecution, printInfo, ec);

    std::string id;
    MoveJCommand moveJ_releaseCartpos({0.40, 0.20, -0.0, M_PI, 0, 0});
    robot.moveAppend({moveJ_releaseCartpos}, id, ec);  //移动到释放苹果姿态
    robot.moveStart(ec);
    waitForFinish(robot, id, 0);
    robot.stop(ec);

    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);
  } catch (const rokae::Exception &e) {
    std::cerr << e.what();
  }

  return 0;
}