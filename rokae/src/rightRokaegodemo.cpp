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
#include <chrono>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include "rokae/utility.h"
#include <string>
#include "ros/ros.h"
#include "comm/serialData.h"
#include <Eigen/Dense>
#include "rokae/model.h"
#include "rokae/planner.h"

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
int main(int argc, char *argv[]) {
  setlocale(LC_ALL,"");
  ros::init(argc,argv,"pick_right");
  ros::NodeHandle nh;

  ros::Publisher pub_serial = nh.advertise<comm::serialData>("ap_robot/serial",10);
  comm::serialData rightClosedata, rightOpendata;//确保脉冲数的大小正确
  rightClosedata.data = {0xaa, 0x55, 0x03, 0x00, 0x03, 0x84, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};
  rightOpendata.data = {0xaa, 0x55, 0x03, 0x01, 0x03, 0x84, 0x50, 0x05, 0x27, 0x10, 0x3c, 0x01, 0x02, 0x03, 0x0d, 0x0a};

  try {
    // *** 1. 连接机器人 ***
    std::string ip = "192.168.2.160";
    std::error_code ec;
    xMateRobot robot(ip);  // 此处连接的是协作6轴机型

    // *** 2. Switch to auto mode and motor on ***
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    // *** 3. set default speed and turning zone ***
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.setDefaultZone(50, ec); // 可选：设置默认转弯区
    robot.setDefaultSpeed(100, ec); // 可选：设置默认速度

    Toolset defaultToolset;
    robot.setToolset(defaultToolset, ec);

    robot.setEventWatcher(Event::moveExecution, printInfo, ec);
    CoordinateType flangeInbase=CoordinateType::flangeInBase;
    
    robot.posture(flangeInbase,ec);
    
    

    std::string id;
    // float a=1.22173;
    MoveJCommand moveJ_gohome({0.23, -0.23, 0.26, M_PI_2,70*M_PI_1, 39*M_PI_1});//移动到chushi姿态
    MoveJCommand moveJ_releaseCartpos({0.08, -0.42, 0.25, -M_PI, M_PI_4, -M_PI});//移动到释放苹果姿态
    MoveJCommand moveJ_godemoline({0.88,-0.08,0.23,-M_PI_2,30*M_PI_1,-103*M_PI_1});//caizhaizitai
    // robot.moveReset(ec);
    // robot.moveAppend({moveJ_releaseCartpos}, id, ec);  
    // robot.moveStart(ec);
    // waitForFinish(robot, id, 0);
    robot.moveReset(ec);
    robot.moveAppend({moveJ_gohome}, id, ec);  
    robot.moveStart(ec);
    waitForFinish(robot, id, 0);

    robot.moveReset(ec);
    robot.moveAppend({moveJ_godemoline}, id, ec);  
    robot.moveStart(ec);
    waitForFinish(robot, id, 1);

    std::cout << " 闭合右手爪！" << std::endl;
    pub_serial.publish(rightClosedata); //闭合手爪
    ros::Duration(2).sleep(); // 等待2s

    robot.moveReset(ec);
    robot.moveAppend({moveJ_gohome},id,ec);
    robot.moveStart(ec);
    waitForFinish(robot, id, 2);

    std::cout << " 打开右手爪！" << std::endl;
    pub_serial.publish(rightOpendata); //打开手爪
    ros::Duration(2).sleep(); // 模拟机械臂执行
    //xunhuanzhixing
  

    robot.stop(ec);
    
    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);
  } catch (const rokae::Exception &e) {
    std::cerr << e.what();
  }

  return 0;
}