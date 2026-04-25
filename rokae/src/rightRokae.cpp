#include "ros/ros.h"
#include "task_assign/Target.h"
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include "rokae/utility.h"
#include <Eigen/Dense>
#include "comm/serialData.h"
#include <fstream>
#include <string>
#include <stdexcept>
using namespace rokae;
using namespace Eigen;
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

Eigen::Matrix4d loadMatrix(const std::string& filePath){
    // 定义一个4x4的矩阵
    Eigen::Matrix4d matrix;

    // 打开文件
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + filePath);
    }

    // 读取文件中的数据到矩阵
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!(file >> matrix(i, j))) {
                throw std::runtime_error("Error reading matrix data from file: " + filePath);
            }
        }
    }

    file.close();
    return matrix;
}

float loadFloat(const std::string& filePath) {
    float value = 0.0f; // 用于存储读取的浮点数

    // 打开文件
    std::ifstream file(filePath);
    if (file.is_open()) {
        // 读取文件中的浮点数
        file >> value;
        
        if (file.fail()) {
            std::cerr << "Error: Failed to read a float from file." << std::endl;
            file.close();
            return 1.0f; // 或根据需要返回默认值
        }
        
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filePath << std::endl;
    }

    return value;
}

int main(int argc, char *argv[]) {
  setlocale(LC_ALL,"");
  ros::init(argc,argv,"pick_right_2");
  ros::NodeHandle nh;

  ros::Publisher pub_serial = nh.advertise<comm::serialData>("ap_robot/serial",10);
  comm::serialData rightClosedata, rightOpendata;//确保脉冲数的大小正确
  rightClosedata.data = {0xaa, 0x55, 0x03, 0x00, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};
  rightOpendata.data = {0xaa, 0x55, 0x03, 0x01, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};

  Eigen::Matrix4d T_cam_to_base = loadMatrix("src/rokae/src/camera_pose_right.txt"); //手眼标定得到的矩阵
  float depth_scale = loadFloat("src/rokae/src/camera_depth_scale_right.txt");

  std::string ip = "192.168.2.160";
  std::error_code ec;  

  try {
    // *** 1. 连接机器人 ***

    xMateRobot robot(ip);  // 此处连接的是协作6轴机型

    // *** 2. 切换到自动模式并上电 ***
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    // *** 3. 设置默认运动速度和转弯区 ***
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.setDefaultZone(0, ec); // 可选：设置默认转弯区
    robot.setDefaultSpeed(200, ec); //可选：设置默认速度

    //use default tool and wobj frame
    Toolset Toolset1;
    Toolset1.end = {{ 0, 0, 0.25}, {0, 0, 0}};
    Toolset1.load.mass = 0.708993;
    robot.setToolset(Toolset1, ec);

    MoveJCommand moveJ_forward({0.65, -0.2, 0.0, 0, M_PI / 2, 0}); //末端朝前
    MoveJCommand moveJ_down({0.3, -0.1, -0.65, -M_PI, 0, -M_PI}); //末端朝下

    //使用标定过的夹爪末端工具坐标系
    // Toolset Toolset1;
    // Toolset1.end = {{ 0, 0, 0.28}, {0, 0, 0}};
    // Toolset1.load.mass = 0.708993;
    // robot.setToolset(Toolset1, ec);
    
    robot.setAvoidSingularity(AvoidSingularityMethod::wrist, true, M_PI_4, ec);

    // 服务回调函数
    auto doReq = [&](task_assign::Target::Request& req, task_assign::Target::Response& resp) -> bool {
      std::cout << "right Camera coordinates1: (" << req.x << ", " << req.y << ", " << req.z << ")" << std::endl;
      
      Eigen::Vector4d p_camera(req.x, req.y, req.z * depth_scale, 1.0); // 相机坐标系下的点 (x_camera, y_camera, z_camera)
      Eigen::Vector4d p_base = T_cam_to_base * p_camera; // 转换到基坐标系下

      // 输出结果
      std::cout << "right Camera coordinates2: (" << p_camera(0) << ", " << p_camera(1) << ", " << p_camera(2) << ")" << std::endl;
      std::cout << "right Base coordinates: (" << p_base(0) << ", " << p_base(1) << ", " << p_base(2) << ")" << std::endl;

      //测试抓取姿态，z轴指向苹果与关节2的连线方向，姿态使用欧拉角ZYX
      double y0 = -0.335;
      Vector3d target(p_base(0), p_base(1) + y0, p_base(2));
      target = target.normalized();
      // double A = -atan2(target(1), target(2)); //目标姿态Z
      // double B = atan2(target(0),sqrt(target(1)*target(1)+target(2)*target(2))); //目标姿态Y
      // double C = 0; //目标姿态X
      double A = 0, B = M_PI / 2, C = 0;

      // std::cout << "目标位置为x:" << target(0) << " y:" << target(1) << " z:" << target(2) << std::endl;
      std::cout << "目标姿态为A:" << A << " B:" << B << " C:" << C << std::endl;

      std::string id;
      // Frame frame({0, 0, req.size / 2, 0, 0, 0}); //工具坐标系z方向偏移苹果直径
      Frame frame({0, 0, -0.08, 0, 0, 0}); //工具坐标系x方向偏移80mm
      CartesianPosition::Offset offset(CartesianPosition::Offset::relTool, frame);
      MoveJCommand moveJ_close({p_base(0), p_base(1), p_base(2), A, B, C}); //先接近目标
      moveJ_close.offset = offset;

      // //不考虑抓取动作
      // std::string id;
      // MoveJCommand moveJ({p_base(0), p_base(1), p_base(2), 0, M_PI_2, 0}); //先接近目标
      // robot.executeCommand({moveJ}, ec);

      robot.moveAppend({moveJ_forward, moveJ_close}, id, ec); //先接近目标
      robot.moveStart(ec);
      waitForFinish(robot, id, 0);

      MoveLCommand moveL_get({p_base(0), p_base(1), p_base(2), A, B, C}); //后进给      
      robot.moveAppend({moveL_get}, id, ec); //后进给
      robot.moveStart(ec);
      waitForFinish(robot, id, 0);

      std::cout << " 闭合右手爪！" << std::endl;
      pub_serial.publish(rightClosedata); //闭合手爪
      ros::Duration(1).sleep(); // 等待2s

      robot.moveAppend({moveJ_down}, id, ec);  //移动到释放苹果姿态
      robot.moveStart(ec);
      waitForFinish(robot, id, 0);
      robot.stop(ec);

      std::cout << " 打开右手爪！" << std::endl;
      pub_serial.publish(rightOpendata); //打开手爪
      ros::Duration(1).sleep(); // 模拟机械臂执行

      // resp.status = 0;
      // ros::Duration(5).sleep(); // 模拟机械臂执行
      return true;
    };

    ros::ServiceServer server = nh.advertiseService<task_assign::Target::Request, task_assign::Target::Response>("right_pickTarget",doReq);
    ros::spin();

    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);
  } catch (const rokae::Exception &e) {
    std::cerr << e.what();
  }
  
  return 0;
}