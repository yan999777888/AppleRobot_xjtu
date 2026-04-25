// // // #include "ros/ros.h"
// // // #include "task_assign/Target.h"
// // // #include <thread>
// // // #include "rokae/robot.h"
// // // #include "comm/serialData.h"
// // // #include <Eigen/Dense>
// // // #include <Eigen/Geometry>
// // // #include <fstream>
// // // #include <vector>
// // // #include <algorithm>
// // // #include <atomic>
// // // #include <cmath>

// // // // PCL
// // // #include <sensor_msgs/PointCloud2.h>
// // // #include <pcl_conversions/pcl_conversions.h>
// // // #include <pcl/point_cloud.h>
// // // #include <pcl/point_types.h>
// // // #include <pcl/common/transforms.h>

// // // using namespace rokae;
// // // using namespace Eigen;
// // // using namespace std;

// // // // =================== 1. 全局配置 ===================
// // // const double DIST_PRE = 0.15; 
// // // const double DIST_RET = 0.10; 
// // // const double MAX_REACH = 1.2; 
// // // const int NUM_SAMPLES = 80; 

// // // // 【姿态锁定】手心朝前
// // // const double HOME_RX = -1.57;
// // // const double HOME_RY = -0.05;
// // // const double HOME_RZ = -1.50;
// // // const vector<double> HOME_POSE_FULL = {0.522, -0.113, 0.484, HOME_RX, HOME_RY, HOME_RZ};

// // // xMateRobot* robot_ptr = nullptr;
// // // ros::Publisher pub_serial;
// // // comm::serialData gripper_close_cmd, gripper_open_cmd;
// // // Eigen::Matrix4d T_cam_to_base;
// // // float depth_scale = 0.9797;
// // // std::atomic<int> g_motion_id_counter(0);

// // // // 障碍物存储
// // // pcl::PointCloud<pcl::PointXYZ>::Ptr g_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// // // bool g_has_obs = false;

// // // struct GraspCandidate {
// // //     int id;
// // //     Vector3d approach_vec; 
// // //     vector<double> rpy;   
// // //     double score;
// // // };

// // // // =================== 2. 辅助函数 ===================

// // // std::string genUniqueId(std::string prefix) {
// // //     return prefix + "_" + std::to_string(g_motion_id_counter++);
// // // }

// // // Matrix4d loadMatrix(string path) {
// // //     Matrix4d m; ifstream f(path);
// // //     if(!f.is_open()) throw runtime_error("标定文件加载失败");
// // //     for(int i=0;i<4;++i) for(int j=0;j<4;++j) f >> m(i,j);
// // //     return m;
// // // }

// // // void waitForFinish(BaseRobot &robot, const std::string &traj_id, int index) {
// // //     std::error_code ec;
// // //     int timeout = 0;
// // //     while(timeout++ < 300) { 
// // //         auto info = robot.queryEventInfo(Event::moveExecution, ec);
// // //         if (!ec) {
// // //             auto _id = any_cast<string>(info.at(EventInfoKey::MoveExecution::ID));
// // //             auto _idx = any_cast<int>(info.at(EventInfoKey::MoveExecution::WaypointIndex));
// // //             if (_id == traj_id && _idx == index) {
// // //                 if(any_cast<bool>(info.at(EventInfoKey::MoveExecution::ReachTarget))) {
// // //                     this_thread::sleep_for(chrono::milliseconds(100)); return; 
// // //                 }
// // //             }
// // //         }
// // //         this_thread::sleep_for(chrono::milliseconds(50));
// // //     }
// // //     throw runtime_error("Motion Timeout");
// // // }

// // // void controlGripper(bool close) {
// // //     pub_serial.publish(close ? gripper_close_cmd : gripper_open_cmd);
// // //     ros::Duration(1).sleep();
// // // }

// // // // =================== 3. 避障与策略 ===================

// // // // 障碍物回调：直接转坐标系
// // // void obstacleCB(const sensor_msgs::PointCloud2ConstPtr& msg) {
// // //     pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
// // //     pcl::fromROSMsg(*msg, *temp);
// // //     pcl::transformPointCloud(*temp, *g_obstacle_cloud, T_cam_to_base.cast<float>());
// // //     g_has_obs = true;
// // // }

// // // // 碰撞检测：圆柱体模型
// // // bool checkCollision(Vector3d start, Vector3d end, double radius) {
// // //     if (!g_has_obs || g_obstacle_cloud->empty()) return false;
    
// // //     Vector3d AB = end - start;
// // //     double len_sq = AB.squaredNorm();
// // //     if(len_sq < 1e-6) return false;

// // //     for (const auto& pt : g_obstacle_cloud->points) {
// // //         Vector3d P(pt.x, pt.y, pt.z);
// // //         Vector3d AP = P - start;
// // //         double t = AP.dot(AB) / len_sq;
        
// // //         if (t < -0.1 || t > 1.0) continue; 
        
// // //         Vector3d Q = start + t * AB;
// // //         if ((P - Q).norm() < radius) return true;
// // //     }
// // //     return false;
// // // }

// // // vector<GraspCandidate> planGraspStrategies(Vector3d center_pos) {
// // //     vector<GraspCandidate> candidates;
// // //     double phi = M_PI * (3.0 - sqrt(5.0)); 
    
// // //     // 1. 保底策略 (正前方)
// // //     candidates.push_back({-1, Vector3d(1.0, 0.0, 0.0), {HOME_RX, HOME_RY, HOME_RZ}, 200.0});
// // //     // 2. 保底策略 (仰角)
// // //     candidates.push_back({-2, Vector3d(1.0, 0.0, 0.3).normalized(), {HOME_RX, HOME_RY, HOME_RZ}, 150.0});

// // //     // 3. 采样策略
// // //     for (int i = 0; i < NUM_SAMPLES; ++i) {
// // //         double yi = 1 - (i / float(NUM_SAMPLES - 1)) * 2; 
// // //         double radius = sqrt(1 - yi * yi);
// // //         double theta = phi * i;
// // //         double xi = cos(theta) * radius; double zi = sin(theta) * radius;
        
// // //         Vector3d approach = -Vector3d(xi, zi, yi).normalized(); 

// // //         // 只保留正面的策略 (防止绕到后面)
// // //         if (approach.x() < -0.1) continue; 
// // //         if (approach.z() < -0.3) continue;

// // //         double score = 100.0;
// // //         score += approach.x() * 50.0; 
// // //         if (approach.z() > 0) score += approach.z() * 20.0; 

// // //         candidates.push_back({i, approach, {HOME_RX, HOME_RY, HOME_RZ}, score});
// // //     }
    
// // //     sort(candidates.begin(), candidates.end(), 
// // //     [](const GraspCandidate& a, const GraspCandidate& b) { return a.score > b.score; });
    
// // //     return candidates;
// // // }

// // // // =================== 4. 业务回调 ===================

// // // bool pickCallBack(task_assign::Target::Request& req, task_assign::Target::Response& resp) {
// // //     if (!robot_ptr) return false;
// // //     std::error_code ec;

// // //     Vector4d p_cam(req.x, req.y, req.z * depth_scale, 1.0);
// // //     Vector3d target = (T_cam_to_base * p_cam).head<3>();
    
// // //     ROS_INFO(">>> 目标: [%.3f, %.3f, %.3f]", target.x(), target.y(), target.z());

// // //     if (target.norm() > MAX_REACH) { 
// // //         ROS_ERROR("目标过远"); resp.success = false; return true; 
// // //     }

// // //     auto strategies = planGraspStrategies(target);
// // //     ROS_INFO("生成策略数: %ld", strategies.size()); 

// // //     bool success = false;
// // //     string id; 

// // //     // 尝试前 10 个策略
// // //     for (int i = 0; i < min((int)strategies.size(), 10); ++i) { 
// // //         auto& task = strategies[i];
        
// // //         Vector3d pre_pos = target - task.approach_vec * DIST_PRE;
        
// // //         // --- 避障检测 (半径4cm) ---
// // //         if (checkCollision(pre_pos, target, 0.04)) {
// // //             ROS_WARN("策略 #%d 路径有障碍，跳过", i+1);
// // //             continue; 
// // //         }
        
// // //         if (pre_pos.z() < 0.10) { ROS_WARN("预备点太低"); continue; }

// // //         ROS_INFO("执行策略 #%d (分: %.1f)...", i+1, task.score);

// // //         try {
// // //             controlGripper(false);
            
// // //             // 1. MoveJ 预备点 (姿态锁死，安全)
// // //             id = genUniqueId("pre"); 
// // //             MoveJCommand cmd_pre({pre_pos.x(), pre_pos.y(), pre_pos.z(), 
// // //                                 task.rpy[0], task.rpy[1], task.rpy[2]});
// // //             cmd_pre.speed = 50; 
// // //             robot_ptr->moveAppend({cmd_pre}, id, ec); robot_ptr->moveStart(ec);
// // //             waitForFinish(*robot_ptr, id, 0); 
            
// // //             // 2. MoveL 进给
// // //             id = genUniqueId("grasp");
// // //             MoveLCommand cmd_grasp({target.x(), target.y(), target.z(), 
// // //                                   task.rpy[0], task.rpy[1], task.rpy[2]});
// // //             cmd_grasp.speed = 15; 
// // //             robot_ptr->moveAppend({cmd_grasp}, id, ec); robot_ptr->moveStart(ec);
// // //             waitForFinish(*robot_ptr, id, 0);

// // //             // 3. 抓取 & 撤退
// // //             controlGripper(true);
            
// // //             id = genUniqueId("back");
// // //             Vector3d ret_pos = target - task.approach_vec * DIST_RET;
// // //             MoveLCommand cmd_back({ret_pos.x(), ret_pos.y(), ret_pos.z(), 
// // //                                  task.rpy[0], task.rpy[1], task.rpy[2]});
// // //             cmd_back.speed = 30;
// // //             robot_ptr->moveAppend({cmd_back}, id, ec); robot_ptr->moveStart(ec);
// // //             waitForFinish(*robot_ptr, id, 0);

// // //             success = true; break; 
// // //         } catch (...) {
// // //             ROS_WARN("运动异常，尝试下一个..."); continue; 
// // //         }
// // //     }

// // //     if (success) {
// // //         id = genUniqueId("drop");
// // //         MoveJCommand cmd_drop({0.522, -0.113, 0.484, HOME_RX, HOME_RY, HOME_RZ}); 
// // //         robot_ptr->moveAppend({cmd_drop}, id, ec); robot_ptr->moveStart(ec);
// // //         waitForFinish(*robot_ptr, id, 0);
// // //         controlGripper(false);
// // //         resp.success = true;
// // //     } else {
// // //         resp.success = false; resp.message = "Failed/Blocked";
// // //         ROS_ERROR("任务失败 (可能所有路径均被遮挡)");
// // //     }
// // //     return true;
// // // }

// // // int main(int argc, char *argv[]) {
// // //     setlocale(LC_ALL,"");
// // //     ros::init(argc, argv, "robot_node_with_obstacle");
// // //     ros::NodeHandle nh;

// // //     pub_serial = nh.advertise<comm::serialData>("ap_robot/serial", 10);
// // //     gripper_close_cmd.data = {0xaa, 0x55, 0x02, 0x00, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};
// // //     gripper_open_cmd.data = {0xaa, 0x55, 0x02, 0x01, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};

// // //     try {
// // //         T_cam_to_base = loadMatrix("/home/y/dual_rokae_ws/src/rokae/src/camera_pose_left.txt");
// // //     } catch (...) { return -1; }

// // //     try {
// // //         robot_ptr = new xMateRobot("192.168.2.161");
// // //         std::error_code ec;
// // //         robot_ptr->setOperateMode(OperateMode::automatic, ec);
// // //         robot_ptr->setPowerState(true, ec);
// // //         robot_ptr->setMotionControlMode(MotionControlMode::NrtCommand, ec);
// // //         Toolset tool; tool.end = {{ 0, 0, 0.25}, {0, 0, 0}}; tool.load.mass = 0.70;
// // //         robot_ptr->setToolset(tool, ec);

// // //         // 订阅障碍物点云
// // //         ros::Subscriber sub = nh.subscribe("ap_robot/obstacles", 1, obstacleCB);

// // //         ROS_INFO(">>> 启动成功 | 姿态锁定 + 避障检测 <<<");
// // //         ros::ServiceServer server = nh.advertiseService("left_pickTarget", pickCallBack);
        
// // //         // 必须双线程
// // //         ros::AsyncSpinner spinner(2);
// // //         spinner.start();
// // //         ros::waitForShutdown();
// // //     } catch (...) { return -1; }

// // //     if(robot_ptr) delete robot_ptr;
// // //     return 0;
// // // }

// // #include "ros/ros.h"
// // #include "task_assign/Target.h"
// // #include <thread>
// // #include "rokae/robot.h"
// // #include "comm/serialData.h"
// // #include <Eigen/Dense>
// // #include <Eigen/Geometry>
// // #include <fstream>
// // #include <vector>
// // #include <algorithm>
// // #include <atomic>
// // #include <cmath>

// // // PCL
// // #include <sensor_msgs/PointCloud2.h>
// // #include <pcl_conversions/pcl_conversions.h>
// // #include <pcl/point_cloud.h>
// // #include <pcl/point_types.h>
// // #include <pcl/common/transforms.h>

// // using namespace rokae;
// // using namespace Eigen;
// // using namespace std;

// // // =================== 1. 全局配置 ===================
// // const double DIST_PRE = 0.15; 
// // const double DIST_RET = 0.10; 
// // const double MAX_REACH = 1.3; 
// // const int NUM_SAMPLES = 100; // 增加采样密度，找缝隙

// // // 【姿态参考】手心朝前 (仅作为 Roll 轴的参考)——需要根据实际情况作修改
// // const double HOME_RX = 2.95950;
// // const double HOME_RY = 0.124145268;
// // const double HOME_RZ = 1.518890209;
// // const vector<double> HOME_POSE_FULL = {0.436, -0.096825, 0.100019, HOME_RX, HOME_RY, HOME_RZ};

// // xMateRobot* robot_ptr = nullptr;
// // ros::Publisher pub_serial;
// // comm::serialData gripper_close_cmd, gripper_open_cmd;
// // Eigen::Matrix4d T_cam_to_base;
// // float depth_scale = 0.9797;
// // std::atomic<int> g_motion_id_counter(0);

// // // 障碍物点云
// // pcl::PointCloud<pcl::PointXYZ>::Ptr g_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// // bool g_has_obs = false;

// // struct GraspCandidate {
// //     int id;
// //     Vector3d approach_vec; 
// //     vector<double> rpy;   
// //     double score;
// // };

// // // =================== 2. 辅助函数 ===================

// // std::string genUniqueId(std::string prefix) {
// //     return prefix + "_" + std::to_string(g_motion_id_counter++);
// // }

// // Matrix4d loadMatrix(string path) {
// //     Matrix4d m; ifstream f(path);
// //     if(!f.is_open()) throw runtime_error("标定文件加载失败");
// //     for(int i=0;i<4;++i) for(int j=0;j<4;++j) f >> m(i,j);
// //     return m;
// // }

// // void waitForFinish(BaseRobot &robot, const std::string &traj_id, int index) {
// //     std::error_code ec;
// //     int timeout = 0;
// //     while(timeout++ < 300) { 
// //         auto info = robot.queryEventInfo(Event::moveExecution, ec);
// //         if (!ec) {
// //             auto _id = any_cast<string>(info.at(EventInfoKey::MoveExecution::ID));
// //             auto _idx = any_cast<int>(info.at(EventInfoKey::MoveExecution::WaypointIndex));
// //             if (_id == traj_id && _idx == index) {
// //                 if(any_cast<bool>(info.at(EventInfoKey::MoveExecution::ReachTarget))) {
// //                     this_thread::sleep_for(chrono::milliseconds(100)); return; 
// //                 }
// //             }
// //         }
// //         this_thread::sleep_for(chrono::milliseconds(50));
// //     }
// //     throw runtime_error("Motion Timeout");
// // }

// // void controlGripper(bool close) {
// //     pub_serial.publish(close ? gripper_close_cmd : gripper_open_cmd);
// //     ros::Duration(1).sleep();
// // }

// // // =================== 3. 感知与避障 ===================

// // void obstacleCB(const sensor_msgs::PointCloud2ConstPtr& msg) {
// //     pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
// //     pcl::fromROSMsg(*msg, *temp);
// //     pcl::transformPointCloud(*temp, *g_obstacle_cloud, T_cam_to_base.cast<float>());
// //     g_has_obs = true;
// // }

// // // 碰撞检测：加粗圆柱体 (半径建议设为 0.08 ~ 0.1)
// // bool checkCollision(Vector3d start, Vector3d end, double radius) {
// //     if (!g_has_obs || g_obstacle_cloud->empty()) return false;
    
// //     Vector3d AB = end - start;
// //     double len_sq = AB.squaredNorm();
// //     if(len_sq < 1e-6) return false;

// //     // 统计碰撞点数量，超过3个点才算碰撞 (防噪点)
// //     int collision_count = 0;

// //     for (const auto& pt : g_obstacle_cloud->points) {
// //         Vector3d P(pt.x, pt.y, pt.z);
// //         Vector3d AP = P - start;
// //         double t = AP.dot(AB) / len_sq;
        
// //         // 检测范围：从预备点到目标点，稍微往两头延伸一点
// //         if (t < -0.05 || t > 1.05) continue; 
        
// //         Vector3d Q = start + t * AB;
// //         if ((P - Q).norm() < radius) {
// //             collision_count++;
// //             if(collision_count > 3) return true; // 确认碰撞
// //         }
// //     }
// //     return false;
// // }

// // // =================== 4. 核心策略 (有限制的灵活姿态) ===================

// // /**
// //  * [姿态计算 - 灵活版]
// //  * 1. Z轴 (Approach): 严格对准苹果 (这样才能绕开障碍)
// //  * 2. X轴 (Wrist): 尽量贴近 HOME 姿态的 X 轴 (防止乱翻腕)
// //  */
// // vector<double> calcPoseSmart(Vector3d approach_vec, Matrix3d home_rot) {
// //     Vector3d z_new = approach_vec.normalized();
// //     Vector3d x_home = home_rot.col(0); // 参考 X 轴

// //     // 叉乘算出 Y，再算出 X
// //     Vector3d y_new = z_new.cross(x_home);
// //     if (y_new.norm() < 0.01) y_new = z_new.cross(Vector3d(0,1,0)); // 防奇异
// //     y_new.normalize();
    
// //     Vector3d x_new = y_new.cross(z_new).normalized();

// //     Matrix3d R_new;
// //     R_new.col(0) = x_new; R_new.col(1) = y_new; R_new.col(2) = z_new;
    
// //     Vector3d e = R_new.eulerAngles(2, 1, 0); 
// //     return {e[2], e[1], e[0]};
// // }

// // vector<GraspCandidate> planGraspStrategies(Vector3d center_pos) {
// //     vector<GraspCandidate> candidates;
// //     double phi = M_PI * (3.0 - sqrt(5.0)); 
    
// //     // 构建 Home 旋转矩阵
// //     Matrix3d home_rot;
// //     home_rot = AngleAxisd(HOME_RX, Vector3d::UnitZ()) * AngleAxisd(HOME_RY, Vector3d::UnitY()) * AngleAxisd(HOME_RZ, Vector3d::UnitX());
// //     Vector3d home_z = home_rot.col(2); // (1, 0, 0) 大概方向

// //     // 1. 斐波那契采样 (在球面上找空隙)
// //     for (int i = 0; i < NUM_SAMPLES; ++i) {
// //         double yi = 1 - (i / float(NUM_SAMPLES - 1)) * 2; 
// //         double radius = sqrt(1 - yi * yi);
// //         double theta = phi * i;
// //         double xi = cos(theta) * radius; double zi = sin(theta) * radius;
        
// //         // approach 是从球面指向球心
// //         // 我们的球面坐标系: Z是高，X是深。
// //         // 所以我们想要 X 为正的向量 (从机器人这边指向苹果)
// //         Vector3d vec_sphere(xi, zi, yi); 
// //         Vector3d approach = -vec_sphere.normalized(); 

// //         // ----------------- 过滤条件 -----------------
        
// //         // 1. 必须是从正面进攻 (X > 0)
// //         // 注意：之前的代码这里可能有坐标系定义混淆。
// //         // 如果你的 T_cam_to_base 是标准的，通常 X+ 是机器人前方。
// //         // 我们这里只保留与 Home 方向大致相同的向量。
// //         double angle_diff = acos(approach.dot(home_z));
        
// //         // [限制] 允许偏离 Home 方向最多 60 度
// //         // 超过60度意味着侧面太厉害，可能会撞基座或奇异
// //         if (angle_diff > M_PI / 3.0) continue; 

// //         // 2. 禁止从正上方下压 (容易压断枝)
// //         if (approach.z() < -0.3) continue;

// //         // ----------------- 评分 -----------------
// //         double score = 100.0;
        
// //         // 越正对 (角度差越小) 分数越高
// //         score -= angle_diff * 50.0; 
        
// //         // 稍微奖励仰角 (从下往上掏比较安全)
// //         if (approach.z() > 0) score += approach.z() * 30.0; 

// //         GraspCandidate task;
// //         task.id = i; 
// //         task.approach_vec = approach; 
        
// //         // [核心改动] 不再锁死 RPY，而是根据 approach 智能计算
// //         // 这样如果 approach 是斜着的，夹爪也会跟着斜过来
// //         task.rpy = calcPoseSmart(approach, home_rot); 
        
// //         task.score = score;
// //         candidates.push_back(task);
// //     }
    
// //     // 排序
// //     sort(candidates.begin(), candidates.end(), 
// //     [](const GraspCandidate& a, const GraspCandidate& b) { return a.score > b.score; });
    
// //     return candidates;
// // }

// // // =================== 5. 业务回调 ===================

// // bool pickCallBack(task_assign::Target::Request& req, task_assign::Target::Response& resp) {
// //     if (!robot_ptr) return false;
// //     std::error_code ec;

// //     Vector4d p_cam(req.x, req.y, req.z * depth_scale, 1.0);
// //     Vector3d target = (T_cam_to_base * p_cam).head<3>();
    
// //     ROS_INFO(">>> 目标: [%.3f, %.3f, %.3f]", target.x(), target.y(), target.z());

// //     if (target.norm() > MAX_REACH) { 
// //         ROS_ERROR("目标过远"); resp.success = false; return true; 
// //     }

// //     auto strategies = planGraspStrategies(target);
// //     ROS_INFO("生成策略数: %ld (尝试避障)", strategies.size()); 

// //     bool success = false;
// //     string id; 

// //     // 尝试前 20 个策略 (多试一点，因为可能有障碍物)
// //     for (int i = 0; i < min((int)strategies.size(), 20); ++i) { 
// //         auto& task = strategies[i];
        
// //         Vector3d pre_pos = target - task.approach_vec * DIST_PRE;
        
// //         // [避障检测] 半径设为 8cm (三指夹爪宽度)
// //         if (checkCollision(pre_pos, target, 0.08)) {
// //             // 只打印警告，不刷屏
// //             if(i < 3) ROS_WARN("策略 #%d 路径受阻，尝试绕行...", i+1);
// //             continue; 
// //         }
        
// //         if (pre_pos.z() < 0.10) continue; // 太低了

// //         ROS_INFO("选中策略 #%d (分: %.1f) | 姿态已调整", i+1, task.score);

// //         try {
// //             controlGripper(false);
            
// //             // 1. MoveJ 预备点
// //             // 因为限制了角度 < 60度，MoveJ 应该能算出来
// //             id = genUniqueId("pre"); 
// //             MoveJCommand cmd_pre({pre_pos.x(), pre_pos.y(), pre_pos.z(), 
// //                                 task.rpy[0], task.rpy[1], task.rpy[2]});
// //             cmd_pre.speed = 60; // 稍微慢点，安全第一
// //             robot_ptr->moveAppend({cmd_pre}, id, ec); robot_ptr->moveStart(ec);
// //             waitForFinish(*robot_ptr, id, 0); 
// //             ROS_INFO("finish pre");

// //             // 2. MoveL 进给
// //             id = genUniqueId("grasp");
// //             MoveLCommand cmd_grasp({target.x(), target.y(), target.z(), 
// //                                   task.rpy[0], task.rpy[1], task.rpy[2]});
// //             cmd_grasp.speed = 60; // 进给慢速
// //             robot_ptr->moveAppend({cmd_grasp}, id, ec); robot_ptr->moveStart(ec);
// //             waitForFinish(*robot_ptr, id, 0);
// //             ROS_INFO("finish grasp");
// //             // 3. 抓取
// //             controlGripper(true);
// //             ROS_INFO("finish gripper");

// //             // 4. MoveL 撤退
// //             id = genUniqueId("back");
// //             Vector3d ret_pos = target - task.approach_vec * DIST_RET;
// //             MoveLCommand cmd_back({ret_pos.x(), ret_pos.y(), ret_pos.z(), 
// //                                  task.rpy[0], task.rpy[1], task.rpy[2]});
// //             cmd_back.speed = 60;
// //             robot_ptr->moveAppend({cmd_back}, id, ec); robot_ptr->moveStart(ec);
// //             waitForFinish(*robot_ptr, id, 0);
// //             ROS_INFO("finish back");
// //             success = true; break; 
// //         } catch (...) {
// //             ROS_WARN("策略执行异常 (可能是姿态奇异)，尝试下一个..."); continue; 
// //         }
// //     }

// //     if (success) {
// //         id = genUniqueId("drop");
// //         MoveJCommand cmd_drop({0.436, -0.096825, -0.100019, HOME_RX, HOME_RY, HOME_RZ}); 
// //         robot_ptr->moveAppend({cmd_drop}, id, ec); robot_ptr->moveStart(ec);
// //         waitForFinish(*robot_ptr, id, 0);
// //         controlGripper(false);
// //         resp.success = true;
// //     } else {
// //         resp.success = false; resp.message = "All blocked";
// //         ROS_ERROR("所有策略均被阻挡或无法执行");
// //     }
// //     return true;
// // }

// // int main(int argc, char *argv[]) {
// //     setlocale(LC_ALL,"");
// //     ros::init(argc, argv, "smart_picking_node");
// //     ros::NodeHandle nh;

// //     pub_serial = nh.advertise<comm::serialData>("ap_robot/serial", 10);
// //     gripper_close_cmd.data = {0xaa, 0x55, 0x02, 0x00, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};
// //     gripper_open_cmd.data = {0xaa, 0x55, 0x02, 0x01, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};

// //     try {
// //         T_cam_to_base = loadMatrix("/home/y/dual_rokae_ws/src/rokae/src/camera_pose_left.txt");
// //     } catch (...) { return -1; }

// //     try {
// //         robot_ptr = new xMateRobot("192.168.2.160");
// //         std::error_code ec;
// //         robot_ptr->setOperateMode(OperateMode::automatic, ec);
// //         robot_ptr->setPowerState(true, ec);
// //         robot_ptr->setMotionControlMode(MotionControlMode::NrtCommand, ec);
        
// //         Toolset tool; tool.end = {{ 0, 0, 0.25}, {0, 0, 0}}; tool.load.mass = 0.70;
// //         robot_ptr->setToolset(tool, ec);

// //         ros::Subscriber sub = nh.subscribe("ap_robot/obstacles", 1, obstacleCB);

// //         ROS_INFO(">>> 启动成功 | 智能避障 + 姿态自适应 <<<");
// //         ros::ServiceServer server = nh.advertiseService("left_pickTarget", pickCallBack);
        
// //         ros::AsyncSpinner spinner(2);
// //         spinner.start();
// //         ros::waitForShutdown();
// //     } catch (...) { return -1; }

// //     if(robot_ptr) delete robot_ptr;
// //     return 0;
// // }


#include "ros/ros.h"
#include "task_assign/Target.h"
#include <thread>
#include "rokae/robot.h"
#include "comm/serialData.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <vector>
#include <algorithm>
#include <atomic>
#include <cmath>

// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace rokae;
using namespace Eigen;
using namespace std;

// =================== 1. 全局配置 ===================
const double DIST_PRE = 0.15; 
const double DIST_RET = 0.10; 
const double MAX_REACH = 1.5; 
const int NUM_SAMPLES = 120; // 稍微增加采样数以适应更灵活的空间

// 【姿态参考】侧装模式下的 Home 点
// 请确保这个 Home 姿态是机械臂在侧装时的一个自然、安全的中间姿态
const double HOME_RX = 1.64318;
const double HOME_RY = 0.732183;
const double HOME_RZ = 0.967122;
const vector<double> HOME_POSE_FULL = {0.706165, -0.244223, 0.000155, HOME_RX, HOME_RY, HOME_RZ};

xMateRobot* robot_ptr = nullptr;
ros::Publisher pub_serial;
comm::serialData gripper_close_cmd, gripper_open_cmd;
Eigen::Matrix4d T_cam_to_base;
float depth_scale = 0.9797;
std::atomic<int> g_motion_id_counter(0);

// 障碍物点云
pcl::PointCloud<pcl::PointXYZ>::Ptr g_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool g_has_obs = false;

struct GraspCandidate {
    int id;
    Vector3d approach_vec; 
    vector<double> rpy;   
    double score;
};

// =================== 2. 辅助函数 ===================

std::string genUniqueId(std::string prefix) {
    return prefix + "_" + std::to_string(g_motion_id_counter++);
}

Matrix4d loadMatrix(string path) {
    Matrix4d m; ifstream f(path);
    if(!f.is_open()) throw runtime_error("标定文件加载失败");
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) f >> m(i,j);
    return m;
}

void waitForFinish(BaseRobot &robot, const std::string &traj_id, int index) {
    std::error_code ec;
    int timeout = 0;
    while(timeout++ < 300) { 
        auto info = robot.queryEventInfo(Event::moveExecution, ec);
        if (!ec) {
            auto _id = any_cast<string>(info.at(EventInfoKey::MoveExecution::ID));
            auto _idx = any_cast<int>(info.at(EventInfoKey::MoveExecution::WaypointIndex));
            if (_id == traj_id && _idx == index) {
                if(any_cast<bool>(info.at(EventInfoKey::MoveExecution::ReachTarget))) {
                    this_thread::sleep_for(chrono::milliseconds(100)); return; 
                }
            }
        }
        this_thread::sleep_for(chrono::milliseconds(50));
    }
    throw runtime_error("Motion Timeout");
}

void controlGripper(bool close) {
    pub_serial.publish(close ? gripper_close_cmd : gripper_open_cmd);
    ros::Duration(1).sleep();
}

// =================== 3. 感知与避障 ===================

void obstacleCB(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp);
    pcl::transformPointCloud(*temp, *g_obstacle_cloud, T_cam_to_base.cast<float>());
    g_has_obs = true;
}

// 碰撞检测：加粗圆柱体 (半径建议设为 0.08 ~ 0.1)
bool checkCollision(Vector3d start, Vector3d end, double radius) {
    if (!g_has_obs || g_obstacle_cloud->empty()) return false;
    
    Vector3d AB = end - start;
    double len_sq = AB.squaredNorm();
    if(len_sq < 1e-6) return false;

    int collision_count = 0;
    for (const auto& pt : g_obstacle_cloud->points) {
        Vector3d P(pt.x, pt.y, pt.z);
        Vector3d AP = P - start;
        double t = AP.dot(AB) / len_sq;
        
        // 检测范围：从预备点到目标点
        if (t < -0.05 || t > 1.05) continue; 
        
        Vector3d Q = start + t * AB;
        if ((P - Q).norm() < radius) {
            collision_count++;
            if(collision_count > 3) return true; // 确认碰撞
        }
    }
    return false;
}

// =================== 4. 核心策略 (侧装适配版) ===================

/**
 * [姿态计算]
 * 侧装模式下，我们依然希望 X 轴(手腕)尽量不要乱转，保持接近 Home 的 X 轴朝向
 */
vector<double> calcPoseSmart(Vector3d approach_vec, Matrix3d home_rot) {
    Vector3d z_new = approach_vec.normalized();
    Vector3d x_home = home_rot.col(0); // 参考 X 轴

    // 叉乘算出 Y，再算出 X
    Vector3d y_new = z_new.cross(x_home);
    if (y_new.norm() < 0.01) y_new = z_new.cross(Vector3d(0,1,0)); // 防奇异
    y_new.normalize();
    
    Vector3d x_new = y_new.cross(z_new).normalized();

    Matrix3d R_new;
    R_new.col(0) = x_new; R_new.col(1) = y_new; R_new.col(2) = z_new;
    
    Vector3d e = R_new.eulerAngles(2, 1, 0); 
    return {e[2], e[1], e[0]};
}

vector<GraspCandidate> planGraspStrategies(Vector3d center_pos) {
    vector<GraspCandidate> candidates;
    double phi = M_PI * (3.0 - sqrt(5.0)); 
    
    // 构建 Home 旋转矩阵
    Matrix3d home_rot;
    home_rot = AngleAxisd(HOME_RX, Vector3d::UnitZ()) * AngleAxisd(HOME_RY, Vector3d::UnitY()) * AngleAxisd(HOME_RZ, Vector3d::UnitX());
    Vector3d home_z = home_rot.col(2); // (1, 0, 0) 大概方向

    // 1. 斐波那契采样
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        double yi = 1 - (i / float(NUM_SAMPLES - 1)) * 2; 
        double radius = sqrt(1 - yi * yi);
        double theta = phi * i;
        double xi = cos(theta) * radius; double zi = sin(theta) * radius;
        
        Vector3d vec_sphere(xi, zi, yi); 
        Vector3d approach = -vec_sphere.normalized(); 

        // ----------------- 侧装模式过滤条件 -----------------
        
        // 1. 【修改】放宽角度限制
        // 侧装时，为了抓取不同高度的物体，可能需要较大的 Z 轴偏转
        // 我们依然保留“大致前方”的限制，防止机械臂往自己基座里面抓，但阈值放宽到 80度
        double angle_diff = acos(approach.dot(home_z));
        if (angle_diff > M_PI / 2.2) continue; // 约 80度

        // 2. 【删除】禁止从正上方下压 (approach.z < -0.3)
        // 侧装时，Z 轴可能是水平的，这个限制没有物理意义了
        // 依靠 checkCollision 来避免撞到侧面的墙壁或地面

        // ----------------- 评分 -----------------
        double score = 100.0;
        
        // 越正对 Home 方向分数越高 (意味着姿态最舒服)
        score -= angle_diff * 40.0; 
        
        // 【删除】Z轴评分偏好
        // 侧装时 Z > 0 不代表"上方"，所以去掉这个偏好，让所有方向平等

        GraspCandidate task;
        task.id = i; 
        task.approach_vec = approach; 
        
        // 姿态计算
        task.rpy = calcPoseSmart(approach, home_rot); 
        
        task.score = score;
        candidates.push_back(task);
    }
    
    // 排序
    sort(candidates.begin(), candidates.end(), 
    [](const GraspCandidate& a, const GraspCandidate& b) { return a.score > b.score; });
    
    return candidates;
}

// =================== 5. 业务回调 ===================

bool pickCallBack(task_assign::Target::Request& req, task_assign::Target::Response& resp) {
    if (!robot_ptr) return false;
    std::error_code ec;

    Vector4d p_cam(req.x, req.y, req.z * depth_scale, 1.0);
    Vector3d target = (T_cam_to_base * p_cam).head<3>();
    
    ROS_INFO(">>> 目标: [%.3f, %.3f, %.3f]", target.x(), target.y(), target.z());

    if (target.norm() > MAX_REACH) { 
        ROS_ERROR("目标过远"); resp.success = false; return true; 
    }

    auto strategies = planGraspStrategies(target);
    ROS_INFO("生成策略数: %ld (侧装无Z轴限制)", strategies.size()); 

    bool success = false;
    string id; 

    // 尝试前 20 个策略
    for (int i = 0; i < min((int)strategies.size(), 20); ++i) { 
        auto& task = strategies[i];
        
        Vector3d pre_pos = target - task.approach_vec * DIST_PRE;
        
        // [避障检测]
        if (checkCollision(pre_pos, target, 0.08)) {
            if(i < 3) ROS_WARN("策略 #%d 路径受阻，尝试绕行...", i+1);
            continue; 
        }
        
        // 【删除】pre_pos.z() < 0.10 的硬编码限制
        // 侧装时，Z < 0.1 可能只是表示在基座的某一边，并不代表撞地
        // 警告：必须确保你的相机能看到侧面的墙或地面，从而生成点云来避障！

        ROS_INFO("选中策略 #%d (分: %.1f)", i+1, task.score);

        try {
            controlGripper(false);
            
            // 1. MoveJ 预备点
            id = genUniqueId("pre"); 
            MoveJCommand cmd_pre({pre_pos.x(), pre_pos.y(), pre_pos.z(), 
                                task.rpy[0], task.rpy[1], task.rpy[2]});
            cmd_pre.speed = 200; 
            robot_ptr->moveAppend({cmd_pre}, id, ec); robot_ptr->moveStart(ec);
            waitForFinish(*robot_ptr, id, 0); 
            ROS_INFO("finish pre");

            // 2. MoveL 进给
            id = genUniqueId("grasp");
            MoveLCommand cmd_grasp({target.x(), target.y(), target.z(), 
                                  task.rpy[0], task.rpy[1], task.rpy[2]});
            cmd_grasp.speed = 200; 
            robot_ptr->moveAppend({cmd_grasp}, id, ec); robot_ptr->moveStart(ec);
            waitForFinish(*robot_ptr, id, 0);
            ROS_INFO("finish grasp");

            // 3. 抓取
            controlGripper(true);
            ROS_INFO("finish gripper");

            // 4. MoveL 撤退
            id = genUniqueId("back");
            Vector3d ret_pos = target - task.approach_vec * DIST_RET;
            MoveLCommand cmd_back({ret_pos.x(), ret_pos.y(), ret_pos.z(), 
                                 task.rpy[0], task.rpy[1], task.rpy[2]});
            cmd_back.speed = 200;
            robot_ptr->moveAppend({cmd_back}, id, ec); robot_ptr->moveStart(ec);
            waitForFinish(*robot_ptr, id, 0);
            ROS_INFO("finish back");
            success = true; break; 
        } catch (...) {
            ROS_WARN("策略执行异常 (可能是姿态奇异)，尝试下一个..."); continue; 
        }
    }

    if (success) {
        id = genUniqueId("drop");
        MoveJCommand cmd_drop({0.285725, -0.630346,0.244551,1.63061,0.707504,00.0520108}); // 使用新的 Home 点回位
        robot_ptr->moveAppend({cmd_drop}, id, ec); robot_ptr->moveStart(ec);
        waitForFinish(*robot_ptr, id, 0);
        controlGripper(false);
        // id = genUniqueId("home");
        // MoveJCommand cmd_home({0.436, -0.096825, 0.100019, HOME_RX, HOME_RY, HOME_RZ}); // 使用新的 Home 点回位
        // robot_ptr->moveAppend({cmd_home}, id, ec); robot_ptr->moveStart(ec);
        // waitForFinish(*robot_ptr, id, 0);
        resp.success = true;
    } else {
        resp.success = false; resp.message = "All blocked";
        ROS_ERROR("所有策略均被阻挡或无法执行");
    }
    return true;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "smart_picking_node");
    ros::NodeHandle nh;

    pub_serial = nh.advertise<comm::serialData>("ap_robot/serial", 10);
    gripper_close_cmd.data = {0xaa, 0x55, 0x02, 0x00, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};
    gripper_open_cmd.data = {0xaa, 0x55, 0x02, 0x01, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};

    try {
        T_cam_to_base = loadMatrix("/home/y/dual_rokae_ws/src/rokae/src/camera_pose_left.txt");
    } catch (...) { return -1; }

    try {
        robot_ptr = new xMateRobot("192.168.2.160"); // IP 已改为 160
        std::error_code ec;
        robot_ptr->setOperateMode(OperateMode::automatic, ec);
        robot_ptr->setPowerState(true, ec);
        robot_ptr->setMotionControlMode(MotionControlMode::NrtCommand, ec);
        
        Toolset tool; tool.end = {{ 0, 0, 0.25}, {0, 0, 0}}; tool.load.mass = 0.70;
        robot_ptr->setToolset(tool, ec);

        ros::Subscriber sub = nh.subscribe("ap_robot/obstacles", 1, obstacleCB);

        ROS_INFO(">>> 启动成功 | 侧装模式 (无Z轴限制) <<<");
        ros::ServiceServer server = nh.advertiseService("left_pickTarget", pickCallBack);
        
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    } catch (...) { return -1; }

    if(robot_ptr) delete robot_ptr;
    return 0;
}



// #include "ros/ros.h"
// #include "task_assign/Target.h"
// #include <thread>
// #include "rokae/robot.h"
// #include "comm/serialData.h"
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
// #include <fstream>
// #include <vector>
// #include <algorithm>
// #include <atomic>
// #include <cmath>

// // PCL
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/common/transforms.h>

// using namespace rokae;
// using namespace Eigen;
// using namespace std;

// // =================== 1. 全局配置 ===================
// const double DIST_PRE = 0.15; 
// const double DIST_RET = 0.10; 
// const double MAX_REACH = 1.3; 
// const int NUM_SAMPLES = 120; 

// // 【新增】X轴最小安全距离 (防止撞到底座或安装支架)
// const double MIN_X = 0.20; 

// // 【姿态参考】侧装模式 Home 点
// const double HOME_RX = 2.95950;
// const double HOME_RY = 0.124145268;
// const double HOME_RZ = 1.518890209;
// const vector<double> HOME_POSE_FULL = {0.436, -0.096825, 0.100019, HOME_RX, HOME_RY, HOME_RZ};

// xMateRobot* robot_ptr = nullptr;
// ros::Publisher pub_serial;
// comm::serialData gripper_close_cmd, gripper_open_cmd;
// Eigen::Matrix4d T_cam_to_base;
// float depth_scale = 0.9797;
// std::atomic<int> g_motion_id_counter(0);

// // 障碍物点云
// pcl::PointCloud<pcl::PointXYZ>::Ptr g_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// bool g_has_obs = false;

// struct GraspCandidate {
//     int id;
//     Vector3d approach_vec; 
//     vector<double> rpy;   
//     double score;
// };

// // =================== 2. 辅助函数 ===================

// std::string genUniqueId(std::string prefix) {
//     return prefix + "_" + std::to_string(g_motion_id_counter++);
// }

// Matrix4d loadMatrix(string path) {
//     Matrix4d m; ifstream f(path);
//     if(!f.is_open()) throw runtime_error("标定文件加载失败");
//     for(int i=0;i<4;++i) for(int j=0;j<4;++j) f >> m(i,j);
//     return m;
// }

// void waitForFinish(BaseRobot &robot, const std::string &traj_id, int index) {
//     std::error_code ec;
//     int timeout = 0;
//     while(timeout++ < 300) { 
//         auto info = robot.queryEventInfo(Event::moveExecution, ec);
//         if (!ec) {
//             auto _id = any_cast<string>(info.at(EventInfoKey::MoveExecution::ID));
//             auto _idx = any_cast<int>(info.at(EventInfoKey::MoveExecution::WaypointIndex));
//             if (_id == traj_id && _idx == index) {
//                 if(any_cast<bool>(info.at(EventInfoKey::MoveExecution::ReachTarget))) {
//                     this_thread::sleep_for(chrono::milliseconds(100)); return; 
//                 }
//             }
//         }
//         this_thread::sleep_for(chrono::milliseconds(50));
//     }
//     throw runtime_error("Motion Timeout");
// }

// void controlGripper(bool close) {
//     pub_serial.publish(close ? gripper_close_cmd : gripper_open_cmd);
//     ros::Duration(1).sleep();
// }

// // =================== 3. 感知与避障 ===================

// void obstacleCB(const sensor_msgs::PointCloud2ConstPtr& msg) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*msg, *temp);
//     pcl::transformPointCloud(*temp, *g_obstacle_cloud, T_cam_to_base.cast<float>());
//     g_has_obs = true;
// }

// // 碰撞检测
// bool checkCollision(Vector3d start, Vector3d end, double radius) {
//     if (!g_has_obs || g_obstacle_cloud->empty()) return false;
    
//     Vector3d AB = end - start;
//     double len_sq = AB.squaredNorm();
//     if(len_sq < 1e-6) return false;

//     int collision_count = 0;
//     for (const auto& pt : g_obstacle_cloud->points) {
//         Vector3d P(pt.x, pt.y, pt.z);
//         Vector3d AP = P - start;
//         double t = AP.dot(AB) / len_sq;
        
//         if (t < -0.05 || t > 1.05) continue; 
        
//         Vector3d Q = start + t * AB;
//         if ((P - Q).norm() < radius) {
//             collision_count++;
//             if(collision_count > 3) return true; 
//         }
//     }
//     return false;
// }

// // =================== 4. 核心策略 (侧装版+X轴限制) ===================

// vector<double> calcPoseSmart(Vector3d approach_vec, Matrix3d home_rot) {
//     Vector3d z_new = approach_vec.normalized();
//     Vector3d x_home = home_rot.col(0); 

//     Vector3d y_new = z_new.cross(x_home);
//     if (y_new.norm() < 0.01) y_new = z_new.cross(Vector3d(0,1,0)); 
//     y_new.normalize();
    
//     Vector3d x_new = y_new.cross(z_new).normalized();

//     Matrix3d R_new;
//     R_new.col(0) = x_new; R_new.col(1) = y_new; R_new.col(2) = z_new;
    
//     Vector3d e = R_new.eulerAngles(2, 1, 0); 
//     return {e[2], e[1], e[0]};
// }

// vector<GraspCandidate> planGraspStrategies(Vector3d center_pos) {
//     vector<GraspCandidate> candidates;
//     double phi = M_PI * (3.0 - sqrt(5.0)); 
    
//     Matrix3d home_rot;
//     home_rot = AngleAxisd(HOME_RX, Vector3d::UnitZ()) * AngleAxisd(HOME_RY, Vector3d::UnitY()) * AngleAxisd(HOME_RZ, Vector3d::UnitX());
//     Vector3d home_z = home_rot.col(2); 

//     for (int i = 0; i < NUM_SAMPLES; ++i) {
//         double yi = 1 - (i / float(NUM_SAMPLES - 1)) * 2; 
//         double radius = sqrt(1 - yi * yi);
//         double theta = phi * i;
//         double xi = cos(theta) * radius; double zi = sin(theta) * radius;
        
//         Vector3d vec_sphere(xi, zi, yi); 
//         Vector3d approach = -vec_sphere.normalized(); 

//         // 侧装角度限制 (80度)
//         double angle_diff = acos(approach.dot(home_z));
//         if (angle_diff > M_PI / 2.2) continue; 

//         double score = 100.0;
//         score -= angle_diff * 40.0; 

//         GraspCandidate task;
//         task.id = i; 
//         task.approach_vec = approach; 
//         task.rpy = calcPoseSmart(approach, home_rot); 
//         task.score = score;
//         candidates.push_back(task);
//     }
    
//     sort(candidates.begin(), candidates.end(), 
//     [](const GraspCandidate& a, const GraspCandidate& b) { return a.score > b.score; });
    
//     return candidates;
// }

// // =================== 5. 业务回调 ===================

// bool pickCallBack(task_assign::Target::Request& req, task_assign::Target::Response& resp) {
//     if (!robot_ptr) return false;
//     std::error_code ec;

//     Vector4d p_cam(req.x, req.y, req.z * depth_scale, 1.0);
//     Vector3d target = (T_cam_to_base * p_cam).head<3>();
    
//     ROS_INFO(">>> 目标: [%.3f, %.3f, %.3f]", target.x(), target.y(), target.z());

//     if (target.norm() > MAX_REACH) { 
//         ROS_ERROR("目标过远"); resp.success = false; return true; 
//     }

//     // 【新增】目标点硬限制：X 不能小于 20cm
//     if (target.x() < MIN_X) {
//         ROS_ERROR("目标距离基座太近 (X < %.2fm)，存在碰撞风险，拒绝执行！", MIN_X);
//         resp.success = false; 
//         return true;
//     }

//     auto strategies = planGraspStrategies(target);
//     ROS_INFO("生成策略数: %ld", strategies.size()); 

//     bool success = false;
//     string id; 

//     for (int i = 0; i < min((int)strategies.size(), 20); ++i) { 
//         auto& task = strategies[i];
        
//         Vector3d pre_pos = target - task.approach_vec * DIST_PRE;
        
//         // 【新增】预备点硬限制：预备点也不能跑到底座里去
//         if (pre_pos.x() < MIN_X) {
//             if(i < 3) ROS_WARN("策略 #%d 预备点离基座太近 (X=%.2f)，跳过", i+1, pre_pos.x());
//             continue; 
//         }

//         // 避障检测
//         if (checkCollision(pre_pos, target, 0.08)) {
//             if(i < 3) ROS_WARN("策略 #%d 路径受阻，尝试绕行...", i+1);
//             continue; 
//         }

//         ROS_INFO("选中策略 #%d (分: %.1f)", i+1, task.score);

//         try {
//             controlGripper(false);
            
//             // 1. MoveJ 预备点
//             id = genUniqueId("pre"); 
//             MoveJCommand cmd_pre({pre_pos.x(), pre_pos.y(), pre_pos.z(), 
//                                 task.rpy[0], task.rpy[1], task.rpy[2]});
//             cmd_pre.speed = 60; 
//             robot_ptr->moveAppend({cmd_pre}, id, ec); robot_ptr->moveStart(ec);
//             waitForFinish(*robot_ptr, id, 0); 
//             ROS_INFO("finish pre");

//             // 2. MoveL 进给
//             id = genUniqueId("grasp");
//             MoveLCommand cmd_grasp({target.x(), target.y(), target.z(), 
//                                   task.rpy[0], task.rpy[1], task.rpy[2]});
//             cmd_grasp.speed = 60; 
//             robot_ptr->moveAppend({cmd_grasp}, id, ec); robot_ptr->moveStart(ec);
//             waitForFinish(*robot_ptr, id, 0);
//             ROS_INFO("finish grasp");

//             // 3. 抓取
//             controlGripper(true);
//             ROS_INFO("finish gripper");

//             // 4. MoveL 撤退
//             id = genUniqueId("back");
//             Vector3d ret_pos = target - task.approach_vec * DIST_RET;
//             MoveLCommand cmd_back({ret_pos.x(), ret_pos.y(), ret_pos.z(), 
//                                  task.rpy[0], task.rpy[1], task.rpy[2]});
//             cmd_back.speed = 60;
//             robot_ptr->moveAppend({cmd_back}, id, ec); robot_ptr->moveStart(ec);
//             waitForFinish(*robot_ptr, id, 0);
//             ROS_INFO("finish back");
//             success = true; break; 
//         } catch (...) {
//             ROS_WARN("策略执行异常 (可能是姿态奇异)，尝试下一个..."); continue; 
//         }
//     }

//     if (success) {
//         id = genUniqueId("drop");
//         MoveJCommand cmd_drop({0.285725, -0.630346,0.244551,1.63061,0.707504,00.0520108}); 
//         robot_ptr->moveAppend({cmd_drop}, id, ec); robot_ptr->moveStart(ec);
//         waitForFinish(*robot_ptr, id, 0);
//         controlGripper(false);
//         resp.success = true;
//     } else {
//         resp.success = false; resp.message = "All blocked";
//         ROS_ERROR("所有策略均被阻挡或不满足安全距离");
//     }
//     return true;
// }

// int main(int argc, char *argv[]) {
//     setlocale(LC_ALL,"");
//     ros::init(argc, argv, "smart_picking_node");
//     ros::NodeHandle nh;

//     pub_serial = nh.advertise<comm::serialData>("ap_robot/serial", 10);
//     gripper_close_cmd.data = {0xaa, 0x55, 0x02, 0x00, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};
//     gripper_open_cmd.data = {0xaa, 0x55, 0x02, 0x01, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};

//     try {
//         T_cam_to_base = loadMatrix("/home/y/dual_rokae_ws/src/rokae/src/camera_pose_left.txt");
//     } catch (...) { return -1; }

//     try {
//         robot_ptr = new xMateRobot("192.168.2.160");
//         std::error_code ec;
//         robot_ptr->setOperateMode(OperateMode::automatic, ec);
//         robot_ptr->setPowerState(true, ec);
//         robot_ptr->setMotionControlMode(MotionControlMode::NrtCommand, ec);
        
//         Toolset tool; tool.end = {{ 0, 0, 0.25}, {0, 0, 0}}; tool.load.mass = 0.70;
//         robot_ptr->setToolset(tool, ec);

//         ros::Subscriber sub = nh.subscribe("ap_robot/obstacles", 1, obstacleCB);

//         ROS_INFO(">>> 启动成功 | 侧装模式 (Min X = 0.2m) <<<");
//         ros::ServiceServer server = nh.advertiseService("left_pickTarget", pickCallBack);
        
//         ros::AsyncSpinner spinner(2);
//         spinner.start();
//         ros::waitForShutdown();
//     } catch (...) { return -1; }

//     if(robot_ptr) delete robot_ptr;
//     return 0;
// }