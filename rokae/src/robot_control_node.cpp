#include "ros/ros.h"
#include "task_assign/Target.h"
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include "rokae/utility.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "comm/serialData.h"
#include <fstream>
#include <string>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <std_msgs/Bool.h>

using namespace rokae;
using namespace Eigen;
using namespace std;

// =================== 1. 运行时参数（ROS param 可覆盖）===================
double DIST_PRE         = 0.15;
double DIST_RET         = 0.10;
double GRIPPER_OPEN_WAIT  = 0.5;
double GRIPPER_CLOSE_WAIT = 0.6;
double MAX_REACH        = 1.5;
double COLLISION_RADIUS = 0.08;
double APPROACH_SPEED   = 60.0;
double TRANSIT_SPEED    = 200.0;
int    NUM_SAMPLES      = 120;

// 右臂 Home 点姿态（侧装模式）
const double HOME_RX = 1.49086;
const double HOME_RY = 0.419559699;
const double HOME_RZ = 1.6362287;
// 右臂放置点（笛卡尔坐标 {x, y, z, rx, ry, rz}，单位 m/rad）
const array<double, 6> DROP_POSE = {0.508949, -0.189169, 0.163698, 1.49086, 0.419559699, 1.6362287};

// =================== 2. 正向运动学（硬编码 DH，从 xMateSR4 xacro 提取）===================
//
// 右臂安装变换：parent=base_link, xyz="0.0 -0.150 0.885", rpy="1.5708 0 0"
// 关节链 DH 偏移（各关节相对于上一关节坐标系的平移）：
//   joint1: 0, 0, 0
//   joint2: 0, 0, 0.355
//   joint3: 0.05, 0, 0.4
//   joint4: -0.05, 0, 0.4
//   joint5: 0, 0.136, 0
//   joint6: 0, 0, 0.1035
//   tool  : 0, 0, 0.25  (夹爪长度)
//
// 各关节轴（相对于自身坐标系）：
//   joint1: Z轴  joint2: Y轴  joint3: -Y轴
//   joint4: Z轴  joint5: -Y轴 joint6: Z轴

// 右臂安装变换矩阵（base_link -> right_base）
static Matrix4d buildRightMountTransform() {
    // xyz=(0, -0.15, 0.885), rpy=(1.5708, 0, 0)
    Matrix4d T = Matrix4d::Identity();
    double rx = 1.5708;
    Matrix3d R = AngleAxisd(rx, Vector3d::UnitX()).toRotationMatrix();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = Vector3d(0.0, -0.150, 0.885);
    return T;
}

// 给定6个关节角（rad），返回各关节原点在 base_link 坐标系下的位置
// 返回 7 个点：right_base + joint1~6 的原点位置（用于胶囊碰撞检测）
// 末尾附加 tool 末端点（index 7）
static vector<Vector3d> forwardKinematicsLinkPositions(const vector<double>& q) {
    // 关节偏移（child joint origin 相对于 parent link 坐标系）
    static const vector<Vector3d> joint_offset = {
        {0.0,   0.0,    0.0   },  // joint1 origin in base frame
        {0.0,   0.0,    0.355 },  // joint2 origin in link1 frame
        {0.05,  0.0,    0.4   },  // joint3 origin in link2 frame
        {-0.05, 0.0,    0.4   },  // joint4 origin in link3 frame
        {0.0,   0.136,  0.0   },  // joint5 origin in link4 frame
        {0.0,   0.0,    0.1035},  // joint6 origin in link5 frame
        {0.0,   0.0,    0.25  },  // tool   origin in link6 frame
    };
    // 各关节旋转轴（joint i 绕该轴转动 q[i]）
    static const vector<Vector3d> joint_axis = {
        { 0,  0,  1},  // joint1: Z
        { 0,  1,  0},  // joint2: Y
        { 0, -1,  0},  // joint3: -Y
        { 0,  0,  1},  // joint4: Z
        { 0, -1,  0},  // joint5: -Y
        { 0,  0,  1},  // joint6: Z
    };

    static const Matrix4d T_mount = buildRightMountTransform();

    // 从 base_link 累积变换
    Matrix4d T = T_mount;

    vector<Vector3d> pts;
    // right_base 原点（即 T_mount 的平移）
    pts.push_back(T.block<3,1>(0,3));

    for (int i = 0; i < 6; ++i) {
        // 先走 joint_offset[i]（在当前坐标系下平移）
        Vector4d p_local(joint_offset[i].x(), joint_offset[i].y(), joint_offset[i].z(), 1.0);
        Vector3d p_world = (T * p_local).head<3>();
        pts.push_back(p_world);

        // 然后绕关节轴旋转 q[i]
        Matrix4d Tj = Matrix4d::Identity();
        Tj.block<3,3>(0,0) = AngleAxisd(q[i], joint_axis[i]).toRotationMatrix();
        T = T * Tj;
    }

    // tool 末端点
    {
        Vector4d p_local(joint_offset[6].x(), joint_offset[6].y(), joint_offset[6].z(), 1.0);
        pts.push_back((T * p_local).head<3>());
    }

    return pts; // 8 个点：base + joint1~6 origin + tool tip
}

// =================== 3. 全局状态 ===================
xMateRobot* robot_ptr = nullptr;
ros::Publisher pub_serial;
comm::serialData leftClosedata, leftOpendata;
Eigen::Matrix4d T_cam_to_base;
float depth_scale = 1.0;
atomic<int> g_motion_id_counter(0);

// 障碍物点云（基座坐标系）
mutex obstacle_mutex;
pcl::PointCloud<pcl::PointXYZ>::Ptr g_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool g_has_obs = false;

// 串口 ack
mutex ack_mutex;
bool g_gripper_ack = false;

// 连杆碰撞监控
mutex joint_mutex;
vector<double> g_joint_positions(6, 0.0);
bool g_joint_valid = false;
atomic<bool> g_link_collision_detected(false);
atomic<bool> g_monitor_active(false);

// 抓取候选结构体
struct GraspCandidate {
    int id;
    Vector3d approach_vec;
    vector<double> rpy;
    double score;
};

// =================== 4. 回调 ===================

void obstacleCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*temp, *transformed, T_cam_to_base.cast<float>());

    lock_guard<mutex> lk(obstacle_mutex);
    g_obstacle_cloud = transformed;
    g_has_obs = true;
}

void serialAckCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        lock_guard<mutex> lk(ack_mutex);
        g_gripper_ack = true;
    }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // 匹配 right_joint1 ~ right_joint6
    static const vector<string> joint_names = {
        "right_joint1","right_joint2","right_joint3",
        "right_joint4","right_joint5","right_joint6"
    };
    vector<double> q(6, 0.0);
    bool found[6] = {};
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < 6; ++j) {
            if (msg->name[i] == joint_names[j]) {
                q[j] = msg->position[i];
                found[j] = true;
            }
        }
    }
    bool all_found = true;
    for (int j = 0; j < 6; ++j) if (!found[j]) { all_found = false; break; }
    if (!all_found) return;

    lock_guard<mutex> lk(joint_mutex);
    g_joint_positions = q;
    g_joint_valid = true;
}

// =================== 5. 辅助函数 ===================

string genId(const string& prefix) {
    return prefix + "_" + to_string(g_motion_id_counter++);
}

Eigen::Matrix4d loadMatrix(const string& path) {
    Matrix4d m;
    ifstream f(path);
    if (!f.is_open()) throw runtime_error("无法打开标定文件: " + path);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            f >> m(i, j);
    return m;
}

float loadFloat(const string& path) {
    float v = 1.0f;
    ifstream f(path);
    if (f.is_open()) f >> v;
    return v;
}

void waitForFinish(BaseRobot& robot, const string& traj_id, int index) {
    error_code ec;
    int timeout = 0;
    while (timeout++ < 600) {
        auto info = robot.queryEventInfo(Event::moveExecution, ec);
        if (!ec) {
            auto _id  = any_cast<string>(info.at(EventInfoKey::MoveExecution::ID));
            auto _idx = any_cast<int>(info.at(EventInfoKey::MoveExecution::WaypointIndex));
            if (auto _ec = any_cast<error_code>(info.at(EventInfoKey::MoveExecution::Error))) {
                ROS_ERROR_STREAM("运动错误 " << _id << ": " << _ec.message());
                return;
            }
            if (_id == traj_id && _idx == index) {
                if (any_cast<bool>(info.at(EventInfoKey::MoveExecution::ReachTarget))) return;
            }
        }
        // 检查连杆碰撞紧急停止
        if (g_link_collision_detected.load()) {
            ROS_ERROR("连杆碰撞监控触发！紧急停止运动");
            robot.stop(ec);
            throw runtime_error("Link collision detected, motion stopped");
        }
        this_thread::sleep_for(chrono::milliseconds(50));
    }
    throw runtime_error("Motion Timeout: " + traj_id);
}

void waitGripper(double timeout_sec) {
    {
        lock_guard<mutex> lk(ack_mutex);
        g_gripper_ack = false;
    }
    ros::Time start = ros::Time::now();
    ros::Rate r(50);
    while (ros::ok()) {
        {
            lock_guard<mutex> lk(ack_mutex);
            if (g_gripper_ack) return;
        }
        if ((ros::Time::now() - start).toSec() >= timeout_sec) return;
        r.sleep();
    }
}

// =================== 6. 碰撞检测 ===================

struct RackAABB {
    double xmin = -0.25, xmax = 0.25;
    double ymin = -0.35, ymax = 0.35;
    double zmin =  0.00, zmax = 0.885;
} g_rack;

bool isInRack(const Vector3d& p, double margin) {
    return (p.x() > g_rack.xmin - margin && p.x() < g_rack.xmax + margin &&
            p.y() > g_rack.ymin - margin && p.y() < g_rack.ymax + margin &&
            p.z() > g_rack.zmin - margin && p.z() < g_rack.zmax + margin);
}

// 胶囊体碰撞：线段 [start,end] 与点云，超过3点即碰撞
bool capsuleHitsCloud(const Vector3d& start, const Vector3d& end, double radius,
                      const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    Vector3d AB = end - start;
    double len_sq = AB.squaredNorm();
    if (len_sq < 1e-6) return false;
    int hit = 0;
    for (const auto& pt : cloud.points) {
        Vector3d P(pt.x, pt.y, pt.z);
        double t = (P - start).dot(AB) / len_sq;
        if (t < -0.05 || t > 1.05) continue;
        if ((P - (start + t * AB)).norm() < radius) {
            if (++hit > 3) return true;
        }
    }
    return false;
}

// 胶囊体碰撞：线段 [start,end] 与 AABB 架子
bool capsuleHitsRack(const Vector3d& start, const Vector3d& end, double radius) {
    int steps = max(1, (int)ceil((end - start).norm() / 0.03));
    for (int i = 0; i <= steps; ++i) {
        double t = (double)i / steps;
        if (isInRack(start + t * (end - start), radius)) return true;
    }
    return false;
}

bool checkDynamicCollision(const Vector3d& start, const Vector3d& end, double radius) {
    lock_guard<mutex> lk(obstacle_mutex);
    if (!g_has_obs || g_obstacle_cloud->empty()) return false;
    return capsuleHitsCloud(start, end, radius, *g_obstacle_cloud);
}

// 末端路径碰撞检测（规划阶段使用）
bool checkCollision(const Vector3d& start, const Vector3d& end, double radius) {
    if (capsuleHitsRack(start, end, radius)) {
        ROS_DEBUG("碰撞检测：命中架子");
        return true;
    }
    return checkDynamicCollision(start, end, radius);
}

// =================== 7. 连杆碰撞监控线程 ===================
//
// 每 50ms 读一次关节角 → 正向运动学 → 对每段连杆做胶囊碰撞检测
// 各段胶囊半径参考连杆实际宽度（保守估计）
static const double LINK_RADIUS[7] = {
    0.08,  // base → link1
    0.07,  // link1 → link2
    0.07,  // link2 → link3
    0.06,  // link3 → link4
    0.06,  // link4 → link5
    0.05,  // link5 → link6
    0.04,  // link6 → tool
};

void linkCollisionMonitorThread() {
    ros::Rate rate(20); // 20 Hz
    while (ros::ok()) {
        if (!g_monitor_active.load()) {
            rate.sleep();
            continue;
        }

        vector<double> q;
        {
            lock_guard<mutex> lk(joint_mutex);
            if (!g_joint_valid) { rate.sleep(); continue; }
            q = g_joint_positions;
        }

        auto pts = forwardKinematicsLinkPositions(q);

        // 检查每段连杆（跳过 base→link1 的固定段，从 link1 开始）
        // pts[0]=right_base, pts[1]=joint1_origin, ..., pts[7]=tool_tip
        bool hit = false;
        for (int seg = 1; seg < 7 && !hit; ++seg) {
            const Vector3d& A = pts[seg];
            const Vector3d& B = pts[seg + 1];
            double r = LINK_RADIUS[seg];

            // 对 AABB 架子检测（连杆穿进架子里很危险）
            if (capsuleHitsRack(A, B, r)) {
                ROS_WARN_THROTTLE(1.0, "连杆 %d 碰到架子！", seg + 1);
                hit = true;
            }

            // 对动态点云检测（仅末端 3 段，近端连杆离相机远且点云稀疏）
            if (!hit && seg >= 4) {
                lock_guard<mutex> lk(obstacle_mutex);
                if (g_has_obs && !g_obstacle_cloud->empty()) {
                    if (capsuleHitsCloud(A, B, r, *g_obstacle_cloud)) {
                        ROS_WARN_THROTTLE(1.0, "连杆 %d 碰到动态障碍！", seg + 1);
                        hit = true;
                    }
                }
            }
        }

        if (hit && !g_link_collision_detected.load()) {
            g_link_collision_detected.store(true);
            ROS_ERROR("连杆碰撞监控：检测到碰撞，已置停止标志");
        }

        rate.sleep();
    }
}

// =================== 8. 抓取策略生成 ===================

vector<double> calcPoseSmart(Vector3d approach_vec, const Matrix3d& home_rot) {
    Vector3d z_new = approach_vec.normalized();
    Vector3d x_home = home_rot.col(0);

    Vector3d y_new = z_new.cross(x_home);
    if (y_new.norm() < 0.01) y_new = z_new.cross(Vector3d(0, 1, 0));
    y_new.normalize();
    Vector3d x_new = y_new.cross(z_new).normalized();

    Matrix3d R;
    R.col(0) = x_new;
    R.col(1) = y_new;
    R.col(2) = z_new;

    Vector3d e = R.eulerAngles(2, 1, 0);
    return {e[2], e[1], e[0]};
}

vector<GraspCandidate> planGraspStrategies(const Vector3d& center_pos) {
    vector<GraspCandidate> candidates;
    double phi = M_PI * (3.0 - sqrt(5.0));

    Matrix3d home_rot;
    home_rot = AngleAxisd(HOME_RX, Vector3d::UnitZ())
             * AngleAxisd(HOME_RY, Vector3d::UnitY())
             * AngleAxisd(HOME_RZ, Vector3d::UnitX());
    Vector3d home_z = home_rot.col(2);

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        double yi = 1.0 - (i / float(NUM_SAMPLES - 1)) * 2.0;
        double r  = sqrt(1.0 - yi * yi);
        double theta = phi * i;
        double xi = cos(theta) * r;
        double zi = sin(theta) * r;

        Vector3d approach = -Vector3d(xi, zi, yi).normalized();

        double angle_diff = acos(max(-1.0, min(1.0, approach.dot(home_z))));
        if (angle_diff > M_PI / 2.2) continue;

        GraspCandidate c;
        c.id           = i;
        c.approach_vec = approach;
        c.rpy          = calcPoseSmart(approach, home_rot);
        c.score        = 100.0 - angle_diff * 40.0;
        candidates.push_back(c);
    }

    sort(candidates.begin(), candidates.end(),
         [](const GraspCandidate& a, const GraspCandidate& b) {
             return a.score > b.score;
         });

    return candidates;
}

// =================== 9. Service 回调 ===================

bool pickCallBack(task_assign::Target::Request& req, task_assign::Target::Response& resp) {
    if (!robot_ptr) {
        resp.success = false;
        resp.message = "Robot not initialized";
        return false;
    }

    // 重置碰撞标志
    g_link_collision_detected.store(false);
    g_monitor_active.store(true);

    error_code ec;

    Vector4d p_cam(req.x, req.y, req.z * depth_scale, 1.0);
    Vector3d target = (T_cam_to_base * p_cam).head<3>();
    ROS_INFO(">>> 目标 (基座系): [%.3f, %.3f, %.3f]", target.x(), target.y(), target.z());

    if (target.norm() > MAX_REACH) {
        ROS_ERROR("目标过远 (%.3fm > %.3fm)", target.norm(), MAX_REACH);
        resp.success = false;
        resp.message = "Target out of reach";
        g_monitor_active.store(false);
        return true;
    }

    if (isInRack(target, 0.0)) {
        ROS_ERROR("目标点在架子包围盒内，拒绝执行");
        resp.success = false;
        resp.message = "Target inside rack";
        g_monitor_active.store(false);
        return true;
    }

    auto strategies = planGraspStrategies(target);
    ROS_INFO("生成 %ld 个候选策略，开始尝试...", strategies.size());

    bool success = false;
    string id;
    const int MAX_ATTEMPTS = 20;

    for (int i = 0; i < min((int)strategies.size(), MAX_ATTEMPTS); ++i) {
        if (g_link_collision_detected.load()) {
            ROS_ERROR("连杆碰撞，中止采摘");
            break;
        }

        auto& task = strategies[i];
        Vector3d pre_pos = target - task.approach_vec * DIST_PRE;

        if (isInRack(pre_pos, COLLISION_RADIUS)) {
            if (i < 3) ROS_WARN("策略 #%d 预备点在架子内，跳过", i + 1);
            continue;
        }
        if (checkCollision(pre_pos, target, COLLISION_RADIUS)) {
            if (i < 3) ROS_WARN("策略 #%d 末端路径有碰撞，跳过", i + 1);
            continue;
        }

        ROS_INFO("执行策略 #%d (score=%.1f) approach=[%.2f, %.2f, %.2f]",
                 i + 1, task.score,
                 task.approach_vec.x(), task.approach_vec.y(), task.approach_vec.z());

        try {
            // (A) 打开夹爪
            pub_serial.publish(leftOpendata);
            waitGripper(GRIPPER_OPEN_WAIT);

            // (B) MoveJ 到预备点 
            id = genId("pre");
            MoveJCommand cmd_pre({pre_pos.x(), pre_pos.y(), pre_pos.z(),
                                  task.rpy[0], task.rpy[1], task.rpy[2]});
            cmd_pre.speed = TRANSIT_SPEED;
            robot_ptr->moveAppend({cmd_pre}, id, ec);
            robot_ptr->moveStart(ec);
            waitForFinish(*robot_ptr, id, 0);

            // (C) MoveL 进给
            id = genId("grasp");
            MoveLCommand cmd_grasp({target.x(), target.y(), target.z(),
                                    task.rpy[0], task.rpy[1], task.rpy[2]});
            cmd_grasp.speed = APPROACH_SPEED;
            robot_ptr->moveAppend({cmd_grasp}, id, ec);
            robot_ptr->moveStart(ec);
            waitForFinish(*robot_ptr, id, 0);

            // (D) 闭合夹爪
            pub_serial.publish(leftClosedata);
            waitGripper(GRIPPER_CLOSE_WAIT);

            // (E) MoveL 撤退
            id = genId("back");
            Vector3d ret_pos = target - task.approach_vec * DIST_RET;
            MoveLCommand cmd_back({ret_pos.x(), ret_pos.y(), ret_pos.z(),
                                   task.rpy[0], task.rpy[1], task.rpy[2]});
            cmd_back.speed = TRANSIT_SPEED;
            robot_ptr->moveAppend({cmd_back}, id, ec);
            robot_ptr->moveStart(ec);
            waitForFinish(*robot_ptr, id, 0);

            success = true;
            break;

        } catch (const exception& e) {
            ROS_WARN("策略 #%d 执行异常 (%s)，尝试下一个", i + 1, e.what());
            g_link_collision_detected.store(false); // 重置，允许继续尝试
        } catch (...) {
            ROS_WARN("策略 #%d 执行异常（未知），尝试下一个", i + 1);
            g_link_collision_detected.store(false);
        }
    }

    g_monitor_active.store(false);

    if (success) {
        // //这个点为则是用笛卡尔坐标，而不是关节坐标
        g_monitor_active.store(true);
        g_link_collision_detected.store(false);
        id = genId("drop");
        MoveJCommand cmd_drop({DROP_POSE[0], DROP_POSE[1], DROP_POSE[2],
                               DROP_POSE[3], DROP_POSE[4], DROP_POSE[5]});
        cmd_drop.speed = TRANSIT_SPEED;
        robot_ptr->moveAppend({cmd_drop}, id, ec);
        robot_ptr->moveStart(ec);
        try {
            waitForFinish(*robot_ptr, id, 0);
        } catch (...) {}
        g_monitor_active.store(false);

        // (G) 释放
        pub_serial.publish(leftOpendata);
        waitGripper(GRIPPER_OPEN_WAIT);

        resp.success = true;
        resp.message = "Pick Success";
        ROS_INFO("采摘完成！");
    } else {
        resp.success = false;
        resp.message = "All strategies blocked or failed";
        ROS_ERROR("所有策略均失败");
    }
    return true;
}

// =================== 10. Main ===================

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "robot_control_node");
    ros::NodeHandle nh;

    nh.param("dist_pre",           DIST_PRE,           0.15);
    nh.param("dist_ret",           DIST_RET,           0.10);
    nh.param("gripper_open_wait",  GRIPPER_OPEN_WAIT,  0.5);
    nh.param("gripper_close_wait", GRIPPER_CLOSE_WAIT, 0.6);
    nh.param("max_reach",          MAX_REACH,          1.5);
    nh.param("collision_radius",   COLLISION_RADIUS,   0.08);
    nh.param("approach_speed",     APPROACH_SPEED,     60.0);
    nh.param("transit_speed",      TRANSIT_SPEED,      200.0);
    nh.param("num_samples",        NUM_SAMPLES,        120);

    nh.param("rack_xmin", g_rack.xmin, -0.25);
    nh.param("rack_xmax", g_rack.xmax,  0.25);
    nh.param("rack_ymin", g_rack.ymin, -0.35);
    nh.param("rack_ymax", g_rack.ymax,  0.35);
    nh.param("rack_zmin", g_rack.zmin,  0.00);
    nh.param("rack_zmax", g_rack.zmax,  0.885);

    // 右臂夹爪串口指令
    pub_serial = nh.advertise<comm::serialData>("ap_robot/serial", 10);
    leftClosedata.data = {0xaa, 0x55, 0x02, 0x00, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};
    leftOpendata.data  = {0xaa, 0x55, 0x02, 0x01, 0x02, 0x58, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};

    ros::Subscriber sub_obs = nh.subscribe("ap_robot/obstacles", 1, obstacleCallback);
    ros::Subscriber sub_ack = nh.subscribe("ap_robot/serial_ack", 10, serialAckCallback);
    ros::Subscriber sub_js  = nh.subscribe("/joint_states", 10, jointStateCallback);

    // 加载右臂标定文件
    try {
        T_cam_to_base = loadMatrix("/home/y/dual_rokae_ws/src/rokae/src/camera_pose_right.txt");
        depth_scale   = loadFloat("/home/y/dual_rokae_ws/src/rokae/src/camera_depth_scale_right.txt");
        ROS_INFO("右臂标定参数加载成功, depth_scale=%.4f", depth_scale);
    } catch (const exception& e) {
        ROS_ERROR("标定文件加载失败: %s", e.what());
        return -1;
    }

    // 启动连杆碰撞监控线程
    thread monitor_thread(linkCollisionMonitorThread);
    monitor_thread.detach();

    // 连接右臂
    error_code ec;
    try {
        robot_ptr = new xMateRobot("192.168.2.160");
        robot_ptr->setOperateMode(OperateMode::automatic, ec);
        robot_ptr->setPowerState(true, ec);
        robot_ptr->setMotionControlMode(MotionControlMode::NrtCommand, ec);
        robot_ptr->setDefaultZone(0, ec);
        robot_ptr->setDefaultSpeed(100, ec);

        Toolset tool;
        tool.end = {{0, 0, 0.25}, {0, 0, 0}};
        tool.load.mass = 0.708993;
        robot_ptr->setToolset(tool, ec);

        ROS_INFO(">>> 右臂连接成功 | 斐波那契策略 + 连杆避障 + 串口ack <<<");

        ros::ServiceServer server = nh.advertiseService("right_pickTarget", pickCallBack);
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();

    } catch (const rokae::Exception& e) {
        ROS_ERROR("ROKAE Exception: %s", e.what());
    }

    if (robot_ptr) {
        robot_ptr->setPowerState(false, ec);
        robot_ptr->disconnectFromRobot(ec);
        delete robot_ptr;
    }
    return 0;
}
