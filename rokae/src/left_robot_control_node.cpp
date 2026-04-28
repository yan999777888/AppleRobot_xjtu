#include "ros/ros.h"
#include "ros/package.h"
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
#include <sstream>
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

// =================== 1. 运行时参数 ===================
double DIST_PRE         = 0.15;
double DIST_RET         = 0.10;
double GRIPPER_OPEN_WAIT  = 0.5;
double GRIPPER_CLOSE_WAIT = 0.6;
double MAX_REACH        = 1.5;
double COLLISION_RADIUS = 0.08;
double APPROACH_SPEED   = 150.0;
double TRANSIT_SPEED    = 300.0;
int    NUM_SAMPLES      = 120;

// 手眼标定补偿偏移（基座系下，单位 m）
// // 用于修正标定矩阵的系统误差，可通过 rosparam 微调
// double CALIB_OFFSET_X = 0.0;
// double CALIB_OFFSET_Y = -0.08;   // 左臂偏右 → Y 正方向补偿
// double CALIB_OFFSET_Z = 0.0;

// 左臂 Home 点姿态参考（采摘示教位 103°,-83°,78°）
// M_PI/180 = 0.017453
const double HOME_RX = -1.419545;   // 103 * π/180
const double HOME_RY =  0.89989;  // -83 * π/180
const double HOME_RZ = -1.341966;   // 78  * π/180

// 左臂放置点与过渡点，由 left_drop.txt 配置
array<double, 6> DROP_POSE = {-0.031550, 0.470405, 0.408085, 1.791179, 1.174973, -2.831744};
vector<array<double, 6>> DROP_TRANSITIONS = {
    {0.252950, 0.470405, 0.408115, 1.791162, 1.175043, -2.831587},
    {0.324507, 0.470405, 0.408083, 1.791389, 1.175060, -2.831360},
    {0.569419, 0.470405, 0.407999, 1.326346, 1.341111, 1.980059},
    {0.667165, 0.294619, 0.407985, 0.648809, 1.353381, 0.960455},
};

// =================== 2. 正向运动学（硬编码 DH）===================
//
// 左臂安装变换：parent=base_link, xyz="0.0 0.150 0.885", rpy="-1.5708 0 0"
// 与右臂的区别：Y 偏移为正（+0.150），绕 X 轴旋转 -90°（而非 +90°）
//
// 关节链 DH 偏移与右臂相同（同款机器人），安装变换不同即可区分。
// 各关节轴在自身坐标系下与右臂相同。

// 左臂安装变换矩阵（base_link -> left_base）
static Matrix4d buildLeftMountTransform() {
    // xyz=(0, +0.15, 0.885), rpy=(-1.5708, 0, 0)
    Matrix4d T = Matrix4d::Identity();
    double rx = -1.5708;
    Matrix3d R = AngleAxisd(rx, Vector3d::UnitX()).toRotationMatrix();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = Vector3d(0.0, 0.150, 0.885);
    return T;
}

// 给定6个关节角（rad），返回各关节原点在 base_link 坐标系下的位置（共8点）
static vector<Vector3d> forwardKinematicsLinkPositions(const vector<double>& q) {
    static const vector<Vector3d> joint_offset = {
        {0.0,   0.0,    0.0   },
        {0.0,   0.0,    0.355 },
        {0.05,  0.0,    0.4   },
        {-0.05, 0.0,    0.4   },
        {0.0,   0.136,  0.0   },
        {0.0,   0.0,    0.1035},
        {0.0,   0.0,    0.25  },
    };
    static const vector<Vector3d> joint_axis = {
        { 0,  0,  1},
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0,  1},
        { 0, -1,  0},
        { 0,  0,  1},
    };

    static const Matrix4d T_mount = buildLeftMountTransform();

    Matrix4d T = T_mount;
    vector<Vector3d> pts;
    pts.push_back(T.block<3,1>(0,3));

    for (int i = 0; i < 6; ++i) {
        Vector4d p_local(joint_offset[i].x(), joint_offset[i].y(), joint_offset[i].z(), 1.0);
        pts.push_back((T * p_local).head<3>());
        Matrix4d Tj = Matrix4d::Identity();
        Tj.block<3,3>(0,0) = AngleAxisd(q[i], joint_axis[i]).toRotationMatrix();
        T = T * Tj;
    }
    Vector4d p_local(joint_offset[6].x(), joint_offset[6].y(), joint_offset[6].z(), 1.0);
    pts.push_back((T * p_local).head<3>());

    return pts;
}

// =================== 3. 全局状态 ===================
xMateRobot* robot_ptr = nullptr;
ros::Publisher pub_serial;
comm::serialData leftClosedata, leftOpendata;
Eigen::Matrix4d T_cam_to_base;
float depth_scale = 1.0;
atomic<int> g_motion_id_counter(0);

mutex obstacle_mutex;
pcl::PointCloud<pcl::PointXYZ>::Ptr g_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool g_has_obs = false;

mutex ack_mutex;
bool g_gripper_ack = false;

mutex joint_mutex;
vector<double> g_joint_positions(6, 0.0);
bool g_joint_valid = false;
atomic<bool> g_link_collision_detected(false);
atomic<bool> g_monitor_active(false);

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
    // 左臂关节名：left_joint1 ~ left_joint6
    static const vector<string> joint_names = {
        "left_joint1","left_joint2","left_joint3",
        "left_joint4","left_joint5","left_joint6"
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
    return prefix + "_L_" + to_string(g_motion_id_counter++);
}

bool parsePoseLine(const string& line, array<double, 6>& pose) {
    auto lb = line.find('[');
    auto rb = line.find(']');
    if (lb == string::npos || rb == string::npos || rb <= lb) return false;

    string values = line.substr(lb + 1, rb - lb - 1);
    replace(values.begin(), values.end(), ',', ' ');
    stringstream ss(values);
    for (double& v : pose) {
        if (!(ss >> v)) return false;
    }
    return true;
}

bool loadDropTrajectory(const string& path) {
    ifstream f(path);
    if (!f.is_open()) return false;

    vector<array<double, 6>> poses;
    string line;
    while (getline(f, line)) {
        if (line.find("position:") == string::npos) continue;
        array<double, 6> pose{};
        if (parsePoseLine(line, pose)) {
            poses.push_back(pose);
        }
    }
    if (poses.empty()) return false;

    DROP_POSE = poses.front();
    DROP_TRANSITIONS.assign(poses.begin() + 1, poses.end());
    return true;
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
                ROS_ERROR_STREAM("左臂运动错误 " << _id << ": " << _ec.message());
                return;
            }
            if (_id == traj_id && _idx == index) {
                if (any_cast<bool>(info.at(EventInfoKey::MoveExecution::ReachTarget))) return;
            }
        }
        if (g_link_collision_detected.load()) {
            ROS_ERROR("左臂连杆碰撞监控触发！紧急停止");
            robot.stop(ec);
            throw runtime_error("Left arm link collision detected");
        }
        this_thread::sleep_for(chrono::milliseconds(50));
    }
    throw runtime_error("Motion Timeout: " + traj_id);
}

void waitGripper(double timeout_sec) {
    { lock_guard<mutex> lk(ack_mutex); g_gripper_ack = false; }
    ros::Time start = ros::Time::now();
    ros::Rate r(50);
    while (ros::ok()) {
        { lock_guard<mutex> lk(ack_mutex); if (g_gripper_ack) return; }
        if ((ros::Time::now() - start).toSec() >= timeout_sec) return;
        r.sleep();
    }
}

// =================== 6. 碰撞检测 ===================

// --- 6a. 苹果架子 AABB ---
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

// --- 6b. 连接座碰撞体 ---
// 连接座建模为竖直圆柱：圆心在 (0, 0)，高度 0~0.885m，半径约 0.15m
// 左臂安装在 y=+0.15 顶端，运动时连杆不能穿入此圆柱
struct MountCylinder {
    double cx     =  0.0;    // 中心 X
    double cy     =  0.0;    // 中心 Y
    double radius =  0.20;   // 圆柱半径（含安全裕度）
    double zmin   =  0.0;    // 底部
    double zmax   =  0.885;  // 顶部
} g_mount;

// 点到竖直圆柱的距离检测
bool isInMountCylinder(const Vector3d& p, double margin) {
    if (p.z() < g_mount.zmin - margin || p.z() > g_mount.zmax + margin) return false;
    double dx = p.x() - g_mount.cx;
    double dy = p.y() - g_mount.cy;
    return (dx*dx + dy*dy) < (g_mount.radius + margin) * (g_mount.radius + margin);
}

// --- 6c. 通用碰撞函数 ---

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
        if ((P - (start + t * AB)).norm() < radius)
            if (++hit > 3) return true;
    }
    return false;
}

bool capsuleHitsRack(const Vector3d& start, const Vector3d& end, double radius) {
    int steps = max(1, (int)ceil((end - start).norm() / 0.03));
    for (int i = 0; i <= steps; ++i) {
        double t = (double)i / steps;
        if (isInRack(start + t * (end - start), radius)) return true;
    }
    return false;
}

// 线段与连接座圆柱的碰撞检测
bool capsuleHitsMountCylinder(const Vector3d& start, const Vector3d& end, double radius) {
    int steps = max(1, (int)ceil((end - start).norm() / 0.03));
    for (int i = 0; i <= steps; ++i) {
        double t = (double)i / steps;
        if (isInMountCylinder(start + t * (end - start), radius)) return true;
    }
    return false;
}

bool checkCollision(const Vector3d& start, const Vector3d& end, double radius) {
    if (capsuleHitsRack(start, end, radius)) return true;
    if (capsuleHitsMountCylinder(start, end, radius)) return true;
    lock_guard<mutex> lk(obstacle_mutex);
    if (!g_has_obs || g_obstacle_cloud->empty()) return false;
    return capsuleHitsCloud(start, end, radius, *g_obstacle_cloud);
}

// =================== 7. 连杆碰撞监控线程 ===================
//
// 左臂安装在 y=+0.150 处，绕 X 轴转 -90°。
// 相比右臂（y=-0.150, +90°），左臂的关节在 base_link 系下 Y 方向对称，
// 但 buildLeftMountTransform() 已处理此差异，FK 结果自动正确。

static const double LINK_RADIUS[7] = {
    0.08, 0.07, 0.07, 0.06, 0.06, 0.06, 0.07,
};

void linkCollisionMonitorThread() {
    ros::Rate rate(20);
    while (ros::ok()) {
        if (!g_monitor_active.load()) { rate.sleep(); continue; }

        vector<double> q;
        {
            lock_guard<mutex> lk(joint_mutex);
            if (!g_joint_valid) { rate.sleep(); continue; }
            q = g_joint_positions;
        }

        auto pts = forwardKinematicsLinkPositions(q);

        bool hit = false;
        for (int seg = 1; seg < 7 && !hit; ++seg) {
            const Vector3d& A = pts[seg];
            const Vector3d& B = pts[seg + 1];
            double r = LINK_RADIUS[seg];

            if (capsuleHitsRack(A, B, r)) {
                ROS_WARN_THROTTLE(1.0, "左臂连杆 %d 碰到架子！", seg + 1);
                hit = true;
            }
            if (!hit && capsuleHitsMountCylinder(A, B, r)) {
                ROS_WARN_THROTTLE(1.0, "左臂连杆 %d 碰到连接座！", seg + 1);
                hit = true;
            }
            if (!hit && seg >= 4) {
                lock_guard<mutex> lk(obstacle_mutex);
                if (g_has_obs && !g_obstacle_cloud->empty())
                    if (capsuleHitsCloud(A, B, r, *g_obstacle_cloud)) {
                        ROS_WARN_THROTTLE(1.0, "左臂连杆 %d 碰到动态障碍！", seg + 1);
                        hit = true;
                    }
            }
        }

        if (hit && !g_link_collision_detected.load()) {
            g_link_collision_detected.store(true);
            ROS_ERROR("左臂连杆碰撞监控：检测到碰撞，已置停止标志");
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
    Matrix3d R; R.col(0) = x_new; R.col(1) = y_new; R.col(2) = z_new;
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
        Vector3d approach = -Vector3d(cos(theta)*r, sin(theta)*r, yi).normalized();

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
         [](const GraspCandidate& a, const GraspCandidate& b) { return a.score > b.score; });
    return candidates;
}

// =================== 9. Service 回调 ===================

bool pickCallBack(task_assign::Target::Request& req, task_assign::Target::Response& resp) {
    if (!robot_ptr) { resp.success = false; resp.message = "Robot not initialized"; return false; }

    g_link_collision_detected.store(false);
    g_monitor_active.store(true);

    error_code ec;

    Vector4d p_cam(req.x, req.y, req.z * depth_scale, 1.0);
    Vector3d target = (T_cam_to_base * p_cam).head<3>();
    // target.x() += CALIB_OFFSET_X;
    // target.y() += CALIB_OFFSET_Y;

    // target.z() += CALIB_OFFSET_Z;
    ROS_INFO("左臂 >>> 目标 (基座系+补偿): [%.3f, %.3f, %.3f]", target.x(), target.y(), target.z());

    if (target.norm() > MAX_REACH) {
        ROS_ERROR("左臂目标过远 (%.3fm)", target.norm());
        resp.success = false; resp.message = "Target out of reach";
        g_monitor_active.store(false); return true;
    }
    if (isInRack(target, 0.0)) {
        ROS_ERROR("左臂目标点在架子包围盒内，拒绝执行");
        resp.success = false; resp.message = "Target inside rack";
        g_monitor_active.store(false); return true;
    }

    auto strategies = planGraspStrategies(target);
    ROS_INFO("左臂生成 %ld 个候选策略", strategies.size());

    bool success = false;
    string id;
    const int MAX_ATTEMPTS = 20;

    for (int i = 0; i < min((int)strategies.size(), MAX_ATTEMPTS); ++i) {
        if (g_link_collision_detected.load()) { ROS_ERROR("左臂连杆碰撞，中止"); break; }

        auto& task = strategies[i];
        Vector3d pre_pos = target - task.approach_vec * DIST_PRE;

        if (isInRack(pre_pos, COLLISION_RADIUS)) {
            if (i < 3) ROS_WARN("左臂策略 #%d 预备点在架子内，跳过", i+1);
            continue;
        }
        if (checkCollision(pre_pos, target, COLLISION_RADIUS)) {
            if (i < 3) ROS_WARN("左臂策略 #%d 路径碰撞，跳过", i+1);
            continue;
        }

        ROS_INFO("左臂执行策略 #%d (score=%.1f)", i+1, task.score);

        try {
            pub_serial.publish(leftOpendata);
            waitGripper(GRIPPER_OPEN_WAIT);

            id = genId("pre");
            MoveJCommand cmd_pre({pre_pos.x(), pre_pos.y(), pre_pos.z(),
                                  task.rpy[0], task.rpy[1], task.rpy[2]});
            cmd_pre.speed = TRANSIT_SPEED;
            robot_ptr->moveAppend({cmd_pre}, id, ec);
            robot_ptr->moveStart(ec);
            waitForFinish(*robot_ptr, id, 0);

            id = genId("grasp");
            MoveLCommand cmd_grasp({target.x(), target.y(), target.z(),
                                    task.rpy[0], task.rpy[1], task.rpy[2]});
            cmd_grasp.speed = APPROACH_SPEED;
            robot_ptr->moveAppend({cmd_grasp}, id, ec);
            robot_ptr->moveStart(ec);
            waitForFinish(*robot_ptr, id, 0);

            pub_serial.publish(leftClosedata);
            waitGripper(GRIPPER_CLOSE_WAIT);

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
            ROS_WARN("左臂策略 #%d 异常 (%s)，尝试下一个", i+1, e.what());
            g_link_collision_detected.store(false);
        } catch (...) {
            ROS_WARN("左臂策略 #%d 未知异常", i+1);
            g_link_collision_detected.store(false);
        }
    }

    g_monitor_active.store(false);

    if (success) {
        g_monitor_active.store(true);
        g_link_collision_detected.store(false);
        auto moveJointPose = [&](const array<double, 6>& pose, const string& prefix) {
            id = genId(prefix);
            MoveJCommand cmd({pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]});
            cmd.speed = TRANSIT_SPEED;
            robot_ptr->moveAppend({cmd}, id, ec);
            robot_ptr->moveStart(ec);
            try {
                waitForFinish(*robot_ptr, id, 0);
                return true;
            } catch (const exception& e) {
                ROS_WARN("左臂放置运动 %s 失败: %s", prefix.c_str(), e.what());
                return false;
            }
        };

        bool drop_ok = true;
        for (auto it = DROP_TRANSITIONS.rbegin(); it != DROP_TRANSITIONS.rend(); ++it) {
            if (!moveJointPose(*it, "drop_in")) { drop_ok = false; break; }
        }
        if (drop_ok) drop_ok = moveJointPose(DROP_POSE, "drop");
        g_monitor_active.store(false);

        if (drop_ok) {
            pub_serial.publish(leftOpendata);
            ros::Duration(1.0).sleep();
            waitGripper(GRIPPER_OPEN_WAIT);

            g_monitor_active.store(true);
            for (const auto& pose : DROP_TRANSITIONS) {
                if (!moveJointPose(pose, "drop_out")) { drop_ok = false; break; }
            }
            g_monitor_active.store(false);
        }

        resp.success = drop_ok;
        resp.message = drop_ok ? "Pick Success" : "Drop motion failed";
        if (drop_ok) ROS_INFO("左臂采摘完成！");
    } else {
        resp.success = false; resp.message = "All strategies blocked or failed";
        ROS_ERROR("左臂所有策略均失败");
    }
    return true;
}

// =================== 10. Main ===================

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "left_robot_control_node");
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

    // nh.param("calib_offset_x",     CALIB_OFFSET_X,     0.0);
    // nh.param("calib_offset_y",     CALIB_OFFSET_Y,     -0.08);
    // nh.param("calib_offset_z",     CALIB_OFFSET_Z,     0.0);

    nh.param("rack_xmin", g_rack.xmin, -0.25);
    nh.param("rack_xmax", g_rack.xmax,  0.25);
    nh.param("rack_ymin", g_rack.ymin, -0.35);
    nh.param("rack_ymax", g_rack.ymax,  0.35);
    nh.param("rack_zmin", g_rack.zmin,  0.00);
    nh.param("rack_zmax", g_rack.zmax,  0.885);

    const string drop_path = ros::package::getPath("rokae") + "/src/left_drop.txt";
    if (loadDropTrajectory(drop_path)) {
        ROS_INFO("左臂放置/过渡点已从 %s 加载", drop_path.c_str());
    } else {
        ROS_WARN("左臂放置/过渡点加载失败，继续使用默认值");
    }

    // 左臂夹爪串口指令（与右臂字节不同，来自 leftRokaegodemo.cpp）
    pub_serial = nh.advertise<comm::serialData>("ap_robot/serial", 10);
    leftClosedata.data = {0xaa, 0x55, 0x02, 0x01, 0x08, 0x34, 0x4e, 0x05, 0xd8, 0xf0, 0xcd, 0x01, 0x02, 0x03, 0x0d, 0x0a};
    leftOpendata.data  = {0xaa, 0x55, 0x02, 0x00, 0x08, 0x34, 0x50, 0x05, 0x27, 0x10, 0x3c, 0x01, 0x02, 0x03, 0x0d, 0x0a};

    ros::Subscriber sub_obs = nh.subscribe("ap_robot/obstacles", 1, obstacleCallback);
    ros::Subscriber sub_ack = nh.subscribe("ap_robot/serial_ack", 10, serialAckCallback);
    ros::Subscriber sub_js  = nh.subscribe("/joint_states", 10, jointStateCallback);

    // 加载左臂标定文件
    try {
        T_cam_to_base = loadMatrix("/home/y/dual_rokae_ws/src/rokae/src/camera_pose_left.txt");
        depth_scale   = loadFloat("/home/y/dual_rokae_ws/src/rokae/src/camera_depth_scale_left.txt");
        ROS_INFO("左臂标定参数加载成功, depth_scale=%.4f", depth_scale);
    } catch (const exception& e) {
        ROS_ERROR("标定文件加载失败: %s", e.what());
        return -1;
    }

    // 启动连杆碰撞监控线程
    thread monitor_thread(linkCollisionMonitorThread);
    monitor_thread.detach();

    // 连接左臂
    error_code ec;
    try {
        robot_ptr = new xMateRobot("192.168.2.161");
        robot_ptr->setOperateMode(OperateMode::automatic, ec);
        robot_ptr->setPowerState(true, ec);
        robot_ptr->setMotionControlMode(MotionControlMode::NrtCommand, ec);
        robot_ptr->setDefaultZone(0, ec);
        robot_ptr->setDefaultSpeed(100, ec);

        Toolset tool;
        tool.end = {{0, 0, 0.25}, {0, 0, 0}};
        tool.load.mass = 0.708993;
        robot_ptr->setToolset(tool, ec);

        ROS_INFO(">>> 左臂连接成功 | 斐波那契策略 + 连杆避障 <<<");

        ros::ServiceServer server = nh.advertiseService("left_pickTarget", pickCallBack);
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();

    } catch (const rokae::Exception& e) {
        ROS_ERROR("左臂 ROKAE Exception: %s", e.what());
    }

    if (robot_ptr) {
        robot_ptr->setPowerState(false, ec);
        robot_ptr->disconnectFromRobot(ec);
        delete robot_ptr;
    }
    return 0;
}
