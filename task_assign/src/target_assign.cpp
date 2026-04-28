#include "ros/ros.h"
#include "task_assign/Target.h"
#include "img_detect/Apple.h"
#include "std_msgs/Bool.h"
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

using namespace std;

// =================== 全局状态 ===================

struct AppleInfo {
    ros::Time stamp;
    uint32_t  id;
    float x, y, z, size;
};

// 最新一帧苹果数据，用互斥锁保护
mutex         g_apple_mutex;
vector<AppleInfo> g_pending;      // 待处理的苹果列表
bool          g_new_data = false; // 是否有新消息

// 底盘抓取允许状态：true = 允许抓取，false = 移动锁定
atomic<bool>  g_base_pick_enabled(false);

// 各臂是否正在采摘（防止重复派遣）
atomic<bool>  g_left_busy(false);
atomic<bool>  g_right_busy(false);

mutex         g_cooldown_mutex;
chrono::steady_clock::time_point g_left_cooldown_until;
chrono::steady_clock::time_point g_right_cooldown_until;
double        g_cooldown_sec = 3.0;

// ROS service clients（在 main 中初始化，线程中使用）
ros::ServiceClient g_left_client;
ros::ServiceClient g_right_client;

// =================== 话题回调 ===================

void appleCallback(const img_detect::Apple::ConstPtr& msg) {
    vector<AppleInfo> apples;
    for (size_t i = 0; i < msg->id.size(); i++) {
        AppleInfo a;
        a.stamp = msg->header.stamp;
        a.id    = msg->id[i];
        a.x     = msg->x[i];
        a.y     = msg->y[i];
        a.z     = msg->z[i];
        a.size  = msg->size[i];
        apples.push_back(a);
    }

    lock_guard<mutex> lk(g_apple_mutex);
    g_pending  = apples;
    g_new_data = true;
}

void basePickEnableCallback(const std_msgs::Bool::ConstPtr& msg) {
    g_base_pick_enabled.store(msg->data);
    ROS_INFO("底盘抓取允许状态: %s", msg->data ? "允许" : "禁止");
}

bool baseReadyForPick() {
    if (!g_base_pick_enabled.load()) {
        ROS_WARN_THROTTLE(1.0, "底盘移动锁定中，暂停派单抓取");
        return false;
    }
    return true;
}

// =================== 采摘线程函数 ===================

// 按 z（深度）从近到远排序，优先抓最近的
void sortByDepth(vector<AppleInfo>& v) {
    sort(v.begin(), v.end(),
         [](const AppleInfo& a, const AppleInfo& b) { return a.z < b.z; });
}

void leftPickThread(AppleInfo apple) {
    task_assign::Target srv;
    srv.request.header.stamp = apple.stamp;
    srv.request.x = apple.x;
    srv.request.y = apple.y;
    srv.request.z = apple.z;

    ROS_INFO("左臂发送目标: x=%.3f y=%.3f z=%.3f", apple.x, apple.y, apple.z);
    if (g_left_client.call(srv))
        ROS_INFO("左臂完成, status=%u", srv.response.status);
    else
        ROS_ERROR("左臂服务调用失败");

    {
        lock_guard<mutex> lk(g_cooldown_mutex);
        g_left_cooldown_until = chrono::steady_clock::now() + chrono::milliseconds((int)(g_cooldown_sec * 1000));
    }
    g_left_busy = false;
}

void rightPickThread(AppleInfo apple) {
    task_assign::Target srv;
    srv.request.header.stamp = apple.stamp;
    srv.request.x = apple.x;
    srv.request.y = apple.y;
    srv.request.z = apple.z;

    ROS_INFO("右臂发送目标: x=%.3f y=%.3f z=%.3f", apple.x, apple.y, apple.z);
    if (g_right_client.call(srv))
        ROS_INFO("右臂完成, status=%u", srv.response.status);
    else
        ROS_ERROR("右臂服务调用失败");

    {
        lock_guard<mutex> lk(g_cooldown_mutex);
        g_right_cooldown_until = chrono::steady_clock::now() + chrono::milliseconds((int)(g_cooldown_sec * 1000));
    }
    g_right_busy = false;
}

// =================== Main ===================

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "task_assign");
    ros::NodeHandle nh;

    nh.param("pick_cooldown_sec", g_cooldown_sec, 3.0);
    ROS_INFO("采摘冷却时间: %.1f 秒", g_cooldown_sec);

    // 等待两臂服务上线再订阅，避免派遣到未就绪的臂
    ROS_INFO("等待左右臂服务...");
    ros::service::waitForService("left_pickTarget");
    ros::service::waitForService("right_pickTarget");
    g_left_client  = nh.serviceClient<task_assign::Target>("left_pickTarget",  true); // persistent
    g_right_client = nh.serviceClient<task_assign::Target>("right_pickTarget", true);
    ROS_INFO("双臂服务就绪，开始任务分配");

    ros::Subscriber sub = nh.subscribe("ap_robot/chatter_appleInfo", 1, appleCallback);
    ros::Subscriber sub_base_pick = nh.subscribe("/ap_robot/base_pick_enabled", 1, basePickEnableCallback);

    ros::Rate rate(10); // 10Hz 轮询

    while (ros::ok()) {
        ros::spinOnce();

        if (!baseReadyForPick()) {
            rate.sleep();
            continue;
        }

        // 取出最新数据
        vector<AppleInfo> apples;
        {
            lock_guard<mutex> lk(g_apple_mutex);
            if (!g_new_data) { rate.sleep(); continue; }
            apples     = g_pending;
            g_new_data = false;
        }

        if (apples.empty()) { rate.sleep(); continue; }

        // 按 x 坐标分区：x < 0 归左臂，x >= 0 归右臂
        vector<AppleInfo> left_apples, right_apples;
        for (auto& a : apples) {
            if (a.x < 0) left_apples.push_back(a);
            else          right_apples.push_back(a);
        }
        sortByDepth(left_apples);
        sortByDepth(right_apples);

        // 左臂空闲且有任务且冷却结束 → 派遣
        if (!g_left_busy && !left_apples.empty()) {
            bool cooled = false;
            {
                lock_guard<mutex> lk(g_cooldown_mutex);
                cooled = chrono::steady_clock::now() >= g_left_cooldown_until;
            }
            if (cooled) {
                g_left_busy = true;
                ROS_INFO("派遣左臂，左区苹果数: %ld", left_apples.size());
                thread(leftPickThread, left_apples.front()).detach();
            }
        }

        // 右臂空闲且有任务且冷却结束 → 派遣
        if (!g_right_busy && !right_apples.empty()) {
            bool cooled = false;
            {
                lock_guard<mutex> lk(g_cooldown_mutex);
                cooled = chrono::steady_clock::now() >= g_right_cooldown_until;
            }
            if (cooled) {
                g_right_busy = true;
                ROS_INFO("派遣右臂，右区苹果数: %ld", right_apples.size());
                thread(rightPickThread, right_apples.front()).detach();
            }
        }

        rate.sleep();
    }

    return 0;
}
