#pragma once

/**
 * coverage_path_executor.hpp
 *
 * 覆盖路径执行节点 — 头文件
 *
 * TF 帧名对应关系 (你的系统):
 *   FASTLIO2:    world_frame=lidar, body_frame=body
 *   Localizer:   map_frame=map,     local_frame=lidar
 *   实际TF链:    map → lidar → body
 *   Nav2参数:    global_frame=map, robot_base_frame=body,
 *                local_costmap.global_frame=lidar
 *
 * 传感器 Topic:
 *   点云:   /fastlio2/body_cloud  (PointCloud2, body frame)
 *   里程计: /fastlio2/lio_odom    (Odometry)
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <auto_construct/srv/set_path_and_start.hpp>

namespace auto_construct
{

// ── 型别别名 ──────────────────────────────────────────────────────────────────
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using NavGoalHandle        = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
using Trigger              = std_srvs::srv::Trigger;
using PoseStamped          = geometry_msgs::msg::PoseStamped;
using SetPathAndStart      = auto_construct::srv::SetPathAndStart;
using LoadMap              = nav2_msgs::srv::LoadMap;

// ─────────────────────────────────────────────────────────────────────────────
// CoveragePathExecutor
// ─────────────────────────────────────────────────────────────────────────────
class CoveragePathExecutor : public rclcpp::Node
{
public:
    explicit CoveragePathExecutor(
        const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{});

    ~CoveragePathExecutor() override;

private:
    // ── 初始化 ────────────────────────────────────────────────────────────────
    void declareAndGetParams();
    void setupInterfaces();
    void printBanner() const;

    // ── YAML 加载 ─────────────────────────────────────────────────────────────
    /**
     * 从文件加载覆盖路径。
     * 支持格式:
     *   path:
     *     frame_id: map          # "undefined" → 回退到 frame_id 参数
     *     poses:
     *       - position:    {x, y, z}
     *         orientation: {x, y, z, w}
     * @return true  加载成功
     * @return false 文件不存在 / 格式错误
     */
    bool loadPath(const std::string & path_file);

    /**
     * 通过 /map_server/load_map 热切换 Nav2 地图。
     * @return true  切换成功
     * @return false 服务不可用 / 文件不存在 / 超时
     */
    bool reloadMap(const std::string & map_file);

    /**
     * 扫描目录，按内容自动识别地图文件和路径文件。
     *   地图文件 — 含 image: 字段 (Nav2 map_server 格式)
     *   路径文件 — 含 poses: 字段 (opennav_coverage 格式)
     * @param[out] map_file   识别到的地图文件完整路径
     * @param[out] path_file  识别到的路径文件完整路径
     * @param[out] error_msg  失败原因
     * @return true  两个文件均找到
     * @return false 目录不存在 / 找不到其中一个文件
     */
    bool discoverFiles(const std::string & map_dir,
                       std::string       & map_file,
                       std::string       & path_file,
                       std::string       & error_msg);

    // ── 控制服务回调 ──────────────────────────────────────────────────────────
    void svcSetPathAndStart(const SetPathAndStart::Request::SharedPtr req,
                            SetPathAndStart::Response::SharedPtr       res);
    void svcStart (const Trigger::Request::SharedPtr req,
                   Trigger::Response::SharedPtr       res);
    void svcPause (const Trigger::Request::SharedPtr req,
                   Trigger::Response::SharedPtr       res);
    void svcResume(const Trigger::Request::SharedPtr req,
                   Trigger::Response::SharedPtr       res);
    void svcCancel(const Trigger::Request::SharedPtr req,
                   Trigger::Response::SharedPtr       res);

    // ── 执行主循环 ────────────────────────────────────────────────────────────
    /** 在独立线程中运行；由 svcStart() / svcSetPathAndStart() 启动。 */
    void runExecution();

    /**
     * 发送 NavigateThroughPoses 并同步阻塞等待结果。
     * @return "SUCCEEDED" | "PAUSED" | "CANCELLED" | "FAILED"
     */
    std::string sendAndWait(const std::vector<PoseStamped> & poses);

    /** 取消当前正在执行的 Nav2 goal（线程安全）。 */
    void cancelCurrentGoal();

    // ── 状态与进度 ────────────────────────────────────────────────────────────
    void printProgress();
    void finish(bool success);
    void resetState();
    void publishStatus(const std::string & status);

    // ── 工具函数（静态） ──────────────────────────────────────────────────────
    static std::string fmtTime(double seconds);
    static std::string progressBar(double pct, int width = 25);

    // ─────────────────────────────────────────────────────────────────────────
    // 参数
    // ─────────────────────────────────────────────────────────────────────────
    std::string path_file_;
    std::string frame_id_;
    bool        skip_on_failure_{true};
    bool        autostart_{false};
    bool        map_loaded_{false};   ///< 是否已通过 load_map 加载过地图

    // ─────────────────────────────────────────────────────────────────────────
    // ROS 接口
    // ─────────────────────────────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
    rclcpp::Client<LoadMap>::SharedPtr                     map_client_;

    rclcpp::Service<SetPathAndStart>::SharedPtr set_path_srv_;
    rclcpp::Service<Trigger>::SharedPtr         start_srv_;
    rclcpp::Service<Trigger>::SharedPtr         pause_srv_;
    rclcpp::Service<Trigger>::SharedPtr         resume_srv_;
    rclcpp::Service<Trigger>::SharedPtr         cancel_srv_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_status_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_progress_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_done_;

    rclcpp::TimerBase::SharedPtr autostart_timer_;

    // ─────────────────────────────────────────────────────────────────────────
    // 路径数据
    // ─────────────────────────────────────────────────────────────────────────
    std::vector<PoseStamped> all_poses_;
    std::size_t              total_{0};

    // ─────────────────────────────────────────────────────────────────────────
    // 执行状态（线程间共享）
    // ─────────────────────────────────────────────────────────────────────────
    std::size_t          resume_index_{0};   ///< 下次从哪个 index 开始发送
    std::size_t          sent_count_{0};     ///< 本次 action 发出的 pose 数量
    std::atomic<int>     last_remaining_{0}; ///< 最新 feedback 中的 remaining

    std::atomic<bool>    paused_{false};
    std::atomic<bool>    cancelled_{false};
    std::atomic<bool>    running_{false};

    NavGoalHandle::SharedPtr goal_handle_;
    std::mutex               goal_mutex_;

    std::thread exec_thread_;
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace auto_construct
