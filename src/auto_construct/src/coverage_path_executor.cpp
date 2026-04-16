/**
 * coverage_path_executor.cpp
 *
 * 覆盖路径执行节点 (C++)
 * 从 YAML 文件加载覆盖路径，通过 Nav2 NavigateThroughPoses Action 完成覆盖作业。
 *
 * 支持的 YAML 格式 (你当前的格式):
 *   boundary: [...]
 *   pillars:  [...]
 *   path:
 *     frame_id: map          # "undefined" 自动回退到 frame_id 参数
 *     poses:
 *       - position:    {x, y, z}
 *         orientation: {x, y, z, w}
 *
 * 发布 Topic:
 *   ~/status   (std_msgs/String)   — IDLE/RUNNING/PAUSED/DONE/FAILED/CANCELLED
 *   ~/progress (std_msgs/Float32)  — 0.0 ~ 100.0
 *   ~/done     (std_msgs/Bool)     — true=成功, false=失败/取消
 *
 * 控制服务:
 *   ~/start    (std_srvs/Trigger)  — 开始
 *   ~/pause    (std_srvs/Trigger)  — 暂停 (保留进度)
 *   ~/resume   (std_srvs/Trigger)  — 继续
 *   ~/cancel   (std_srvs/Trigger)  — 取消
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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace fs = std::filesystem;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using NavGoalHandle        = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
using Trigger              = std_srvs::srv::Trigger;
using PoseStamped          = geometry_msgs::msg::PoseStamped;

// ─────────────────────────────────────────────────────────────────────────────
// 工具函数
// ─────────────────────────────────────────────────────────────────────────────
static std::string fmtTime(double seconds)
{
    int s = static_cast<int>(seconds);
    if (s < 60)   return std::to_string(s) + "s";
    if (s < 3600) return std::to_string(s / 60) + "m" + std::to_string(s % 60) + "s";
    return std::to_string(s / 3600) + "h" + std::to_string((s % 3600) / 60) + "m";
}

static std::string progressBar(double pct, int width = 25)
{
    int filled = static_cast<int>(width * pct / 100.0);
    std::string bar(filled, '#');
    bar += std::string(width - filled, '-');
    return "[" + bar + "]";
}

// ─────────────────────────────────────────────────────────────────────────────
// 节点类
// ─────────────────────────────────────────────────────────────────────────────
class CoveragePathExecutor : public rclcpp::Node
{
public:
    explicit CoveragePathExecutor(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
    : Node("coverage_path_executor", opts)
    {
        // ── 参数 ──────────────────────────────────────────────────────────────
        declare_parameter<std::string>("path_file",       "");
        declare_parameter<std::string>("frame_id",        "map");
        declare_parameter<bool>       ("skip_on_failure", true);
        declare_parameter<bool>       ("autostart",       false);

        path_file_        = get_parameter("path_file").as_string();
        frame_id_         = get_parameter("frame_id").as_string();
        skip_on_failure_  = get_parameter("skip_on_failure").as_bool();
        autostart_        = get_parameter("autostart").as_bool();

        // ── 回调组 (Service 与 Action 可并发) ──────────────────────────────
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // ── Action Client ─────────────────────────────────────────────────
        nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
            this, "navigate_through_poses", cb_group_);

        // ── 控制服务 ──────────────────────────────────────────────────────
        const auto qos = rmw_qos_profile_services_default;
        start_srv_  = create_service<Trigger>("~/start",
            [this](auto rq, auto rs){ svcStart(rq, rs);  }, qos, cb_group_);
        pause_srv_  = create_service<Trigger>("~/pause",
            [this](auto rq, auto rs){ svcPause(rq, rs);  }, qos, cb_group_);
        resume_srv_ = create_service<Trigger>("~/resume",
            [this](auto rq, auto rs){ svcResume(rq, rs); }, qos, cb_group_);
        cancel_srv_ = create_service<Trigger>("~/cancel",
            [this](auto rq, auto rs){ svcCancel(rq, rs); }, qos, cb_group_);

        // ── 状态发布 ──────────────────────────────────────────────────────
        pub_status_   = create_publisher<std_msgs::msg::String> ("~/status",   10);
        pub_progress_ = create_publisher<std_msgs::msg::Float32>("~/progress", 10);
        pub_done_     = create_publisher<std_msgs::msg::Bool>   ("~/done",     10);

        // ── 加载路径 ──────────────────────────────────────────────────────
        if (!path_file_.empty()) {
            loadPath(path_file_);
        }

        printBanner();

        if (autostart_ && !all_poses_.empty()) {
            RCLCPP_INFO(get_logger(), "autostart=true，3 秒后自动开始...");
            autostart_timer_ = create_wall_timer(
                std::chrono::seconds(3),
                [this]() {
                    autostart_timer_->cancel();
                    auto rq = std::make_shared<Trigger::Request>();
                    auto rs = std::make_shared<Trigger::Response>();
                    svcStart(rq, rs);
                });
        }
    }

    ~CoveragePathExecutor() override
    {
        cancelled_.store(true);
        if (exec_thread_.joinable()) {
            exec_thread_.join();
        }
    }

private:
    // ─────────────────────────────────────────────────────────────────────────
    // YAML 加载
    // ─────────────────────────────────────────────────────────────────────────
    bool loadPath(const std::string & path_file)
    {
        if (!fs::exists(path_file)) {
            RCLCPP_ERROR(get_logger(), "路径文件不存在: %s", path_file.c_str());
            return false;
        }

        try {
            YAML::Node root = YAML::LoadFile(path_file);

            // 定位 path 节（支持顶层 path 键或整个文件就是路径）
            YAML::Node sect = root["path"] ? root["path"] : root;

            if (!sect["poses"]) {
                RCLCPP_ERROR(get_logger(), "YAML 中未找到 'poses' 字段");
                return false;
            }

            // frame_id 处理：如果是 "undefined" 则回退到参数值
            if (sect["frame_id"]) {
                std::string yf = sect["frame_id"].as<std::string>();
                if (yf != "undefined" && !yf.empty()) {
                    frame_id_ = yf;
                } else {
                    RCLCPP_WARN(get_logger(),
                        "YAML frame_id='%s'，使用参数 frame_id='%s'",
                        yf.c_str(), frame_id_.c_str());
                }
            }

            // 解析 poses
            all_poses_.clear();
            for (const auto & p : sect["poses"]) {
                PoseStamped ps;
                ps.header.frame_id     = frame_id_;
                ps.pose.position.x     = p["position"]["x"].as<double>();
                ps.pose.position.y     = p["position"]["y"].as<double>();
                ps.pose.position.z     = p["position"]["z"].as<double>();
                ps.pose.orientation.x  = p["orientation"]["x"].as<double>();
                ps.pose.orientation.y  = p["orientation"]["y"].as<double>();
                ps.pose.orientation.z  = p["orientation"]["z"].as<double>();
                ps.pose.orientation.w  = p["orientation"]["w"].as<double>();
                all_poses_.push_back(ps);
            }

            total_ = all_poses_.size();
            RCLCPP_INFO(get_logger(),
                "路径加载成功: %zu 个路径点 (frame_id=%s)",
                total_, frame_id_.c_str());
            return true;

        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "路径解析异常: %s", e.what());
            return false;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 控制服务回调
    // ─────────────────────────────────────────────────────────────────────────
    void svcStart(const Trigger::Request::SharedPtr /*req*/,
                  Trigger::Response::SharedPtr       res)
    {
        if (running_.load()) {
            res->success = false;
            res->message = "任务已在执行中";
            return;
        }
        if (all_poses_.empty()) {
            res->success = false;
            res->message = "路径为空，请检查 path_file 参数";
            return;
        }
        resetState();
        exec_thread_ = std::thread(&CoveragePathExecutor::runExecution, this);
        exec_thread_.detach();
        res->success = true;
        res->message = "开始覆盖导航，共 " + std::to_string(total_) + " 个路径点";
    }

    void svcPause(const Trigger::Request::SharedPtr /*req*/,
                  Trigger::Response::SharedPtr       res)
    {
        if (!running_.load() || paused_.load()) {
            res->success = false;
            res->message = "当前未运行或已暂停";
            return;
        }
        paused_.store(true);
        cancelCurrentGoal();
        publishStatus("PAUSED");
        res->success = true;
        res->message = "已暂停，进度 " + std::to_string(resume_index_) +
                       "/" + std::to_string(total_);
    }

    void svcResume(const Trigger::Request::SharedPtr /*req*/,
                   Trigger::Response::SharedPtr       res)
    {
        if (!running_.load() || !paused_.load()) {
            res->success = false;
            res->message = "当前未暂停";
            return;
        }
        paused_.store(false);
        publishStatus("RUNNING");
        res->success = true;
        res->message = "继续执行，从第 " + std::to_string(resume_index_) + " 个路径点";
    }

    void svcCancel(const Trigger::Request::SharedPtr /*req*/,
                   Trigger::Response::SharedPtr       res)
    {
        cancelled_.store(true);
        paused_.store(false);
        cancelCurrentGoal();
        publishStatus("CANCELLED");
        res->success = true;
        res->message = "任务已取消";
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 执行主循环（独立线程）
    // ─────────────────────────────────────────────────────────────────────────
    void runExecution()
    {
        running_.store(true);
        start_time_ = std::chrono::steady_clock::now();
        publishStatus("RUNNING");

        RCLCPP_INFO(get_logger(), "等待 navigate_through_poses action server...");
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(15))) {
            RCLCPP_ERROR(get_logger(), "等待超时，请确认 Nav2 已启动并激活");
            finish(false);
            return;
        }
        RCLCPP_INFO(get_logger(), "Nav2 已就绪，开始覆盖导航");

        while (resume_index_ < total_ && !cancelled_.load()) {
            // 等待 resume
            while (paused_.load() && !cancelled_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (cancelled_.load()) break;

            // 构造剩余 poses 切片
            std::vector<PoseStamped> remaining(
                all_poses_.begin() + static_cast<long>(resume_index_),
                all_poses_.end());
            sent_count_     = remaining.size();
            last_remaining_ = static_cast<int>(sent_count_);

            const std::string result = sendAndWait(remaining);

            if (result == "SUCCEEDED") {
                resume_index_ = total_;
                break;
            }

            if (result == "PAUSED") {
                // 根据 feedback remaining 计算实际走到了哪里
                size_t done = sent_count_ -
                              static_cast<size_t>(std::max(0, last_remaining_));
                resume_index_ += done;
                RCLCPP_INFO(get_logger(),
                    "已暂停，当前进度 %zu/%zu，等待继续指令...",
                    resume_index_, total_);
                continue;   // 外层循环继续等待 paused_=false
            }

            if (result == "CANCELLED") {
                break;
            }

            // FAILED
            {
                size_t done      = sent_count_ -
                                   static_cast<size_t>(std::max(0, last_remaining_));
                size_t failed_at = resume_index_ + done;

                if (skip_on_failure_) {
                    RCLCPP_WARN(get_logger(),
                        "路径点 #%zu 导航失败，跳过继续执行...", failed_at);
                    resume_index_ = failed_at + 1;
                } else {
                    RCLCPP_ERROR(get_logger(),
                        "路径点 #%zu 导航失败，任务终止", failed_at);
                    finish(false);
                    return;
                }
            }
        }

        const bool success = (resume_index_ >= total_) && !cancelled_.load();
        finish(success);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 发送 Action 并同步等待结果
    // 返回: "SUCCEEDED" | "FAILED" | "PAUSED" | "CANCELLED"
    // ─────────────────────────────────────────────────────────────────────────
    std::string sendAndWait(const std::vector<PoseStamped> & poses)
    {
        // 使用 shared_ptr 确保 lambda 持有引用的生命周期与 future 一致
        auto sync_mutex  = std::make_shared<std::mutex>();
        auto sync_cv     = std::make_shared<std::condition_variable>();
        auto result_str  = std::make_shared<std::string>("FAILED");
        auto done_flag   = std::make_shared<bool>(false);

        NavigateThroughPoses::Goal goal;
        goal.poses = poses;

        auto send_opts = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions{};

        // ① 目标被接受/拒绝
        send_opts.goal_response_callback =
            [this, sync_mutex, sync_cv, result_str, done_flag]
            (const NavGoalHandle::SharedPtr & gh)
        {
            if (!gh) {
                RCLCPP_ERROR(get_logger(), "目标被 Nav2 拒绝 (server 未就绪?)");
                std::lock_guard<std::mutex> lk(*sync_mutex);
                *result_str = "FAILED";
                *done_flag  = true;
                sync_cv->notify_one();
                return;
            }
            std::lock_guard<std::mutex> lk(goal_mutex_);
            goal_handle_ = gh;
        };

        // ② feedback：更新 remaining，打印进度
        send_opts.feedback_callback =
            [this](NavGoalHandle::SharedPtr /*gh*/,
                   const NavigateThroughPoses::Feedback::ConstSharedPtr fb)
        {
            last_remaining_ = fb->number_of_poses_remaining;
            printProgress();
        };

        // ③ 结果
        send_opts.result_callback =
            [this, sync_mutex, sync_cv, result_str, done_flag]
            (const NavGoalHandle::WrappedResult & res)
        {
            std::lock_guard<std::mutex> lk(*sync_mutex);
            switch (res.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    *result_str = "SUCCEEDED";
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    // 区分"我们主动暂停"和"彻底取消"
                    *result_str = paused_.load() ? "PAUSED" : "CANCELLED";
                    break;
                default:
                    *result_str = "FAILED";
            }
            *done_flag = true;
            sync_cv->notify_one();
        };

        nav_client_->async_send_goal(goal, send_opts);

        // 阻塞等待结果，每 100ms 检查 pause/cancel
        std::unique_lock<std::mutex> ul(*sync_mutex);
        while (!sync_cv->wait_for(ul, std::chrono::milliseconds(100),
                                  [&done_flag] { return *done_flag; })) {
            if ((paused_.load() || cancelled_.load()) && !(*done_flag)) {
                ul.unlock();
                cancelCurrentGoal();
                ul.lock();
            }
        }

        return *result_str;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 取消当前 Goal
    // ─────────────────────────────────────────────────────────────────────────
    void cancelCurrentGoal()
    {
        NavGoalHandle::SharedPtr gh;
        {
            std::lock_guard<std::mutex> lk(goal_mutex_);
            gh          = goal_handle_;
            goal_handle_ = nullptr;   // 防止重复取消
        }
        if (gh) {
            nav_client_->async_cancel_goal(gh);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 进度显示（终端 + topic）
    // ─────────────────────────────────────────────────────────────────────────
    void printProgress()
    {
        if (total_ == 0) return;

        const size_t done = resume_index_ +
            (sent_count_ - static_cast<size_t>(std::max(0, last_remaining_)));
        const double pct  = std::min(100.0,
            static_cast<double>(done) / static_cast<double>(total_) * 100.0);

        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        std::string eta;
        if (pct > 0.5) {
            eta = " | ETA: " + fmtTime(elapsed / pct * (100.0 - pct));
        }

        printf("\r%s %5.1f%% | %zu/%zu pts | 耗时: %s%s   ",
               progressBar(pct).c_str(), pct,
               done, total_,
               fmtTime(elapsed).c_str(), eta.c_str());
        fflush(stdout);

        auto msg = std_msgs::msg::Float32{};
        msg.data = static_cast<float>(pct);
        pub_progress_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 任务结束
    // ─────────────────────────────────────────────────────────────────────────
    void finish(bool success)
    {
        printf("\n");
        running_.store(false);

        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (success) {
            RCLCPP_INFO(get_logger(),
                "覆盖导航完成! %zu 个路径点，耗时 %s",
                total_, fmtTime(elapsed).c_str());
            publishStatus("DONE");
        } else {
            RCLCPP_WARN(get_logger(),
                "覆盖导航结束 (失败/取消)，耗时 %s",
                fmtTime(elapsed).c_str());
            publishStatus("FAILED");
        }

        auto msg = std_msgs::msg::Bool{};
        msg.data = success;
        pub_done_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 工具
    // ─────────────────────────────────────────────────────────────────────────
    void resetState()
    {
        resume_index_   = 0;
        sent_count_     = 0;
        last_remaining_ = 0;
        paused_.store(false);
        cancelled_.store(false);
        running_.store(false);
        std::lock_guard<std::mutex> lk(goal_mutex_);
        goal_handle_ = nullptr;
    }

    void publishStatus(const std::string & s)
    {
        auto msg = std_msgs::msg::String{};
        msg.data = s;
        pub_status_->publish(msg);
        RCLCPP_INFO(get_logger(), "[状态] %s", s.c_str());
    }

    void printBanner()
    {
        const std::string sep(56, '-');
        RCLCPP_INFO(get_logger(), "%s", sep.c_str());
        RCLCPP_INFO(get_logger(), "  CoveragePathExecutor (C++) 已启动");
        RCLCPP_INFO(get_logger(), "  路径文件:   %s",
            path_file_.empty() ? "(未指定)" : path_file_.c_str());
        RCLCPP_INFO(get_logger(), "  总路径点:   %zu",  total_);
        RCLCPP_INFO(get_logger(), "  坐标系:     %s",   frame_id_.c_str());
        RCLCPP_INFO(get_logger(), "  跳过失败:   %s",   skip_on_failure_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  ~/start  ~/pause  ~/resume  ~/cancel");
        RCLCPP_INFO(get_logger(), "%s", sep.c_str());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 成员变量
    // ─────────────────────────────────────────────────────────────────────────
    // 参数
    std::string path_file_;
    std::string frame_id_;
    bool        skip_on_failure_{true};
    bool        autostart_{false};

    // ROS 接口
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
    rclcpp::Service<Trigger>::SharedPtr start_srv_, pause_srv_, resume_srv_, cancel_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_status_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_progress_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_done_;
    rclcpp::TimerBase::SharedPtr autostart_timer_;

    // 路径数据
    std::vector<PoseStamped> all_poses_;
    size_t total_{0};

    // 执行状态
    size_t             resume_index_{0};
    size_t             sent_count_{0};
    std::atomic<int>   last_remaining_{0};
    std::atomic<bool>  paused_{false};
    std::atomic<bool>  cancelled_{false};
    std::atomic<bool>  running_{false};

    NavGoalHandle::SharedPtr goal_handle_;
    std::mutex               goal_mutex_;

    std::thread exec_thread_;
    std::chrono::steady_clock::time_point start_time_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoveragePathExecutor>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
