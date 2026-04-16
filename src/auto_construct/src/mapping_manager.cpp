#include "auto_construct/mapping_manager.hpp"

#include <cerrno>
#include <cstring>
#include <signal.h>
#include <sys/wait.h>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

MappingManager::MappingManager() : Node("mapping_manager")
{
  // Reentrant 回调组：允许 finish_mapping 在等待 PGO future 时
  // executor 仍能调度其他回调（含 client 的响应回调），避免死锁
  cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  srv_start_mapping_ = this->create_service<std_srvs::srv::Trigger>(
    "/sys/start_mapping",
    std::bind(&MappingManager::handle_start_mapping, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_finish_mapping_ = this->create_service<std_srvs::srv::Trigger>(
    "/sys/finish_mapping",
    std::bind(&MappingManager::handle_finish_mapping, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_start_navigation_ = this->create_service<std_srvs::srv::Trigger>(
    "/sys/start_navigation",
    std::bind(&MappingManager::handle_start_navigation, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_stop_all_ = this->create_service<std_srvs::srv::Trigger>(
    "/sys/stop_all",
    std::bind(&MappingManager::handle_stop_all, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);



  // client 同样挂在 Reentrant 回调组，响应回调才能被 executor 调度
  client_save_map_ = this->create_client<std_srvs::srv::Trigger>(
    "/save_map",
    rmw_qos_profile_services_default,
    cb_group_);

  RCLCPP_INFO(this->get_logger(), "🟢 MappingManager 已就绪");
}

MappingManager::~MappingManager()
{
  stop_current_process();
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_start_mapping
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_start_mapping(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  // ── 阶段 1：持锁检查并切换为 TRANSITIONING ───────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::MAPPING) {
      res->success = true;
      res->message = "已在建图模式";
      return;
    }
    if (current_mode_ == RobotMode::TRANSITIONING) {
      res->success = false;
      res->message = "系统正在切换模式，请稍候";
      return;
    }
    current_mode_  = RobotMode::TRANSITIONING;
    has_saved_map_ = false;
  }

  // ── 阶段 2：释放锁后执行阻塞操作（stop + fork）───────────────────────────
  stop_current_process();

  bool ok = start_launch_process("mapping_plugin.launch.py");

  // ── 阶段 3：更新最终状态 ─────────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (ok) {
      current_mode_ = RobotMode::MAPPING;
      res->success  = true;
      res->message  = "建图模式已拉起";
      RCLCPP_INFO(this->get_logger(), "🗺️  %s", res->message.c_str());
    } else {
      current_mode_ = RobotMode::IDLE;
      res->success  = false;
      res->message  = "建图 launch 启动失败";
      RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_finish_mapping
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_finish_mapping(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  // ── 阶段 1：持锁检查并切换为 TRANSITIONING ───────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ != RobotMode::MAPPING) {
      res->success = false;
      res->message = "错误：当前不在建图模式";
      return;
    }
    current_mode_ = RobotMode::TRANSITIONING;
  }

  // ── 阶段 2：等待 PGO 服务上线（释放锁，不阻塞其他 service）───────────────
  RCLCPP_INFO(this->get_logger(), "呼叫 PGO 存图...");

  if (!client_save_map_->wait_for_service(3s)) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::MAPPING;
    res->success  = false;
    res->message  = "存图失败：未响应";
    RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    return;
  }

  // ── 阶段 3：发起异步请求，用 future.wait_for 轮询等待 ────────────────────
  // ✅ 正确做法：不调用 spin_until_future_complete(this, ...)
  //    而是直接 wait_for，让 MultiThreadedExecutor 的其他线程处理响应回调
  auto future = client_save_map_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());

  constexpr auto kPgoTimeout  = 30s;
  constexpr auto kPollInterval = std::chrono::milliseconds(50);
  auto deadline = std::chrono::steady_clock::now() + kPgoTimeout;

  while (std::chrono::steady_clock::now() < deadline) {
    if (future.wait_for(kPollInterval) == std::future_status::ready) {
      break;
    }
    // 还未就绪，继续等待（executor 其他线程会处理 client 响应回调）
  }

  if (future.wait_for(0s) != std::future_status::ready) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::MAPPING;
    res->success  = false;
    res->message  = "存图超时（30s），请检查 PGO 节点状态";
    RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    return;
  }

  auto pgo_res = future.get();
  if (!pgo_res->success) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::MAPPING;
    res->success  = false;
    res->message  = "PGO 报错：" + pgo_res->message;
    RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    return;
  }

  // ── 阶段 4：存图成功，停止建图进程 ──────────────────────────────────────
  RCLCPP_INFO(this->get_logger(), "✅ 地图落盘成功，清理建图进程...");
  stop_current_process();

  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    has_saved_map_ = true;
    current_mode_  = RobotMode::IDLE;
  }

  res->success = true;
  res->message = "建图结束，地图已保存";
  RCLCPP_INFO(this->get_logger(), "✅ %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_start_navigation
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_start_navigation(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  // ── 阶段 1：持锁检查并切换为 TRANSITIONING ───────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!has_saved_map_) {
      res->success = false;
      res->message = "拒绝：必须先完成建图存图才能导航";
      return;
    }
    if (current_mode_ == RobotMode::NAVIGATION) {
      res->success = true;
      res->message = "已在导航模式";
      return;
    }
    if (current_mode_ == RobotMode::TRANSITIONING) {
      res->success = false;
      res->message = "系统正在切换模式，请稍候";
      return;
    }
    current_mode_ = RobotMode::TRANSITIONING;
  }

  // ── 阶段 2：释放锁后执行阻塞操作 ────────────────────────────────────────
  stop_current_process();

  bool ok = start_launch_process("nav_plugin.launch.py");

  // ── 阶段 3：更新最终状态 ─────────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (ok) {
      current_mode_ = RobotMode::NAVIGATION;
      res->success  = true;
      res->message  = "导航模式已拉起";
      RCLCPP_INFO(this->get_logger(), "🧭  %s", res->message.c_str());
    } else {
      current_mode_ = RobotMode::IDLE;
      res->success  = false;
      res->message  = "导航 launch 启动失败";
      RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_stop_all
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_stop_all(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  // ── 阶段 1：持锁检查并切换为 TRANSITIONING ───────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::IDLE) {
      res->success = true;
      res->message = "已是 IDLE 状态";
      return;
    }
    current_mode_ = RobotMode::TRANSITIONING;
  }

  // ── 阶段 2：释放锁后停止进程（内部有阻塞等待，不能持锁）─────────────────
  stop_current_process();

  // ── 阶段 3：更新状态 ─────────────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::IDLE;
  }

  res->success = true;
  res->message = "已紧急关闭所有任务";
  RCLCPP_WARN(this->get_logger(), "🛑 %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Process Management
// ─────────────────────────────────────────────────────────────────────────────

bool MappingManager::start_launch_process(const std::string & launch_file)
{
  // ⚠️ 调用此函数前必须已释放 state_mtx_（内部无锁）
  pid_t pid = ::fork();

  if (pid < 0) {
    RCLCPP_ERROR(this->get_logger(), "fork() 失败: %s", strerror(errno));
    return false;
  }

  if (pid == 0) {
    // ── 子进程 ────────────────────────────────────────────────────────────
    // setpgid(0,0)：成为新进程组的组长
    // 之后 kill(-pid, SIG) 可以将信号发给整个进程组（含 launch 的所有子节点）
    ::setpgid(0, 0);
    ::execlp("ros2", "ros2", "launch", "auto_construct",
             launch_file.c_str(), nullptr);
    // execlp 仅失败才返回
    ::_exit(EXIT_FAILURE);
  }

  // ── 父进程 ────────────────────────────────────────────────────────────────
  current_pid_ = pid;
  RCLCPP_INFO(this->get_logger(),
    "▶ 启动 [%s] PID=%d", launch_file.c_str(), pid);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::stop_current_process()
{
  // ⚠️ 调用此函数前必须已释放 state_mtx_（内部有阻塞等待）
  if (current_pid_ <= 0) return;

  const pid_t pid = current_pid_;
  current_pid_ = -1;  // 先置 -1，防止析构时重复 kill

  RCLCPP_INFO(this->get_logger(), "⏹ 终止进程组 PGID=%d (SIGINT)", pid);

  // 向整个进程组发 SIGINT（ROS2 节点会响应 SIGINT 做优雅退出）
  ::kill(-pid, SIGINT);

  constexpr int kTimeoutMs   = 5000;
  constexpr int kIntervalMs  = 100;
  int elapsed_ms = 0;
  int status     = 0;

  while (elapsed_ms < kTimeoutMs) {
    pid_t r = ::waitpid(pid, &status, WNOHANG);
    if (r == pid) {
      RCLCPP_INFO(this->get_logger(), "✅ 进程组 %d 已退出", pid);
      return;
    }
    if (r == -1 && errno == ECHILD) {
      return;  // 子进程已不存在
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
    elapsed_ms += kIntervalMs;
  }

  // 超时 → SIGKILL
  RCLCPP_WARN(this->get_logger(),
    "⚠ 进程组 %d 未响应 SIGINT，发送 SIGKILL", pid);
  ::kill(-pid, SIGKILL);
  ::waitpid(pid, &status, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MappingManager>();

  // ✅ MultiThreadedExecutor + Reentrant 回调组 的组合：
  //
  //  线程 A：执行 handle_finish_mapping，在 future.wait_for 轮询
  //  线程 B：执行 /save_map client 的响应回调，将 future 置为 ready
  //
  // 两个线程并发，future.wait_for 轮询退出，不会死锁。
  // 线程数建议 >= 2，保证至少一个线程处理 client 响应。
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions{}, 4);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}