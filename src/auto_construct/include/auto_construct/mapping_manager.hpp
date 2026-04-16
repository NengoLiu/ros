#ifndef MAPPING_MANAGER_HPP_
#define MAPPING_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <mutex>
#include <sys/types.h>
#include <unistd.h>

// ─────────────────────────────────────────────────────────────────────────────
enum class RobotMode { IDLE, MAPPING, NAVIGATION, TRANSITIONING };

inline const char * mode_str(RobotMode m)
{
  switch (m) {
    case RobotMode::IDLE:          return "IDLE";
    case RobotMode::MAPPING:       return "MAPPING";
    case RobotMode::NAVIGATION:    return "NAVIGATION";
    case RobotMode::TRANSITIONING: return "TRANSITIONING";
  }
  return "UNKNOWN";
}

// ─────────────────────────────────────────────────────────────────────────────
class MappingManager : public rclcpp::Node
{
public:
  MappingManager();
  ~MappingManager() override;

private:
  // ── Service 回调 ─────────────────────────────────────────────────────────
  void handle_start_mapping(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_finish_mapping(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_start_navigation(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_stop_all(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ── 进程管理 ─────────────────────────────────────────────────────────────
  bool start_launch_process(const std::string & launch_file);
  void stop_current_process();

  // ── 状态 ─────────────────────────────────────────────────────────────────
  // ⚠️ 原版三个裸变量在 Reentrant + MultiThreadedExecutor 下存在数据竞争
  //    用 mutex 统一保护，并增加 TRANSITIONING 防止并发切换
  std::mutex          state_mtx_;
  RobotMode           current_mode_  {RobotMode::IDLE};
  bool                has_saved_map_ {false};
  pid_t               current_pid_   {-1};

  // ── ROS 对象 ─────────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_mapping_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_finish_mapping_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_navigation_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_all_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr  client_save_map_;
};

#endif  // MAPPING_MANAGER_HPP_