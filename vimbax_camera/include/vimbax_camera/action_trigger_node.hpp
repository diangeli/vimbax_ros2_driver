#ifndef VIMBAX_CAMERA__ACTION_TRIGGER_NODE_HPP_
#define VIMBAX_CAMERA__ACTION_TRIGGER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera/loader/vmbc_api.hpp>

namespace vimbax_camera
{

class ActionTriggerNode : public rclcpp::Node
{
public:
  explicit ActionTriggerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~ActionTriggerNode() override;

private:
  struct Window
  {
    uint64_t sent = 0;
    uint64_t failed = 0;
    uint64_t late = 0;
    std::vector<double> wake_lateness_ms;
  };

  void send_actions();
  void log_summary();

  std::shared_ptr<VmbCAPI> api_;
  VmbHandle_t transport_layer_handle_{};
  int64_t period_ns_;

  std::mutex window_mutex_;
  Window window_;
  std::chrono::steady_clock::time_point window_start_;

  std::atomic<bool> running_{true};
  rclcpp::TimerBase::SharedPtr summary_timer_;
  std::thread sender_;
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__ACTION_TRIGGER_NODE_HPP_
