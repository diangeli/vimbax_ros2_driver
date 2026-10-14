#ifndef VIMBAX_CAMERA__SYNC_MONITOR_NODE_HPP_
#define VIMBAX_CAMERA__SYNC_MONITOR_NODE_HPP_

#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <vimbax_camera/stamp_pairer.hpp>

namespace vimbax_camera
{

class SyncMonitorNode : public rclcpp::Node
{
public:
  explicit SyncMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void log_summary();

  double warn_offset_ms_;

  std::mutex pairer_mutex_;
  StampPairer pairer_;
  std::chrono::steady_clock::time_point window_start_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr ir_subscription_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__SYNC_MONITOR_NODE_HPP_
