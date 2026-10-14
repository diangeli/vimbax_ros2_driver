#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <string>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

#include <vimbax_camera/percentile.hpp>
#include <vimbax_camera/sync_monitor_node.hpp>

namespace vimbax_camera
{

using sensor_msgs::msg::CameraInfo;

constexpr double kNsPerMs = 1e6;

SyncMonitorNode::SyncMonitorNode(const rclcpp::NodeOptions & options)
: Node("sync_monitor", options),
  warn_offset_ms_{declare_parameter<double>("warn_offset_ms", 1.0)},
  pairer_{static_cast<int64_t>(declare_parameter<double>("max_pair_offset_ms", 25.0) * kNsPerMs)},
  window_start_{std::chrono::steady_clock::now()}
{
  auto const rgb_topic = declare_parameter<std::string>(
    "rgb_camera_info_topic", "/vimbax_camera_rgb/camera_info");
  auto const ir_topic = declare_parameter<std::string>(
    "ir_camera_info_topic", "/vimbax_camera_ir/camera_info");
  auto const qos = rclcpp::QoS{rclcpp::KeepLast{30}}.best_effort();

  rgb_subscription_ = create_subscription<CameraInfo>(
    rgb_topic, qos, [this](CameraInfo::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(pairer_mutex_);
      pairer_.add_rgb(rclcpp::Time{msg->header.stamp}.nanoseconds());
    });
  ir_subscription_ = create_subscription<CameraInfo>(
    ir_topic, qos, [this](CameraInfo::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(pairer_mutex_);
      pairer_.add_ir(rclcpp::Time{msg->header.stamp}.nanoseconds());
    });
  summary_timer_ = create_wall_timer(std::chrono::seconds(30), [this] {log_summary();});
}

void SyncMonitorNode::log_summary()
{
  StampPairer::Window window;
  auto const end = std::chrono::steady_clock::now();
  double seconds = 0.0;
  {
    std::lock_guard<std::mutex> lock(pairer_mutex_);
    window = pairer_.take_window();
    seconds = std::chrono::duration<double>(end - window_start_).count();
    window_start_ = end;
  }

  std::vector<double> abs_dt_ms;
  abs_dt_ms.reserve(window.rgb_minus_ir_ns.size());
  for (auto const dt_ns : window.rgb_minus_ir_ns) {
    abs_dt_ms.push_back(std::abs(static_cast<double>(dt_ns)) / kNsPerMs);
  }
  auto const pairs = window.rgb_minus_ir_ns.size();
  auto const mean_dt_ms = pairs == 0 ? 0.0 :
    std::accumulate(
    window.rgb_minus_ir_ns.begin(), window.rgb_minus_ir_ns.end(), 0.0) / pairs / kNsPerMs;
  auto const p95 = percentile(abs_dt_ms, 0.95);
  auto const max_abs_dt_ms = abs_dt_ms.empty() ? 0.0 :
    *std::max_element(abs_dt_ms.begin(), abs_dt_ms.end());

  char line[256];
  std::snprintf(
    line, sizeof(line),
    "last %.1f s: rgb %lu, ir %lu, pairs %lu (%.1f Hz), unmatched rgb %lu ir %lu, "
    "|dt| p50 %.3f ms p95 %.3f ms max %.3f ms, mean dt(rgb-ir) %.3f ms",
    seconds, window.rgb, window.ir, pairs, seconds > 0.0 ? pairs / seconds : 0.0,
    window.unmatched_rgb, window.unmatched_ir, percentile(abs_dt_ms, 0.5), p95, max_abs_dt_ms,
    mean_dt_ms);
  if (p95 > warn_offset_ms_) {
    RCLCPP_WARN(get_logger(), "%s", line);
  } else {
    RCLCPP_INFO(get_logger(), "%s", line);
  }
}

}  // namespace vimbax_camera

RCLCPP_COMPONENTS_REGISTER_NODE(vimbax_camera::SyncMonitorNode)
