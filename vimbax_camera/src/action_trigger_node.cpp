#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <pthread.h>
#include <time.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

#include <vimbax_camera/action_trigger_node.hpp>
#include <vimbax_camera/percentile.hpp>
#include <vimbax_camera/vimbax_camera_helper.hpp>

namespace vimbax_camera
{

using helper::vmb_error_to_string;

constexpr int kSenderPriority = 50;
constexpr int64_t kNsPerSec = 1000000000;

static std::optional<uint32_t> ipv4_directed_broadcast(const std::string & interface_name)
{
  ifaddrs * addresses = nullptr;
  if (getifaddrs(&addresses) != 0) {
    return std::nullopt;
  }
  std::optional<uint32_t> broadcast;
  for (auto * entry = addresses; entry != nullptr; entry = entry->ifa_next) {
    if (entry->ifa_addr && entry->ifa_netmask && entry->ifa_addr->sa_family == AF_INET &&
      interface_name == entry->ifa_name)
    {
      auto const address = ntohl(reinterpret_cast<sockaddr_in *>(entry->ifa_addr)->sin_addr.s_addr);
      auto const netmask = ntohl(
        reinterpret_cast<sockaddr_in *>(entry->ifa_netmask)->sin_addr.s_addr);
      broadcast = (address & netmask) | ~netmask;
      break;
    }
  }
  freeifaddrs(addresses);
  return broadcast;
}

static std::string ipv4_to_string(uint32_t address)
{
  in_addr const network_order{htonl(address)};
  char text[INET_ADDRSTRLEN]{};
  inet_ntop(AF_INET, &network_order, text, sizeof(text));
  return text;
}

static int64_t monotonic_ns()
{
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return now.tv_sec * kNsPerSec + now.tv_nsec;
}

ActionTriggerNode::ActionTriggerNode(const rclcpp::NodeOptions & options)
: Node("action_trigger", options),
  api_{VmbCAPI::get_instance()},
  period_ns_{static_cast<int64_t>(kNsPerSec / declare_parameter<double>("rate_hz", 30.0))},
  window_start_{std::chrono::steady_clock::now()}
{
  auto const device_key = declare_parameter<int64_t>("device_key", 1);
  auto const group_key = declare_parameter<int64_t>("group_key", 1);
  auto const group_mask = declare_parameter<int64_t>("group_mask", 1);
  auto const interface_id = declare_parameter<std::string>("interface_id", "");

  auto const fail = [this](const std::string & message) {
      RCLCPP_FATAL(get_logger(), "%s", message.c_str());
      throw std::runtime_error(message);
    };

  if (!api_) {
    fail("failed to load VmbC API");
  }

  auto const interfaces = api_->interface_list_get();
  if (!interfaces) {
    fail("listing interfaces failed: " + std::string{vmb_error_to_string(interfaces.error().code)});
  }

  if (interface_id.empty()) {
    fail("interface_id is required (the network interface the cameras are on, e.g. eth0)");
  }

  for (auto const & info : *interfaces) {
    if (info.interfaceType == VmbTransportLayerTypeGEV && interface_id == info.interfaceIdString) {
      transport_layer_handle_ = info.transportLayerHandle;
      break;
    }
  }
  if (!transport_layer_handle_) {
    fail("GigE interface '" + interface_id + "' not found");
  }

  auto const broadcast = ipv4_directed_broadcast(interface_id);
  if (!broadcast) {
    fail("interface '" + interface_id + "' has no IPv4 address");
  }
  RCLCPP_INFO(
    get_logger(), "sending action commands via the GigE transport layer to %s (%s)",
    ipv4_to_string(*broadcast).c_str(), interface_id.c_str());

  // The interface module's ActionCommand stalls ~170 ms every 2.5 s; the transport layer's does not.
  const std::pair<const char *, int64_t> action_features[] = {
    {"ActionDeviceKey", device_key},
    {"ActionGroupKey", group_key},
    {"ActionGroupMask", group_mask},
    {"GevActionDestinationIPAddress", *broadcast},
  };
  for (auto const & [name, value] : action_features) {
    if (!api_->feature_int_set(transport_layer_handle_, name, value)) {
      fail(std::string{"setting "} + name + " failed");
    }
  }

  summary_timer_ = create_wall_timer(std::chrono::seconds(30), [this] {log_summary();});
  sender_ = std::thread([this] {send_actions();});
}

ActionTriggerNode::~ActionTriggerNode()
{
  running_ = false;
  if (sender_.joinable()) {
    sender_.join();
  }
}

void ActionTriggerNode::send_actions()
{
  sched_param const param{kSenderPriority};
  auto const sched_err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (sched_err != 0) {
    RCLCPP_WARN(
      get_logger(), "SCHED_FIFO priority %d unavailable (%s), sending with default scheduling",
      kSenderPriority, std::strerror(sched_err));
  }

  auto deadline_ns = monotonic_ns() + period_ns_;
  while (running_) {
    timespec const deadline{
      static_cast<time_t>(deadline_ns / kNsPerSec), static_cast<long>(deadline_ns % kNsPerSec)};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr) == EINTR) {
    }
    auto const wake_ns = monotonic_ns();
    auto const err = api_->FeatureCommandRun(transport_layer_handle_, "ActionCommand");

    auto const lateness_ns = wake_ns - deadline_ns;
    auto const late = lateness_ns > period_ns_;
    deadline_ns = (late ? wake_ns : deadline_ns) + period_ns_;

    std::lock_guard<std::mutex> lock(window_mutex_);
    if (err == VmbErrorSuccess) {
      ++window_.sent;
    } else {
      ++window_.failed;
    }
    window_.late += late;
    window_.wake_lateness_ms.push_back(lateness_ns / 1e6);
  }
}

void ActionTriggerNode::log_summary()
{
  Window window;
  auto const end = std::chrono::steady_clock::now();
  double seconds = 0.0;
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    std::swap(window, window_);
    seconds = std::chrono::duration<double>(end - window_start_).count();
    window_start_ = end;
  }
  auto const max_lateness_ms = window.wake_lateness_ms.empty() ? 0.0 :
    *std::max_element(window.wake_lateness_ms.begin(), window.wake_lateness_ms.end());
  RCLCPP_INFO(
    get_logger(),
    "last %.1f s: sent %lu (%.1f Hz), failed %lu, late %lu, "
    "wake lateness p50 %.3f ms p95 %.3f ms max %.3f ms",
    seconds, window.sent, seconds > 0.0 ? window.sent / seconds : 0.0, window.failed,
    window.late, percentile(window.wake_lateness_ms, 0.5),
    percentile(window.wake_lateness_ms, 0.95), max_lateness_ms);
}

}  // namespace vimbax_camera

RCLCPP_COMPONENTS_REGISTER_NODE(vimbax_camera::ActionTriggerNode)
