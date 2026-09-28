#include "camera_recorder/recorder_node.hpp"

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <type_traits>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include "camera_recorder/bag_uri.hpp"

namespace camera_recorder
{

RecorderNode::RecorderNode(const rclcpp::NodeOptions & options)
: Node("recorder", options),
  queue_(static_cast<std::size_t>(declare_parameter<std::int64_t>("queue_frames", 600)))
{
  const auto output_dir = declare_parameter<std::string>("output_dir", "~/bags");
  const auto image_topics = declare_parameter("image_topics", std::vector<std::string>{});
  const auto camera_info_topics =
    declare_parameter("camera_info_topics", std::vector<std::string>{});
  const auto generic_topics = declare_parameter("generic_topics", std::vector<std::string>{});
  const auto home = std::getenv("HOME");
  const auto wall_time = std::time(nullptr);
  std::tm local_time{};
  localtime_r(&wall_time, &local_time);

  rosbag2_storage::StorageOptions storage;
  storage.uri = bag_uri(output_dir, home ? home : "", local_time);
  storage.storage_id = declare_parameter<std::string>("storage_id", "sqlite3");
  storage.max_cache_size =
    static_cast<std::uint64_t>(declare_parameter<std::int64_t>("max_cache_size_mb", 0)) *
    1024 * 1024;
  storage.max_bagfile_duration =
    static_cast<std::uint64_t>(declare_parameter<std::int64_t>("max_bagfile_duration_s", 60));
  writer_.open(storage, {"cdr", "cdr"});
  RCLCPP_INFO(get_logger(), "recording to %s (%s)", storage.uri.c_str(), storage.storage_id.c_str());

  const auto resolve = [this](const std::string & name) {
      return get_node_topics_interface()->resolve_topic_name(name);
    };
  for (const auto & name : image_topics) {
    topics_.push_back({resolve(name), "sensor_msgs/msg/Image", {}});
  }
  for (const auto & name : camera_info_topics) {
    topics_.push_back({resolve(name), "sensor_msgs/msg/CameraInfo", {}});
  }
  for (const auto & name : generic_topics) {
    pending_generic_.push_back(topics_.size());
    topics_.push_back({resolve(name), "", {}});
  }

  const auto typed_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  for (std::size_t i = 0; i < image_topics.size() + camera_info_topics.size(); ++i) {
    writer_.create_topic({topics_[i].name, topics_[i].type, "cdr", ""});
  }
  for (std::size_t i = 0; i < image_topics.size(); ++i) {
    subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::Image>(
        topics_[i].name, typed_qos,
        [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) {enqueue(i, std::move(msg));}));
  }
  for (std::size_t i = image_topics.size(); i < image_topics.size() + camera_info_topics.size();
    ++i)
  {
    subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::CameraInfo>(
        topics_[i].name, typed_qos,
        [this, i](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          enqueue(i, std::move(msg));
        }));
  }
  subscribe_pending_generic_topics();
  if (!pending_generic_.empty()) {
    generic_retry_timer_ =
      create_wall_timer(std::chrono::seconds(1), [this] {subscribe_pending_generic_topics();});
  }

  window_start_ = std::chrono::steady_clock::now();
  summary_timer_ = create_wall_timer(std::chrono::seconds(30), [this] {log_summary();});
  worker_ = std::thread([this] {write_items();});
}

RecorderNode::~RecorderNode()
{
  queue_.close();
  worker_.join();
  writer_.close();
  log_summary();
}

void RecorderNode::enqueue(std::size_t topic, Message message)
{
  const bool queued = queue_.try_push({topic, std::move(message), now()});
  const auto depth = queue_.size();
  std::lock_guard<std::mutex> lock(window_mutex_);
  auto & window = topics_[topic].window;
  ++window.received;
  window.dropped += !queued;
  max_queue_depth_ = std::max(max_queue_depth_, depth);
}

void RecorderNode::subscribe_pending_generic_topics()
{
  const auto advertised = get_topic_names_and_types();
  std::vector<std::size_t> still_pending;
  for (const auto i : pending_generic_) {
    auto & topic = topics_[i];
    const auto found = advertised.find(topic.name);
    if (found == advertised.end() || found->second.empty()) {
      still_pending.push_back(i);
      continue;
    }
    topic.type = found->second.front();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    const auto publishers = get_publishers_info_by_topic(topic.name);
    if (std::any_of(
        publishers.begin(), publishers.end(), [](const rclcpp::TopicEndpointInfo & p) {
          return p.qos_profile().reliability() == rclcpp::ReliabilityPolicy::BestEffort;
        }))
    {
      qos.best_effort();
    }
    writer_.create_topic({topic.name, topic.type, "cdr", ""});
    subscriptions_.push_back(
      create_generic_subscription(
        topic.name, topic.type, qos,
        [this, i](std::shared_ptr<rclcpp::SerializedMessage> msg) {enqueue(i, std::move(msg));}));
    RCLCPP_INFO(
      get_logger(), "subscribed to %s (%s, %s)", topic.name.c_str(), topic.type.c_str(),
      qos.reliability() == rclcpp::ReliabilityPolicy::BestEffort ? "best effort" : "reliable");
  }
  pending_generic_ = std::move(still_pending);
  if (pending_generic_.empty() && generic_retry_timer_) {
    generic_retry_timer_->cancel();
  }
}

void RecorderNode::write_items()
{
  while (auto item = queue_.pop()) {
    const auto serialized = std::visit(
      [](const auto & msg) {
        using M = std::decay_t<decltype(msg)>;
        if constexpr (std::is_same_v<M, std::shared_ptr<rclcpp::SerializedMessage>>) {
          return msg;
        } else {
          auto out = std::make_shared<rclcpp::SerializedMessage>();
          rclcpp::Serialization<std::remove_const_t<typename M::element_type>>().serialize_message(
            msg.get(), out.get());
          return out;
        }
      }, item->message);
    const auto & topic = topics_[item->topic];
    // Writer::write releases the serialized buffer, leaving size() at 0.
    const auto bytes = serialized->size();
    const auto start = std::chrono::steady_clock::now();
    writer_.write(serialized, topic.name, topic.type, item->received);
    const double write_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    std::lock_guard<std::mutex> lock(window_mutex_);
    auto & window = topics_[item->topic].window;
    ++window.written;
    window.bytes += bytes;
    window.write_ms.push_back(write_ms);
  }
}

void RecorderNode::log_summary()
{
  std::vector<Window> windows;
  std::size_t max_depth = 0;
  const auto end = std::chrono::steady_clock::now();
  double seconds = 0.0;
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    for (auto & topic : topics_) {
      windows.push_back(std::exchange(topic.window, {}));
    }
    max_depth = std::exchange(max_queue_depth_, 0);
    seconds = std::chrono::duration<double>(end - window_start_).count();
    window_start_ = end;
  }
  for (std::size_t i = 0; i < topics_.size(); ++i) {
    auto & window = windows[i];
    std::sort(window.write_ms.begin(), window.write_ms.end());
    const double write_p95 = window.write_ms.empty() ? 0.0 :
      window.write_ms[static_cast<std::size_t>(0.95 * (window.write_ms.size() - 1) + 0.5)];
    RCLCPP_INFO(
      get_logger(),
      "%s last %.1f s: received %lu, written %lu (%.1f Hz), dropped %lu, write p95 %.2f ms, "
      "%.1f MB/s",
      topics_[i].name.c_str(), seconds, window.received, window.written,
      seconds > 0.0 ? window.written / seconds : 0.0, window.dropped, write_p95,
      seconds > 0.0 ? window.bytes / seconds / 1e6 : 0.0);
  }
  RCLCPP_INFO(get_logger(), "queue depth now %zu, max %zu in the last %.1f s", queue_.size(),
    max_depth, seconds);
}

}  // namespace camera_recorder

RCLCPP_COMPONENTS_REGISTER_NODE(camera_recorder::RecorderNode)
