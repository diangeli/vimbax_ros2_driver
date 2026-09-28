#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "camera_recorder/bounded_queue.hpp"

namespace camera_recorder
{

class RecorderNode : public rclcpp::Node
{
public:
  explicit RecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RecorderNode() override;

private:
  using Message = std::variant<
    sensor_msgs::msg::Image::ConstSharedPtr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr,
    std::shared_ptr<rclcpp::SerializedMessage>>;

  struct Item
  {
    std::size_t topic;
    Message message;
    rclcpp::Time received;
  };

  struct Window
  {
    std::uint64_t received = 0;
    std::uint64_t written = 0;
    std::uint64_t dropped = 0;
    std::uint64_t bytes = 0;
    std::vector<double> write_ms;
  };

  struct Topic
  {
    std::string name;
    std::string type;
    Window window;
  };

  void enqueue(std::size_t topic, Message message);
  void subscribe_pending_generic_topics();
  void write_items();
  void log_summary();

  rosbag2_cpp::Writer writer_;
  std::vector<Topic> topics_;
  std::vector<std::size_t> pending_generic_;

  std::mutex window_mutex_;
  std::size_t max_queue_depth_ = 0;
  std::chrono::steady_clock::time_point window_start_;

  BoundedQueue<Item> queue_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr generic_retry_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  std::thread worker_;
};

}  // namespace camera_recorder
