#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>

namespace camera_sync {

// Configuration constants
constexpr const char* MASTER_CAMERA_TOPIC = "/vimbax_camera_ir/camera_info";
constexpr const char* SLAVE_CAMERA_TOPIC = "/vimbax_camera_rgb/camera_info";
constexpr int QUEUE_SIZE = 1; // Use last message
constexpr std::chrono::milliseconds SYNC_LOOP_RATE{10}; // Check at 100Hz
constexpr int MAX_NODE_RECREATIONS = 1000;
constexpr int CONSECUTIVE_SYNC_SUCCESSES_REQUIRED = 10;
constexpr int MAX_CONSECUTIVE_FAILURES = 10;

class CameraSubscriber : public rclcpp::Node {
public:
    CameraSubscriber(const std::string& node_name, const std::string& topic)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(false)),
          last_stamp_sec_(0),
          last_stamp_nsec_(0),
          has_new_message_(false) {
        RCLCPP_INFO(get_logger(), "Creating subscriber for topic: %s", topic.c_str());
        auto qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE));
        sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            topic, qos,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                this->last_stamp_sec_ = msg->header.stamp.sec;
                this->last_stamp_nsec_ = msg->header.stamp.nanosec;
                this->has_new_message_ = true;
            });
    }

    ~CameraSubscriber() {
        RCLCPP_INFO(get_logger(), "Destroying subscriber for topic: %s", sub_->get_topic_name());
    }

    bool has_new_message() {
        return has_new_message_.exchange(false); // Atomically get and reset flag
    }

    double get_timestamp_sec() const {
        return static_cast<double>(last_stamp_sec_) + static_cast<double>(last_stamp_nsec_) / 1e9;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_;
    std::atomic<int32_t> last_stamp_sec_;
    std::atomic<uint32_t> last_stamp_nsec_;
    std::atomic<bool> has_new_message_;
};

class SyncManager : public rclcpp::Node {
public:
    enum class SyncState { ATTEMPTING_SYNC, MONITORING, FAILED };

    explicit SyncManager(double time_threshold_ms)
        : Node("sync_manager"),
          time_threshold_ms_(time_threshold_ms),
          state_(SyncState::ATTEMPTING_SYNC),
          consecutive_successes_(0),
          consecutive_failures_(0),
          recreation_count_(0)
          {
        this->declare_parameter<bool>("verbose_monitoring", false);
        this->get_parameter("verbose_monitoring", verbose_monitoring_);

        RCLCPP_INFO(get_logger(), "SyncManager started with threshold: %.2f ms. Verbose monitoring: %s", 
                    time_threshold_ms_, verbose_monitoring_ ? "true" : "false");
        master_node_ = std::make_shared<CameraSubscriber>("master_camera_node", MASTER_CAMERA_TOPIC);

        recreate_slave_node(false); // Initial creation

        sync_timer_ = create_wall_timer(SYNC_LOOP_RATE, std::bind(&SyncManager::sync_loop, this));
    }

private:
    void recreate_slave_node(bool is_recreation = true) {
        if (is_recreation) {
            RCLCPP_WARN(get_logger(), "Max consecutive failures reached. Recreating slave node.");
            if (slave_node_) {
                RCLCPP_INFO(get_logger(), "Deleting slave node...");
                slave_node_.reset();
            }
            RCLCPP_INFO(get_logger(), "Waiting for node to be fully destroyed...");
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Give time for cleanup
            recreation_count_++;
        }

        RCLCPP_INFO(get_logger(), "Creating slave node. Recreation #%d", recreation_count_);
        slave_node_ = std::make_shared<CameraSubscriber>("slave_camera_node", SLAVE_CAMERA_TOPIC);
        consecutive_successes_ = 0;
        consecutive_failures_ = 0;
    }

    void sync_loop() {
        // Always spin nodes to clear message queues
        rclcpp::spin_some(master_node_);
        if(slave_node_) {
            rclcpp::spin_some(slave_node_);
        }

        if (state_ == SyncState::ATTEMPTING_SYNC) {
            run_sync_attempt();
        }
        // No action needed for other states in this high-frequency loop
    }

    void run_sync_attempt() {
        if (recreation_count_ >= MAX_NODE_RECREATIONS) {
            state_ = SyncState::FAILED;
            RCLCPP_ERROR(get_logger(), "Exceeded max recreations. Giving up.");
            sync_timer_->cancel();
            return;
        }

        if (!master_node_->has_new_message() || !slave_node_ || !slave_node_->has_new_message()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for new messages from both cameras to attempt sync...");
            return;
        }

        double master_ts = master_node_->get_timestamp_sec();
        double slave_ts = slave_node_->get_timestamp_sec();
        double diff_ms = std::abs(master_ts - slave_ts) * 1000.0;

        if (diff_ms <= time_threshold_ms_) {
            consecutive_failures_ = 0;
            consecutive_successes_++;
            RCLCPP_INFO(get_logger(), "Sync success %d/%d. Time difference: %.4f ms", consecutive_successes_, CONSECUTIVE_SYNC_SUCCESSES_REQUIRED, diff_ms);
            
            if (consecutive_successes_ >= CONSECUTIVE_SYNC_SUCCESSES_REQUIRED) {
                RCLCPP_INFO(get_logger(), "Synchronization stable! Final difference: %.4f ms", diff_ms);
                RCLCPP_INFO(get_logger(), "Total recreations: %d", recreation_count_);
                state_ = SyncState::MONITORING;
                sync_timer_->cancel(); // Stop attempting to sync
                monitoring_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&SyncManager::run_monitoring, this));
            }
        } else {
            RCLCPP_INFO(get_logger(), "Sync failed. Time difference: %.4f ms. Resetting success streak.", diff_ms);
            consecutive_successes_ = 0;
            consecutive_failures_++;
            if (consecutive_failures_ >= MAX_CONSECUTIVE_FAILURES) {
                recreate_slave_node();
            }
        }
    }

    void run_monitoring() {
        rclcpp::spin_some(master_node_);
        if (slave_node_) {
            rclcpp::spin_some(slave_node_);
        }

        if (verbose_monitoring_) {
            double master_ts = master_node_->get_timestamp_sec();
            double slave_ts = slave_node_->get_timestamp_sec();
            double diff_ms = (master_ts - slave_ts) * 1000.0;

            RCLCPP_INFO(get_logger(), "[Monitoring] Time difference: %.4f ms", diff_ms);
        }
    }

    double time_threshold_ms_;
    SyncState state_;
    int consecutive_successes_;
    int consecutive_failures_;
    int recreation_count_;
    bool verbose_monitoring_;

    std::shared_ptr<CameraSubscriber> master_node_;
    std::shared_ptr<CameraSubscriber> slave_node_;
    rclcpp::TimerBase::SharedPtr sync_timer_;
    rclcpp::TimerBase::SharedPtr monitoring_timer_;
};

} // namespace camera_sync

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: %s <time_threshold_ms>", argv[0]);
        return 1;
    }

    double time_threshold_ms = 1.0;
    try {
        time_threshold_ms = std::stod(argv[1]);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid argument for time threshold. Using default 1.0 ms. Error: %s", e.what());
    }

    auto sync_manager_node = std::make_shared<camera_sync::SyncManager>(time_threshold_ms);
    rclcpp::spin(sync_manager_node);
    rclcpp::shutdown();
    return 0;
}
