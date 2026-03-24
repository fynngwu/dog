#include "ros2_command_source.hpp"
#include <iostream>
#include <cstring>

namespace minimal {

ROS2CommandSource::ROS2CommandSource() : running_(false) {}

ROS2CommandSource::~ROS2CommandSource() {
    Stop();
}

bool ROS2CommandSource::Initialize() {
#ifdef ROS2_FOUND
    // Initialize ROS2 if not already
    if (!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
        did_init_ = true;
    }

    // Create node and subscription
    node_ = std::make_shared<rclcpp::Node>("minimal_dog_cmd");
    sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            OnTwist(msg);
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // Start spin thread
    running_ = true;
    spin_thread_ = std::thread(&ROS2CommandSource::SpinLoop, this);

    std::cout << "[ROS2Command] Subscribed to /cmd_vel" << std::endl;
    return true;
#else
    std::cerr << "[ROS2Command] ROS2 not available, command will be zero" << std::endl;
    return false;
#endif
}

void ROS2CommandSource::OnTwist(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_vx_ = static_cast<float>(msg->linear.x) * kVxScale;
    cmd_vy_ = static_cast<float>(msg->linear.y) * kVyScale;
    cmd_yaw_ = static_cast<float>(msg->angular.z) * kYawScale;
    last_msg_tp_ = std::chrono::steady_clock::now();
    received_once_ = true;
}

void ROS2CommandSource::SpinLoop() {
#ifdef ROS2_FOUND
    while (running_ && rclcpp::ok()) {
        executor_->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
#endif
}

std::array<float, 3> ROS2CommandSource::GetCommand() const {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    if (!received_once_) {
        return {0.0f, 0.0f, 0.0f};
    }

    auto now = std::chrono::steady_clock::now();
    auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_msg_tp_).count();

    if (dt_ms > kDeadmanMs) {
        return {0.0f, 0.0f, 0.0f};
    }

    return {cmd_vx_, cmd_vy_, cmd_yaw_};
}

void ROS2CommandSource::Stop() {
    running_ = false;
#ifdef ROS2_FOUND
    // Cancel executor and shutdown ROS2 first, then join thread
    // This ensures the spin thread can exit even if blocked
    if (executor_) {
        executor_->cancel();
    }
    if (did_init_) {
        rclcpp::shutdown();
        did_init_ = false;
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    executor_.reset();
    node_.reset();
#endif
}

}  // namespace minimal
