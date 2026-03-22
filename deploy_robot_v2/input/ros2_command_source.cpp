#include "ros2_command_source.hpp"

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

Ros2CommandSource::Ros2CommandSource(const std::string& topic,
                                     float vx_scale,
                                     float vy_scale,
                                     float yaw_scale,
                                     int deadman_ms)
    : topic_(topic)
    , vx_scale_(vx_scale)
    , vy_scale_(vy_scale)
    , yaw_scale_(yaw_scale)
    , deadman_ms_(deadman_ms) {
    if (!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
        owns_ros_init_ = true;
    }

    node_ = std::make_shared<rclcpp::Node>("policy_cmd_source");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            OnTwist(msg);
        });

    std::cout << "[ROS2] Listening topic: " << topic_
              << " (Twist: linear.x -> vx, linear.y -> vy, angular.z -> yaw)"
              << std::endl;

    spin_thread_ = std::thread(&Ros2CommandSource::SpinLoop, this);
}

Ros2CommandSource::~Ros2CommandSource() {
    running_ = false;

    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }

    if (executor_ && node_) {
        executor_->remove_node(node_);
    }

    if (owns_ros_init_ && rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void Ros2CommandSource::OnTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_[0] = static_cast<float>(msg->linear.x)  * vx_scale_;
    cmd_[1] = static_cast<float>(msg->linear.y)  * vy_scale_;
    cmd_[2] = static_cast<float>(msg->angular.z) * yaw_scale_;
    last_msg_tp_ = std::chrono::steady_clock::now();
    received_once_ = true;
}

void Ros2CommandSource::SpinLoop() {
    while (running_ && rclcpp::ok()) {
        executor_->spin_some();
        std::this_thread::sleep_for(5ms);
    }
}

std::vector<float> Ros2CommandSource::GetCommand() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!received_once_) {
        return {0.0f, 0.0f, 0.0f};
    }

    auto now = std::chrono::steady_clock::now();
    auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_msg_tp_).count();
    if (dt_ms > deadman_ms_) {
        return {0.0f, 0.0f, 0.0f};
    }

    return {cmd_[0], cmd_[1], cmd_[2]};
}

bool Ros2CommandSource::IsReady() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_once_) {
        return false;
    }

    auto now = std::chrono::steady_clock::now();
    auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_msg_tp_).count();
    return dt_ms <= deadman_ms_;
}

void Ros2CommandSource::Update() {
    // 留空；实际 spin 在独立线程里跑
}