#pragma once
#include <array>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp/rclcpp.hpp>
#include <geometry_msgs/geometry_msgs/msg/twist.hpp>
#endif

namespace minimal {

/**
 * @brief ROS2 cmd_vel command source for minimal_dog
 *
 * Subscribes to /cmd_vel topic and provides velocity commands.
 * Features:
 * - Deadman timeout (200ms default)
 * - Thread-safe command access
 * - Configurable scaling factors
 * - Graceful fallback when ROS2 is unavailable
 */
class ROS2CommandSource {
public:
    ROS2CommandSource();
    ~ROS2CommandSource();

    /**
     * @brief Initialize ROS2 node and subscription
     * @return true if ROS2 is available and initialized, false otherwise
     */
    bool Initialize();

    /**
     * @brief Get current command [vx, vy, yaw_rate]
     * @return Array of 3 floats, zero if no valid command
     */
    std::array<float, 3> GetCommand() const;

    /**
     * @brief Stop the spin thread and cleanup
     */
    void Stop();

private:
#ifdef ROS2_FOUND
    void OnTwist(const geometry_msgs::msg::Twist::SharedPtr msg);
    void SpinLoop();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    std::thread spin_thread_;
#endif

    mutable std::mutex cmd_mutex_;
    float cmd_vx_ = 0.0f;
    float cmd_vy_ = 0.0f;
    float cmd_yaw_ = 0.0f;

    std::atomic<bool> running_;
    std::chrono::steady_clock::time_point last_msg_tp_;
    bool received_once_ = false;
    bool did_init_ = false;  // Track whether we called rclcpp::init()

    static constexpr int kDeadmanMs = 200;
    static constexpr float kVxScale = 0.5f;
    static constexpr float kVyScale = 0.5f;
    static constexpr float kYawScale = 0.5f;
};

}  // namespace minimal
