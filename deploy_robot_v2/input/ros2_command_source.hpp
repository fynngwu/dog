#pragma once

#include "input_source.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief ROS2 命令输入源
 *
 * 订阅 geometry_msgs/Twist 话题，映射为速度命令：
 * - linear.x -> vx
 * - linear.y -> vy
 * - angular.z -> yaw_rate
 *
 * 安全机制：
 * - deadman timeout: 超时后自动归零
 */
class Ros2CommandSource : public ICommandSource {
public:
    /**
     * @brief 构造函数
     * @param topic Twist 话题名称
     * @param vx_scale vx 缩放因子
     * @param vy_scale vy 缩放因子
     * @param yaw_scale yaw 缩放因子
     * @param deadman_ms deadman 超时时间 (毫秒)
     */
    Ros2CommandSource(const std::string& topic = "/policy_cmd",
                      float vx_scale = 1.0f,
                      float vy_scale = 1.0f,
                      float yaw_scale = 1.0f,
                      int deadman_ms = 200);

    ~Ros2CommandSource() override;

    std::vector<float> GetCommand() const override;
    bool IsReady() const override;
    void Update() override;
    const char* Name() const override { return "ros2"; }

private:
    void SpinLoop();
    void OnTwist(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
    std::string topic_;
    float vx_scale_{1.0f};
    float vy_scale_{1.0f};
    float yaw_scale_{1.0f};
    int deadman_ms_{200};

    bool owns_ros_init_{false};
    bool running_{true};
    bool received_once_{false};

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;

    mutable std::mutex mutex_;
    std::array<float, 3> cmd_{0.0f, 0.0f, 0.0f};
    std::chrono::steady_clock::time_point last_msg_tp_{std::chrono::steady_clock::now()};
};