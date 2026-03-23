#pragma once
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include "robstride.hpp"
#include "can_interface.hpp"

/**
 * @brief 机器人启动管理器（精简版，无 IMU 依赖）
 *
 * 负责安全的电机上电流程：
 * 1. 绑定 CAN 接口
 * 2. 绑定电机
 * 3. 使能电机
 * 4. 开启自动上报
 * 5. 等待反馈在线
 * 6. 平滑移动到目标姿态
 */
class RobotBringup {
public:
    using StopCondition = std::function<bool()>;

    explicit RobotBringup(std::shared_ptr<RobstrideController> controller);
    ~RobotBringup() = default;

    bool BindAllCAN();
    bool BindAllMotors(std::vector<int>& motor_indices);
    bool EnableAll(const std::vector<int>& motor_indices, bool low_gain = false, int retry_count = 3);
    bool EnableAutoReportAll(const std::vector<int>& motor_indices);
    bool WaitForFeedback(const std::vector<int>& motor_indices, int timeout_ms = 3000);
    std::vector<float> ReadCurrentAbsPositions(const std::vector<int>& motor_indices) const;

    bool SmoothMoveAbs(const std::vector<int>& motor_indices,
                       const std::vector<float>& from_abs,
                       const std::vector<float>& to_abs,
                       float duration_sec = 2.0f,
                       int hz = 100,
                       StopCondition stop_condition = nullptr);

    bool MoveCurrentToOffsets(const std::vector<int>& motor_indices,
                              float duration_sec = 2.0f,
                              StopCondition stop_condition = nullptr);

    bool HoldOffsets(const std::vector<int>& motor_indices);
    bool EmergencyHoldAll(const std::vector<int>& motor_indices, int repeat = 3);

    std::shared_ptr<RobstrideController> GetController() const { return controller_; }

private:
    std::shared_ptr<RobstrideController> controller_;
    std::vector<std::shared_ptr<CANInterface>> can_ifaces_;
};