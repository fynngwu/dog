#pragma once
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include "robstride.hpp"
#include "can_interface.hpp"

/**
 * @brief 机器人启动管理器
 *
 * 负责安全的电机上电流程：
 * 1. 绑定 CAN 接口
 * 2. 绑定电机
 * 3. 使能电机
 * 4. 开启自动上报
 * 5. 等待反馈在线
 * 6. 平滑移动到目标姿态
 *
 * 注意：此类不包含 SetZero 操作，SetZero 应该只在单独的校准工具中使用
 */
class RobotBringup {
public:
    // 停止条件类型：返回 true 表示应该停止
    using StopCondition = std::function<bool()>;

    explicit RobotBringup(std::shared_ptr<RobstrideController> controller);
    ~RobotBringup() = default;

    /**
     * @brief 绑定所有 CAN 接口
     * @return true 成功, false 失败
     */
    bool BindAllCAN();

    /**
     * @brief 绑定所有电机
     * @param motor_indices 输出：电机索引数组 (12个)
     * @return true 成功, false 失败
     */
    bool BindAllMotors(std::vector<int>& motor_indices);

    /**
     * @brief 使能所有电机
     * @param motor_indices 电机索引数组
     * @return true 成功, false 失败
     */
    bool EnableAll(const std::vector<int>& motor_indices);

    /**
     * @brief 开启所有电机的自动上报
     * @param motor_indices 电机索引数组
     * @return true 成功, false 失败
     */
    bool EnableAutoReportAll(const std::vector<int>& motor_indices);

    /**
     * @brief 等待所有电机反馈在线
     * @param motor_indices 电机索引数组
     * @param timeout_ms 超时时间 (毫秒)
     * @return true 所有电机在线, false 超时
     */
    bool WaitForFeedback(const std::vector<int>& motor_indices, int timeout_ms = 3000);

    /**
     * @brief 读取当前所有电机的绝对位置
     * @param motor_indices 电机索引数组
     * @return 位置数组 (弧度)
     */
    std::vector<float> ReadCurrentAbsPositions(const std::vector<int>& motor_indices) const;

    /**
     * @brief 平滑移动电机到目标位置
     * @param motor_indices 电机索引数组
     * @param from_abs 起始位置
     * @param to_abs 目标位置
     * @param duration_sec 移动时间 (秒)
     * @param hz 控制频率
     * @param stop_condition 可选的停止条件函数，返回 true 时中断移动
     * @return true 成功完成, false 被中断或失败
     */
    bool SmoothMoveAbs(const std::vector<int>& motor_indices,
                       const std::vector<float>& from_abs,
                       const std::vector<float>& to_abs,
                       float duration_sec = 2.0f,
                       int hz = 100,
                       StopCondition stop_condition = nullptr);

    /**
     * @brief 从当前姿态平滑移动到 offset 姿态
     * @param motor_indices 电机索引数组
     * @param duration_sec 移动时间 (秒)
     * @param stop_condition 可选的停止条件函数
     * @return true 成功
     */
    bool MoveCurrentToOffsets(const std::vector<int>& motor_indices,
                              float duration_sec = 2.0f,
                              StopCondition stop_condition = nullptr);

    /**
     * @brief 保持电机在 offset 位置
     * @param motor_indices 电机索引数组
     */
    void HoldOffsets(const std::vector<int>& motor_indices);

    /**
     * @brief 获取控制器指针
     */
    std::shared_ptr<RobstrideController> GetController() const { return controller_; }

private:
    std::shared_ptr<RobstrideController> controller_;
    std::vector<std::shared_ptr<CANInterface>> can_ifaces_;
};