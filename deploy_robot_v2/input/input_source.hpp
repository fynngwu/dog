#pragma once
#include <vector>
#include <memory>
#include <string>
#include "observations.hpp"

/**
 * @brief 命令输入源接口
 *
 * 抽象不同的输入源（gamepad、keyboard、ROS topic 等），
 * 为 CommandComponent 提供统一的命令获取接口。
 *
 * 命令格式：[vx, vy, yaw_rate]
 * - vx: 前进速度 (m/s)，正值前进，负值后退
 * - vy: 横移速度 (m/s)，正值左移，负值右移
 * - yaw_rate: 转向角速度 (rad/s)，正值左转，负值右转
 */
class ICommandSource {
public:
    virtual ~ICommandSource() = default;

    /**
     * @brief 更新输入源状态（主循环调用）
     */
    virtual void Update() = 0;

    /**
     * @brief 获取当前命令
     * @return [vx, vy, yaw_rate]
     */
    virtual std::vector<float> GetCommand() const = 0;

    /**
     * @brief 检查输入源是否就绪
     * @return true 就绪
     */
    virtual bool IsReady() const = 0;

    /**
     * @brief 获取输入源名称
     */
    virtual const char* Name() const = 0;
};

/**
 * @brief 创建输入源
 * @param input_mode 输入模式: "gamepad", "keyboard", "auto"
 * @return 输入源智能指针
 */
std::shared_ptr<ICommandSource> CreateInputSource(const std::string& input_mode);