#pragma once
#include "input_source.hpp"
#include <memory>

// 前向声明
class Gamepad;

/**
 * @brief Gamepad 命令输入源
 *
 * 将 Gamepad 轴值映射为速度命令：
 * - axis1 (左摇杆上下) -> vx
 * - axis0 (左摇杆左右) -> vy (默认关闭)
 * - axis3 (右摇杆左右) -> yaw_rate
 */
class GamepadCommandSource : public ICommandSource {
public:
    /**
     * @brief 构造函数
     * @param gamepad Gamepad 实例
     * @param vx_scale vx 缩放因子，默认 0.5
     * @param vy_scale vy 缩放因子，默认 0（关闭横移）
     * @param yaw_scale yaw 缩放因子，默认 0.5
     */
    explicit GamepadCommandSource(std::shared_ptr<Gamepad> gamepad,
                                   float vx_scale = 0.5f,
                                   float vy_scale = 0.0f,
                                   float yaw_scale = 0.5f);

    ~GamepadCommandSource() override = default;

    void Update() override {}
    std::vector<float> GetCommand() const override;
    bool IsReady() const override;
    const char* Name() const override { return "gamepad"; }

private:
    std::shared_ptr<Gamepad> gamepad_;
    float vx_scale_;
    float vy_scale_;
    float yaw_scale_;
};