#include "gamepad_command_source.hpp"
#include "observations.hpp"

GamepadCommandSource::GamepadCommandSource(std::shared_ptr<Gamepad> gamepad,
                                           float vx_scale,
                                           float vy_scale,
                                           float yaw_scale)
    : gamepad_(std::move(gamepad))
    , vx_scale_(vx_scale)
    , vy_scale_(vy_scale)
    , yaw_scale_(yaw_scale) {
}

std::vector<float> GamepadCommandSource::GetCommand() const {
    std::vector<float> cmd(3, 0.0f);
    if (!gamepad_ || !gamepad_->IsFresh(500)) {
        return cmd;
    }

    // 与现有 CommandComponent::GetObs() 保持一致
    // axis1: 前进/后退，正值后退，负值前进，所以取负
    cmd[0] = -gamepad_->GetAxis(1) * vx_scale_;
    // axis0: 横移，默认关闭 (vy_scale_ = 0)
    cmd[1] = -gamepad_->GetAxis(0) * vy_scale_;
    // axis3: 转向，正值右转，负值左转，所以取负
    cmd[2] = -gamepad_->GetAxis(3) * yaw_scale_;

    return cmd;
}

bool GamepadCommandSource::IsReady() const {
    return gamepad_ && gamepad_->IsFresh(500);
}